// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef POSITIONSENSORCLASS_H
#define POSITIONSENSORCLASS_H

#include <mars/core_state.h>
#include <mars/ekf.h>
#include <mars/sensors/bind_sensor_data.h>
#include <mars/sensors/position/position_measurement_type.h>
#include <mars/sensors/position/position_sensor_state_type.h>
#include <mars/sensors/update_sensor_abs_class.h>
#include <mars/time.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

namespace mars
{
using PositionSensorData = BindSensorData<PositionSensorStateType>;

class PositionSensorClass : public UpdateSensorAbsClass
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PositionSensorClass(const std::string& name, std::shared_ptr<CoreState> core_states)
  {
    name_ = name;
    core_states_ = std::move(core_states);
    const_ref_to_nav_ = false;
    initial_calib_provided_ = false;

    // chi2
    chi2_.set_dof(3);

    std::cout << "Created: [" << this->name_ << "] Sensor" << std::endl;
  }

  virtual ~PositionSensorClass() = default;

  PositionSensorStateType get_state(const std::shared_ptr<void>& sensor_data)
  {
    PositionSensorData data = *static_cast<PositionSensorData*>(sensor_data.get());
    return data.state_;
  }

  Eigen::MatrixXd get_covariance(const std::shared_ptr<void>& sensor_data)
  {
    PositionSensorData data = *static_cast<PositionSensorData*>(sensor_data.get());
    return data.get_full_cov();
  }

  void set_initial_calib(std::shared_ptr<void> calibration)
  {
    initial_calib_ = calibration;
    initial_calib_provided_ = true;
  }

  BufferDataType Initialize(const Time& timestamp, std::shared_ptr<void> sensor_data,
                            std::shared_ptr<CoreType> latest_core_data)
  {
    PositionMeasurementType measurement = *static_cast<PositionMeasurementType*>(sensor_data.get());

    PositionSensorData sensor_state;
    std::string calibration_type;

    if (this->initial_calib_provided_)
    {
      calibration_type = "Given";

      PositionSensorData calib = *static_cast<PositionSensorData*>(initial_calib_.get());

      sensor_state.state_ = calib.state_;
      sensor_state.sensor_cov_ = calib.sensor_cov_;
    }
    else
    {
      calibration_type = "Auto";
      std::cout << "Position calibration AUTO init not implemented yet" << std::endl;
      exit(EXIT_FAILURE);
    }

    // Bypass core state for the returned object
    BufferDataType result(std::make_shared<CoreType>(*latest_core_data.get()),
                          std::make_shared<PositionSensorData>(sensor_state));

    is_initialized_ = true;

    std::cout << "Info: Initialized [" << name_ << "] with [" << calibration_type << "] Calibration at t=" << timestamp
              << std::endl;

    std::cout << "Info: [" << name_ << "] Calibration(rounded):" << std::endl;
    std::cout << "\tPosition[m]: [" << sensor_state.state_.p_ip_.transpose() << " ]" << std::endl;

    return result;
  }

  bool CalcUpdate(const Time& /*timestamp*/, std::shared_ptr<void> measurement, const CoreStateType& prior_core_state,
                  std::shared_ptr<void> latest_sensor_data, const Eigen::MatrixXd& prior_cov,
                  BufferDataType* new_state_data)
  {
    // Cast the sensor measurement and prior state information
    PositionMeasurementType* meas = static_cast<PositionMeasurementType*>(measurement.get());
    PositionSensorData* prior_sensor_data = static_cast<PositionSensorData*>(latest_sensor_data.get());

    // Decompose sensor measurement
    Eigen::Vector3d p_meas = meas->position_;

    // Extract sensor state
    PositionSensorStateType prior_sensor_state(prior_sensor_data->state_);

    // Generate measurement noise matrix
    const Eigen::Matrix<double, 3, 3> R_meas = this->R_.asDiagonal();

    const int size_of_core_state = CoreStateType::size_error_;
    const int size_of_sensor_state = prior_sensor_state.cov_size_;
    const int size_of_full_error_state = size_of_core_state + size_of_sensor_state;
    const Eigen::MatrixXd P = prior_cov;
    assert(P.size() == size_of_full_error_state * size_of_full_error_state);

    // Calculate the measurement jacobian H
    const Eigen::Matrix3d I_3 = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d P_wi = prior_core_state.p_wi_;
    const Eigen::Matrix3d R_wi = prior_core_state.q_wi_.toRotationMatrix();
    const Eigen::Vector3d P_ip = prior_sensor_state.p_ip_;

    // Position
    const Eigen::Matrix3d Hp_pwi = I_3;
    const Eigen::Matrix3d Hp_vwi = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hp_rwi = -R_wi * Utils::Skew(P_ip);
    const Eigen::Matrix3d Hp_bw = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hp_ba = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hp_pip = R_wi;

    // Assemble the jacobian for the position (horizontal)
    // H_p = [Hp_pwi Hp_vwi Hp_rwi Hp_bw Hp_ba Hp_pig ];
    Eigen::MatrixXd H(3, Hp_pwi.cols() + Hp_vwi.cols() + Hp_rwi.cols() + Hp_bw.cols() + Hp_ba.cols() + Hp_pip.cols());

    H << Hp_pwi, Hp_vwi, Hp_rwi, Hp_bw, Hp_ba, Hp_pip;

    // Calculate the residual z = z~ - (estimate)
    // Position
    const Eigen::Vector3d p_est = P_wi + R_wi * P_ip;
    const Eigen::Vector3d res = p_meas - p_est;

    // Perform EKF calculations
    mars::Ekf ekf(H, R_meas, res, P);
    const Eigen::MatrixXd correction = ekf.CalculateCorrection(chi2_);
    assert(correction.size() == size_of_full_error_state * 1);

    // Perform Chi2 test
    if (!chi2_.passed_ && chi2_.do_test_)
    {
      chi2_.PrintReport(name_);
      return false;
    }

    Eigen::MatrixXd P_updated = ekf.CalculateCovUpdate();
    assert(P_updated.size() == size_of_full_error_state * size_of_full_error_state);
    P_updated = Utils::EnforceMatrixSymmetry(P_updated);

    // Apply Core Correction
    CoreStateVector core_correction = correction.block(0, 0, CoreStateType::size_error_, 1);
    CoreStateType corrected_core_state = CoreStateType::ApplyCorrection(prior_core_state, core_correction);

    // Apply Sensor Correction
    const Eigen::MatrixXd sensor_correction = correction.block(size_of_core_state, 0, size_of_sensor_state, 1);
    const PositionSensorStateType corrected_sensor_state = ApplyCorrection(prior_sensor_state, sensor_correction);

    // Return Results
    // CoreState data
    CoreType core_data;
    core_data.cov_ = P_updated.block(0, 0, CoreStateType::size_error_, CoreStateType::size_error_);
    core_data.state_ = corrected_core_state;

    // SensorState data
    std::shared_ptr<PositionSensorData> sensor_data(std::make_shared<PositionSensorData>());
    sensor_data->set_cov(P_updated);
    sensor_data->state_ = corrected_sensor_state;

    BufferDataType state_entry(std::make_shared<CoreType>(core_data), sensor_data);

    if (const_ref_to_nav_)
    {
      // corrected_sensor_data.ref_to_nav = prior_ref_to_nav;
    }
    else
    {
      // TODO also estimate ref to nav
    }

    *new_state_data = state_entry;

    return true;
  }

  PositionSensorStateType ApplyCorrection(const PositionSensorStateType& prior_sensor_state,
                                          const Eigen::MatrixXd& correction)
  {
    // state + error state correction

    PositionSensorStateType corrected_sensor_state;
    corrected_sensor_state.p_ip_ = prior_sensor_state.p_ip_ + correction.block(0, 0, 3, 1);
    return corrected_sensor_state;
  }
};
}  // namespace mars

#endif  // POSITIONSENSORCLASS_H
