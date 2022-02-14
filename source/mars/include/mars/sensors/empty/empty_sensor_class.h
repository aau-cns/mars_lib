// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef GPSVELSENSORCLASS_H
#define GPSVELSENSORCLASS_H

#include <mars/core_state.h>
#include <mars/ekf.h>
#include <mars/sensors/bind_sensor_data.h>
#include <mars/sensors/empty/empty_measurement_type.h>
#include <mars/sensors/empty/empty_sensor_state_type.h>
#include <mars/sensors/update_sensor_abs_class.h>
#include <mars/time.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

namespace mars
{
using EmptySensorData = BindSensorData<EmptySensorStateType>;

class EmptySensorClass : public UpdateSensorAbsClass
{
private:
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EmptySensorClass(std::string name, std::shared_ptr<CoreState> core_states)
  {
    name_ = name;
    core_states_ = core_states;
    const_ref_to_nav_ = false;
    initial_calib_provided_ = false;

    std::cout << "Created: [" << this->name_ << "] Sensor" << std::endl;
  }

  EmptySensorStateType get_state(std::shared_ptr<void> sensor_data)
  {
    EmptySensorData data = *static_cast<EmptySensorData*>(sensor_data.get());
    return data.state_;
  }

  Eigen::MatrixXd get_covariance(std::shared_ptr<void> sensor_data)
  {
    EmptySensorData data = *static_cast<EmptySensorData*>(sensor_data.get());
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
    EmptyMeasurementType measurement = *static_cast<EmptyMeasurementType*>(sensor_data.get());

    EmptySensorData sensor_state;
    std::string calibration_type;

    if (this->initial_calib_provided_)
    {
      calibration_type = "Given";

      EmptySensorData calib = *static_cast<EmptySensorData*>(initial_calib_.get());

      sensor_state.state_ = calib.state_;
      sensor_state.sensor_cov_ = calib.sensor_cov_;
    }
    else
    {
      std::cout << "Empty calibration AUTO init not implemented yet" << std::endl;
      exit(EXIT_FAILURE);
    }

    // Bypass core state for the returned object
    BufferDataType result(std::make_shared<CoreType>(*latest_core_data.get()),
                          std::make_shared<EmptySensorData>(sensor_state));

    is_initialized_ = true;

    std::cout << "Info: Initialized [" << name_ << "] with [" << calibration_type << "] Calibration at t=" << timestamp
              << std::endl;

    std::cout << "Info: [" << name_ << "] Calibration(rounded):" << std::endl;
    std::cout << "\tPosition[m]: [" << sensor_state.state_.value_.transpose() << " ]" << std::endl;

    return result;
  }

  bool CalcUpdate(const Time& timestamp, std::shared_ptr<void> measurement, const CoreStateType& prior_core_state,
                  std::shared_ptr<void> latest_sensor_data, const Eigen::MatrixXd& prior_cov,
                  BufferDataType* new_state_data)
  {
    // Cast the sensor measurement and prior state information
    EmptyMeasurementType* meas = static_cast<EmptyMeasurementType*>(measurement.get());
    EmptySensorData* prior_sensor_data = static_cast<EmptySensorData*>(latest_sensor_data.get());

    // Extract sensor state
    EmptySensorStateType prior_sensor_state(prior_sensor_data->state_);

    // Return Results
    // CoreState data
    CoreType core_data;
    core_data.cov_ = prior_cov.block(0, 0, CoreStateType::size_error_, CoreStateType::size_error_);
    core_data.state_ = prior_core_state;

    // SensorState data
    std::shared_ptr<EmptySensorData> sensor_data(std::make_shared<EmptySensorData>());
    sensor_data->set_cov(prior_cov);
    sensor_data->state_ = prior_sensor_state;

    BufferDataType state_entry(std::make_shared<CoreType>(core_data), sensor_data);

    *new_state_data = state_entry;

    return true;

    //    // Generate measurement noise matrix
    //    Eigen::MatrixXd R_meas(R_.asDiagonal());

    //    const int size_of_core_state = CoreStateType::size_error_;
    //    const int size_of_sensor_state = prior_sensor_state.cov_size_;
    //    const int size_of_full_error_state = size_of_core_state + size_of_sensor_state;
    //    const Eigen::MatrixXd P = prior_cov;
    //    assert(P.size() == size_of_full_error_state * size_of_full_error_state);

    //    // Calculate the measurement jacobian H
    //    const Eigen::Matrix3d I_3 = Eigen::Matrix3d::Identity();
    //    const Eigen::Matrix3d O_3 = Eigen::Matrix3d::Zero();

    //    const Eigen::Vector3d P_wi = prior_core_state.p_wi_;
    //    const Eigen::Vector3d V_wi = prior_core_state.v_wi_;
    //    const Eigen::Matrix3d R_wi = prior_core_state.q_wi_.toRotationMatrix();

    //    // Dummy
    //    const Eigen::Matrix3d Hp_pwi = O_3;
    //    const Eigen::Matrix3d Hp_vwi = O_3;
    //    const Eigen::Matrix3d Hp_rwi = O_3;
    //    const Eigen::Matrix3d Hp_bw = O_3;
    //    const Eigen::Matrix3d Hp_ba = O_3;

    //    const Eigen::Matrix3d Hp_pig = O_3;
    //    const Eigen::Matrix3d Hp_pgw_w = O_3;
    //    const Eigen::Matrix3d Hp_rgw_w = O_3;

    //    const int num_states = static_cast<int>(Hp_pwi.cols() + Hp_vwi.cols() + Hp_rwi.cols() + Hp_bw.cols() +
    //                                            Hp_ba.cols() + Hp_pig.cols() + Hp_pgw_w.cols() + Hp_rgw_w.cols());

    //    // Assemble the jacobian for the position (horizontal)
    //    Eigen::MatrixXd H_p(3, num_states);
    //    H_p << Hp_pwi, Hp_vwi, Hp_rwi, Hp_bw, Hp_ba, Hp_pig, Hp_pgw_w, Hp_rgw_w;

    //    // Combine all jacobians (vertical)
    //    Eigen::MatrixXd H(H_p.rows(), H_p.cols());
    //    H << H_p;

    //    // Calculate the residual z = z~ - (estimate)
    //    const Eigen::Vector3d res_p(Eigen::Vector3d::Zero());

    //    // Combine residuals (vertical)
    //    Eigen::MatrixXd res(res_p.rows(), 1);
    //    res << res_p;

    //    // Perform EKF calculations
    //    mars::Ekf ekf(H, R_meas, res, P);
    //    const Eigen::MatrixXd correction = ekf.CalculateCorrection();
    //    assert(correction.size() == size_of_full_error_state * 1);

    //    Eigen::MatrixXd P_updated = ekf.CalculateCovUpdate();
    //    assert(P_updated.size() == size_of_full_error_state * size_of_full_error_state);
    //    P_updated = Utils::EnforceMatrixSymmetry(P_updated);

    //    // Apply Core Correction
    //    CoreStateVector core_correction = correction.block(0, 0, CoreStateType::size_error_, 1);
    //    CoreStateType corrected_core_state = CoreStateType::ApplyCorrection(prior_core_state, core_correction);

    //    // Apply Sensor Correction
    //    const Eigen::MatrixXd sensor_correction = correction.block(size_of_core_state, 0, size_of_sensor_state, 1);
    //    const EmptySensorStateType corrected_sensor_state = ApplyCorrection(prior_sensor_state, sensor_correction);

    //    // Return Results
    //    // CoreState data
    //    CoreType core_data;
    //    core_data.cov_ = P_updated.block(0, 0, CoreStateType::size_error_, CoreStateType::size_error_);
    //    core_data.state_ = corrected_core_state;

    //    // SensorState data
    //    std::shared_ptr<EmptySensorData> sensor_data(std::make_shared<EmptySensorData>());
    //    sensor_data->set_cov(P_updated);
    //    sensor_data->state_ = corrected_sensor_state;

    //    BufferDataType state_entry(std::make_shared<CoreType>(core_data), sensor_data);

    //    *new_state_data = state_entry;

    //    return true;
  }

  EmptySensorStateType ApplyCorrection(const EmptySensorStateType& prior_sensor_state,
                                       const Eigen::MatrixXd& correction)
  {
    // state + error state correction
    // with quaternion from small angle approx -> new state

    EmptySensorStateType corrected_sensor_state;
    corrected_sensor_state.value_ = prior_sensor_state.value_ + correction.block(0, 0, 3, 1);
    return corrected_sensor_state;
  }
};
}

#endif  // GPSVELSENSORCLASS_H
