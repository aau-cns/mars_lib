// Copyright (C) 2024 Christian Brommer, Control of Networked Systems, University of Klagenfurt,
// Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef VELOCITYSENSORCLASS_H
#define VELOCITYSENSORCLASS_H

#include <mars/core_state.h>
#include <mars/ekf.h>
#include <mars/sensors/bind_sensor_data.h>
#include <mars/sensors/update_sensor_abs_class.h>
#include <mars/sensors/velocity/velocity_measurement_type.h>
#include <mars/sensors/velocity/velocity_sensor_state_type.h>
#include <mars/time.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

namespace mars
{
using VelocitySensorData = BindSensorData<VelocitySensorStateType>;

class VelocitySensorClass : public UpdateSensorAbsClass
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VelocitySensorClass(const std::string& name, std::shared_ptr<CoreState> core_states)
  {
    name_ = name;
    core_states_ = std::move(core_states);
    const_ref_to_nav_ = false;
    initial_calib_provided_ = false;

    // chi2
    chi2_.set_dof(3);

    std::cout << "Created: [" << this->name_ << "] Sensor" << std::endl;
  }

  virtual ~VelocitySensorClass() = default;

  VelocitySensorStateType get_state(const std::shared_ptr<void>& sensor_data)
  {
    VelocitySensorData data = *static_cast<VelocitySensorData*>(sensor_data.get());
    return data.state_;
  }

  Eigen::MatrixXd get_covariance(const std::shared_ptr<void>& sensor_data)
  {
    VelocitySensorData data = *static_cast<VelocitySensorData*>(sensor_data.get());
    return data.get_full_cov();
  }

  void set_initial_calib(std::shared_ptr<void> calibration)
  {
    initial_calib_ = calibration;
    initial_calib_provided_ = true;
  }

  BufferDataType Initialize(const Time& timestamp, std::shared_ptr<void> /*sensor_data*/,
                            std::shared_ptr<CoreType> latest_core_data)
  {
    // VelocityMeasurementType measurement = *static_cast<VelocityMeasurementType*>(sensor_data.get());

    VelocitySensorData sensor_state;
    std::string calibration_type;

    if (this->initial_calib_provided_)
    {
      calibration_type = "Given";

      VelocitySensorData calib = *static_cast<VelocitySensorData*>(initial_calib_.get());

      sensor_state.state_ = calib.state_;
      sensor_state.sensor_cov_ = calib.sensor_cov_;
    }
    else
    {
      calibration_type = "Auto";

      std::cout << "Velocity calibration AUTO init not implemented yet" << std::endl;
      exit(EXIT_FAILURE);
    }

    // Bypass core state for the returned object
    BufferDataType result(std::make_shared<CoreType>(*latest_core_data.get()),
                          std::make_shared<VelocitySensorData>(sensor_state));

    is_initialized_ = true;

    std::cout << "Info: Initialized [" << name_ << "] with [" << calibration_type << "] Calibration at t=" << timestamp
              << std::endl;
    std::cout << "\tPosition[m]: [" << sensor_state.state_.p_iv_.transpose() << " ]" << std::endl;

    if (!initial_calib_provided_)
    {
      std::cout << "Info: [" << name_ << "] Calibration(rounded):" << std::endl;
      std::cout << "\tPosition[m]: [" << sensor_state.state_.p_iv_.transpose() << " ]" << std::endl;
    }

    return result;
  }

  bool CalcUpdate(const Time& /*timestamp*/, std::shared_ptr<void> measurement, const CoreStateType& prior_core_state,
                  std::shared_ptr<void> latest_sensor_data, const Eigen::MatrixXd& prior_cov,
                  BufferDataType* new_state_data)
  {
    // Cast the sensor measurement and prior state information
    VelocityMeasurementType* meas = static_cast<VelocityMeasurementType*>(measurement.get());
    VelocitySensorData* prior_sensor_data = static_cast<VelocitySensorData*>(latest_sensor_data.get());

    // Decompose sensor measurement
    Eigen::Vector3d v_meas = meas->velocity_;

    // Extract sensor state
    VelocitySensorStateType prior_sensor_state(prior_sensor_data->state_);

    // Generate measurement noise matrix and check
    // if noisevalues from the measurement object should be used
    Eigen::MatrixXd R_meas_dyn;
    if (meas->has_meas_noise && use_dynamic_meas_noise_)
    {
      meas->get_meas_noise(&R_meas_dyn);
    }
    else
    {
      R_meas_dyn = this->R_.asDiagonal();
    }

    const Eigen::Matrix<double, 3, 3> R_meas = R_meas_dyn;

    const int size_of_core_state = CoreStateType::size_error_;
    const int size_of_sensor_state = prior_sensor_state.cov_size_;
    const int size_of_full_error_state = size_of_core_state + size_of_sensor_state;
    const Eigen::MatrixXd P = prior_cov;
    assert(P.size() == size_of_full_error_state * size_of_full_error_state);

    // Calculate the measurement jacobian H
    const Eigen::Matrix3d I_3 = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d O_3 = Eigen::Matrix3d::Zero();

    const Eigen::Vector3d omega_i = prior_core_state.w_m_;  ///< Angular Velocity of the IMU Frame

    // const Eigen::Vector3d P_wi = prior_core_state.p_wi_;
    const Eigen::Vector3d V_wi = prior_core_state.v_wi_;
    const Eigen::Vector3d b_w = prior_core_state.b_w_;
    const Eigen::Matrix3d R_wi = prior_core_state.q_wi_.toRotationMatrix();
    const Eigen::Vector3d P_iv = prior_sensor_state.p_iv_;

    const Eigen::Matrix3d Hv_pwi = O_3;
    const Eigen::Matrix3d Hv_vwi = I_3;
    const Eigen::Matrix3d Hv_rwi = -R_wi * Utils::Skew(Utils::Skew(omega_i - b_w) * P_iv);
    const Eigen::Matrix3d Hv_bw = O_3;
    const Eigen::Matrix3d Hv_ba = O_3;

    const Eigen::Matrix3d Hv_piv = R_wi * Utils::Skew(omega_i - b_w);

    // Assemble the jacobian for the velocity (horizontal)
    const int num_states =
        static_cast<int>(Hv_pwi.cols() + Hv_vwi.cols() + Hv_rwi.cols() + Hv_bw.cols() + Hv_ba.cols() + Hv_piv.cols());

    // Combine all jacobians (vertical)
    Eigen::MatrixXd H(3, num_states);
    H << Hv_pwi, Hv_vwi, Hv_rwi, Hv_bw, Hv_ba, Hv_piv;

    Eigen::Vector3d v_est;
    v_est = V_wi + R_wi * Utils::Skew(omega_i - b_w) * P_iv;

    // Calculate the residual z = z~ - (estimate)
    // Velocity
    const Eigen::Vector3d res_v = v_meas - v_est;

    // Combine residuals (vertical)
    residual_ = Eigen::MatrixXd(res_v.rows(), 1);
    residual_ << res_v;

    // Perform EKF calculations
    mars::Ekf ekf(H, R_meas, residual_, P);
    const Eigen::MatrixXd correction = ekf.CalculateCorrection(&chi2_);
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
    const VelocitySensorStateType corrected_sensor_state = ApplyCorrection(prior_sensor_state, sensor_correction);

    // Return Results
    // CoreState data
    CoreType core_data;
    core_data.cov_ = P_updated.block(0, 0, CoreStateType::size_error_, CoreStateType::size_error_);
    core_data.state_ = corrected_core_state;

    // SensorState data
    std::shared_ptr<VelocitySensorData> sensor_data(std::make_shared<VelocitySensorData>());
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

  VelocitySensorStateType ApplyCorrection(const VelocitySensorStateType& prior_sensor_state,
                                          const Eigen::MatrixXd& correction)
  {
    // state + error state correction
    // with quaternion from small angle approx -> new state

    VelocitySensorStateType corrected_sensor_state;
    corrected_sensor_state.p_iv_ = prior_sensor_state.p_iv_ + correction.block(0, 0, 3, 1);
    return corrected_sensor_state;
  }
};  // namespace mars
}  // namespace mars

#endif  // VELOCITYSENSORCLASS_H
