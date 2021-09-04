// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef POSESENSORCLASS_H
#define POSESENSORCLASS_H

#include <mars/core_state.h>
#include <mars/ekf.h>
#include <mars/general_functions/utils.h>
#include <mars/sensors/bind_sensor_data.h>
#include <mars/sensors/pose/pose_measurement_type.h>
#include <mars/sensors/pose/pose_sensor_state_type.h>
#include <mars/sensors/update_sensor_abs_class.h>
#include <mars/time.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/core_state_type.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

namespace mars
{
using PoseSensorData = BindSensorData<PoseSensorStateType>;

class PoseSensorClass : public UpdateSensorAbsClass
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PoseSensorClass(std::string name, std::shared_ptr<CoreState> core_states)
  {
    name_ = std::move(name);
    core_states_ = core_states;
    const_ref_to_nav_ = false;
    initial_calib_provided_ = false;

    std::cout << "Created: [" << this->name_ << "] Sensor" << std::endl;
  }

  virtual ~PoseSensorClass() = default;

  PoseSensorStateType get_state(std::shared_ptr<void> sensor_data)
  {
    PoseSensorData data = *static_cast<PoseSensorData*>(sensor_data.get());
    return data.state_;
  }

  Eigen::MatrixXd get_covariance(std::shared_ptr<void> sensor_data)
  {
    PoseSensorData data = *static_cast<PoseSensorData*>(sensor_data.get());
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
    PoseMeasurementType measurement = *static_cast<PoseMeasurementType*>(sensor_data.get());

    PoseSensorData sensor_state;
    std::string calibration_type;

    if (this->initial_calib_provided_)
    {
      calibration_type = "Given";

      PoseSensorData calib = *static_cast<PoseSensorData*>(initial_calib_.get());

      sensor_state.state_ = calib.state_;
      sensor_state.sensor_cov_ = calib.sensor_cov_;
    }
    else
    {
      //      calibration_type = "Auto";

      //      Eigen::Vector3d p_wp(measurement.position_);
      //      Eigen::Quaterniond q_wp(measurement.orientation_);

      //      Eigen::Vector3d p_wi(latest_core_data->state_.p_wi_);
      //      Eigen::Quaterniond q_wi(latest_core_data->state_.q_wi_);
      //      Eigen::Matrix3d r_wi(q_wi.toRotationMatrix());

      //      Eigen::Vector3d p_ip = r_wi.transpose() * (p_wp - p_wi);
      //      Eigen::Quaterniond q_ip = q_wi.conjugate() * q_wp;

      //      // Calibration, position / rotation imu-pose
      //      sensor_state.state_.p_ip_ = p_ip;
      //      sensor_state.state_.q_ip_ = q_ip;

      //      // The covariance should enclose the initialization with a 3 Sigma bound
      //      Eigen::Matrix<double, 6, 1> std;
      //      std << 1, 1, 1, (35 * M_PI / 180), (35 * M_PI / 180), (35 * M_PI / 180);
      //      sensor_state.sensor_cov_ = std.cwiseProduct(std).asDiagonal();

      std::cout << "Pose calibration AUTO init not implemented yet" << std::endl;
      exit(EXIT_FAILURE);
    }

    // Bypass core state for the returned object
    BufferDataType result(std::make_shared<CoreType>(*latest_core_data.get()),
                          std::make_shared<PoseSensorData>(sensor_state));

    is_initialized_ = true;

    std::cout << "Info: Initialized [" << name_ << "] with [" << calibration_type << "] Calibration at t=" << timestamp
              << std::endl;

    if (!initial_calib_provided_)
    {
      std::cout << "Info: [" << name_ << "] Calibration(rounded):" << std::endl;
      std::cout << "\tPosition[m]: [" << sensor_state.state_.p_ip_.transpose() << " ]" << std::endl;
      std::cout << "\tOrientation[deg]: ["
                << sensor_state.state_.q_ip_.toRotationMatrix().eulerAngles(0, 1, 2).transpose() * (180 / M_PI) << " ]"
                << std::endl;
    }

    return result;
  }

  bool CalcUpdate(const Time& /*timestamp*/, std::shared_ptr<void> measurement, const CoreStateType& prior_core_state,
                  std::shared_ptr<void> latest_sensor_data, const Eigen::MatrixXd& prior_cov,
                  BufferDataType* new_state_data)
  {
    // Cast the sensor measurement and prior state information
    PoseMeasurementType* meas = static_cast<PoseMeasurementType*>(measurement.get());
    PoseSensorData* prior_sensor_data = static_cast<PoseSensorData*>(latest_sensor_data.get());

    // Decompose sensor measurement
    Eigen::Vector3d p_meas = meas->position_;
    Eigen::Quaternion<double> q_meas = meas->orientation_;

    // Extract sensor state
    PoseSensorStateType prior_sensor_state(prior_sensor_data->state_);

    // Generate measurement noise matrix
    const Eigen::Matrix<double, 6, 6> R_meas = this->R_.asDiagonal();

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
    const Eigen::Matrix3d R_ip = prior_sensor_state.q_ip_.toRotationMatrix();

    // Position
    const Eigen::Matrix3d Hp_pwi = I_3;
    const Eigen::Matrix3d Hp_vwi = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hp_rwi = -R_wi * Utils::Skew(P_ip);
    const Eigen::Matrix3d Hp_bw = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hp_ba = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hp_ip = R_wi;
    const Eigen::Matrix3d Hp_rip = Eigen::Matrix3d::Zero();

    // Assemble the jacobian for the position (horizontal)
    // H_p = [Hp_pwi Hp_vwi Hp_rwi Hp_bw Hp_ba Hp_ip Hp_rip];
    Eigen::MatrixXd H_p(3, Hp_pwi.cols() + Hp_vwi.cols() + Hp_rwi.cols() + Hp_bw.cols() + Hp_ba.cols() + Hp_ip.cols() +
                               Hp_rip.cols());
    H_p << Hp_pwi, Hp_vwi, Hp_rwi, Hp_bw, Hp_ba, Hp_ip, Hp_rip;

    // Orientation
    const Eigen::Matrix3d Hr_pwi = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hr_vwi = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hr_rwi = R_ip.transpose();
    const Eigen::Matrix3d Hr_bw = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hr_ba = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hr_pip = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hr_rip = I_3;

    // Assemble the jacobian for the orientation (horizontal)
    // H_r = [Hr_pwi Hr_vwi Hr_rwi Hr_bw Hr_ba Hr_pip Hr_rip];
    Eigen::MatrixXd H_r(3, Hr_pwi.cols() + Hr_vwi.cols() + Hr_rwi.cols() + Hr_bw.cols() + Hr_ba.cols() + Hr_pip.cols() +
                               Hr_rip.cols());
    H_r << Hr_pwi, Hr_vwi, Hr_rwi, Hr_bw, Hr_ba, Hr_pip, Hr_rip;

    // Combine all jacobians (vertical)
    Eigen::MatrixXd H(H_p.rows() + H_r.rows(), H_r.cols());
    H << H_p, H_r;

    // Calculate the residual z = z~ - (estimate)
    // Position
    const Eigen::Vector3d p_est = P_wi + R_wi * P_ip;
    const Eigen::Vector3d res_p = p_meas - p_est;
    // Orientation
    const Eigen::Quaternion<double> q_est = prior_core_state.q_wi_ * prior_sensor_state.q_ip_;
    const Eigen::Quaternion<double> res_q = q_est.inverse() * q_meas;
    const Eigen::Vector3d res_r = 2 * res_q.vec() / res_q.w();

    // Combine residuals (vertical)
    Eigen::MatrixXd res(res_p.rows() + res_r.rows(), 1);
    res << res_p, res_r;

    // Perform EKF calculations
    mars::Ekf ekf(H, R_meas, res, P);
    const Eigen::MatrixXd correction = ekf.CalculateCorrection();
    assert(correction.size() == size_of_full_error_state * 1);

    Eigen::MatrixXd P_updated = ekf.CalculateCovUpdate();
    assert(P_updated.size() == size_of_full_error_state * size_of_full_error_state);
    P_updated = Utils::EnforceMatrixSymmetry(P_updated);

    // Apply Core Correction
    CoreStateVector core_correction = correction.block(0, 0, CoreStateType::size_error_, 1);
    CoreStateType corrected_core_state = CoreStateType::ApplyCorrection(prior_core_state, core_correction);

    // Apply Sensor Correction
    const Eigen::MatrixXd sensor_correction = correction.block(size_of_core_state, 0, size_of_sensor_state, 1);
    const PoseSensorStateType corrected_sensor_state = ApplyCorrection(prior_sensor_state, sensor_correction);

    // Return Results
    // CoreState data
    CoreType core_data;
    core_data.cov_ = P_updated.block(0, 0, CoreStateType::size_error_, CoreStateType::size_error_);
    core_data.state_ = corrected_core_state;

    // SensorState data
    std::shared_ptr<PoseSensorData> sensor_data(std::make_shared<PoseSensorData>());
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

  PoseSensorStateType ApplyCorrection(const PoseSensorStateType& prior_sensor_state, const Eigen::MatrixXd& correction)
  {
    // state + error state correction
    // with quaternion from small angle approx -> new state

    PoseSensorStateType corrected_sensor_state;
    corrected_sensor_state.p_ip_ = prior_sensor_state.p_ip_ + correction.block(0, 0, 3, 1);
    corrected_sensor_state.q_ip_ =
        Utils::ApplySmallAngleQuatCorr(prior_sensor_state.q_ip_, correction.block(3, 0, 3, 1));
    return corrected_sensor_state;
  }
};
}

#endif  // POSESENSORCLASS_H
