// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef MAG_SENSOR_CLASS_H
#define MAG_SENSOR_CLASS_H

#include <mars/core_state.h>
#include <mars/ekf.h>
#include <mars/general_functions/utils.h>
#include <mars/sensors/bind_sensor_data.h>
#include <mars/sensors/mag/mag_measurement_type.h>
#include <mars/sensors/mag/mag_sensor_state_type.h>
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
using MagSensorData = BindSensorData<MagSensorStateType>;

class MagSensorClass : public UpdateSensorAbsClass
{
private:
  bool normalize_{ false };

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MagSensorClass(std::string name, std::shared_ptr<CoreState> core_states)
  {
    name_ = std::move(name);
    core_states_ = core_states;
    const_ref_to_nav_ = false;
    initial_calib_provided_ = false;

    // Sensor specific information
    // setup_sensor_properties();
    std::cout << "Created: [" << this->name_ << "] Sensor" << std::endl;
  }

  MagSensorStateType get_state(std::shared_ptr<void> sensor_data)
  {
    MagSensorData data = *static_cast<MagSensorData*>(sensor_data.get());
    return data.state_;
  }

  Eigen::MatrixXd get_covariance(std::shared_ptr<void> sensor_data)
  {
    MagSensorData data = *static_cast<MagSensorData*>(sensor_data.get());
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
    MagMeasurementType measurement = *static_cast<MagMeasurementType*>(sensor_data.get());

    MagSensorData sensor_state;
    std::string calibration_type;

    if (this->initial_calib_provided_)
    {
      calibration_type = "Given";

      MagSensorData calib = *static_cast<MagSensorData*>(initial_calib_.get());

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
    }

    // Bypass core state for the returned object
    BufferDataType result(std::make_shared<CoreType>(*latest_core_data.get()),
                          std::make_shared<MagSensorData>(sensor_state));

    // TODO
    // sensor_data.ref_to_nav = 0; //obj.calc_ref_to_nav(measurement, latest_core_state);

    is_initialized_ = true;

    std::cout << "Info: Initialized [" << name_ << "] with [" << calibration_type << "] Calibration at t=" << timestamp
              << std::endl;

    if (!initial_calib_provided_)
    {
      std::cout << "Info: [" << name_ << "] Calibration(rounded):" << std::endl;
      std::cout << "\tMag Vector []: [" << sensor_state.state_.mag_.transpose() << " ]" << std::endl;
      std::cout << "\tOrientation Mag in IMU [deg]: ["
                << sensor_state.state_.q_im_.toRotationMatrix().eulerAngles(0, 1, 2).transpose() * (180 / M_PI) << " ]"
                << std::endl;
    }

    return result;
  }

  bool CalcUpdate(const Time& /*timestamp*/, std::shared_ptr<void> measurement, const CoreStateType& prior_core_state,
                  std::shared_ptr<void> latest_sensor_data, const Eigen::MatrixXd& prior_cov,
                  BufferDataType* new_state_data)
  {
    // Cast the sensor measurement and prior state information
    MagMeasurementType* meas = static_cast<MagMeasurementType*>(measurement.get());
    MagSensorData* prior_sensor_data = static_cast<MagSensorData*>(latest_sensor_data.get());

    // Decompose sensor measurement
    Eigen::Vector3d mag_meas;

    if (normalize_)
    {
      mag_meas = meas->mag_vector_ / meas->mag_vector_.norm();
    }
    else
    {
      mag_meas = meas->mag_vector_;
    }

    // Extract sensor state
    MagSensorStateType prior_sensor_state(prior_sensor_data->state_);

    // Generate measurement noise matrix
    const Eigen::Matrix<double, 3, 3> R_meas = this->R_.asDiagonal();

    const int size_of_core_state = CoreStateType::size_error_;
    const int size_of_sensor_state = prior_sensor_state.cov_size_;
    const int size_of_full_error_state = size_of_core_state + size_of_sensor_state;
    const Eigen::MatrixXd P = prior_cov;
    assert(P.size() == size_of_full_error_state * size_of_full_error_state);

    // Calculate the measurement jacobian H
    // const Eigen::Matrix3d I_3 = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d R_wi = prior_core_state.q_wi_.toRotationMatrix();
    const Eigen::Vector3d mag_w = prior_sensor_state.mag_;
    const Eigen::Matrix3d R_im = prior_sensor_state.q_im_.toRotationMatrix();

    // Orientation
    const Eigen::Matrix3d Hm_pwi = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hm_vwi = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hm_rwi = R_im.transpose() * Utils::Skew(R_wi.transpose() * mag_w);
    const Eigen::Matrix3d Hm_bw = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hm_ba = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hm_mag = R_im.transpose() * R_wi.transpose();
    const Eigen::Matrix3d Hm_rim = Utils::Skew(R_im.transpose() * R_wi.transpose() * mag_w);

    // Assemble the jacobian for the orientation (horizontal)
    // H_r = [Hr_pwi Hr_vwi Hr_rwi Hr_bw Hr_ba Hr_mag Hr_rim];
    Eigen::MatrixXd H(3, Hm_pwi.cols() + Hm_vwi.cols() + Hm_rwi.cols() + Hm_bw.cols() + Hm_ba.cols() + Hm_mag.cols() +
                             Hm_rim.cols());
    H << Hm_pwi, Hm_vwi, Hm_rwi, Hm_bw, Hm_ba, Hm_mag, Hm_rim;

    // Calculate the residual z = z~ - (estimate)
    // Position
    const Eigen::Vector3d mag_est = R_im.transpose() * R_wi.transpose() * mag_w;
    const Eigen::Vector3d res = mag_meas - mag_est;

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
    const MagSensorStateType corrected_sensor_state = ApplyCorrection(prior_sensor_state, sensor_correction);

    // Return Results
    // CoreState data
    CoreType core_data;
    core_data.cov_ = P_updated.block(0, 0, CoreStateType::size_error_, CoreStateType::size_error_);
    core_data.state_ = corrected_core_state;

    // SensorState data
    std::shared_ptr<MagSensorData> sensor_data(std::make_shared<MagSensorData>());
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

  MagSensorStateType ApplyCorrection(const MagSensorStateType& prior_sensor_state, const Eigen::MatrixXd& correction)
  {
    // state + error state correction
    // with quaternion from small angle approx -> new state

    MagSensorStateType corrected_sensor_state;
    corrected_sensor_state.mag_ = prior_sensor_state.mag_ + correction.block(0, 0, 3, 1);
    corrected_sensor_state.q_im_ =
        Utils::ApplySmallAngleQuatCorr(prior_sensor_state.q_im_, correction.block(3, 0, 3, 1));
    return corrected_sensor_state;
  }

  void set_normalize(const bool& normalize)
  {
    normalize_ = normalize;
  }
};
}

#endif  // MAG_SENSOR_CLASS_H
