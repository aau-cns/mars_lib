// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@ieee.org>

#ifndef ATTITUDE_SENSOR_CLASS_H
#define ATTITUDE_SENSOR_CLASS_H

#include <mars/core_state.h>
#include <mars/ekf.h>
#include <mars/general_functions/utils.h>
#include <mars/sensors/attitude/attitude_measurement_type.h>
#include <mars/sensors/attitude/attitude_sensor_state_type.h>
#include <mars/sensors/bind_sensor_data.h>
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
using AttitudeSensorData = BindSensorData<AttitudeSensorStateType>;

enum class AttitudeSensorType
{
  RP_TYPE,   //!< aviation attitude, roll and pitch only
  RPY_TYPE,  //!< full orientation, roll, pitch, and yaw
};

inline std::ostream& operator<<(std::ostream& os, AttitudeSensorType type)
{
  switch (type)
  {
    case AttitudeSensorType::RP_TYPE:
      os << "ROLL/PITCH";
      break;
    case AttitudeSensorType::RPY_TYPE:
      os << "ROLL/PITCH/YAW";
      break;
  }
  return os;
}

class AttitudeSensorClass : public UpdateSensorAbsClass
{
public:
private:
  AttitudeSensorType attitude_type_{ AttitudeSensorType::RPY_TYPE };

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AttitudeSensorClass(std::string name, std::shared_ptr<CoreState> core_states,
                      AttitudeSensorType type = AttitudeSensorType::RPY_TYPE)
  {
    name_ = std::move(name);
    core_states_ = core_states;
    const_ref_to_nav_ = false;
    initial_calib_provided_ = false;

    // set sensor type
    attitude_type_ = type;

    // Sensor specific information
    // setup_sensor_properties();
    std::cout << "Created: [" << this->name_ << "] Sensor (type: " << attitude_type_ << ")" << std::endl;
  }

  AttitudeSensorStateType get_state(std::shared_ptr<void> sensor_data)
  {
    AttitudeSensorData data = *static_cast<AttitudeSensorData*>(sensor_data.get());
    return data.state_;
  }

  Eigen::MatrixXd get_covariance(std::shared_ptr<void> sensor_data)
  {
    AttitudeSensorData data = *static_cast<AttitudeSensorData*>(sensor_data.get());
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
    AttitudeMeasurementType measurement = *static_cast<AttitudeMeasurementType*>(sensor_data.get());

    AttitudeSensorData sensor_state;
    std::string calibration_type;

    if (this->initial_calib_provided_)
    {
      calibration_type = "Given";

      AttitudeSensorData calib = *static_cast<AttitudeSensorData*>(initial_calib_.get());

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
                          std::make_shared<AttitudeSensorData>(sensor_state));

    // TODO
    // sensor_data.ref_to_nav = 0; //obj.calc_ref_to_nav(measurement, latest_core_state);

    is_initialized_ = true;

    std::cout << "Info: Initialized [" << name_ << "] with [" << calibration_type << "] Calibration at t=" << timestamp
              << std::endl;

    if (!initial_calib_provided_)
    {
      std::cout << "Info: [" << name_ << "] Calibration(rounded):" << std::endl;
      std::cout << "\tOrientation[deg]: ["
                << sensor_state.state_.q_aw_.toRotationMatrix().eulerAngles(0, 1, 2).transpose() * (180 / M_PI) << " ]"
                << std::endl;
    }

    return result;
  }

  bool CalcUpdate(const Time& timestamp, std::shared_ptr<void> measurement, const CoreStateType& prior_core_state,
                  std::shared_ptr<void> latest_sensor_data, const Eigen::MatrixXd& prior_cov,
                  BufferDataType* new_state_data)
  {
    switch (attitude_type_)
    {
      case AttitudeSensorType::RP_TYPE:
        return CalcUpdateRP(timestamp, measurement, prior_core_state, latest_sensor_data, prior_cov, new_state_data);
        break;
      case AttitudeSensorType::RPY_TYPE:
        return CalcUpdateRPY(timestamp, measurement, prior_core_state, latest_sensor_data, prior_cov, new_state_data);
        break;
    }
  }

  bool CalcUpdateRP(const Time& /*timestamp*/, std::shared_ptr<void> measurement, const CoreStateType& prior_core_state,
                    std::shared_ptr<void> latest_sensor_data, const Eigen::MatrixXd& prior_cov,
                    BufferDataType* new_state_data)
  {
    // Cast the sensor measurement and prior state information
    AttitudeMeasurementType* meas = static_cast<AttitudeMeasurementType*>(measurement.get());
    AttitudeSensorData* prior_sensor_data = static_cast<AttitudeSensorData*>(latest_sensor_data.get());

    // Decompose sensor measurement
    Eigen::Vector2d rp_meas = meas->attitude_.get_rp();

    // Extract sensor state
    AttitudeSensorStateType prior_sensor_state(prior_sensor_data->state_);

    // Generate measurement noise matrix
    const Eigen::Matrix<double, 2, 2> R_meas = this->R_.asDiagonal();

    const int size_of_core_state = CoreStateType::size_error_;
    const int size_of_sensor_state = prior_sensor_state.cov_size_;
    const int size_of_full_error_state = size_of_core_state + size_of_sensor_state;
    const Eigen::MatrixXd P = prior_cov;
    assert(P.size() == size_of_full_error_state * size_of_full_error_state);

    // Calculate the measurement jacobian H
    typedef Eigen::Matrix<double, 2, 3> Matrix23d_t;
    Matrix23d_t I_23;
    I_23 << 1., 0., 0., 0., 1., 0.;
    const Matrix23d_t Z_23 = Matrix23d_t::Zero();
    const Eigen::Matrix3d I_3 = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d R_wi = prior_core_state.q_wi_.toRotationMatrix();

    // Orientation
    const Matrix23d_t Hm_pwi = Z_23;
    const Matrix23d_t Hm_vwi = Z_23;
    const Matrix23d_t Hm_rwi = I_23;
    const Matrix23d_t Hm_bw = Z_23;
    const Matrix23d_t Hm_ba = Z_23;

    // Assemble the jacobian for the orientation (horizontal)
    // H_r = [Hr_pwi Hr_vwi Hr_rwi Hr_bw Hr_ba Hr_mag Hr_rim];
    Eigen::MatrixXd H(2, Hm_pwi.cols() + Hm_vwi.cols() + Hm_rwi.cols() + Hm_bw.cols() + Hm_ba.cols());
    H << Hm_pwi, Hm_vwi, Hm_rwi, Hm_bw, Hm_ba;

    // Calculate the residual z = z~ - (estimate)
    // Orientation
    const Eigen::Vector3d rpy_est = mars::Utils::RPYFromRotMat(R_wi);
    const Eigen::Vector2d rp_est(rpy_est(0), rpy_est(1));  // = R_wi; // TODO
    const Eigen::Vector2d res = rp_meas - rp_est;

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
    const AttitudeSensorStateType corrected_sensor_state = ApplyCorrection(prior_sensor_state, sensor_correction);

    // Return Results
    // CoreState data
    CoreType core_data;
    core_data.cov_ = P_updated.block(0, 0, CoreStateType::size_error_, CoreStateType::size_error_);
    core_data.state_ = corrected_core_state;

    // SensorState data
    std::shared_ptr<AttitudeSensorData> sensor_data(std::make_shared<AttitudeSensorData>());
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

  bool CalcUpdateRPY(const Time& /*timestamp*/, std::shared_ptr<void> measurement,
                     const CoreStateType& prior_core_state, std::shared_ptr<void> latest_sensor_data,
                     const Eigen::MatrixXd& prior_cov, BufferDataType* new_state_data)
  {
    // Cast the sensor measurement and prior state information
    AttitudeMeasurementType* meas = static_cast<AttitudeMeasurementType*>(measurement.get());
    AttitudeSensorData* prior_sensor_data = static_cast<AttitudeSensorData*>(latest_sensor_data.get());

    // Decompose sensor measurement
    Eigen::Quaterniond q_meas = meas->attitude_.quaternion_;

    // Extract sensor state
    AttitudeSensorStateType prior_sensor_state(prior_sensor_data->state_);

    // Generate measurement noise matrix
    const Eigen::Matrix<double, 3, 3> R_meas = this->R_.asDiagonal();
    const int size_of_core_state = CoreStateType::size_error_;
    const int size_of_sensor_state = prior_sensor_state.cov_size_;
    const int size_of_full_error_state = size_of_core_state + size_of_sensor_state;
    const Eigen::MatrixXd P = prior_cov;
    assert(P.size() == size_of_full_error_state * size_of_full_error_state);

    // Calculate the measurement jacobian H
    const Eigen::Matrix3d I_3 = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d Z_3 = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d R_wi = prior_core_state.q_wi_.toRotationMatrix();
    const Eigen::Matrix3d R_aw = prior_sensor_state.q_aw_.toRotationMatrix();
    const Eigen::Matrix3d R_ib = prior_sensor_state.q_ib_.toRotationMatrix();

    // Orientation
    const Eigen::Matrix3d Hr_pwi = Z_3;
    const Eigen::Matrix3d Hr_vwi = Z_3;
    const Eigen::Matrix3d Hr_rwi = R_ib.transpose();
    const Eigen::Matrix3d Hr_bw = Z_3;
    const Eigen::Matrix3d Hr_ba = Z_3;

    const Eigen::Matrix3d Hr_raw = R_ib.transpose() * R_wi.transpose();
    const Eigen::Matrix3d Hr_rib = I_3;

    // Assemble the jacobian for the orientation (horizontal)
    // H_r = [Hr_pwi Hr_vwi Hr_rwi Hr_bw Hr_ba Hr_raw];
    Eigen::MatrixXd H(3, Hr_pwi.cols() + Hr_vwi.cols() + Hr_rwi.cols() + Hr_bw.cols() + Hr_ba.cols() + Hr_raw.cols());
    H << Hr_pwi, Hr_vwi, Hr_rwi, Hr_bw, Hr_ba, Hr_raw;
    Hr_rib;

    // Calculate the residual z = z~ - (estimate)
    // Orientation
    const Eigen::Quaternion<double> q_est =
        prior_sensor_state.q_aw_ * prior_core_state.q_wi_ * prior_sensor_state.q_ib_;
    const Eigen::Quaternion<double> res_q = q_est.inverse() * q_meas;
    const Eigen::Vector3d res = 2 * res_q.vec() / res_q.w();

    // std::cout << "INFO: [" << name_ << "] Residual: " << std::endl;
    // std::cout << "        " << res.transpose() << " (" << res.norm() << ")" << std::endl;

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
    const AttitudeSensorStateType corrected_sensor_state = ApplyCorrection(prior_sensor_state, sensor_correction);

    // Return Results
    // CoreState data
    CoreType core_data;
    core_data.cov_ = P_updated.block(0, 0, CoreStateType::size_error_, CoreStateType::size_error_);
    core_data.state_ = corrected_core_state;

    // SensorState data
    std::shared_ptr<AttitudeSensorData> sensor_data(std::make_shared<AttitudeSensorData>());
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

  AttitudeSensorStateType ApplyCorrection(const AttitudeSensorStateType& prior_sensor_state,
                                          const Eigen::MatrixXd& correction)
  {
    // state + error state correction
    // with quaternion from small angle approx -> new state

    // q_aw   [0,1,2] 0:2

    AttitudeSensorStateType corrected_sensor_state;
    corrected_sensor_state.q_aw_ =
        Utils::ApplySmallAngleQuatCorr(prior_sensor_state.q_aw_, correction.block(0, 0, 3, 1));
    corrected_sensor_state.q_ib_ =
        Utils::ApplySmallAngleQuatCorr(prior_sensor_state.q_ib_, correction.block(3, 0, 3, 1));

    return corrected_sensor_state;
  }
};
}  // namespace mars

#endif  // ATTITUDE_SENSOR_CLASS_H
