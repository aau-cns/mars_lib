// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef VISIONSENSORCLASS_H
#define VISIONSENSORCLASS_H

#include <mars/core_state.h>
#include <mars/ekf.h>
#include <mars/general_functions/utils.h>
#include <mars/sensors/bind_sensor_data.h>
#include <mars/sensors/update_sensor_abs_class.h>
#include <mars/sensors/vision/vision_measurement_type.h>
#include <mars/sensors/vision/vision_sensor_state_type.h>
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
using VisionSensorData = BindSensorData<VisionSensorStateType>;

class VisionSensorClass : public UpdateSensorAbsClass
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool update_scale_{ true };

  VisionSensorClass(const std::string& name, std::shared_ptr<CoreState> core_states, bool update_scale = true)
  {
    name_ = name;
    core_states_ = std::move(core_states);
    const_ref_to_nav_ = false;
    initial_calib_provided_ = false;
    update_scale_ = update_scale;

    // chi2
    chi2_.set_dof(6);

    std::cout << "Created: [" << this->name_ << "] Sensor" << std::endl;
  }

  virtual ~VisionSensorClass() = default;

  VisionSensorStateType get_state(const std::shared_ptr<void>& sensor_data)
  {
    VisionSensorData data = *static_cast<VisionSensorData*>(sensor_data.get());
    return data.state_;
  }

  Eigen::MatrixXd get_covariance(const std::shared_ptr<void>& sensor_data)
  {
    VisionSensorData data = *static_cast<VisionSensorData*>(sensor_data.get());
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
    // VisionMeasurementType measurement = *static_cast<VisionMeasurementType*>(sensor_data.get());

    VisionSensorData sensor_state;
    std::string calibration_type;

    if (this->initial_calib_provided_)
    {
      calibration_type = "Given";

      VisionSensorData calib = *static_cast<VisionSensorData*>(initial_calib_.get());

      sensor_state.state_ = calib.state_;
      sensor_state.sensor_cov_ = calib.sensor_cov_;
    }
    else
    {
      calibration_type = "Auto";

      //      Eigen::Vector3d p_wp(measurement.position_);
      //      Eigen::Quaterniond q_wp(measurement.orientation_);

      //      Eigen::Vector3d p_wi(latest_core_data->state_.p_wi_);
      //      Eigen::Quaterniond q_wi(latest_core_data->state_.q_wi_);
      //      Eigen::Matrix3d r_wi(q_wi.toRotationMatrix());

      //      Eigen::Vector3d p_ip = r_wi.transpose() * (p_wp - p_wi);
      //      Eigen::Quaterniond q_ip = q_wi.conjugate() * q_wp;

      //      // Calibration, position / rotation imu-pose
      //      sensor_state.state_.p_vw_ = p_ip;
      //      sensor_state.state_.q_vw_ = q_ip;
      //      sensor_state.state_.p_ic_ = p_ip;
      //      sensor_state.state_.q_ic_ = q_ip;
      //      sensor_state.state_.lambda_ = 1;

      //      // The covariance should enclose the initialization with a 3 Sigma bound
      //      Eigen::Matrix<double, 13, 1> std;
      //      std << 1, 1, 1, (35 * M_PI / 180), (35 * M_PI / 180), (35 * M_PI / 180);
      //      sensor_state.sensor_cov_ = std.cwiseProduct(std).asDiagonal();
      std::cout << "Vision calibration AUTO init not implemented yet" << std::endl;
      exit(EXIT_FAILURE);
    }

    // Bypass core state for the returned object
    BufferDataType result(std::make_shared<CoreType>(*latest_core_data.get()),
                          std::make_shared<VisionSensorData>(sensor_state));

    is_initialized_ = true;

    std::cout << "Info: Initialized [" << name_ << "] with [" << calibration_type << "] Calibration at t=" << timestamp
              << std::endl;

    if (!initial_calib_provided_)
    {
      std::cout << "Info: [" << name_ << "] Calibration(rounded):" << std::endl;
      std::cout << "\tP_vw[m]: [" << sensor_state.state_.p_vw_.transpose() << " ]" << std::endl;
      std::cout << "\tR_vw[deg]: ["
                << sensor_state.state_.q_vw_.toRotationMatrix().eulerAngles(0, 1, 2).transpose() * (180 / M_PI) << " ]"
                << std::endl;
      std::cout << "\tP_ic[m]: [" << sensor_state.state_.p_ic_.transpose() << " ]" << std::endl;
      std::cout << "\tR_ic[deg]: ["
                << sensor_state.state_.q_ic_.toRotationMatrix().eulerAngles(0, 1, 2).transpose() * (180 / M_PI) << " ]"
                << std::endl;
      std::cout << "\tLambda[%]: [" << sensor_state.state_.lambda_ * 100 << " ]" << std::endl;
    }

    return result;
  }

  bool CalcUpdate(const Time& /*timestamp*/, std::shared_ptr<void> measurement, const CoreStateType& prior_core_state,
                  std::shared_ptr<void> latest_sensor_data, const Eigen::MatrixXd& prior_cov,
                  BufferDataType* new_state_data)
  {
    // Cast the sensor measurement and prior state information
    VisionMeasurementType* meas = static_cast<VisionMeasurementType*>(measurement.get());
    VisionSensorData* prior_sensor_data = static_cast<VisionSensorData*>(latest_sensor_data.get());

    // Decompose sensor measurement
    Eigen::Vector3d p_meas = meas->position_;
    Eigen::Quaternion<double> q_meas = meas->orientation_;

    // Extract sensor state
    VisionSensorStateType prior_sensor_state(prior_sensor_data->state_);

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
    const Eigen::Matrix<double, 6, 6> R_meas = R_meas_dyn;

    const int size_of_core_state = CoreStateType::size_error_;
    const int size_of_sensor_state = prior_sensor_state.cov_size_;
    const int size_of_full_error_state = size_of_core_state + size_of_sensor_state;
    const Eigen::MatrixXd P = prior_cov;
    assert(P.size() == size_of_full_error_state * size_of_full_error_state);

    // Calculate the measurement jacobian H
    const Eigen::Matrix3d I_3 = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d P_wi = prior_core_state.p_wi_;
    const Eigen::Matrix3d R_wi = prior_core_state.q_wi_.toRotationMatrix();
    const Eigen::Vector3d P_vw = prior_sensor_state.p_vw_;
    const Eigen::Matrix3d R_vw = prior_sensor_state.q_vw_.toRotationMatrix();
    const Eigen::Vector3d P_ic = prior_sensor_state.p_ic_;
    const Eigen::Matrix3d R_ic = prior_sensor_state.q_ic_.toRotationMatrix();
    const double L = prior_sensor_state.lambda_;

    // Position
    const Eigen::Matrix3d Hp_pwi = L * R_vw;
    const Eigen::Matrix3d Hp_vwi = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hp_rwi = -L * R_vw * R_wi * Utils::Skew(P_ic);
    const Eigen::Matrix3d Hp_bw = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hp_ba = Eigen::Matrix3d::Zero();

    const Eigen::Matrix3d Hp_pvw = I_3 * L;
    const Eigen::Matrix3d Hp_rvw = -L * R_vw * Utils::Skew(P_wi + R_wi * P_ic);
    const Eigen::Matrix3d Hp_pic = L * R_vw * R_wi;
    const Eigen::Matrix3d Hp_ric = Eigen::Matrix3d::Zero();
    Eigen::Vector3d Hp_lambda;
    if (update_scale_)
    {
      Hp_lambda = P_vw + R_vw * (P_wi + R_wi * P_ic);
    }
    else
    {
      Hp_lambda = Eigen::Vector3d::Zero();
    }
    // Assemble the jacobian for the position (horizontal)
    // H_p = [Hp_pwi Hp_vwi Hp_rwi Hp_bw Hp_ba Hp_ip Hp_rip];
    Eigen::MatrixXd H_p(3, Hp_pwi.cols() + Hp_vwi.cols() + Hp_rwi.cols() + Hp_bw.cols() + Hp_ba.cols() + Hp_pvw.cols() +
                               Hp_rvw.cols() + Hp_pic.cols() + Hp_ric.cols() + Hp_lambda.cols());

    H_p << Hp_pwi, Hp_vwi, Hp_rwi, Hp_bw, Hp_ba, Hp_pvw, Hp_rvw, Hp_pic, Hp_ric, Hp_lambda;

    // Orientation
    const Eigen::Matrix3d Hr_pwi = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hr_vwi = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hr_rwi = R_ic.transpose();
    const Eigen::Matrix3d Hr_bw = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hr_ba = Eigen::Matrix3d::Zero();

    const Eigen::Matrix3d Hr_pvw = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hr_rvw = R_ic.transpose() * R_wi.transpose();
    const Eigen::Matrix3d Hr_pic = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hr_ric = I_3;
    const Eigen::Vector3d Hr_lambda = Eigen::Vector3d::Zero();

    // Assemble the jacobian for the orientation (horizontal)
    // H_r = [Hr_pwi Hr_vwi Hr_rwi Hr_bw Hr_ba Hr_pip Hr_rip];
    Eigen::MatrixXd H_r(3, Hr_pwi.cols() + Hr_vwi.cols() + Hr_rwi.cols() + Hr_bw.cols() + Hr_ba.cols() + Hr_pvw.cols() +
                               Hr_rvw.cols() + Hr_pic.cols() + Hr_ric.cols() + Hr_lambda.cols());
    H_r << Hr_pwi, Hr_vwi, Hr_rwi, Hr_bw, Hr_ba, Hr_pvw, Hr_rvw, Hr_pic, Hr_ric, Hr_lambda;

    // Combine all jacobians (vertical)
    Eigen::MatrixXd H(H_p.rows() + H_r.rows(), H_r.cols());
    H << H_p, H_r;

    // Calculate the residual z = z~ - (estimate)
    // Position
    const Eigen::Vector3d p_est = (P_vw + R_vw * (P_wi + R_wi * P_ic)) * L;
    const Eigen::Vector3d res_p = p_meas - p_est;

    // Orientation
    const Eigen::Quaternion<double> q_est =
        prior_sensor_state.q_vw_ * prior_core_state.q_wi_ * prior_sensor_state.q_ic_;
    const Eigen::Quaternion<double> res_q = q_est.inverse() * q_meas;
    const Eigen::Vector3d res_r = 2 * res_q.vec() / res_q.w();

    // Combine residuals (vertical)
    residual_ = Eigen::MatrixXd(res_p.rows() + res_r.rows(), 1);
    residual_ << res_p, res_r;

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
    const VisionSensorStateType corrected_sensor_state = ApplyCorrection(prior_sensor_state, sensor_correction);

    // Return Results
    // CoreState data
    CoreType core_data;
    core_data.cov_ = P_updated.block(0, 0, CoreStateType::size_error_, CoreStateType::size_error_);
    core_data.state_ = corrected_core_state;

    // SensorState data
    std::shared_ptr<VisionSensorData> sensor_data(std::make_shared<VisionSensorData>());
    sensor_data->set_cov(P_updated);
    sensor_data->state_ = corrected_sensor_state;

    BufferDataType state_entry(std::make_shared<CoreType>(core_data), sensor_data);

    if (const_ref_to_nav_)
    {
      // corrected_sensor_data.ref_to_nav = prior_ref_to_nav;
    }
    else
    {
      // TODO(chb) also estimate ref to nav
    }

    *new_state_data = state_entry;

    return true;
  }

  VisionSensorStateType ApplyCorrection(const VisionSensorStateType& prior_sensor_state,
                                        const Eigen::MatrixXd& correction)
  {
    // state + error state correction
    // with quaternion from small angle approx -> new state

    // p_vw   [0,1,2] 0:2
    // q_vw   [3,4,5] 3:5
    // p_ic   [6,7,8] 6:8
    // q_ic   [9,10,11] 9:11
    // lambda [12] 12

    VisionSensorStateType corrected_sensor_state;
    corrected_sensor_state.p_vw_ = prior_sensor_state.p_vw_ + correction.block(0, 0, 3, 1);
    corrected_sensor_state.q_vw_ =
        Utils::ApplySmallAngleQuatCorr(prior_sensor_state.q_vw_, correction.block(3, 0, 3, 1));

    corrected_sensor_state.p_ic_ = prior_sensor_state.p_ic_ + correction.block(6, 0, 3, 1);
    corrected_sensor_state.q_ic_ =
        Utils::ApplySmallAngleQuatCorr(prior_sensor_state.q_ic_, correction.block(9, 0, 3, 1));

    corrected_sensor_state.lambda_ = prior_sensor_state.lambda_ + correction(12);

    return corrected_sensor_state;
  }
};
}  // namespace mars

#endif  // VISIONSENSORCLASS_H
