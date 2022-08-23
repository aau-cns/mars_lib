// Copyright (C) 2021 Martin Scheiber and Christian Brommer, Control of Networked Systems, University of Klagenfurt,
// Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <martin.scheiber@ieee.org>
// and <christian.brommer@ieee.org>.

#ifndef BODYVELSENSORCLASS_H
#define BODYVELSENSORCLASS_H

#include <mars/core_state.h>
#include <mars/ekf.h>
#include <mars/general_functions/utils.h>
#include <mars/sensors/bind_sensor_data.h>
#include <mars/sensors/bodyvel/bodyvel_measurement_type.h>
#include <mars/sensors/bodyvel/bodyvel_sensor_state_type.h>
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
using BodyvelSensorData = BindSensorData<BodyvelSensorStateType>;

class BodyvelSensorClass : public UpdateSensorAbsClass
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BodyvelSensorClass(std::string name, std::shared_ptr<CoreState> core_states)
  {
    name_ = std::move(name);
    core_states_ = core_states;
    const_ref_to_nav_ = false;
    initial_calib_provided_ = false;

    // chi2
    chi2_.set_dof(3);

    std::cout << "Created: [" << this->name_ << "] Sensor" << std::endl;
  }

  virtual ~BodyvelSensorClass() = default;

  BodyvelSensorStateType get_state(std::shared_ptr<void> sensor_data)
  {
    BodyvelSensorData data = *static_cast<BodyvelSensorData*>(sensor_data.get());
    return data.state_;
  }

  Eigen::MatrixXd get_covariance(std::shared_ptr<void> sensor_data)
  {
    BodyvelSensorData data = *static_cast<BodyvelSensorData*>(sensor_data.get());
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
    BodyvelMeasurementType measurement = *static_cast<BodyvelMeasurementType*>(sensor_data.get());

    BodyvelSensorData sensor_state;
    std::string calibration_type;

    if (this->initial_calib_provided_)
    {
      calibration_type = "Given";

      BodyvelSensorData calib = *static_cast<BodyvelSensorData*>(initial_calib_.get());

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

      std::cout << "Bodyvel calibration AUTO init not implemented yet" << std::endl;
      exit(EXIT_FAILURE);
    }

    // Bypass core state for the returned object
    BufferDataType result(std::make_shared<CoreType>(*latest_core_data.get()),
                          std::make_shared<BodyvelSensorData>(sensor_state));

    is_initialized_ = true;

    std::cout << "Info: Initialized [" << name_ << "] with [" << calibration_type << "] Calibration at t=" << timestamp
              << std::endl;

    if (!initial_calib_provided_)
    {
      std::cout << "Info: [" << name_ << "] Calibration(rounded):" << std::endl;
      std::cout << "\tPosition[m]: [" << sensor_state.state_.p_ib_.transpose() << " ]" << std::endl;
      std::cout << "\tOrientation[deg]: ["
                << sensor_state.state_.q_ib_.toRotationMatrix().eulerAngles(0, 1, 2).transpose() * (180 / M_PI) << " ]"
                << std::endl;
    }

    return result;
  }

  bool CalcUpdate(const Time& /*timestamp*/, std::shared_ptr<void> measurement, const CoreStateType& prior_core_state,
                  std::shared_ptr<void> latest_sensor_data, const Eigen::MatrixXd& prior_cov,
                  BufferDataType* new_state_data)
  {
    // Cast the sensor measurement and prior state information
    BodyvelMeasurementType* meas = static_cast<BodyvelMeasurementType*>(measurement.get());
    BodyvelSensorData* prior_sensor_data = static_cast<BodyvelSensorData*>(latest_sensor_data.get());

    // Decompose sensor measurement
    Eigen::Vector3d v_meas = meas->velocity_;

    // Extract sensor state
    BodyvelSensorStateType prior_sensor_state(prior_sensor_data->state_);

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
    const Eigen::Vector3d P_wi = prior_core_state.p_wi_;
    const Eigen::Vector3d V_wi = prior_core_state.v_wi_;
    const Eigen::Matrix3d R_wi = prior_core_state.q_wi_.toRotationMatrix();
    const Eigen::Vector3d P_ib = prior_sensor_state.p_ib_;
    const Eigen::Matrix3d R_ib = prior_sensor_state.q_ib_.toRotationMatrix();

    const Eigen::Vector3d w_wi = prior_core_state.w_m_ - prior_core_state.b_w_;
    const Eigen::Matrix3d w_wi_skew = Utils::Skew(w_wi);

    // Linear Velocity
    const Eigen::Matrix3d Hv_pwi = Z_3;
    const Eigen::Matrix3d Hv_vwi = R_ib.transpose() * R_wi.transpose();
    const Eigen::Matrix3d Hv_rwi = R_ib.transpose() * Utils::Skew(R_wi.transpose() * V_wi);
    const Eigen::Matrix3d Hv_bw = R_ib.transpose() * Utils::Skew(P_ib);  // Z_3;
    const Eigen::Matrix3d Hv_ba = Z_3;
    const Eigen::Matrix3d Hv_pib = R_ib.transpose() * w_wi_skew;
    const Eigen::Matrix3d Hv_rib =
        Utils::Skew(R_ib.transpose() * R_wi.transpose() * V_wi) + Utils::Skew(R_ib.transpose() * w_wi_skew * P_ib);

    // Assemble the jacobian for the velocity (horizontal)
    // H_v = [Hv_pwi Hv_vwi Hv_rwi Hv_bw Hv_ba Hv_pib Hv_rib];
    Eigen::MatrixXd H_v(3, Hv_pwi.cols() + Hv_vwi.cols() + Hv_rwi.cols() + Hv_bw.cols() + Hv_ba.cols() + Hv_pib.cols() +
                               Hv_rib.cols());
    H_v << Hv_pwi, Hv_vwi, Hv_rwi, Hv_bw, Hv_ba, Hv_pib, Hv_rib;

    // Combine all jacobians (vertical)
    Eigen::MatrixXd H(H_v.rows(), H_v.cols());
    H << H_v;

    // Calculate the residual z = z~ - (estimate)
    // Velocity
    const Eigen::Vector3d v_est = R_ib.transpose() * R_wi.transpose() * V_wi + R_ib.transpose() * w_wi_skew * P_ib;
    const Eigen::Vector3d res = v_meas - v_est;

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
    const BodyvelSensorStateType corrected_sensor_state = ApplyCorrection(prior_sensor_state, sensor_correction);

    // Return Results
    // CoreState data
    CoreType core_data;
    core_data.cov_ = P_updated.block(0, 0, CoreStateType::size_error_, CoreStateType::size_error_);
    core_data.state_ = corrected_core_state;

    // SensorState data
    std::shared_ptr<BodyvelSensorData> sensor_data(std::make_shared<BodyvelSensorData>());
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

  BodyvelSensorStateType ApplyCorrection(const BodyvelSensorStateType& prior_sensor_state,
                                         const Eigen::MatrixXd& correction)
  {
    // state + error state correction
    // with quaternion from small angle approx -> new state

    BodyvelSensorStateType corrected_sensor_state;
    corrected_sensor_state.p_ib_ = prior_sensor_state.p_ib_ + correction.block(0, 0, 3, 1);
    corrected_sensor_state.q_ib_ =
        Utils::ApplySmallAngleQuatCorr(prior_sensor_state.q_ib_, correction.block(3, 0, 3, 1));
    return corrected_sensor_state;
  }
};
}  // namespace mars

#endif  // BODYVELSENSORCLASS_H
