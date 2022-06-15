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
#include <mars/sensors/gps/gps_conversion.h>
#include <mars/sensors/gps_w_vel/gps_w_vel_measurement_type.h>
#include <mars/sensors/gps_w_vel/gps_w_vel_sensor_state_type.h>
#include <mars/sensors/update_sensor_abs_class.h>
#include <mars/time.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

namespace mars
{
using GpsVelSensorData = BindSensorData<GpsVelSensorStateType>;

class GpsVelSensorClass : public UpdateSensorAbsClass
{
private:
  Eigen::Vector3d v_rot_axis_{ 1, 0, 0 };
  bool use_vel_rot_{ false };
  double vel_rot_thr_{ 0.3 };

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GpsConversion gps_conversion_;
  bool using_external_gps_reference_;
  bool gps_reference_is_set_;

  GpsVelSensorClass(std::string name, std::shared_ptr<CoreState> core_states)
  {
    name_ = name;
    core_states_ = core_states;
    const_ref_to_nav_ = false;
    initial_calib_provided_ = false;
    using_external_gps_reference_ = false;
    gps_reference_is_set_ = false;

    chi2_.set_dof(6);

    std::cout << "Created: [" << this->name_ << "] Sensor" << std::endl;
  }

  void set_v_rot_axis(const Eigen::Vector3d& vec)
  {
    v_rot_axis_ = vec.normalized();
  }

  void set_use_vel_rot(const bool& value)
  {
    use_vel_rot_ = value;
  }

  void set_vel_rot_thr(const double& value)
  {
    vel_rot_thr_ = fabs(value);
  }

  GpsVelSensorStateType get_state(std::shared_ptr<void> sensor_data)
  {
    GpsVelSensorData data = *static_cast<GpsVelSensorData*>(sensor_data.get());
    return data.state_;
  }

  Eigen::MatrixXd get_covariance(std::shared_ptr<void> sensor_data)
  {
    GpsVelSensorData data = *static_cast<GpsVelSensorData*>(sensor_data.get());
    return data.get_full_cov();
  }

  void set_initial_calib(std::shared_ptr<void> calibration)
  {
    initial_calib_ = calibration;
    initial_calib_provided_ = true;
  }

  void set_gps_reference_coordinates(const double& latitude, const double& longitude, const double& altitude)
  {
    set_gps_reference_coordinates(mars::GpsCoordinates(latitude, longitude, altitude));
  }

  void set_gps_reference_coordinates(const mars::GpsCoordinates& gps_reference)
  {
    if (!gps_reference_is_set_)
    {
      gps_conversion_.set_gps_reference(gps_reference);
      gps_reference_is_set_ = true;
      using_external_gps_reference_ = true;
      std::cout << "Info: [" << name_ << "] Set External GPS Reference: \n" << gps_reference << std::endl;
    }
    else
    {
      std::cout << "Warning: [" << name_ << "] "
                << "Trying to set GPS reference but reference was already set. Action has no effect." << std::endl;
    }
  }

  BufferDataType Initialize(const Time& timestamp, std::shared_ptr<void> sensor_data,
                            std::shared_ptr<CoreType> latest_core_data)
  {
    GpsVelMeasurementType measurement = *static_cast<GpsVelMeasurementType*>(sensor_data.get());

    if (!gps_reference_is_set_)
    {
      GpsCoordinates gps_reference(measurement.coordinates_.latitude_, measurement.coordinates_.longitude_,
                                   measurement.coordinates_.altitude_);

      gps_conversion_.set_gps_reference(gps_reference);
      gps_reference_is_set_ = true;

      std::cout << "Info: [" << name_ << "] Set Internal GPS Reference: \n" << gps_reference << std::endl;
    }

    GpsVelSensorData sensor_state;
    std::string calibration_type;

    if (this->initial_calib_provided_)
    {
      calibration_type = "Given";

      GpsVelSensorData calib = *static_cast<GpsVelSensorData*>(initial_calib_.get());

      sensor_state.state_ = calib.state_;
      sensor_state.sensor_cov_ = calib.sensor_cov_;
    }
    else
    {
      calibration_type = "Auto";
      std::cout << "GPS calibration AUTO init not implemented yet" << std::endl;
      exit(EXIT_FAILURE);
    }

    // Bypass core state for the returned object
    BufferDataType result(std::make_shared<CoreType>(*latest_core_data.get()),
                          std::make_shared<GpsVelSensorData>(sensor_state));

    is_initialized_ = true;

    std::cout << "Info: Initialized [" << name_ << "] with [" << calibration_type << "] Calibration at t=" << timestamp
              << std::endl;

    std::cout << "Info: [" << name_ << "] Calibration(rounded):" << std::endl;
    std::cout << "\tPosition[m]: [" << sensor_state.state_.p_ig_.transpose() << " ]" << std::endl;
    std::cout << "\tReference: \n" << gps_conversion_.get_gps_reference() << std::endl;

    return result;
  }

  bool CalcUpdate(const Time& timestamp, std::shared_ptr<void> measurement, const CoreStateType& prior_core_state,
                  std::shared_ptr<void> latest_sensor_data, const Eigen::MatrixXd& prior_cov,
                  BufferDataType* new_state_data)
  {
    // Cast the sensor measurement and prior state information
    GpsVelMeasurementType* meas = static_cast<GpsVelMeasurementType*>(measurement.get());
    GpsVelSensorData* prior_sensor_data = static_cast<GpsVelSensorData*>(latest_sensor_data.get());

    // Decompose sensor measurement
    Eigen::Vector3d p_meas = gps_conversion_.get_enu(meas->coordinates_);
    Eigen::Vector3d v_meas = meas->velocity_;

    // Extract sensor state
    GpsVelSensorStateType prior_sensor_state(prior_sensor_data->state_);

    // Generate measurement noise matrix
    Eigen::MatrixXd R_meas(R_.asDiagonal());

    const int size_of_core_state = CoreStateType::size_error_;
    const int size_of_sensor_state = prior_sensor_state.cov_size_;
    const int size_of_full_error_state = size_of_core_state + size_of_sensor_state;
    const Eigen::MatrixXd P = prior_cov;
    assert(P.size() == size_of_full_error_state * size_of_full_error_state);

    // Calculate the measurement jacobian H
    const Eigen::Matrix3d I_3 = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d O_3 = Eigen::Matrix3d::Zero();

    const Eigen::Vector3d omega_i = prior_core_state.w_m_;  ///< Angular Velocity of the IMU Frame

    const Eigen::Vector3d P_wi = prior_core_state.p_wi_;
    const Eigen::Vector3d V_wi = prior_core_state.v_wi_;
    const Eigen::Vector3d b_w = prior_core_state.b_w_;
    const Eigen::Matrix3d R_wi = prior_core_state.q_wi_.toRotationMatrix();
    const Eigen::Vector3d P_ig = prior_sensor_state.p_ig_;

    const Eigen::Vector3d P_gw_w = prior_sensor_state.p_gw_w_;
    const Eigen::Matrix3d R_gw_w = prior_sensor_state.q_gw_w_.toRotationMatrix();

    // Position
    const Eigen::Matrix3d Hp_pwi = R_gw_w;
    const Eigen::Matrix3d Hp_vwi = O_3;
    const Eigen::Matrix3d Hp_rwi = -R_gw_w * R_wi * Utils::Skew(P_ig);
    const Eigen::Matrix3d Hp_bw = O_3;
    const Eigen::Matrix3d Hp_ba = O_3;

    const Eigen::Matrix3d Hp_pig = R_gw_w * R_wi;
    const Eigen::Matrix3d Hp_pgw_w = O_3;
    const Eigen::Matrix3d Hp_rgw_w = O_3;

    const int num_states = static_cast<int>(Hp_pwi.cols() + Hp_vwi.cols() + Hp_rwi.cols() + Hp_bw.cols() +
                                            Hp_ba.cols() + Hp_pig.cols() + Hp_pgw_w.cols() + Hp_rgw_w.cols());

    // Assemble the jacobian for the position (horizontal)
    Eigen::MatrixXd H_p(3, num_states);
    H_p << Hp_pwi, Hp_vwi, Hp_rwi, Hp_bw, Hp_ba, Hp_pig, Hp_pgw_w, Hp_rgw_w;

    // Assemble the jacobian for the velocity (horizontal)
    Eigen::MatrixXd H_v(3, num_states);
    Eigen::Vector3d v_est;

    if (use_vel_rot_ && (v_meas.norm() > vel_rot_thr_))
    {
      // Velocity
      const Eigen::Vector3d mu = V_wi + R_wi * Utils::Skew(omega_i - b_w) * P_ig;
      const Eigen::Vector3d d_mu = mu / mu.norm();
      const Eigen::Vector3d alpha = v_rot_axis_;

      const Eigen::Matrix3d Hv_pwi = O_3;
      const Eigen::Matrix3d Hv_vwi = R_wi * alpha * d_mu.transpose();
      const Eigen::Matrix3d Hv_rwi =
          -R_wi * Utils::Skew(alpha) * mu.norm() -
          R_wi * alpha * d_mu.transpose() * R_wi * Utils::Skew(Utils::Skew(omega_i - b_w) * P_ig);

      const Eigen::Matrix3d Hv_bw = O_3;
      const Eigen::Matrix3d Hv_ba = O_3;

      const Eigen::Matrix3d Hv_pig = R_wi * alpha * d_mu.transpose() * R_wi * Utils::Skew(omega_i - b_w);
      const Eigen::Matrix3d Hv_pgw_w = O_3;
      const Eigen::Matrix3d Hv_rgw_w = O_3;

      H_v << Hv_pwi, Hv_vwi, Hv_rwi, Hv_bw, Hv_ba, Hv_pig, Hv_pgw_w, Hv_rgw_w;
      v_est = R_wi * alpha * (mu).norm();
    }
    else
    {
      const Eigen::Matrix3d Hv_pwi = O_3;
      const Eigen::Matrix3d Hv_vwi = I_3;
      const Eigen::Matrix3d Hv_rwi = -R_wi * Utils::Skew(Utils::Skew(omega_i - b_w) * P_ig);
      const Eigen::Matrix3d Hv_bw = O_3;
      const Eigen::Matrix3d Hv_ba = O_3;

      const Eigen::Matrix3d Hv_pig = R_wi * Utils::Skew(omega_i - b_w);
      const Eigen::Matrix3d Hv_pgw_w = O_3;
      const Eigen::Matrix3d Hv_rgw_w = O_3;

      H_v << Hv_pwi, Hv_vwi, Hv_rwi, Hv_bw, Hv_ba, Hv_pig, Hv_pgw_w, Hv_rgw_w;
      v_est = V_wi + R_wi * Utils::Skew(omega_i - b_w) * P_ig;
    }

    // Combine all jacobians (vertical)
    Eigen::MatrixXd H(H_p.rows() + H_v.rows(), H_v.cols());
    H << H_p, H_v;

    // Calculate the residual z = z~ - (estimate)
    // Position
    const Eigen::Vector3d p_est = P_gw_w + R_gw_w * (P_wi + R_wi * P_ig);
    const Eigen::Vector3d res_p = p_meas - p_est;

    // Velocity
    const Eigen::Vector3d res_v = v_meas - v_est;

    // Combine residuals (vertical)
    Eigen::MatrixXd res(res_p.rows() + res_v.rows(), 1);
    res << res_p, res_v;

    // Perform EKF calculations
    mars::Ekf ekf(H, R_meas, res, P);
    const Eigen::MatrixXd correction = ekf.CalculateCorrection(chi2_);
    assert(correction.size() == size_of_full_error_state * 1);

    // Check Chi2 test results
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
    const GpsVelSensorStateType corrected_sensor_state = ApplyCorrection(prior_sensor_state, sensor_correction);

    // Return Results
    // CoreState data
    CoreType core_data;
    core_data.cov_ = P_updated.block(0, 0, CoreStateType::size_error_, CoreStateType::size_error_);
    core_data.state_ = corrected_core_state;

    // SensorState data
    std::shared_ptr<GpsVelSensorData> sensor_data(std::make_shared<GpsVelSensorData>());
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

  GpsVelSensorStateType ApplyCorrection(const GpsVelSensorStateType& prior_sensor_state,
                                        const Eigen::MatrixXd& correction)
  {
    // state + error state correction
    // with quaternion from small angle approx -> new state

    GpsVelSensorStateType corrected_sensor_state;
    corrected_sensor_state.p_ig_ = prior_sensor_state.p_ig_ + correction.block(0, 0, 3, 1);
    corrected_sensor_state.p_gw_w_ = prior_sensor_state.p_gw_w_ + correction.block(3, 0, 3, 1);
    corrected_sensor_state.q_gw_w_ =
        Utils::ApplySmallAngleQuatCorr(prior_sensor_state.q_gw_w_, correction.block(6, 0, 3, 1));
    return corrected_sensor_state;
  }
};
}  // namespace mars

#endif  // GPSVELSENSORCLASS_H
