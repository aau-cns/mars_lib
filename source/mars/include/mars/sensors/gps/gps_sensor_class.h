// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef GPSSENSORCLASS_H
#define GPSSENSORCLASS_H

#include <mars/core_state.h>
#include <mars/ekf.h>
#include <mars/sensors/bind_sensor_data.h>
#include <mars/sensors/gps/gps_conversion.h>
#include <mars/sensors/gps/gps_measurement_type.h>
#include <mars/sensors/gps/gps_sensor_state_type.h>
#include <mars/sensors/update_sensor_abs_class.h>
#include <mars/time.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

namespace mars
{
using GpsSensorData = BindSensorData<GpsSensorStateType>;

class GpsSensorClass : public UpdateSensorAbsClass
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GpsConversion gps_conversion_;
  bool using_external_gps_reference_;
  bool gps_reference_is_set_;

  GpsSensorClass(const std::string& name, std::shared_ptr<CoreState> core_states)
  {
    name_ = name;
    core_states_ = std::move(core_states);
    const_ref_to_nav_ = false;
    initial_calib_provided_ = false;
    using_external_gps_reference_ = false;
    gps_reference_is_set_ = false;

    // chi2
    chi2_.set_dof(3);

    std::cout << "Created: [" << this->name_ << "] Sensor" << std::endl;
  }

  virtual ~GpsSensorClass() = default;

  GpsSensorStateType get_state(const std::shared_ptr<void>& sensor_data)
  {
    GpsSensorData data = *static_cast<GpsSensorData*>(sensor_data.get());
    return data.state_;
  }

  Eigen::MatrixXd get_covariance(const std::shared_ptr<void>& sensor_data)
  {
    GpsSensorData data = *static_cast<GpsSensorData*>(sensor_data.get());
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
    GpsMeasurementType measurement = *static_cast<GpsMeasurementType*>(sensor_data.get());

    if (!gps_reference_is_set_)
    {
      GpsCoordinates gps_reference(measurement.coordinates_.latitude_, measurement.coordinates_.longitude_,
                                   measurement.coordinates_.altitude_);

      gps_conversion_.set_gps_reference(gps_reference);
      gps_reference_is_set_ = true;

      std::cout << "Info: [" << name_ << "] Set Internal GPS Reference: \n" << gps_reference << std::endl;
    }

    GpsSensorData sensor_state;
    std::string calibration_type;

    if (this->initial_calib_provided_)
    {
      calibration_type = "Given";

      GpsSensorData calib = *static_cast<GpsSensorData*>(initial_calib_.get());

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
                          std::make_shared<GpsSensorData>(sensor_state));

    is_initialized_ = true;

    std::cout << "Info: Initialized [" << name_ << "] with [" << calibration_type << "] Calibration at t=" << timestamp
              << std::endl;

    std::cout << "Info: [" << name_ << "] Calibration(rounded):" << std::endl;
    std::cout << "\tPosition[m]: [" << sensor_state.state_.p_ig_.transpose() << " ]" << std::endl;
    std::cout << "\tReference: \n" << gps_conversion_.get_gps_reference() << std::endl;

    return result;
  }

  bool CalcUpdate(const Time& /*timestamp*/, std::shared_ptr<void> measurement, const CoreStateType& prior_core_state,
                  std::shared_ptr<void> latest_sensor_data, const Eigen::MatrixXd& prior_cov,
                  BufferDataType* new_state_data)
  {
    // Cast the sensor measurement and prior state information
    GpsMeasurementType* meas = static_cast<GpsMeasurementType*>(measurement.get());
    GpsSensorData* prior_sensor_data = static_cast<GpsSensorData*>(latest_sensor_data.get());

    // Decompose sensor measurement
    Eigen::Vector3d p_meas = gps_conversion_.get_enu(meas->coordinates_);

    // Extract sensor state
    GpsSensorStateType prior_sensor_state(prior_sensor_data->state_);

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
    // const Eigen::Matrix3d I_3 = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d P_wi = prior_core_state.p_wi_;
    const Eigen::Matrix3d R_wi = prior_core_state.q_wi_.toRotationMatrix();
    const Eigen::Vector3d P_ig = prior_sensor_state.p_ig_;

    const Eigen::Vector3d P_gw_w = prior_sensor_state.p_gw_w_;
    const Eigen::Matrix3d R_gw_w = prior_sensor_state.q_gw_w_.toRotationMatrix();

    // Position
    const Eigen::Matrix3d Hp_pwi = R_gw_w;
    const Eigen::Matrix3d Hp_vwi = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hp_rwi = -R_gw_w * R_wi * Utils::Skew(P_ig);
    const Eigen::Matrix3d Hp_bw = Eigen::Matrix3d::Zero();
    const Eigen::Matrix3d Hp_ba = Eigen::Matrix3d::Zero();

    const Eigen::Matrix3d Hp_pig = R_gw_w * R_wi;
    const Eigen::Matrix3d Hp_pgw_w = Eigen::Matrix3d::Zero();  // I_3;
    const Eigen::Matrix3d Hp_rgw_w = Eigen::Matrix3d::Zero();  // R_gw_w * Utils::Skew(P_wi + R_wi * P_ig);

    // Assemble the jacobian for the position (horizontal)
    // H_p = [Hp_pwi Hp_vwi Hp_rwi Hp_bw Hp_ba Hp_pig Hp_pgw_w Hp_rgw_w ];
    Eigen::MatrixXd H(3, Hp_pwi.cols() + Hp_vwi.cols() + Hp_rwi.cols() + Hp_bw.cols() + Hp_ba.cols() + Hp_pig.cols() +
                             Hp_pgw_w.cols() + Hp_rgw_w.cols());

    H << Hp_pwi, Hp_vwi, Hp_rwi, Hp_bw, Hp_ba, Hp_pig, Hp_pgw_w, Hp_rgw_w;

    // Calculate the residual z = z~ - (estimate)
    // Position
    const Eigen::Vector3d p_est = P_gw_w + R_gw_w * (P_wi + R_wi * P_ig);
    residual_ = Eigen::MatrixXd(p_est.rows(), 1);
    residual_ = p_meas - p_est;

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
    const GpsSensorStateType corrected_sensor_state = ApplyCorrection(prior_sensor_state, sensor_correction);

    // Return Results
    // CoreState data
    CoreType core_data;
    core_data.cov_ = P_updated.block(0, 0, CoreStateType::size_error_, CoreStateType::size_error_);
    core_data.state_ = corrected_core_state;

    // SensorState data
    std::shared_ptr<GpsSensorData> sensor_data(std::make_shared<GpsSensorData>());
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

  GpsSensorStateType ApplyCorrection(const GpsSensorStateType& prior_sensor_state, const Eigen::MatrixXd& correction)
  {
    // state + error state correction
    // with quaternion from small angle approx -> new state

    GpsSensorStateType corrected_sensor_state;
    corrected_sensor_state.p_ig_ = prior_sensor_state.p_ig_ + correction.block(0, 0, 3, 1);
    corrected_sensor_state.p_gw_w_ = prior_sensor_state.p_gw_w_ + correction.block(3, 0, 3, 1);
    corrected_sensor_state.q_gw_w_ =
        Utils::ApplySmallAngleQuatCorr(prior_sensor_state.q_gw_w_, correction.block(6, 0, 3, 1));
    return corrected_sensor_state;
  }
};
}  // namespace mars

#endif  // GPSSENSORCLASS_H
