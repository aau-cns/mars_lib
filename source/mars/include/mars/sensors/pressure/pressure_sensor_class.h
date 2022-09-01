// Copyright (C) 2022 Martin Scheiber, Christian Brommer,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <christian.brommer@ieee.org>
// and <martin.scheiber@ieee.org>

#ifndef PRESSURESENSORCLASS_H
#define PRESSURESENSORCLASS_H

#include <mars/core_state.h>
#include <mars/ekf.h>
#include <mars/sensors/bind_sensor_data.h>
#include <mars/sensors/pressure/pressure_conversion.h>
#include <mars/sensors/pressure/pressure_measurement_type.h>
#include <mars/sensors/pressure/pressure_sensor_state_type.h>
#include <mars/sensors/update_sensor_abs_class.h>
#include <mars/time.h>
#include <mars/type_definitions/buffer_data_type.h>

#include <iostream>
#include <memory>
#include <string>

namespace mars
{
using PressureSensorData = BindSensorData<PressureSensorStateType>;

class PressureSensorClass : public UpdateSensorAbsClass
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PressureConversion pressure_conversion_;
  bool pressure_reference_is_set_;

  PressureSensorClass(const std::string& name, std::shared_ptr<CoreState> core_states)
  {
    name_ = name;
    core_states_ = std::move(core_states);
    const_ref_to_nav_ = false;
    initial_calib_provided_ = false;
    pressure_reference_is_set_ = false;

    // chi2
    chi2_.set_dof(1);

    std::cout << "Created: [" << this->name_ << "] Sensor" << std::endl;
  }

  virtual ~PressureSensorClass() = default;

  PressureSensorStateType get_state(std::shared_ptr<void> sensor_data)
  {
    PressureSensorData data = *static_cast<PressureSensorData*>(sensor_data.get());
    return data.state_;
  }

  Eigen::MatrixXd get_covariance(const std::shared_ptr<void>& sensor_data)
  {
    PressureSensorData data = *static_cast<PressureSensorData*>(sensor_data.get());
    return data.get_full_cov();
  }

  void set_initial_calib(std::shared_ptr<void> calibration)
  {
    initial_calib_ = calibration;
    initial_calib_provided_ = true;
  }

  void set_pressure_reference(const double& pressure, const double& temperature)
  {
    set_pressure_reference(mars::Pressure(pressure, temperature, mars::Pressure::Type::GAS));
  }

  void set_pressure_reference(const mars::Pressure& pressure)
  {
    if (!pressure_reference_is_set_)
    {
      pressure_conversion_.set_pressure_reference(pressure);
      pressure_reference_is_set_ = true;
      std::cout << "Info: [" << name_ << "] Set pressure reference: \n" << pressure << std::endl;
    }
    else
    {
      std::cout << "Warning: [" << name_ << "] "
                << "Trying to set pressure reference but reference was already set. Action has no effect." << std::endl;
    }
  }

  BufferDataType Initialize(const Time& timestamp, std::shared_ptr<void> sensor_data,
                            std::shared_ptr<CoreType> latest_core_data)
  {
    PressureMeasurementType measurement = *static_cast<PressureMeasurementType*>(sensor_data.get());

    if (!pressure_reference_is_set_)
    {
      Pressure pressure(measurement.pressure_.data_, measurement.pressure_.temperature_K_, measurement.pressure_.type_);

      pressure_conversion_.set_pressure_reference(pressure);
      pressure_reference_is_set_ = true;
      std::cout << "Info: [" << name_ << "] Set pressure reference: \n" << pressure << std::endl;
    }

    PressureSensorData sensor_state;
    std::string calibration_type;

    if (this->initial_calib_provided_)
    {
      calibration_type = "Given";

      PressureSensorData calib = *static_cast<PressureSensorData*>(initial_calib_.get());

      sensor_state.state_ = calib.state_;
      sensor_state.sensor_cov_ = calib.sensor_cov_;
    }
    else
    {
      calibration_type = "Auto";
      std::cout << "Pressure calibration AUTO init not implemented yet" << std::endl;
      exit(EXIT_FAILURE);
    }

    // Bypass core state for the returned object
    BufferDataType result(std::make_shared<CoreType>(*latest_core_data.get()),
                          std::make_shared<PressureSensorData>(sensor_state));

    is_initialized_ = true;

    std::cout << "Info: Initialized [" << name_ << "] with [" << calibration_type << "] Calibration at t=" << timestamp
              << std::endl;

    std::cout << "Info: [" << name_ << "] Calibration(rounded):" << std::endl;
    std::cout << "\tPosition[m]: [" << sensor_state.state_.p_ip_.transpose() << " ]" << std::endl;
    std::cout << "\tBias[m]:     " << sensor_state.state_.bias_p_ << std::endl;

    return result;
  }

  bool CalcUpdate(const Time& /*timestamp*/, std::shared_ptr<void> measurement, const CoreStateType& prior_core_state,
                  std::shared_ptr<void> latest_sensor_data, const Eigen::MatrixXd& prior_cov,
                  BufferDataType* new_state_data)
  {
    // Cast the sensor measurement and prior state information
    PressureMeasurementType* meas = static_cast<PressureMeasurementType*>(measurement.get());
    PressureSensorData* prior_sensor_data = static_cast<PressureSensorData*>(latest_sensor_data.get());

    // Decompose sensor measurement
    Eigen::Matrix<double, 1, 1> h_meas = pressure_conversion_.get_height(meas->pressure_);

    // Extract sensor state
    PressureSensorStateType prior_sensor_state(prior_sensor_data->state_);

    // Generate measurement noise matrix
    const Eigen::Matrix<double, 1, 1> R_meas = this->R_.asDiagonal();

    const int size_of_core_state = CoreStateType::size_error_;
    const int size_of_sensor_state = prior_sensor_state.cov_size_;
    const int size_of_full_error_state = size_of_core_state + size_of_sensor_state;
    const Eigen::MatrixXd P = prior_cov;
    assert(P.size() == size_of_full_error_state * size_of_full_error_state);

    // Calculate the measurement jacobian H
    typedef Eigen::Matrix<double, 1, 3> Matrix13d_t;
    typedef Eigen::Matrix<double, 1, 1> Matrix1d_t;
    const Matrix1d_t I_1{ 1 };
    const Matrix13d_t I_el3{ 0, 0, 1 };
    const Matrix13d_t Z_el3{ 0, 0, 0 };
    const Eigen::Vector3d P_wi = prior_core_state.p_wi_;
    const Eigen::Matrix3d R_wi = prior_core_state.q_wi_.toRotationMatrix();
    const Eigen::Vector3d P_ip = prior_sensor_state.p_ip_;
    const Eigen::Vector3d bias_p = Eigen::Vector3d(0, 0, prior_sensor_state.bias_p_);

    // Position
    const Matrix13d_t Hp_pwi = I_el3;
    const Matrix13d_t Hp_vwi = Z_el3;
    const Eigen::Matrix3d Hp_rwi3 = -R_wi * Utils::Skew(P_ip);
    const Matrix13d_t Hp_rwi = I_el3 * Hp_rwi3;
    const Matrix13d_t Hp_bw = Z_el3;
    const Matrix13d_t Hp_ba = Z_el3;

    // with bias
    const Eigen::Matrix3d Hp_pip3 = R_wi;
    const Matrix13d_t Hp_pip = I_el3 * Hp_pip3;
    const Matrix1d_t Hp_biasp = I_1;

    // H_p = [Hp_pwi Hp_vwi Hp_rwi Hp_bw Hp_ba Hp_pip Hp_biasp];
    Eigen::MatrixXd H(1, Hp_pwi.cols() + Hp_vwi.cols() + Hp_rwi.cols() + Hp_bw.cols() + Hp_ba.cols() + Hp_pip.cols() +
                             Hp_biasp.cols());

    H << Hp_pwi, Hp_vwi, Hp_rwi, Hp_bw, Hp_ba, Hp_pip, Hp_biasp;

    // Calculate the residual z = z~ - (estimate)
    // Position
    const Eigen::Vector3d h_est3 = P_wi + R_wi * P_ip + bias_p;
    const Matrix1d_t h_est = I_el3 * h_est3;
    const Matrix1d_t res = h_meas - h_est;

    // Perform EKF calculations
    mars::Ekf ekf(H, R_meas, res, P);
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
    const PressureSensorStateType corrected_sensor_state = ApplyCorrection(prior_sensor_state, sensor_correction);

    // Return Results
    // CoreState data
    CoreType core_data;
    core_data.cov_ = P_updated.block(0, 0, CoreStateType::size_error_, CoreStateType::size_error_);
    core_data.state_ = corrected_core_state;

    // SensorState data
    std::shared_ptr<PressureSensorData> sensor_data(std::make_shared<PressureSensorData>());
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

  PressureSensorStateType ApplyCorrection(const PressureSensorStateType& prior_sensor_state,
                                          const Eigen::MatrixXd& correction)
  {
    // state + error state correction
    PressureSensorStateType corrected_sensor_state;
    corrected_sensor_state.p_ip_ = prior_sensor_state.p_ip_ + correction.block(0, 0, 3, 1);
    corrected_sensor_state.bias_p_ = prior_sensor_state.bias_p_ + correction(3);

    return corrected_sensor_state;
  }
};
}  // namespace mars

#endif  // PRESSURESENSORCLASS_H
