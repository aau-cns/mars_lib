// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef PRESSURESENSORCLASS_H
#define PRESSURESENSORCLASS_H

#include <mars/core_state.h>
#include <mars/ekf.h>
#include <mars/sensors/bind_sensor_data.h>
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

  PressureSensorClass(std::string name, std::shared_ptr<CoreState> core_states)
  {
    name_ = name;
    core_states_ = core_states;
    const_ref_to_nav_ = false;
    initial_calib_provided_ = false;

    std::cout << "Created: [" << this->name_ << "] Sensor" << std::endl;
  }

  PressureSensorStateType get_state(std::shared_ptr<void> sensor_data)
  {
    PressureSensorData data = *static_cast<PressureSensorData*>(sensor_data.get());
    return data.state_;
  }

  Eigen::MatrixXd get_covariance(std::shared_ptr<void> sensor_data)
  {
    PressureSensorData data = *static_cast<PressureSensorData*>(sensor_data.get());
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
    PressureMeasurementType measurement = *static_cast<PressureMeasurementType*>(sensor_data.get());

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

    return result;
  }

  bool CalcUpdate(const Time& /*timestamp*/, std::shared_ptr<void> measurement, const CoreStateType& prior_core_state,
                  std::shared_ptr<void> latest_sensor_data, const Eigen::MatrixXd& prior_cov,
                  BufferDataType* new_state_data)
  {

    // DEBUG
//    std::cout << "[SensorUpdate]: " << name_ << " - called" << std::endl;

    // Cast the sensor measurement and prior state information
    PressureMeasurementType* meas = static_cast<PressureMeasurementType*>(measurement.get());
    PressureSensorData* prior_sensor_data = static_cast<PressureSensorData*>(latest_sensor_data.get());

    // DEBUG
//    std::cout << "[SensorUpdate]: " << name_ << " - retrieving measurment and prior state" << std::endl;

    // Decompose sensor measurement
    Eigen::Matrix<double, 1, 1> h_meas = meas->height_;

    // Extract sensor state
    PressureSensorStateType prior_sensor_state(prior_sensor_data->state_);

    // DEBUG
//    std::cout << "[SensorUpdate]: " << name_ << " - retrieving noise matrix R_meas" << std::endl;

    // Generate measurement noise matrix
    const Eigen::Matrix<double, 1, 1> R_meas = this->R_.asDiagonal();

//    std::cout << "[SensorUpdate]: " << name_ << " - ..... " << this->R_.transpose() << std::endl;

    // DEBUG
//    std::cout << "[SensorUpdate]: " << name_ << " - retrieving prior cov matrix P_prior" << std::endl;

    const int size_of_core_state = CoreStateType::size_error_;
    const int size_of_sensor_state = prior_sensor_state.cov_size_;
    const int size_of_full_error_state = size_of_core_state + size_of_sensor_state;
    const Eigen::MatrixXd P = prior_cov;
    assert(P.size() == size_of_full_error_state * size_of_full_error_state);

    {
      // debugging to circumnvent update
      CoreType core_data;
      core_data.cov_ = P.block(0, 0, CoreStateType::size_error_, CoreStateType::size_error_);
      core_data.state_ = prior_core_state;

      BufferDataType state_entry(std::make_shared<CoreType>(core_data), latest_sensor_data);
      *new_state_data = state_entry;
      return true;
    }

  // DEBUG
//    std::cout << "[SensorUpdate]: " << name_ << " - calculating measurement jacobian H" << std::endl;


    // Calculate the measurement jacobian H
    //    double a
    typedef Eigen::Matrix<double, 1, 3> Matrix13d_t;
    const Matrix13d_t I_el3{ 0, 0, 1 };
    const Matrix13d_t Z_el3{ 0, 0, 0 };
    const Eigen::Vector3d P_wi = prior_core_state.p_wi_;
    const Eigen::Matrix3d R_wi = prior_core_state.q_wi_.toRotationMatrix();
    const Eigen::Vector3d P_ip = prior_sensor_state.p_ip_;

    // Position
    const Matrix13d_t Hp_pwi = I_el3;
    const Matrix13d_t Hp_vwi = Z_el3;
    const Eigen::Matrix3d Hp_rwi3 = -R_wi * Utils::Skew(P_ip);
    const Matrix13d_t Hp_rwi = I_el3 * Hp_rwi3;
    const Matrix13d_t Hp_bw = Z_el3;
    const Matrix13d_t Hp_ba = Z_el3;
    const Eigen::Matrix3d Hp_pip3 = R_wi;
    const Matrix13d_t Hp_pip = I_el3 * Hp_pip3;

    // Assemble the jacobian for the position (horizontal)
    // H_p = [Hp_pwi Hp_vwi Hp_rwi Hp_bw Hp_ba Hp_pig ];
    Eigen::MatrixXd H(1, Hp_pwi.cols() + Hp_vwi.cols() + Hp_rwi.cols() + Hp_bw.cols() + Hp_ba.cols() + Hp_pip.cols());

    H << Hp_pwi, Hp_vwi, Hp_rwi, Hp_bw, Hp_ba, Hp_pip;
    H.setZero();

//    std::cout << "[SensorUpdate]: " << name_ << " - ..... " << H << std::endl;

    // DEBUG
//    std::cout << "[SensorUpdate]: " << name_ << " - calculating residual res" << std::endl;

    // Calculate the residual z = z~ - (estimate)
    // Position
    const Eigen::Vector3d h_est3 = P_wi + R_wi * P_ip;
    const Eigen::Matrix<double, 1, 1> h_est = I_el3 * h_est3;
    const Eigen::Matrix<double, 1, 1> res = Eigen::Matrix<double, 1, 1>(0.0); //h_meas - h_est;
//    std::cout << "[SensorUpdate]: " << name_ << " - .... " << res << std::endl;

    // DEBUG
//    std::cout << "[SensorUpdate]: " << name_ << " - calculate EKF correction" << std::endl;

    // Perform EKF calculations
    mars::Ekf ekf(H, R_meas, res, P);
    Eigen::MatrixXd correction = ekf.CalculateCorrection();
    assert(correction.size() == size_of_full_error_state * 1);
    correction.setZero();
//    std::cout << "[SensorUpdate]: " << name_ << " - ..... " << correction.transpose() << std::endl;

    // DEBUG
//    std::cout << "[SensorUpdate]: " << name_ << " - calculate EKF COV Update" << std::endl;

    Eigen::MatrixXd P_updated = ekf.CalculateCovUpdate();
    assert(P_updated.size() == size_of_full_error_state * size_of_full_error_state);
    P_updated = Utils::EnforceMatrixSymmetry(P_updated);
    P_updated = P;
//    std::cout << "[SensorUpdate]: " << name_ << " - .....\n " << P << std::endl;

    // DEBUG
//    std::cout << "[SensorUpdate]: " << name_ << " - apply core correction" << std::endl;

    // Apply Core Correction
    CoreStateVector core_correction = correction.block(0, 0, CoreStateType::size_error_, 1);
    CoreStateType corrected_core_state = CoreStateType::ApplyCorrection(prior_core_state, core_correction);

    // DEBUG
//    std::cout << "[SensorUpdate]: " << name_ << " - apply sensor correction" << std::endl;

    // Apply Sensor Correction
    const Eigen::MatrixXd sensor_correction = correction.block(size_of_core_state, 0, size_of_sensor_state, 1);
    const PressureSensorStateType corrected_sensor_state = ApplyCorrection(prior_sensor_state, sensor_correction);

    // DEBUG
//    std::cout << "[SensorUpdate]: " << name_ << " - return results" << std::endl;

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
    return corrected_sensor_state;
  }
};
}  // namespace mars

#endif  // PRESSURESENSORCLASS_H
