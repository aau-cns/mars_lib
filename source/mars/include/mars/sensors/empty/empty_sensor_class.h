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
#include <mars/sensors/empty/empty_measurement_type.h>
#include <mars/sensors/empty/empty_sensor_state_type.h>
#include <mars/sensors/update_sensor_abs_class.h>
#include <mars/time.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

namespace mars
{
using EmptySensorData = BindSensorData<EmptySensorStateType>;

class EmptySensorClass : public UpdateSensorAbsClass
{
private:
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EmptySensorClass(const std::string& name, std::shared_ptr<CoreState> core_states)
  {
    name_ = name;
    core_states_ = std::move(core_states);
    const_ref_to_nav_ = false;
    initial_calib_provided_ = false;

    std::cout << "Created: [" << this->name_ << "] Sensor" << std::endl;
  }

  virtual ~EmptySensorClass() = default;

  EmptySensorStateType get_state(const std::shared_ptr<void>& sensor_data)
  {
    EmptySensorData data = *static_cast<EmptySensorData*>(sensor_data.get());
    return data.state_;
  }

  Eigen::MatrixXd get_covariance(const std::shared_ptr<void>& sensor_data)
  {
    EmptySensorData data = *static_cast<EmptySensorData*>(sensor_data.get());
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
    // EmptyMeasurementType measurement = *static_cast<EmptyMeasurementType*>(sensor_data.get());

    EmptySensorData sensor_state;
    std::string calibration_type;

    if (this->initial_calib_provided_)
    {
      calibration_type = "Given";

      EmptySensorData calib = *static_cast<EmptySensorData*>(initial_calib_.get());

      sensor_state.state_ = calib.state_;
      sensor_state.sensor_cov_ = calib.sensor_cov_;
    }
    else
    {
      std::cout << "Empty calibration AUTO init not implemented yet" << std::endl;
      exit(EXIT_FAILURE);
    }

    // Bypass core state for the returned object
    BufferDataType result(std::make_shared<CoreType>(*latest_core_data.get()),
                          std::make_shared<EmptySensorData>(sensor_state));

    is_initialized_ = true;

    std::cout << "Info: Initialized [" << name_ << "] with [" << calibration_type << "] Calibration at t=" << timestamp
              << std::endl;

    std::cout << "Info: [" << name_ << "] Calibration(rounded):" << std::endl;
    std::cout << "\tPosition[m]: [" << sensor_state.state_.value_.transpose() << " ]" << std::endl;

    return result;
  }

  bool CalcUpdate(const Time& /*timestamp*/, std::shared_ptr<void> /*measurement*/,
                  const CoreStateType& prior_core_state, std::shared_ptr<void> latest_sensor_data,
                  const Eigen::MatrixXd& prior_cov, BufferDataType* new_state_data)
  {
    // Cast the sensor measurement and prior state information
    // EmptyMeasurementType* meas = static_cast<EmptyMeasurementType*>(measurement.get());
    EmptySensorData* prior_sensor_data = static_cast<EmptySensorData*>(latest_sensor_data.get());

    // Extract sensor state
    EmptySensorStateType prior_sensor_state(prior_sensor_data->state_);

    // Return Results
    // CoreState data
    CoreType core_data;
    core_data.cov_ = prior_cov.block(0, 0, CoreStateType::size_error_, CoreStateType::size_error_);
    core_data.state_ = prior_core_state;

    // SensorState data
    std::shared_ptr<EmptySensorData> sensor_data(std::make_shared<EmptySensorData>());
    sensor_data->set_cov(prior_cov);
    sensor_data->state_ = prior_sensor_state;

    BufferDataType state_entry(std::make_shared<CoreType>(core_data), sensor_data);

    *new_state_data = state_entry;

    return true;
  }

  EmptySensorStateType ApplyCorrection(const EmptySensorStateType& prior_sensor_state,
                                       const Eigen::MatrixXd& correction)
  {
    // state + error state correction
    // with quaternion from small angle approx -> new state

    EmptySensorStateType corrected_sensor_state;
    corrected_sensor_state.value_ = prior_sensor_state.value_ + correction.block(0, 0, 3, 1);
    return corrected_sensor_state;
  }
};
}  // namespace mars

#endif  // GPSVELSENSORCLASS_H
