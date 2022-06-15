// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef IMU_SENSOR_CLASS_H
#define IMU_SENSOR_CLASS_H

#include <mars/sensors/sensor_abs_class.h>
#include <mars/time.h>
#include <Eigen/Dense>
#include <iostream>
#include <string>

namespace mars
{
class ImuSensorClass : public SensorAbsClass
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuSensorClass(std::string name)
  {
    name_ = name;
    std::cout << "Created: [" << this->name_ << "] Sensor" << std::endl;
  }

  Eigen::MatrixXd get_covariance(std::shared_ptr<void> sensor_data)
  {
  }

  void set_initial_calib(std::shared_ptr<void> calibration)
  {
  }

  BufferDataType Initialize(const Time& timestamp, std::shared_ptr<void> measurement,
                            std::shared_ptr<CoreType> latest_core_data)
  {
  }

  bool CalcUpdate(const Time& timestamp, std::shared_ptr<void> measurement, const CoreStateType& prior_core_state_data,
                  std::shared_ptr<void> latest_sensor_data, const Eigen::MatrixXd& prior_cov,
                  BufferDataType* new_state_data)
  {
    return false;
  }
};
}  // namespace mars

#endif  // IMU_SENSOR_CLASS_H
