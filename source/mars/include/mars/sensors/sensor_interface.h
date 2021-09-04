// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef SENSORINTERFACE_H
#define SENSORINTERFACE_H

#include <mars/time.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/core_type.h>
#include <Eigen/Dense>
#include <memory>

namespace mars
{
class SensorInterface
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // virtual ~SensorInterface();

  ///
  /// \brief set_initial_calib Sets the calibration of an individual sensor
  /// \param calibration
  ///
  virtual void set_initial_calib(std::shared_ptr<void> calibration) = 0;

  ///
  /// \brief Initialize the state of an individual sensor
  /// \param timestamp current timestamp
  /// \param measurement current sensor measurement
  /// \param latest_core_data
  /// \return
  ///
  virtual BufferDataType Initialize(const Time& timestamp, std::shared_ptr<void> measurement,
                                    std::shared_ptr<CoreType> latest_core_data) = 0;
  ///
  /// \brief CalcUpdate Calculates the update for an individual sensor definition
  /// \param timestamp current timestamp
  /// \param measurement current sensor measurement
  /// \param prior_core_state_data
  /// \param latest_sensor_data
  /// \param prior_cov Prior covariance containing core, sensor and sensor cross covariance
  /// \return True if the update was successful, false if the update was rejected
  ///
  virtual bool CalcUpdate(const Time& timestamp, std::shared_ptr<void> measurement,
                          const CoreStateType& prior_core_state_data, std::shared_ptr<void> latest_sensor_data,
                          const Eigen::MatrixXd& prior_cov, BufferDataType* new_state_data) = 0;
  ///
  /// \brief get_covariance Resolves a void pointer to the covariance matrix of the corresponding sensor type
  /// Each sensor is responsible to cast its own data type
  ///
  /// \param sensor_data
  /// \return Covariance matrix contained in the sensor data struct
  ///
  virtual Eigen::MatrixXd get_covariance(std::shared_ptr<void> sensor_data) = 0;

protected:
  // SensorInterface(); // construction for child classes only
};
}
#endif  // SENSORINTERFACE_H
