// Copyright (C) 2022-2023 Martin Scheiber, Christian Brommer,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <christian.brommer@ieee.org>
// and <martin.scheiber@ieee.org>.

#ifndef PRESSURE_UTILS_H
#define PRESSURE_UTILS_H

#include <mars/buffer.h>
#include <mars/sensors/pressure/pressure_conversion.h>
#include <mars/sensors/pressure/pressure_measurement_type.h>
#include <mars/sensors/sensor_abs_class.h>

namespace mars
{
///
/// \brief Pressure initalization object to calcualte mean initial pressure.
///
class PressureInit
{
private:
  double init_duration_{ 1.0 };     ///< Duration in sec used to average initialization
  bool b_is_initialized_{ false };  ///< Flag to determine if initialization was performed successfully
  bool b_verbose_{ false };         ///< Flag to enable verbos output

public:
  PressureInit() = default;

  ///
  /// \param init_duration Duration in seconds used to calculate mean over. Use 0.0 if only the last measurement should
  /// be used.
  ///
  PressureInit(const double& init_duration);

  void Reset();

  ///
  /// \brief Calculates the mean pressure of the given sensor_handles's measurement.
  ///
  /// \param sensor_handle mars::SensorAbsClass sensor handle describing which's sensor measurements to use
  /// \param buffer mars::Buffer with measurements stored
  /// \param cur_meas current measurement (latest)
  /// \param cur_time current time (to calcualte mean given the #init_duration_)
  /// \return Pressure mean pressure
  ///
  Pressure get_press_mean(const std::shared_ptr<SensorAbsClass>& sensor_handle, const Buffer& buffer,
                          const Pressure& cur_meas, const Time& cur_time);

  bool IsDone();
};  // class PressureInit
}  // namespace mars

#endif  // PRESSURE_UTILS_H
