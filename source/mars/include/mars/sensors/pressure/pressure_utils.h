// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@aau.at>

#ifndef PRESSURE_UTILS_H
#define PRESSURE_UTILS_H

#include <mars/buffer.h>
#include <mars/sensors/pressure/pressure_conversion.h>
#include <mars/sensors/pressure/pressure_measurement_type.h>
#include <mars/sensors/sensor_abs_class.h>

namespace mars
{
class PressureInit
{
private:
  double init_duration_{ 1.0 };
  bool b_is_initialized_{ false };
  bool b_verbose_{ false };

public:
  PressureInit() = default;
  PressureInit(const double& init_duration);

  void Reset();

  Pressure get_press_mean(const std::shared_ptr<SensorAbsClass>& sensor_handle, const Buffer& buffer,
                          const Pressure& cur_meas, const Time& cur_time);

  bool IsDone();
};  // class PressureInit
}  // namespace mars

#endif  // PRESSURE_UTILS_H
