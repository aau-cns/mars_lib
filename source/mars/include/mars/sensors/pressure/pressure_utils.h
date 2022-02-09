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
  PressureInit(double init_duration) : init_duration_(init_duration){};

  void Reset()
  {
    b_is_initialized_ = false;
  }

  Pressure get_press_mean(const std::shared_ptr<SensorAbsClass> sensor_handle, const Buffer& buffer,
                          const Pressure& cur_meas, const Time& cur_time)
  {
    if (b_verbose_)
      std::cout << "[PressureInit]: trying to initialize pressure" << std::endl;

    // if the init duration is smaller than 0.0 then only use 'current' measurement
    if (init_duration_ < 0.0)
    {
      b_is_initialized_;
      return cur_meas;
    }

    // get ordered values from buffer
    std::vector<const mars::BufferEntryType*> pressure_entries;
    buffer.get_sensor_handle_measurements(sensor_handle, pressure_entries);

    // check if enough measurements where recorded
    if (pressure_entries.empty() || (cur_time - pressure_entries.at(0)->timestamp_).get_seconds() < init_duration_)
    {
      if (b_verbose_)
      {
        std::cout << "[PressureInit]: could not init sensor (empty? " << pressure_entries.empty() << ")" << std::endl;
      }
      return cur_meas;
    }

    // calcualte average pressure/height
    Pressure avg_pressure = cur_meas;
    uint cnt_meas = 1;
    for (std::vector<const mars::BufferEntryType*>::reverse_iterator it = pressure_entries.rbegin();
         it != pressure_entries.rend(); ++it)
    {
      if ((cur_time - (*it)->timestamp_).get_seconds() <= init_duration_)
      {
        const PressureMeasurementType meas = *static_cast<PressureMeasurementType*>((*it)->data_.sensor_.get());

        avg_pressure += meas.pressure_;
        cnt_meas++;
      }
      else
      {
        // in this case all other entries will not fullfill requirement of buffer so break early
        break;
      }
    }

    // compute average
    avg_pressure /= cnt_meas;
    b_is_initialized_ = true;

    if (b_verbose_)
    {
      std::cout << "[PressureInit]: finished initialization" << std::endl;
    }

    return avg_pressure;
  }

  bool IsDone()
  {
    return b_is_initialized_;
  }
};  // class PressureInit
}  // namespace mars

#endif  // PRESSURE_UTILS_H
