// Copyright (C) 2021 Martin Scheiber, Christian Brommer,
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

#ifndef PRESSURE_UTILS_H
#define PRESSURE_UTILS_H

#include <mars/buffer.h>
#include <mars/sensors/gps/gps_conversion.h>
#include <mars/sensors/gps/gps_measurement_type.h>
#include <mars/sensors/sensor_abs_class.h>

namespace mars
{
///
/// \brief The GPSInit class is an initializer for calculating the average coordinates over a given time window.
///
class GPSInit
{
private:
  double init_duration_{ 1.0 };     ///< window time to use measurements from
  bool b_is_initialized_{ false };  ///< flag that determines if any previous initialization was successful
  bool b_verbose_{ false };         ///< enable/disable output (currently hardcoded)

public:
  GPSInit() = default;
  GPSInit(const double& init_duration);

  ///
  /// \brief Reset resets the initialization routine
  ///
  void Reset();

  ///
  /// \brief get_gps_mean calculates the mean over the given time window using the measurements in the buffer
  /// \param sensor_handle sensor handle to use
  /// \param buffer mars::Buffer to retrieve measurements from
  /// \param cur_meas current measurement (returned if none found in buffer)
  /// \param cur_time current time (returned if none found in buffer)
  /// \return the mean of the GPSCoordinates within the time window
  ///
  GpsCoordinates get_gps_mean(const std::shared_ptr<SensorAbsClass>& sensor_handle, const Buffer& buffer,
                              const GpsCoordinates& cur_meas, const Time& cur_time);

  ///
  /// \brief set_done sets the routine to done, even if no calculation was yet performed or unsuccesful
  ///
  void set_done();

  ///
  /// \brief IsDone returns the value of b_is_initialized_
  /// \return true if initialization was succesful
  ///
  bool IsDone() const;
};  // class PressureInit
}  // namespace mars

#endif  // PRESSURE_UTILS_H
