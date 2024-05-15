// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef SENSORABSCLASS_H
#define SENSORABSCLASS_H

#include <mars/sensors/sensor_interface.h>
#include <string>

namespace mars
{
class SensorAbsClass : public SensorInterface
{
public:
  int id_{ -1 };
  std::string name_;              ///< Name of the individual sensor instance
  bool is_initialized_{ false };  ///< True if the sensor has been initialized
  bool do_update_{ true };        ///< True if updates should be performed with the sensor
  int type_{ -1 };  ///< Future feature, holds information such as position or orientation for highlevel decissions
  bool const_ref_to_nav_{ true };   ///< True if the reference should not be estimated
  bool ref_to_nav_given_{ false };  ///< True if the reference to the navigation frame is given by initial calibration
  bool use_dynamic_meas_noise_{ false };  ///< True if dynamic noise values from measurements should be used

  ///
  /// \brief operator << Overload the << operator for easy printing of the sensor information
  ///
  friend std::ostream& operator<<(std::ostream& out, const SensorAbsClass& sensor)
  {
    out << "\tSensor: " << sensor.name_ << std::endl;
    out << "\tInitialized: " << sensor.is_initialized_ << std::endl;
    out << "\tDo update: " << sensor.do_update_ << std::endl;
    out << "\tReference to nav: " << sensor.const_ref_to_nav_ << std::endl;
    out << "\tUse dynamic noise: " << sensor.use_dynamic_meas_noise_ << std::endl;
    return out;
  }
};
}  // namespace mars
#endif  // SENSORABSCLASS_H
