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
  int type_{ -1 };  ///< Future feature, holds information such as position or orientation for highlevel decissions
};
}
#endif  // SENSORABSCLASS_H
