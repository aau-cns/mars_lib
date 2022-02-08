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

#ifndef PRESSUREMEASUREMENTTYPE_H
#define PRESSUREMEASUREMENTTYPE_H

#include <mars/sensors/pressure/pressure_conversion.h>

#include <Eigen/Dense>

namespace mars
{
class PressureMeasurementType
{
public:
  Pressure pressure_;

  PressureMeasurementType(double height)  // : height_(height)
  {
    pressure_.type_ = Pressure::Type::HEIGHT;
    pressure_.data_ = height;
  }

  PressureMeasurementType(double pressure, double temperature)
    : PressureMeasurementType(pressure, temperature, Pressure::Type::GAS)
  {
  }

  PressureMeasurementType(double pressure, double temperature, Pressure::Type type)
  {
    pressure_.type_ = Pressure::Type::GAS;
    pressure_.data_ = pressure;
    pressure_.temperature_K_ = temperature;
  }
};
}  // namespace mars
#endif  // PRESSUREMEASUREMENTTYPE_H
