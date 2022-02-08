// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@aau.at>

#include "pressure_conversion.h"

namespace mars
{
std::ostream& operator<<(std::ostream& out, const Pressure::Type& type)
{
  switch (type)
  {
    case Pressure::Type::FLUID:
      out << "FLUID";
      break;
    case Pressure::Type::GAS:
      out << "GAS";
      break;
    case Pressure::Type::HEIGHT:
      out << "HEIGHT";
      break;
  }

  return out;
}

std::ostream& operator<<(std::ostream& out, const Pressure& pressure)
{
  out << "p [PA]: " << pressure.data_ << std::endl;
  out << "T [K]:  " << pressure.temperature_K_ << std::endl;
  out << "type:   " << pressure.type_ << std::endl;

  return out;
}

PressureConversion::PressureConversion(Pressure pressure, GasPressureOptions gas_options) : gas_options_(gas_options)
{
  set_pressure_reference(pressure);
}

void PressureConversion::set_pressure_reference(Pressure pressure)
{
  // set pressure reference
  reference_ = pressure;
}

PressureConversion::Matrix1d PressureConversion::get_height(Pressure pressure)
{
  switch (pressure.type_)
  {
    case mars::Pressure::Type::FLUID:
      return Matrix1d(get_height_fluid(pressure));
    case mars::Pressure::Type::GAS:
      return Matrix1d(get_height_gas(pressure));
    case mars::Pressure::Type::HEIGHT:
      return Matrix1d(pressure.data_);
  }
}

double PressureConversion::get_height_fluid(const Pressure& pressure)
{
  /// \todo TODO(scm): implement this
  return -1.0;
}

double PressureConversion::get_height_gas(const Pressure& pressure)
{
  return gas_options_.rOverMg *
         (gas_options_.ln_P0PslT - (std::log(pressure.data_) - gas_options_.ln_Psl) * pressure.temperature_K_);
}
}  // namespace mars
