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
    case Pressure::Type::LIQUID:
      out << "LIQUID";
      break;
    case Pressure::Type::GAS:
      out << "GAS";
      break;
    case Pressure::Type::HEIGHT:
      out << "HEIGHT";
      break;
    default:
      out << "UNKNOWN";
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

PressureConversion::PressureConversion(Pressure pressure, MediumPressureOptions medium_options)
  : medium_options_(medium_options)
{
  set_pressure_reference(pressure);
}

void PressureConversion::set_pressure_reference(Pressure pressure)
{
  // set pressure reference
  reference_ = pressure;
  medium_options_.update_constants(reference_);
  reference_is_set_ = true;
  medium_options_.PrintGasOptions();
}

PressureConversion::Matrix1d PressureConversion::get_height(Pressure pressure)
{
  switch (pressure.type_)
  {
    case mars::Pressure::Type::LIQUID:
      return Matrix1d(get_height_liquid(pressure));
    case mars::Pressure::Type::GAS:
      return Matrix1d(get_height_gas(pressure));
    case mars::Pressure::Type::HEIGHT:
      return Matrix1d(pressure.data_);
    default:
      std::cout << "Error: [PressureConversion] Cannot return height (unknown type)" << std::endl;
      return Matrix1d(-1);
  }
}

double PressureConversion::get_height_liquid(const Pressure& pressure)
{
  return medium_options_.OneOverGRho * pressure.data_;
}

double PressureConversion::get_height_gas(const Pressure& pressure)
{
  return medium_options_.rOverMg *
         (medium_options_.ln_P0PslT - (std::log(pressure.data_) - medium_options_.ln_Psl) * pressure.temperature_K_);
}
}  // namespace mars
