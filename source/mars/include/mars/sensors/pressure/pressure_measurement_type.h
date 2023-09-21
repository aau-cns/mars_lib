// Copyright (C) 2021-2023 Martin Scheiber, Christian Brommer,
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

#ifndef PRESSURE_MEASUREMENT_TYPE_H
#define PRESSURE_MEASUREMENT_TYPE_H

#include <mars/sensors/measurement_base_class.h>
#include <mars/sensors/pressure/pressure_conversion.h>
#include <Eigen/Dense>
#include <utility>

namespace mars
{
class PressureMeasurementType : public BaseMeas
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Pressure pressure_;  ///< Raw pressure measurement [Pascal] including the ambient temperature in [K]

  PressureMeasurementType(const double& height)
  {
    pressure_.type_ = Pressure::Type::HEIGHT;
    pressure_.data_ = height;
  }

  PressureMeasurementType(const double& pressure, const double& temperature)
    : PressureMeasurementType(pressure, temperature, Pressure::Type::GAS)
  {
  }

  PressureMeasurementType(const double& pressure, const double& temperature, const Pressure::Type& type)
  {
    pressure_.type_ = type;
    pressure_.data_ = pressure;
    pressure_.temperature_K_ = temperature;
  }

  static std::string get_csv_header_string()
  {
    std::stringstream os;
    os << "t, ";
    os << "pressure, ";
    os << "temperature, ";
    os << "type";

    return os.str();
  }

  std::string to_csv_string(const double& timestamp) const
  {
    std::stringstream os;
    os.precision(17);
    os << timestamp;

    os << ", " << pressure_.data_;
    os << ", " << pressure_.temperature_K_;
    os << ", " << pressure_.type_;

    return os.str();
  }
};
}  // namespace mars

#endif  // PRESSURE_MEASUREMENT_TYPE_H
