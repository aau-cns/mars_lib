// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@aau.at>

#ifndef PRESSURECONVERSION_H
#define PRESSURECONVERSION_H

#include <Eigen/Dense>

namespace mars
{
struct Pressure
{
  enum class Type
  {
    FLUID,
    GAS,
    HEIGHT
  };
  inline friend std::ostream& operator<<(std::ostream& out, const Type& type);

  Pressure() = default;
  Pressure(double pressure, double temperature, Type type) : data_(pressure), temperature_K_(temperature), type_(type)
  {
  }

  double data_{ 0 };
  double temperature_K_{ 0 };
  Type type_{ Type::GAS };

  friend std::ostream& operator<<(std::ostream& out, const Pressure& pressure);
};

struct GasPressureOptions
{
  GasPressureOptions()
  {
    set_constants();
  }
  GasPressureOptions(double P_sl, double M, double r, double g) : P_sl(P_sl), M(M), r(r), g(g)
  {
    set_constants();
  }

  const double P_sl{ 101325 };  // Pascal
  const double M{ 0.0289644 };  // Kg*mol
  const double r{ 8.31432 };    // Nm/mol*K
  const double g{ 9.80665 };

  double rOverMg;
  double ln_Psl;
  double ln_P0PslT;

  void set_constants()
  {
    rOverMg = r / (M * g);
    ln_Psl = std::log(P_sl);
  }

  void update_constants(Pressure p0)
  {
    ln_P0PslT = (std::log(p0.data_) - ln_Psl) * p0.temperature_K_;
  }
};

class PressureConversion
{
public:
  typedef Eigen::Matrix<double, 1, 1> Matrix1d;

  PressureConversion() = default;
  PressureConversion(Pressure pressure) : PressureConversion(pressure, GasPressureOptions()){};
  PressureConversion(Pressure pressure, GasPressureOptions gas_options);

  void set_pressure_reference(Pressure pressure);

  Matrix1d get_height(Pressure pressure);

private:
  Pressure reference_;
  GasPressureOptions gas_options_;
  bool reference_is_set_{ false };

  double get_height_fluid(const Pressure& pressure);
  double get_height_gas(const Pressure& pressure);
};

}  // namespace mars

#endif  // PRESSURECONVERSION_H
