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
#include <iostream>

namespace mars
{
///
/// \brief The Pressure struct describes the raw pressure measurement used for conversion later
///
struct Pressure
{
  ///
  /// \brief The Type enum determines the type of pressure measurement used for conversion.
  ///
  enum class Type
  {
    LIQUID,
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

  inline Pressure operator+(const Pressure& pressure)
  {
    return { data_ + pressure.data_, temperature_K_ + pressure.temperature_K_, type_ };
  }

  inline void operator+=(const Pressure& pressure)
  {
    data_ += pressure.data_;
    temperature_K_ += pressure.temperature_K_;
  }

  inline void operator/=(const double& n)
  {
    if (n == 0)
    {
      throw std::overflow_error("Divide by zero exception");
    }
    data_ /= n;
    temperature_K_ /= n;
  }
};

///
/// \brief The MediumPressureOptions struct contains all medium-related (gas, liquid, fluid, etc.) variables needed for
/// pressure calculation.
///
struct MediumPressureOptions
{
  MediumPressureOptions()
  {
    set_constants();
  }
  MediumPressureOptions(double P_sl, double M, double r, double g) : P_sl(P_sl), M(M), r(r), g(g)
  {
    set_constants();
  }

  // general constants
  const double g{ 9.80665 };  //!< gravity constant [m/s^2]

  // gas constants
  const double P_sl{ 101325 };  //!< (gas) pressure at sealevel [Pascal]
  const double M{ 0.0289644 };  //!< (gas) [Kg*mol]
  const double r{ 8.31432 };    //!< (gas) [Nm/mol*K]

  // liquid constants
  const double rho{ 997 };  //!< (liquid) density of the medium [kg/m^2]

  // gas variables
  double rOverMg;
  double ln_Psl;
  double ln_P0PslT;

  // liquid variables
  double OneOverGRho;

  void set_constants()
  {
    rOverMg = r / (M * g);
    ln_Psl = std::log(P_sl);
    OneOverGRho = 1.0 / (g * rho);
  }

  void update_constants(Pressure p0)
  {
    ln_P0PslT = (std::log(p0.data_) - ln_Psl) * p0.temperature_K_;
  }

  void PrintGasOptions()
  {
    std::cout << "Medium Options:\n"
              << "\tg:          " << g << " m/s^2\n"
              << "\tGas Options:\n"
              << "\t  P_sl:     " << P_sl << " Pa\n"
              << "\t  M:        " << M << " Kg*mol\n"
              << "\t  r:        " << r << " Nm/mol*K\n"
              << "\t  rOverMg:  " << rOverMg << " Nm s^2 / mol^2 Kg Km\n"
              << "\t  ln_Psl:   " << ln_Psl << " log(Pa)\n"
              << "\tLiquid Options:\n"
              << "\t  rho:      " << rho << " Pa\n"
              << "\t  1/gRho:   " << OneOverGRho << " Pa\n"
              << std::endl;
  }
};

class PressureConversion
{
public:
  typedef Eigen::Matrix<double, 1, 1> Matrix1d;

  PressureConversion() = default;
  PressureConversion(Pressure pressure) : PressureConversion(pressure, MediumPressureOptions()){};
  PressureConversion(Pressure pressure, MediumPressureOptions gas_options);

  void set_pressure_reference(Pressure pressure);

  Matrix1d get_height(Pressure pressure);

private:
  Pressure reference_;
  MediumPressureOptions medium_options_;
  bool reference_is_set_{ false };

  double get_height_liquid(const Pressure& pressure);
  double get_height_gas(const Pressure& pressure);
};

}  // namespace mars

#endif  // PRESSURECONVERSION_H
