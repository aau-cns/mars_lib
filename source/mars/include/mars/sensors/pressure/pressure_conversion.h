// Copyright (C) 2021-2023 Martin Scheiber, Alessandro Fornasier, Christian Brommer,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <christian.brommer@ieee.org>
// <alessandro.fornasier@ieee.org> and <martin.scheiber@ieee.org>.

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
    LIQUID,  ///< pressure measurement is in liquid medium, e.g., water
    GAS,     ///< pressure measruement is in gas medium, e.g., air
    HEIGHT   ///< pressure measurement has already been converted to height by sensor
  };
  inline friend std::ostream& operator<<(std::ostream& out, const Type& type);

  ///
  /// \brief Default constructor initializing values to 0, and type to Type::GAS.
  ///
  Pressure() = default;

  ///
  /// \brief Detailed constructor with pressure value
  ///
  /// \param pressure measured pressure value
  /// \param temperature temperature at time of measurement
  /// \param type Type of measurement
  ///
  Pressure(double pressure) : data_(pressure)
  {
  }

  ///
  /// \brief Detailed constructor.
  ///
  /// \param pressure measured pressure value
  /// \param temperature temperature at time of measurement
  /// \param type Type of measurement
  ///
  Pressure(double pressure, double temperature, Type type) : data_(pressure), temperature_K_(temperature), type_(type)
  {
  }

  double data_{ 0 };           ///< measurement data
  double temperature_K_{ 0 };  ///< ambient temperature when measurement data was observed
  Type type_{ Type::GAS };     ///< type of the measurement

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
/// \see https://en.wikipedia.org/wiki/Barometric_formula#Pressure_equations
///
struct MediumPressureOptions
{
  ///
  /// \brief Default constructor for MediumPressureOptions and sets all constants to default values.
  ///
  MediumPressureOptions()
  {
    set_constants();
  }

  ///
  /// \brief Detailted constructor for Pressure::Type::GAS constants
  ///
  /// \param P_sl pressure at sealevel in [Pascal] (default: 101325)
  /// \param M molar mass of dry air in [Kg/mol] (default: 0.289644)
  /// \param r universal gas constant in [Nm/(mol*K)] (default 8.3144598)
  /// \param g gravity constant in [m/s^2] (default: 9.80665)
  /// \see https://en.wikipedia.org/wiki/Barometric_formula#Pressure_equations
  ///
  MediumPressureOptions(double P_sl, double M, double r, double g) : g(g), P_sl(P_sl), M(M), r(r)
  {
    set_constants();
  }

  ///
  /// \brief Detailted constructor for Pressure::Type::LIQUID constants
  ///
  /// \param rho density of the liquid [kg/m^3] (default: 997)
  /// \param g gravity constant in [m/s^2] (default: 9.80665)
  /// \see https://en.wikipedia.org/wiki/Barometric_formula#Pressure_equations
  ///
  MediumPressureOptions(double rho, double g) : g(g), rho(rho)
  {
    set_constants();
  }

  // general constants
  const double g{ 9.80665 };  ///< gravity constant [m/s^2]

  // gas constants
  const double P_sl{ 101325 };  ///< (gas) pressure at sealevel [Pascal]
  const double M{ 0.0289644 };  ///< (gas) molar mass of dry air [Kg/mol]
  const double r{ 8.31432 };    ///< (gas) universal gas constant [Nm/mol*K]

  // liquid constants
  const double rho{ 997 };  ///< (liquid) density of the medium [kg/m^3]

  // gas variables
  double rOverMg;    ///< (gas) = #r/(#M*#g), for faster calculations
  double ln_Psl;     ///< (gas) = log(#P_sl), log of the pressure at sealevel
  double ln_P0PslT;  ///< (gas) = (log(P_meas) - log(#P_sl)) * T_meas

  // liquid variables
  double OneOverGRho;  ///< (liquid) = 1/(#g*#rho)

  ///
  /// \brief Sets a couple of constants for faster calculations.
  ///
  void set_constants()
  {
    rOverMg = r / (M * g);
    ln_Psl = std::log(P_sl);
    OneOverGRho = 1.0 / (g * rho);
  }

  ///
  /// \brief Updates the constants related to the reference pressure P_0, where height = 0.
  ///
  /// \param p0 Pressure to set as reference pressure
  ///
  void update_constants(Pressure p0)
  {
    ln_P0PslT = (std::log(p0.data_) - ln_Psl) * p0.temperature_K_;
  }

  ///
  /// \brief Print a list of all gas options currently set.
  ///
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

  ///
  /// \brief Default constructor for creating a PressureConversion object.
  ///
  /// The #medium_options_ are initialized to their default values.
  /// \see MediumPressureOptions
  ///
  PressureConversion() = default;

  ///
  /// \brief Detailed constructor to set also the pressure reference.
  ///
  /// \param pressure Pressure to set as reference for height = 0.
  ///
  /// The #medium_options_ are initialized to their default values.
  /// \see MediumPressureOptions
  ///
  PressureConversion(Pressure pressure) : PressureConversion(pressure, MediumPressureOptions()){};

  ///
  /// \brief Detailed constructor to set the pressure reference and medium options.
  ///
  /// \param pressure Pressure to set as reference for height = 0.
  /// \param gas_options MediumPressureOptions to use
  ///
  /// Use this constructor if your medium is different than either air (gas) or water (liquid).
  ///
  PressureConversion(Pressure pressure, MediumPressureOptions gas_options);

  ///
  /// \brief Setter function to set the reference pressure after construction of object.
  ///
  /// \param pressure Pressure reference to set
  ///
  void set_pressure_reference(Pressure pressure);

  ///
  /// \brief Converts the given pressure measurement to a height (distance) value.
  ///
  /// \param pressure Pressure to convert to height
  /// \return Matrix1d
  ///
  Matrix1d get_height(Pressure pressure);

private:
  Pressure reference_;                    ///< reference pressure for h=0
  MediumPressureOptions medium_options_;  ///< pressure medium (gas or liquid) options to use
  bool reference_is_set_{ false };        ///< flag to determine if the #reference_ has been set

  ///
  /// \brief Converts the given pressure measurement to a height based
  ///
  /// \param pressure Pressure of type Pressure::Type::LIQUID to convert to height.
  /// \return double the height of the measurement
  ///
  /// The measurement is converted assuming that the height is directly proportional to the pressure, .i.e,
  ///
  ///   h = \frac{P}{\rho g},
  ///
  /// with P being the measured pressure, \rho the liquid's density, g the gravity at the surface of
  /// the liquid, and h the calculated height.
  ///
  /// \see https://en.wikipedia.org/wiki/Pressure#Liquid_pressure
  ///
  double get_height_liquid(const Pressure& pressure);

  ///
  /// \brief Converts the given pressure measurement to a height based
  ///
  /// \param pressure Pressure of type Pressure::Type::GAS to convert to height.
  /// \return double the height of the measurement
  ///
  /// The measurement is converted assuming that the height is directly proportional to the pressure, .i.e,
  ///
  /// h_start = ((log(P_0) - log(P_sl))*T_0 ) * (R)/(M*g),
  ///
  /// h_now   = ((log(P) - log(P_sl))*T ) * (R)/(M*g),
  ///
  /// h = h_now-h_start,
  ///
  /// with P being the measured pressure, P_0 the measured initial pressure (for h=0), P_sl the medium's pressure at
  /// sealevel, T the measured temperature, T_0 the measured initial temperature, R the universal gas constant, M the
  /// molar mass of the medium, g the gravity at sealevel, and h the calculated height.
  ///
  /// \see https://en.wikipedia.org/wiki/Barometric_formula#Pressure_equations
  ///
  double get_height_gas(const Pressure& pressure);
};

}  // namespace mars

#endif  // PRESSURECONVERSION_H
