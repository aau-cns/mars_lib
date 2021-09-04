// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include "gps_conversion.h"
#include <cmath>
#include <Eigen/Dense>

namespace mars
{
std::ostream& operator<<(std::ostream& out, const GpsCoordinates& coordinates)
{
  out << "Lat:\t" << coordinates.latitude_ << std::endl;
  out << "Long:\t" << coordinates.longitude_ << std::endl;
  out << "Alt:\t" << coordinates.altitude_ << std::endl;

  return out;
}

Eigen::Matrix<double, 3, 1> mars::GpsConversion::get_enu(mars::GpsCoordinates coordinates)
{
  return WGS84ToENU(coordinates);
}

GpsConversion::GpsConversion(mars::GpsCoordinates coordinates)
{
  ecef_ref_orientation_.setIdentity();
  ecef_ref_point_.setZero();

  set_gps_reference(coordinates);
}

GpsCoordinates mars::GpsConversion::get_gps_reference()
{
  return reference_;
}

void GpsConversion::set_gps_reference(mars::GpsCoordinates coordinates)
{
  // set gps reference coordinates
  reference_ = coordinates;

  // set ecef reference, position and orientation
  const double rad_lat = deg2rad(coordinates.latitude_);
  const double rad_long = deg2rad(coordinates.longitude_);

  const double s_lat = sin(rad_lat);
  const double c_lat = cos(rad_lat);

  const double s_long = sin(rad_long);
  const double c_long = cos(rad_long);

  Eigen::Matrix3d R;
  R(0, 0) = -s_long;
  R(0, 1) = c_long;
  R(0, 2) = 0;

  R(1, 0) = -s_lat * c_long;
  R(1, 1) = -s_lat * s_long;
  R(1, 2) = c_lat;

  R(2, 0) = c_lat * c_long;
  R(2, 1) = c_lat * s_long;
  R(2, 2) = s_lat;

  ecef_ref_orientation_ = R;
  ecef_ref_point_ = WGS84ToECEF(coordinates);
}

double GpsConversion::deg2rad(const double& deg)
{
  return (M_PI / 180) * deg;
}

Eigen::Matrix<double, 3, 1> GpsConversion::WGS84ToENU(const mars::GpsCoordinates& coordinates)
{
  return ECEFToENU(WGS84ToECEF(coordinates));
}

Eigen::Matrix<double, 3, 1> GpsConversion::ECEFToENU(const Eigen::Matrix<double, 3, 1>& ecef)
{
  Eigen::Matrix<double, 3, 1> enu = ecef_ref_orientation_ * (ecef - ecef_ref_point_);
  return enu;
}

Eigen::Matrix<double, 3, 1> GpsConversion::WGS84ToECEF(const mars::GpsCoordinates& coordinates)
{
  // WGS84 ellipsoid constants
  constexpr double a = 6378137.0;             // semi-major axis
  constexpr double ecc = 8.1819190842622e-2;  // eccentricity of this ellipsoid
  constexpr double ecc_sq = ecc * ecc;

  const double rad_lat = deg2rad(coordinates.latitude_);
  const double rad_long = deg2rad(coordinates.longitude_);

  const double s_lat = sin(rad_lat);
  const double c_lat = cos(rad_lat);
  const double s_long = sin(rad_long);
  const double c_long = cos(rad_long);

  const double N = a / sqrt(1 - ecc_sq * s_lat * s_lat);

  Eigen::Matrix<double, 3, 1> ecef;
  const double h = coordinates.altitude_;
  ecef(0) = (N + h) * c_lat * c_long;
  ecef(1) = (N + h) * c_lat * s_long;
  ecef(2) = (N * (1 - ecc_sq) + h) * s_lat;

  return ecef;
}
}
