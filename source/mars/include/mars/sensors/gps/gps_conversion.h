// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef GPS_CONVERSION_H
#define GPS_CONVERSION_H

#include <Eigen/Dense>

namespace mars
{
///
/// \brief The GpsCoordinates struct
///
struct GpsCoordinates
{
  GpsCoordinates() = default;
  GpsCoordinates(double latitude, double longitude, double altitude)
    : latitude_(latitude), longitude_(longitude), altitude_(altitude)
  {
  }
  double latitude_{ 0 };
  double longitude_{ 0 };
  double altitude_{ 0 };

  friend std::ostream& operator<<(std::ostream& out, const GpsCoordinates& coordinates);
};

///
/// \brief The GpsConversion class
///
/// \note Crassidis, J. L. (2006). Sigma-point Kalman filtering for integrated GPS and inertial navigation.
///
class GpsConversion
{
public:
  GpsConversion(mars::GpsCoordinates coordinates);
  GpsConversion() = default;

  ///
  /// \brief get_enu get current GPS reference coordinates
  /// \param coordinates GPS coordinates
  /// \return ENU local position
  ///
  Eigen::Matrix<double, 3, 1> get_enu(mars::GpsCoordinates coordinates);

  ///
  /// \brief get_gps_reference
  /// \return GPS reference coordinates
  ///
  mars::GpsCoordinates get_gps_reference();

  ///
  /// \brief set_gps_reference
  /// \param coordinates GPS coordinates
  ///
  void set_gps_reference(mars::GpsCoordinates coordinates);

private:
  GpsCoordinates reference_;  ///< GPS reference coordinates
  Eigen::Matrix3d ecef_ref_orientation_;
  Eigen::Matrix<double, 3, 1> ecef_ref_point_;
  bool reference_is_set{ false };
  ///
  /// \brief deg2rad
  /// \param deg
  /// \return rad
  ///
  double deg2rad(const double& deg);

  ///
  /// \brief WGS84ToECEF World Geodetic System 1984 model (WGS-84) to Earth-Centered-Earth-Fixed (ECEF)
  /// \param coordinates GPS coordinates
  /// \return ecef position
  ///
  Eigen::Matrix<double, 3, 1> WGS84ToECEF(const mars::GpsCoordinates& coordinates);

  ///
  /// \brief ECEFToENU Earth-Centered-Earth-Fixed (ECEF) to East-North-Up (ENU)
  /// \param ecef ecef reference
  /// \return ENU local position
  ///
  Eigen::Matrix<double, 3, 1> ECEFToENU(const Eigen::Matrix<double, 3, 1>& ecef);

  ///
  /// \brief WGS84ToENU World Geodetic System 1984 model (WGS-84) to East-North-Up (ENU) based on given reference
  /// coordinates 'reference_'
  /// \param coordinates GPS coordinates
  /// \return ENU local position
  ///
  Eigen::Matrix<double, 3, 1> WGS84ToENU(const mars::GpsCoordinates& coordinates);
};
}
#endif  // GPS_CONVERSION_H
