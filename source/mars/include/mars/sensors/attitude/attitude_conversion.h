// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@ieee.org>

#ifndef ATTITUDE_CONVERSION_H
#define ATTITUDE_CONVERSION_H

#include <eigen3/Eigen/Dense>

namespace mars
{
///
/// \brief The Attitude struct
///
struct Attitude
{
  Attitude() = default;
  Attitude(Eigen::Quaterniond quaternion) : quaternion_(quaternion)
  {
  }
  Attitude(Eigen::Matrix3d rotation_matrix) : quaternion_(rotation_matrix)
  {
  }
  Attitude(Eigen::Vector3d vec, std::string order = "XYZ")
  {
    // scoping
    using AAd = Eigen::AngleAxisd;
    using v3d = Eigen::Vector3d;

    /// \todo TODO other orders need to be implemented
    if (order == "XYZ")
    {
      quaternion_ = AAd(vec(0), v3d::UnitX()) * AAd(vec(1), v3d::UnitY()) * AAd(vec(2), v3d::UnitZ());
    }
    else if (order == "ZYX")
    {
      quaternion_ = AAd(vec(2), v3d::UnitZ()) * AAd(vec(1), v3d::UnitY()) * AAd(vec(0), v3d::UnitX());
    }
  }

  Eigen::Quaterniond quaternion_;

  Eigen::Matrix3d get_rotation_matrix()
  {
    return quaternion_.toRotationMatrix();
  }

  Eigen::Vector2d get_rp()
  {
    Eigen::Vector3d rpy = mars::Utils::RPYFromRotMat(quaternion_.toRotationMatrix());
    return Eigen::Vector2d(rpy(0), rpy(1));
  }

  friend std::ostream& operator<<(std::ostream& out, const Attitude& attitude);
};

/////
///// \brief The AttitudeConversion class
/////
///// \note Crassidis, J. L. (2006). Sigma-point Kalman filtering for integrated GPS and inertial navigation.
/////
// class AttitudeConversion
//{
// public:
//  AttitudeConversion(mars::Attitude attitude);
//  AttitudeConversion() = default;

//  Eigen::Quaternion<double> get_quaternion(mars::Attitude attitude);
//  Eigen::Matrix<double, 3, 3> get_rotation_matrix(mars::Attitude attitude);

//  ///
//  /// \brief get_gps_reference
//  /// \return GPS reference coordinates
//  ///
//  mars::GpsCoordinates get_gps_reference();

//  ///
//  /// \brief set_gps_reference
//  /// \param coordinates GPS coordinates
//  ///
//  void set_attitude_reference(mars::Attitude attitude);

// private:
//  Attitude reference_;  ///< reference attiude
//  Eigen::Matrix3d ecef_ref_orientation_;
//  Eigen::Matrix<double, 3, 1> ecef_ref_point_;
//  bool reference_is_set{ false };

//  ///
//  /// \brief deg2rad
//  /// \param deg
//  /// \return rad
//  ///
//  double deg2rad(const double& deg);

//  ///
//  /// \brief WGS84ToECEF World Geodetic System 1984 model (WGS-84) to Earth-Centered-Earth-Fixed (ECEF)
//  /// \param coordinates GPS coordinates
//  /// \return ecef position
//  ///
//  Eigen::Matrix<double, 3, 1> WGS84ToECEF(const mars::GpsCoordinates& coordinates);

//  ///
//  /// \brief ECEFToENU Earth-Centered-Earth-Fixed (ECEF) to East-North-Up (ENU)
//  /// \param ecef ecef reference
//  /// \return ENU local position
//  ///
//  Eigen::Matrix<double, 3, 1> ECEFToENU(const Eigen::Matrix<double, 3, 1>& ecef);

//  ///
//  /// \brief WGS84ToENU World Geodetic System 1984 model (WGS-84) to East-North-Up (ENU) based on given reference
//  /// coordinates 'reference_'
//  /// \param coordinates GPS coordinates
//  /// \return ENU local position
//  ///
//  Eigen::Matrix<double, 3, 1> WGS84ToENU(const mars::GpsCoordinates& coordinates);
//};

}  // namespace mars
#endif  // ATTITUDE_CONVERSION_H
