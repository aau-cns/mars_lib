// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace mars
{
class Utils
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Utils();

  ///
  /// \brief skew generate the skew symmetric matrix of v
  /// \param v 3d vector
  /// \return skew symmetric matrix
  ///
  static Eigen::Matrix3d Skew(const Eigen::Vector3d& v);

  ///
  /// \brief mat_exp Calculation of the matrix exponential, Taylor series cut-off at specified order
  /// \param mat Matrix for Taylor series
  /// \param order Order at wich the series is cut-off
  /// \return
  ///
  static Eigen::Matrix4d MatExp(const Eigen::Matrix4d& A, const int& order = 4);

  ///
  /// \brief omega_mat ight multiplication
  /// \param v
  /// \return
  ///
  /// \note Reference: Joan Sola - Quaternion Kinematics for the error-state kalman filter - Equation(199)
  ///
  static Eigen::Matrix4d OmegaMat(const Eigen::Vector3d& v);

  ///
  /// \brief QuatFromSmallAngle
  /// \param v
  /// \return
  ///
  static Eigen::Quaterniond QuatFromSmallAngle(const Eigen::Vector3d& d_theta_vec);

  ///
  /// \brief ApplySmallAngleQuatCorr
  /// \param q_prior
  /// \param correction
  /// \return
  ///
  static Eigen::Quaterniond ApplySmallAngleQuatCorr(const Eigen::Quaterniond& q_prior,
                                                    const Eigen::Vector3d& correction);

  ///
  /// \brief RPYFromRotMat derives the roll pitch and yaw angle from a rotation matrix (in that order)
  /// \param rot_mat 3x3 rotation matrix
  /// \return Vector with [roll, pitch, yaw]
  ///
  static Eigen::Vector3d RPYFromRotMat(const Eigen::Matrix3d& rot_mat);

  ///
  /// \brief check_cov Performs tests for the properties of a given covariance matrix
  /// \param cov_mat
  /// \param description Used to associate the warning with the given covariance
  /// \return true if the covariance matrix is valid, false otherwise
  ///
  static bool CheckCov(const Eigen::MatrixXd& cov_mat, const std::string& description, const bool& check_cond = false);

  ///
  /// \brief EnforceMatrixSymmetry
  /// \param mat_in
  /// \return
  ///
  static Eigen::MatrixXd EnforceMatrixSymmetry(const Eigen::Ref<const Eigen::MatrixXd>& mat_in);

  ///
  /// \brief quaternionAverage without weights
  /// \param std::vector<Eigen::Quaterniond> vector of quaternion being averaged
  /// \return Eigen::Quaterniond averaged quaternion
  ///
  /// \note Reference: Markley et al., Averaging Quaternions, Journal of Guidance, Control, and Dynamics, 30(4):1193-1196, June 2007
  ///
  static Eigen::Quaterniond quaternionAverage(const std::vector<Eigen::Quaterniond>& quats);

};
}  // namespace mars

#endif  // UTILS_H
