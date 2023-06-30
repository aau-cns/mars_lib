// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include <mars/general_functions/utils.h>
#include <mars_lib/mars_lib-version.h>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <cmath>
#include <iostream>

namespace mars
{
Utils::Utils() = default;

std::string Utils::get_mars_version_string()
{
  return std::string(std::string(MARS_LIB_VERSION) + "-" + std::string(MARS_LIB_VERSION_REVISION));
}

void Utils::PrintMarsVersion()
{
  std::cout << "MaRS Version: " << Utils::get_mars_version_string() << std::endl;
}

Eigen::MatrixXd Utils::EnforceMatrixSymmetry(const Eigen::Ref<const Eigen::MatrixXd>& mat_in)
{
  Eigen::MatrixXd mat_out = (mat_in + mat_in.transpose()) / 2;
  return mat_out;
}

void Utils::TransformImu(const IMUMeasurementType& prev, const IMUMeasurementType& now, const double& dt,
                         const Eigen::Vector3d& p_ab, const Eigen::Quaterniond& q_ab, IMUMeasurementType& result)
{
  const Eigen::Matrix3d R_ba = q_ab.toRotationMatrix().transpose();
  const Eigen::Vector3d w_a_now(now.angular_velocity_);
  const Eigen::Vector3d w_a_prev(prev.angular_velocity_);
  const Eigen::Vector3d acc_a(now.linear_acceleration_);

  // Transformation of angular velocity
  const Eigen::Vector3d w_b = R_ba * w_a_now;

  // Transformation of linear acceleration
  Eigen::Vector3d acc_b;

  if (dt > 0)
  {
    // Account for angular acceleration
    const Eigen::Vector3d dw_a = (w_a_now - w_a_prev) / dt;
    acc_b = R_ba * (acc_a + (Skew(w_a_now) * Skew(w_a_now) + Skew(dw_a)) * p_ab);
  }
  else
  {
    // Since dt is zero, do not concider angular acceleration
    acc_b = R_ba * (acc_a + (Skew(w_a_now) * Skew(w_a_now)) * p_ab);
  }

  // Map return values
  result.angular_velocity_ = w_b;
  result.linear_acceleration_ = acc_b;
  return;
}

void Utils::TransformImu(const IMUMeasurementType& now, const Eigen::Vector3d& p_ab, const Eigen::Quaterniond& q_ab,
                         IMUMeasurementType& result)
{
  const double dt = 0;            // Setting dt=0 ignores angular acceleration
  const IMUMeasurementType prev;  // Empty previous IMU measurement

  TransformImu(prev, now, dt, p_ab, q_ab, result);
}

Eigen::Matrix3d Utils::Skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d res;
  res << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return res;
}

Eigen::Matrix4d Utils::MatExp(const Eigen::Matrix4d& A, const int& order)
{
  Eigen::Matrix4d matexp(Eigen::Matrix4d::Identity());  // initial condition with k=0

  int div = 1;
  Eigen::Matrix4d a_loop = A;

  for (int k = 1; k <= order; k++)
  {
    div = div * k;  // factorial(k)
    matexp = matexp + a_loop / div;
    a_loop = a_loop * A;  // adding one exponent each iteration
  }

  return matexp;
}

Eigen::Matrix4d Utils::OmegaMat(const Eigen::Vector3d& v)
{
  Eigen::Matrix4d res;
  res.setZero();

  res.block(0, 1, 1, 3) = -v.transpose();
  res.block(1, 0, 3, 1) = v;
  res.block(1, 1, 3, 3) = -Skew(v);

  return res;
}

Eigen::Quaterniond Utils::QuatFromSmallAngle(const Eigen::Vector3d& d_theta_vec)
{
  const double d_theta_vec_norm = d_theta_vec.norm();
  const double d_theta_squared = d_theta_vec_norm * d_theta_vec_norm;

  if (d_theta_squared / 4.0 < 1.0)
  {
    return Eigen::Quaterniond(sqrt(1 - d_theta_squared / 4.0), d_theta_vec[0] * 0.5, d_theta_vec[1] * 0.5,
                              d_theta_vec[2] * 0.5);
  }
  else
  {
    // In this case the error angle vector is not small at all.
    std::cout << "Warning: The error angle vector is not small" << std::endl;

    double w = 1.0 / sqrt(1 + d_theta_squared / 4.0);
    double f = w * 0.5;
    return Eigen::Quaterniond(w, d_theta_vec[0] * f, d_theta_vec[1] * f, d_theta_vec[2] * f);
  }
}

Eigen::Quaterniond Utils::ApplySmallAngleQuatCorr(const Eigen::Quaterniond& q_prior, const Eigen::Vector3d& correction)
{
  return (q_prior * QuatFromSmallAngle(correction)).normalized();
}

bool Utils::CheckCov(const Eigen::MatrixXd& cov_mat, const std::string& description, const bool& /*check_cond*/)
{
  bool result = true;
  std::string info("[" + description + "]" + ": ");

  bool is_symmetric = Eigen::MatrixXd::Zero(cov_mat.rows(), cov_mat.cols()).isApprox(cov_mat - cov_mat.transpose());
  if (!is_symmetric)
  {
    std::cout << "Warning: " << info << "The covariance matrix is not symmetric" << std::endl;
    result = false;
  }

  double det = cov_mat.determinant();
  if (det <= 0)
  {
    std::cout << "Warning: " << info << "The determinant of the covariance matrix is not positive (" << det << ")"
              << std::endl;
    result = false;
  }

  double min_eigenvalue = Eigen::EigenSolver<Eigen::MatrixXd>(cov_mat).eigenvalues().real().minCoeff();
  if (min_eigenvalue < 0)
  {
    std::cout << "Warning: " << info << "The covariance matrix is not positive semidefinite min: " << min_eigenvalue
              << std::endl;
    result = false;
  }

  return result;
}

Eigen::Vector3d Utils::RPYFromRotMat(const Eigen::Matrix3d& rot_mat)
{
  // according to this post, mat.eulerAngles returns the correct angles
  Eigen::Vector3d ypr = rot_mat.eulerAngles(2, 1, 0);
  return { ypr(2), ypr(1), ypr(0) };
}

Eigen::Quaterniond Utils::quaternionAverage(const std::vector<Eigen::Quaterniond>& quats)
{
  // Define matrix A
  Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

  // Loop through quaternions
  for (const auto& it : quats)
  {
    // Assign quaternion
    Eigen::Quaterniond q = it.normalized();

    // Flip is w coefficient is negative
    if (q.w() < 0)
    {
      q = Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
    }

    // Build matrix A (A = A + q*q')
    A = A + q.coeffs() * q.coeffs().transpose();
  }

  // Scale A
  A = (1.0 / quats.size()) * A;

  // Compute Eigenvalues and Eigenvectors
  Eigen::EigenSolver<Eigen::Matrix4d> eigs(A);
  Eigen::Matrix4d D = eigs.pseudoEigenvalueMatrix();
  Eigen::Matrix4d V = eigs.pseudoEigenvectors();

  // Get max eigenvalue
  int col_index, row_index;
  D.maxCoeff(&row_index, &col_index);
  assert(row_index == col_index);

  // Get averaged quaternion
  Eigen::Quaterniond qavg = Eigen::Quaterniond(V.col(col_index));

  return qavg;
}

Eigen::Quaterniond Utils::NormalizeQuaternion(const Eigen::Quaterniond& quat, std::string note)
{
  if (abs(quat.norm() - 1) > 0.01)
  {
    std::cout << "[Utils] Warning, quaternion not normalized in: " << note << "Norm: " << quat.norm() << std::endl;
  }

  return quat.normalized();
}

Eigen::Quaterniond Utils::NormalizeQuaternion(const double& w, const double& x, const double& y, const double& z,
                                              std::string note)
{
  return NormalizeQuaternion(Eigen::Quaterniond(w, x, y, z), note);
}

}  // namespace mars
