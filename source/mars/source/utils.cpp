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

#include <Eigen/Dense>
#include <Eigen/LU>
#include <cmath>
#include <iostream>

namespace mars
{
Utils::Utils() = default;

Eigen::MatrixXd Utils::EnforceMatrixSymmetry(const Eigen::Ref<const Eigen::MatrixXd>& mat_in)
{
  Eigen::MatrixXd mat_out = (mat_in + mat_in.transpose()) / 2;
  return mat_out;
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
  return Eigen::Vector3d(ypr(2), ypr(1), ypr(0));
}
}  // namespace mars
