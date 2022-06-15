// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include <mars/nearest_cov.h>
#include <Eigen/Dense>

namespace mars
{
NearestCov::NearestCov(const Eigen::MatrixXd& covariance) : cov_mat_(covariance)
{
  // Ensure the matrix is square
  assert(covariance.rows() == covariance.cols());
}

Eigen::MatrixXd NearestCov::EigenCorrectionUsingCovariance(NearestCovMethod method)
{
  Eigen::EigenSolver<Eigen::MatrixXd> vd(cov_mat_);

  Eigen::EigenSolver<Eigen::MatrixXd>::EigenvectorsType V(vd.eigenvectors());
  Eigen::EigenSolver<Eigen::MatrixXd>::EigenvalueType D(vd.eigenvalues());

  if (!(V.imag().isZero() && D.imag().isZero()))
  {
    std::cout << "Warning: Eigenvalue decomposition has imaginary components" << std::endl;
  }

  Eigen::MatrixXd V_real(V.real());
  Eigen::VectorXd D_real(D.real());

  // determine if the matrix is already positive-semi-definite
  bool no_negative_eigenvalues = true;
  for (int k = 0; k < D_real.size(); k++)
  {
    if (D_real[k] < 0)
    {
      no_negative_eigenvalues = false;
    }
  }

  if (no_negative_eigenvalues)
  {
    return cov_mat_;
  }

  Eigen::VectorXd D_corrected(D_real);

  // Correct the covariance matrix
  switch (method)
  {
    case NearestCovMethod::abs:
      // replace negative Eigenvalues with their absolut value
      for (int k = 0; k < D_corrected.size(); k++)
      {
        if (D_corrected[k] < 0)
        {
          D_corrected[k] = std::abs(D_corrected[k]);
        }
      }
      break;

    case NearestCovMethod::zero:
      // replace negative Eigenvalues with zero
      for (int k = 0; k < D_corrected.size(); k++)
      {
        if (D_corrected[k] < 0)
        {
          D_corrected[k] = 0.0;
        }
      }
      break;

    case NearestCovMethod::delta:
      // replace negative Eigenvalues with a positive delta
      for (int k = 0; k < D_corrected.size(); k++)
      {
        if (D_corrected[k] < 0)
        {
          D_corrected[k] = delta_;
        }
      }
      break;

    case NearestCovMethod::none:
      // do not perform any changes
      return cov_mat_;
      break;

    default:
      std::cout << "Warning: Unexpected method for nearest_cov" << std::endl;
      break;
  }

  Eigen::MatrixXd result(V_real * D_corrected.asDiagonal() * V_real.inverse());
  return result;
}

Eigen::MatrixXd NearestCov::EigenCorrectionUsingCorrelation(NearestCovMethod method)
{
  // TODO
  return {};
}
}  // namespace mars
