// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef NEARESTCOV_H
#define NEARESTCOV_H

#include <mars/ekf.h>
#include <Eigen/Dense>

namespace NearestCovMethods
{
///
/// \brief The NearestCovMethod enum defines the method to be used by the NearesCov algorythmus
///
enum NearestCovMethod
{
  abs,    ///< Correcting negative eigenvaules to their absolut
  zero,   ///< Correcting negative eigenvaules to zero
  delta,  ///< Correcting negative eigenvaules to a defined delta
  none    ///< No Correction, direct passtrough
};
}  // namespace NearestCovMethods

namespace mars
{
using NearestCovMethod = NearestCovMethods::NearestCovMethod;
///
/// \brief The NearestCov class generates a PSD Cov for a given pseudo-cov matrix
///
/// This class provides methods to generare a positive-semi-definite (PSD)
/// covariance matrix for a given non-positive-semi-definite covariance matric (pseude covariance).
///
class NearestCov
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::MatrixXd cov_mat_;  ///< Input pseudo covariance
  double delta_{ 0.005 };    ///< default correction for the delta method

  ///
  /// \brief NearestCov Constructor
  /// \param covariance Input pseudo covariance
  ///
  NearestCov(const Eigen::MatrixXd& covariance);

  ///
  /// \brief EigenCorrectionUsingCovariance
  /// \param method Determines methode for the eigen covariance correction
  /// \return Corrected covariance
  ///
  Eigen::MatrixXd EigenCorrectionUsingCovariance(NearestCovMethod method);

  ///
  /// \brief EigenCorrectionUsingCorrelation
  /// \param method Determines methode for the eigen covariance correction
  /// \return Corrected covariance
  ///
  Eigen::MatrixXd EigenCorrectionUsingCorrelation(NearestCovMethod method);
};
}  // namespace mars

#endif  // NEARESTCOV_H
