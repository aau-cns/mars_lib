// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include <mars/ekf.h>
#include <mars/general_functions/utils.h>
#include <Eigen/Dense>

namespace mars
{
Eigen::MatrixXd Ekf::CalculateCorrection()
{
  // Calculate innovation
  S_ = H_ * P_ * H_.transpose() + R_;
  S_ = Utils::EnforceMatrixSymmetry(S_);

  // Calculate Klamen Gain
  K_ = P_ * H_.transpose() * S_.inverse();

  // Calculate Correction
  Eigen::MatrixXd correction = K_ * res_;

  return correction;
}

Eigen::MatrixXd Ekf::CalculateCovUpdate()
{
  // Calculate ErrorState Covariance
  int64_t state_size = H_.cols();

  Eigen::MatrixXd I_state = Eigen::MatrixXd::Identity(state_size, state_size);

  Eigen::MatrixXd KH = I_state - K_ * H_;
  Eigen::MatrixXd updated_P = KH * P_ * KH.transpose() + K_ * R_ * K_.transpose();

  return updated_P;
}
}
