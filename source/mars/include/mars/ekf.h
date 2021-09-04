// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef EKF_HPP
#define EKF_HPP

#include <Eigen/Dense>
#include <iostream>

namespace mars
{
class Ekf
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Ekf(const Eigen::Ref<const Eigen::MatrixXd>& H, const Eigen::Ref<const Eigen::MatrixXd>& R,
      const Eigen::Ref<const Eigen::MatrixXd>& res, const Eigen::Ref<const Eigen::MatrixXd>& P)
  {
    this->H_ = H;
    this->R_ = R;
    this->res_ = res;
    this->P_ = P;
    // S_.setZero();
    // K_.setZero();
  }

  Eigen::MatrixXd H_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd res_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd S_;
  Eigen::MatrixXd K_;

  Eigen::MatrixXd CalculateCorrection();

  Eigen::MatrixXd CalculateCovUpdate();
};
}

#endif  // EKF_HPP
