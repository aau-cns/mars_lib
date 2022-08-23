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
Eigen::MatrixXd Ekf::CalculateStateCorrection()
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

Eigen::MatrixXd Ekf::CalculateCorrection()
{
  return CalculateStateCorrection();
}

Eigen::MatrixXd Ekf::CalculateCorrection(Chi2* chi2)
{
  Eigen::MatrixXd corr = CalculateStateCorrection();

  if (chi2->do_test_)
  {
    chi2->CalculateChi2(res_, S_);
  }

  return corr;
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

Chi2::Chi2() : dist_(3)  // Using 3 as a dummy value
{
}

Chi2::Chi2(const int& dof, const double& chi_value) : dist_(dof)
{
  set_dof(dof);
  set_chi_value(chi_value);
  CalculateUcv();
}

void Chi2::set_dof(const int& value)
{
  dof_ = abs(value);
  dist_ = boost::math::chi_squared(dof_);
  CalculateUcv();
}

void Chi2::set_chi_value(const double& value)
{
  chi_value_ = value;
  CalculateUcv();
}

void Chi2::CalculateUcv()
{
  ucv_ = boost::math::quantile(complement(dist_, chi_value_));
}

void Chi2::ActivateTest(const bool& value)
{
  this->do_test_ = value;
}

bool Chi2::CalculateChi2(const Eigen::MatrixXd& res, const Eigen::MatrixXd& S)
{
  // Determine whether or not the test passed
  double X2 = (res.transpose() * S.inverse() * res).value();
  passed_ = X2 < ucv_;  // boolean expression

  last_res_ = res;
  last_X2_ = X2;
  return passed_;
}

void Chi2::PrintReport(const std::string& name)
{
  if (do_test_)
  {
    std::cout << name << " - Chi Squared Detected" << std::endl;
    std::cout << "Res: " << last_res_.transpose() << std::endl;
    std::cout << "X2 = " << last_X2_ << ", ucv = " << ucv_ << std::endl;
  }
}
}  // namespace mars
