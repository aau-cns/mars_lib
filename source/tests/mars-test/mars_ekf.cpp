// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include <gmock/gmock.h>
#include <mars/ekf.h>
#include <Eigen/Dense>

class mars_Ekf_test : public testing::Test
{
public:
};

TEST_F(mars_Ekf_test, CTOR_EKF)
{
  Eigen::Matrix4d H = Eigen::Matrix4d::Random();
  Eigen::Matrix4d R = Eigen::Matrix4d::Random();
  Eigen::MatrixXd res = Eigen::MatrixXd::Random(4, 1);
  Eigen::Matrix4d P = Eigen::Matrix4d::Random();

  mars::Ekf test(H, R, res, P);
  test.CalculateCorrection();
  test.CalculateCovUpdate();
}
