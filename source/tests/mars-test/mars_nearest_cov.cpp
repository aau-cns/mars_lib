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
#include <mars/nearest_cov.h>
#include <Eigen/Dense>

class mars_nearest_cov_test : public testing::Test
{
public:
  Eigen::Matrix4d cov_neg_eig_, cov_pos_eig_, cov_neg_eig_corrected_abs_, cov_neg_eig_corrected_zero_;

  mars_nearest_cov_test()
  {
    cov_neg_eig_ << -1.40568651843696, -0.935075903395594, 0.424486554379189, 0.248762452350135, -0.935075903395594,
        0.287583771888950, -0.317163902917219, 0.773592140380323, 0.424486554379189, -0.317163902917219,
        0.262285343806545, -0.319415112935589, 0.248762452350135, 0.773592140380323, -0.319415112935589,
        -1.81506837287796;

    cov_pos_eig_ << 1.45788020788541, 0.233473041912030, 0.673436221843917, 0.437984464848577, 0.233473041912030,
        0.541459598894344, -0.297932294411096, -0.0591811009600831, 0.673436221843917, -0.297932294411096,
        1.02672375867165, 0.664523387818831, 0.437984464848577, -0.0591811009600831, 0.664523387818831,
        1.91119265069775;

    cov_neg_eig_corrected_abs_ << 1.67167304507162, 0.342825989696312, -0.0241702004802464, -0.424014738367605,
        0.342825989696312, 1.03225030996644, -0.571041234257830, -0.383341439290196, -0.0241702004802463,
        -0.571041234257831, 0.349029764335636, 0.0557433933629866, -0.424014738367605, -0.383341439290196,
        0.0557433933629864, 1.93053885913165;

    cov_neg_eig_corrected_zero_ << 0.132993263317330, -0.296124956849641, 0.200158176949471, -0.0876261430087349,
        -0.296124956849641, 0.659917040927696, -0.444102568587525, 0.195125350545063, 0.200158176949471,
        -0.444102568587525, 0.305657554071091, -0.131835859786301, -0.0876261430087348, 0.195125350545063,
        -0.131835859786301, 0.0577352431268429;
  }
};

TEST_F(mars_nearest_cov_test, CTOR_NEAREST_COV)
{
  mars::NearestCov cov_pos_eig(cov_pos_eig_);

  // Test assertion on non-square cov matrix
  Eigen::Matrix<double, 3, 4> non_square_mat;
  EXPECT_DEBUG_DEATH(mars::NearestCov cov_non_square(non_square_mat), "");
}

TEST_F(mars_nearest_cov_test, CORRECT_TROUGH_COV)
{
  mars::NearestCov cov_pos_eig(cov_pos_eig_);

  EXPECT_TRUE(cov_pos_eig_.isApprox(cov_pos_eig.EigenCorrectionUsingCovariance(mars::NearestCovMethod::abs), 1e-15));
  EXPECT_TRUE(cov_pos_eig_.isApprox(cov_pos_eig.EigenCorrectionUsingCovariance(mars::NearestCovMethod::zero), 1e-15));
  EXPECT_TRUE(cov_pos_eig_.isApprox(cov_pos_eig.EigenCorrectionUsingCovariance(mars::NearestCovMethod::delta), 1e-15));

  mars::NearestCov cov_neg_eig(cov_neg_eig_);

  EXPECT_TRUE(cov_neg_eig_corrected_abs_.isApprox(
      cov_neg_eig.EigenCorrectionUsingCovariance(mars::NearestCovMethod::abs), 1e-14));
  EXPECT_TRUE(cov_neg_eig_corrected_zero_.isApprox(
      cov_neg_eig.EigenCorrectionUsingCovariance(mars::NearestCovMethod::zero), 1e-14));
}
