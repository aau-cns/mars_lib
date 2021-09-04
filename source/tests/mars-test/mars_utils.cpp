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
#include <mars/general_functions/utils.h>
#include <Eigen/Dense>

class mars_utils_test : public testing::Test
{
public:
  Eigen::Matrix4d non_sym_mat, sym_mat, negative_eigen_mat, negative_det;

  mars_utils_test()
  {
    non_sym_mat << 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1;

    sym_mat = (non_sym_mat + non_sym_mat.transpose()) / 2;

    negative_eigen_mat << 2, -1, -4, 9, -1, 3, 3, 5, -4, 3, 1, -4, 9, 5, -4, 3;

    negative_det << 2, -1, -4, 9, -1, 3, 3, -10, -4, 3, 10, -4, 9, -10, -4, 3;
  }
};

TEST_F(mars_utils_test, QUAT_FROM_SMALL_ANGLE)
{
  Eigen::Vector3d small_angles(0.1, 0.1, 0.1);
  Eigen::Quaterniond result = mars::Utils::QuatFromSmallAngle(small_angles);
  Eigen::Quaterniond correct_result(0.996242942258564, 0.0500, 0.0500, 0.0500);

  EXPECT_TRUE(result.coeffs().isApprox(correct_result.coeffs()));
}

TEST_F(mars_utils_test, APPLY_SMALL_ANGLE_QUAT_CORR)
{
  Eigen::Vector3d small_angles(0.1, 0.1, 0.1);
  Eigen::Quaterniond prior_quaternion(0.105409255338946, 0.210818510677892, 0.632455532033676, 0.737864787372622);
  Eigen::Quaterniond correct_result(0.025956285175946, 0.210026453360312, 0.661702136682618, 0.719281198460249);
  Eigen::Quaterniond result = mars::Utils::ApplySmallAngleQuatCorr(prior_quaternion, small_angles);

  EXPECT_TRUE(result.coeffs().isApprox(correct_result.coeffs()));
}
TEST_F(mars_utils_test, GENERAL_FUNCTIONS)
{
  Eigen::Matrix3d skew_result;
  Eigen::Matrix4d omega_mat_result;
  Eigen::Vector3d test_vector;

  test_vector << 1, 2, 3;
  skew_result << 0, -3, 2, 3, 0, -1, -2, 1, 0;
  omega_mat_result << 0, -1, -2, -3, 1, 0, 3, -2, 2, -3, 0, 1, 3, 2, -1, 0;

  EXPECT_EQ(mars::Utils::Skew(test_vector), skew_result);
  EXPECT_EQ(mars::Utils::OmegaMat(test_vector), omega_mat_result);
}

TEST_F(mars_utils_test, MATRIX_EXPONENTIAL)
{
  int order = 5;
  Eigen::Matrix4d mat_exp_result;
  Eigen::Matrix4d test_mat;

  test_mat << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;
  mat_exp_result << 39700.6666666667, 44956.6666666667, 50213.6666666667, 55470.6666666667, 91726.0666666667,
      103872.600000000, 116017.133333333, 128162.666666667, 143752.466666667, 162786.533333333, 181821.600000000,
      200854.666666667, 195778.866666667, 221701.466666667, 247624.066666667, 273547.666666667;
  EXPECT_TRUE(mars::Utils::MatExp(test_mat, order).isApprox(mat_exp_result));
}

TEST_F(mars_utils_test, COV_CHECK)
{
  // Symmetry
  EXPECT_TRUE(mars::Utils::CheckCov(sym_mat, "Symmetric"));
  EXPECT_FALSE(mars::Utils::CheckCov(non_sym_mat, "Non Symmetric"));
  EXPECT_FALSE(mars::Utils::CheckCov(negative_eigen_mat, "Non-Positive Semidefinite"));
  EXPECT_FALSE(mars::Utils::CheckCov(negative_det, "Negative Determinant"));
}

TEST_F(mars_utils_test, ENFORCE_SYMMETRY)
{
  Eigen::Matrix4d non_symmetric, result_symmetric;
  non_symmetric << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;
  result_symmetric << 1, 3.5, 6, 8.5, 3.5, 6, 8.5, 11, 6, 8.5, 11, 13.5, 8.5, 11, 13.5, 16;

  mars::Utils::EnforceMatrixSymmetry(Eigen::Matrix3d::Random());
  ASSERT_TRUE(result_symmetric.isApprox(mars::Utils::EnforceMatrixSymmetry(non_symmetric)));
}
