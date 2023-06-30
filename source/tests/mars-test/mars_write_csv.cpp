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
#include <mars/data_utils/write_csv.h>

class mars_write_csv_test : public testing::Test
{
public:
};

TEST_F(mars_write_csv_test, WRITE_COV_TO_CSV)
{
  // Even Elements with Fixed Size Matrix - Cov String
  // Full Covariance A
  //    1, 2, 3, 4,
  //    5, 6, 7, 8,
  //    9, 10, 11, 12,
  //    13, 14, 15, 16
  Eigen::Matrix4d cov_a;
  cov_a << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;

  std::string a_str_result(mars::WriteCsv::cov_mat_to_csv(cov_a));
  std::string a_str_expect(", 1, 5, 9, 13, 6, 10, 14, 11, 15, 16");

  EXPECT_EQ(a_str_result, a_str_expect);

  // Uneven Elements - Cov String
  // Full Covariance B
  //    1,  2,  3,  4,  5,  6,  7,
  //    2,  8,  9, 10, 11, 12, 13,
  //    3,  9, 14, 15, 16, 17, 18,
  //    4, 10, 15, 19, 20, 21, 22,
  //    5, 11, 16, 20, 23, 24, 25,
  //    6, 12, 17, 21, 24, 26, 27,
  //    7, 13, 18, 22, 25, 27, 28,
  Eigen::Matrix<double, 7, 7> cov_b;
  cov_b << 1, 2, 3, 4, 5, 6, 7, 2, 8, 9, 10, 11, 12, 13, 3, 9, 14, 15, 16, 17, 18, 4, 10, 15, 19, 20, 21, 22, 5, 11, 16,
      20, 23, 24, 25, 6, 12, 17, 21, 24, 26, 27, 7, 13, 18, 22, 25, 27, 28;

  std::string b_str_result(mars::WriteCsv::cov_mat_to_csv(cov_b));
  std::string b_str_expect(", 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, "
                           "25, 26, 27, 28");
  EXPECT_EQ(b_str_result, b_str_expect);

  // Check Non-Square Rejection - no output
  Eigen::Matrix<double, 4, 3> cov_non_sqr(Eigen::Matrix<double, 4, 3>::Random());

  std::string non_sqr_str_result(mars::WriteCsv::cov_mat_to_csv(cov_non_sqr));
  std::string non_sqr_str_expect("");
  EXPECT_EQ(non_sqr_str_result, non_sqr_str_expect);
}

TEST_F(mars_write_csv_test, WRITE_COV_HEADER_TO_CSV)
{
  // Test 4 state cov header string
  //      11,12,13,14
  //        ,22,23,24
  //           ,33,34
  //              ,44
  std::string cov4_str_result(mars::WriteCsv::get_cov_header_string(4));
  std::string cov4_str_expect(", p_1_1, p_1_2, p_1_3, p_1_4, p_2_2, p_2_3, p_2_4, p_3_3, p_3_4, p_4_4");
  EXPECT_EQ(cov4_str_result, cov4_str_expect);

  // Test 6 state cov header string
  //      11,12,13,14,15,16
  //        ,22,23,24,25,26
  //           ,33,34,35,36
  //              ,44,45,46
  //                 ,55,56
  //                    ,66
  std::string cov6_str_result(mars::WriteCsv::get_cov_header_string(6));
  std::string cov6_str_expect(", p_1_1, p_1_2, p_1_3, p_1_4, p_1_5, p_1_6, p_2_2, p_2_3, p_2_4, p_2_5, p_2_6, p_3_3, "
                              "p_3_4, p_3_5, "
                              "p_3_6, p_4_4, p_4_5, p_4_6, p_5_5, p_5_6, p_6_6");
  EXPECT_EQ(cov6_str_result, cov6_str_expect);
}
