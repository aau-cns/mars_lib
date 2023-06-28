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
#include <mars/sensors/gps/gps_conversion.h>
#include <Eigen/Dense>

class mars_gps_test : public testing::Test
{
public:
};

TEST_F(mars_gps_test, CTOR_GPS)
{
  mars::GpsCoordinates reference(46.614798, 14.2628073, 18);
  mars::GpsConversion gps_conversion(reference);

  // Test with three local test points
  mars::GpsCoordinates coordinates_1(46.613186, 14.2614695, 19);
  Eigen::Matrix<double, 3, 1> enu_gt_1(-102.4800036920105, -179.1949193682265, 0.9966573797104);

  mars::GpsCoordinates coordinates_2(46.624435, 14.306053, 17);
  Eigen::Matrix<double, 3, 1> enu_gt_2(3312.079520417935, 1072.193122997238, -1.948681026472);

  mars::GpsCoordinates coordinates_3(46.6181, 14.227668, 17);
  Eigen::Matrix<double, 3, 1> enu_gt_3(-2691.544711864501, 367.662214924263, -1.577514958162);

  EXPECT_TRUE(enu_gt_1.isApprox(gps_conversion.get_enu(coordinates_1), 1e-5));
  EXPECT_TRUE(enu_gt_2.isApprox(gps_conversion.get_enu(coordinates_2), 1e-5));
  EXPECT_TRUE(enu_gt_3.isApprox(gps_conversion.get_enu(coordinates_3), 1e-5));

  std::cout << coordinates_3 << std::endl;
}

TEST_F(mars_gps_test, COORDINATE_ADDITION)
{
  mars::GpsCoordinates coord1(1, 2, 3);
  mars::GpsCoordinates coord2(1, 2, 3);

  mars::GpsCoordinates sum1 = coord1 + coord2;
  mars::GpsCoordinates sum2 = coord2 + coord1;

  EXPECT_EQ(sum1.longitude_, sum2.longitude_);
  EXPECT_EQ(sum1.latitude_, sum2.latitude_);
  EXPECT_EQ(sum1.altitude_, sum2.altitude_);
  EXPECT_EQ(sum1.latitude_, 2);
  EXPECT_EQ(sum1.longitude_, 4);
  EXPECT_EQ(sum1.altitude_, 6);

  mars::GpsCoordinates sum3 = coord1;
  sum3 += coord2;

  EXPECT_EQ(sum1.longitude_, sum3.longitude_);
  EXPECT_EQ(sum1.latitude_, sum3.latitude_);
  EXPECT_EQ(sum1.altitude_, sum3.altitude_);
}
