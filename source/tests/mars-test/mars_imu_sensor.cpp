// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef MARS_IMU_SENSOR_CPP
#define MARS_IMU_SENSOR_CPP

#include <gmock/gmock.h>
#include <mars/sensors/imu/imu_measurement_type.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <Eigen/Dense>
#include <memory>

class mars_imu_sensor_test : public testing::Test
{
public:
};

TEST_F(mars_imu_sensor_test, CTOR_)
{
  Eigen::Vector3d linear_acceleration_1 = { 1, 2, 3 };
  Eigen::Vector3d angular_velocity_1 = { 4, 5, 6 };
  mars::IMUMeasurementType imu_measurement_1(linear_acceleration_1, angular_velocity_1);
  mars::IMUMeasurementType imu_measurement_2(linear_acceleration_1, angular_velocity_1);

  ASSERT_EQ(imu_measurement_1.linear_acceleration_, linear_acceleration_1);
  ASSERT_EQ(imu_measurement_1.angular_velocity_, angular_velocity_1);
  ASSERT_EQ(imu_measurement_1, imu_measurement_2);

  // Copy constructor
  mars::IMUMeasurementType imu_measurement_3(imu_measurement_1);
  ASSERT_EQ(imu_measurement_1, imu_measurement_3);
}

#endif  // MARS_IMU_SENSOR_CPP
