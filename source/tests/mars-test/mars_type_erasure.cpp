// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef MARS_TYPE_ERASURE_CPP
#define MARS_TYPE_ERASURE_CPP

#include <gmock/gmock.h>
#include <mars/sensors/imu/imu_measurement_type.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <Eigen/Dense>
#include <memory>

class mars_type_erasure_test : public testing::Test
{
public:
};

TEST_F(mars_type_erasure_test, IMU_MEASUREMENT)
{
  Eigen::Vector3d linear_acceleration = { 1, 2, 3 };
  Eigen::Vector3d angular_velocity = { 4, 5, 6 };

  mars::IMUMeasurementType imu_measurement(linear_acceleration, angular_velocity);
  std::shared_ptr<void> test = std::make_shared<mars::IMUMeasurementType>(imu_measurement);
  mars::IMUMeasurementType resolved_void = *(static_cast<mars::IMUMeasurementType*>(test.get()));

  ASSERT_EQ(imu_measurement, resolved_void);
}

#endif  // MARS_TYPE_ERASURE_CPP
