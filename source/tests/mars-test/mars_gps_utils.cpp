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
#include <mars/sensors/gps/gps_sensor_class.h>
#include <mars/sensors/gps/gps_utils.h>
#include <mars/sensors/pose/pose_sensor_class.h>
#include <Eigen/Dense>

class mars_gps_utils_test : public testing::Test
{
public:
};

TEST_F(mars_gps_utils_test, INIT_RESET)
{
  mars::GPSInit gps_init;
  EXPECT_FALSE(gps_init.IsDone());

  // set done
  gps_init.set_done();
  EXPECT_TRUE(gps_init.IsDone());

  // reset
  gps_init.Reset();
  EXPECT_FALSE(gps_init.IsDone());
}

TEST_F(mars_gps_utils_test, AVG_CALC)
{
  // create mag init
  mars::GPSInit gps_init(4);
  mars::GPSInit gps_init2(2);

  // create buffer, sensor class and measurements
  const int max_buffer_size = 6;
  mars::Buffer buffer(max_buffer_size);

  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose1_sensor_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_1", core_states_sptr);
  std::shared_ptr<mars::PoseSensorClass> gps1_sensor_sptr =
      std::make_shared<mars::PoseSensorClass>("GPS", core_states_sptr);

  // dummy data for pose sensor
  int core_dummy = 13;
  int sensor_dummy = 15;
  mars::BufferDataType data_dummy(std::make_shared<int>(core_dummy), std::make_shared<int>(sensor_dummy));

  // gps data
  mars::GpsMeasurementType meas_gps1(1.0, 2.5, 3.0);
  mars::GpsMeasurementType meas_gps2(-5.0, 4.0, 3.0);
  mars::GpsMeasurementType meas_gps3(3.0, 5.5, 3.0);
  mars::BufferDataType data_gps1, data_gps2, data_gps3;
  data_gps1.set_sensor_data(std::make_shared<mars::GpsMeasurementType>(meas_gps1));
  data_gps2.set_sensor_data(std::make_shared<mars::GpsMeasurementType>(meas_gps2));
  data_gps3.set_sensor_data(std::make_shared<mars::GpsMeasurementType>(meas_gps3));

  // add measurements to buffer
  buffer.AddEntrySorted(mars::BufferEntryType(1, data_dummy, pose1_sensor_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(2, data_dummy, pose1_sensor_sptr, mars::BufferMetadataType::measurement));

  // perform update without measurements in buffer
  mars::GpsCoordinates gps_mean = gps_init.get_gps_mean(gps1_sensor_sptr, buffer, meas_gps1.coordinates_, 1);
  EXPECT_EQ(gps_mean.longitude_, meas_gps1.coordinates_.longitude_);
  EXPECT_EQ(gps_mean.latitude_, meas_gps1.coordinates_.latitude_);
  EXPECT_EQ(gps_mean.altitude_, meas_gps1.coordinates_.altitude_);
  EXPECT_FALSE(gps_init.IsDone());  // as no entries in buffer

  // reset for next init
  gps_init.Reset();

  // continue to fill buffer
  buffer.AddEntrySorted(mars::BufferEntryType(1, data_gps1, gps1_sensor_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(3, data_gps2, gps1_sensor_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(4, data_dummy, pose1_sensor_sptr, mars::BufferMetadataType::measurement));

  // test mean again
  gps_mean = gps_init.get_gps_mean(gps1_sensor_sptr, buffer, meas_gps3.coordinates_, 5);
  EXPECT_EQ(gps_mean.latitude_, (1.0 - 5.0 + 3.0) / 3.0);
  EXPECT_EQ(gps_mean.longitude_, (2.5 + 4.0 + 5.5) / 3.0);
  EXPECT_EQ(gps_mean.altitude_, 3.0);
  EXPECT_TRUE(gps_init.IsDone());

  // test different duration
  gps_mean = gps_init2.get_gps_mean(gps1_sensor_sptr, buffer, meas_gps3.coordinates_, 5);
  EXPECT_EQ(gps_mean.latitude_, (-5.0 + 3.0) / 2.0);
  EXPECT_EQ(gps_mean.longitude_, (4.0 + 5.5) / 2.0);
  EXPECT_EQ(gps_mean.altitude_, 3.0);
  EXPECT_TRUE(gps_init2.IsDone());
}
