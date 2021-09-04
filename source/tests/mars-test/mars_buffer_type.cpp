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
#include <mars/core_state.h>
#include <mars/sensors/imu/imu_measurement_type.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars/sensors/pose/pose_sensor_class.h>
#include <mars/type_definitions/base_states.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <Eigen/Dense>

class mars_buffer_type_test : public testing::Test
{
public:
};

TEST_F(mars_buffer_type_test, CTOR)
{
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose", core_states_sptr);
  int core_dummy = 13;
  int sensor_dummy = 15;
  mars::BufferDataType data(std::make_shared<int>(core_dummy), std::make_shared<int>(sensor_dummy));

  mars::BufferEntryType entry(1, data, pose_sensor_sptr, 1);

  // copy constructor
  mars::BufferEntryType copy_of_entry(entry);
}

TEST_F(mars_buffer_type_test, TIME_COMPARISON)
{
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose", core_states_sptr);

  int core_dummy = 13;
  int sensor_dummy = 15;
  mars::BufferDataType data(std::make_shared<int>(core_dummy), std::make_shared<int>(sensor_dummy));

  int timestamp_1 = 1;
  int timestamp_2 = 2;

  mars::BufferEntryType entry_1(timestamp_1, data, pose_sensor_sptr, 1);
  mars::BufferEntryType entry_2(timestamp_2, data, pose_sensor_sptr, 1);

  ASSERT_GT(entry_2, entry_1);
  ASSERT_LT(entry_1, entry_2);
}

TEST_F(mars_buffer_type_test, DATA_STORAGE)
{
  std::shared_ptr<mars::ImuSensorClass> imu_sensor_sptr = std::make_shared<mars::ImuSensorClass>("IMU");
  mars::BufferEntryType buffer_entry;

  mars::IMUMeasurementType imu_meas(Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(4, 5, 6));

  // Intentionally limit the scope
  // the unique pointer ensures deletion when leaving the scope
  {
    std::unique_ptr<mars::BufferDataType> data(new mars::BufferDataType);
    data->set_sensor_data(std::make_shared<mars::IMUMeasurementType>(imu_meas));
    buffer_entry = mars::BufferEntryType(1, *data, imu_sensor_sptr, mars::BufferMetadataType::measurement);
  }

  mars::IMUMeasurementType imu_meas_return = *static_cast<mars::IMUMeasurementType*>(buffer_entry.data_.sensor_.get());

  ASSERT_EQ(imu_meas, imu_meas_return);
  ASSERT_EQ(imu_sensor_sptr.get(), buffer_entry.sensor_.get());
}
