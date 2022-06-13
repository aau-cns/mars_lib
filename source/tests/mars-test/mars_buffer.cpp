// Copyright (C) 2021 Christian Brommer and Martin Scheiber, Control of Networked Systems, University of Klagenfurt,
// Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>
// and <martin.scheiber@ieee.org>

#include <gmock/gmock.h>
#include <mars/buffer.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars/sensors/pose/pose_measurement_type.h>
#include <mars/sensors/pose/pose_sensor_class.h>
#include <mars/sensors/position/position_measurement_type.h>
#include <mars/sensors/position/position_sensor_class.h>
#include <mars/time.h>
#include <mars/type_definitions/buffer_entry_type.h>

#include <Eigen/Dense>

class mars_buffer_test : public testing::Test
{
public:
};

///
/// \brief Test that the CTOR arguments are handled correct
///
TEST_F(mars_buffer_test, CTOR)
{
  // Test buffer size setting by class init argument
  mars::Buffer buffer_ctor_1(100);
  ASSERT_EQ(buffer_ctor_1.get_max_buffer_size(), 100);
  mars::Buffer buffer_ctor_2(-1 * 100);
  ASSERT_EQ(buffer_ctor_2.get_max_buffer_size(), 100);

  // Test setter of max buffer size
  mars::Buffer buffer_setter_1(100);
  buffer_setter_1.set_max_buffer_size(200);
  ASSERT_EQ(buffer_setter_1.get_max_buffer_size(), 200);
  buffer_setter_1.set_max_buffer_size(-1 * 200);
  ASSERT_EQ(buffer_setter_1.get_max_buffer_size(), 200);
}

///
/// \brief Ensure that all entry getter return false on an empty buffer
///
TEST_F(mars_buffer_test, GETTER_EMPTY_BUFFER_RETURN)
{
  const int max_buffer_size = 100;
  mars::Buffer buffer(max_buffer_size);

  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose", core_states_sptr);

  try
  {
    ASSERT_EQ(buffer.IsEmpty(), 1);
  }
  catch (const std::exception& e)
  {
    std::cout << e.what();
    exit(EXIT_FAILURE);
  }

  // Empty buffer return tests
  mars::BufferEntryType latest_state;
  int latest_state_return = buffer.get_latest_state(&latest_state);
  ASSERT_EQ(latest_state_return, 0);

  mars::BufferEntryType oldest_state;
  int oldest_state_return = buffer.get_oldest_state(&oldest_state);
  ASSERT_EQ(oldest_state_return, 0);

  mars::BufferEntryType latest_entry;
  int latest_entry_return = buffer.get_latest_entry(&latest_entry);
  ASSERT_EQ(latest_entry_return, 0);

  mars::BufferEntryType latest_init_state;
  int latest_init_state_return = buffer.get_latest_init_state(&latest_init_state);
  ASSERT_EQ(latest_init_state_return, 0);

  mars::BufferEntryType latest_sensor_handle_state;
  int latest_sensor_handle_state_return =
      buffer.get_latest_sensor_handle_state(pose_sensor_sptr, &latest_sensor_handle_state);
  ASSERT_EQ(latest_sensor_handle_state_return, 0);

  // With index argument
  int latest_sensor_state_idx;
  buffer.get_latest_sensor_handle_state(pose_sensor_sptr, &latest_sensor_handle_state, &latest_sensor_state_idx);
  ASSERT_EQ(latest_sensor_state_idx, -1);

  mars::BufferEntryType latest_sensor_handle_measurement;
  int latest_sensor_handle_measurement_return =
      buffer.get_latest_sensor_handle_measurement(pose_sensor_sptr, &latest_sensor_handle_measurement);
  ASSERT_EQ(latest_sensor_handle_measurement_return, 0);

  mars::BufferEntryType closest_state;
  mars::Time timestamp(1);
  int closest_state_return = buffer.get_closest_state(timestamp, &closest_state);
  ASSERT_EQ(closest_state_return, 0);

  // With index argument
  int closest_state_index;
  closest_state_return = buffer.get_closest_state(timestamp, &closest_state, &closest_state_index);
  ASSERT_EQ(closest_state_index, -1);

  mars::BufferEntryType entry_at_idx;
  int index = 1;
  int entry_at_idx_return = buffer.get_entry_at_idx(index, &entry_at_idx);
  ASSERT_EQ(entry_at_idx_return, 0);
}

///
/// \brief Test that old entrys are removed if max_buffer_size is reached
///
TEST_F(mars_buffer_test, STORAGE_MAX_ENTRY)
{
  // Force removal of overflowing buffer entrys
  const int num_test_entrys = 20;
  const int max_buffer_size = 10;
  mars::Buffer buffer(max_buffer_size);

  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_1_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_1", core_states_sptr);

  mars::Time timestamp(0);
  int core_dummy = 13;
  int sensor_dummy = 15;
  mars::BufferDataType data(std::make_shared<int>(core_dummy), std::make_shared<int>(sensor_dummy));

  for (int k = num_test_entrys; k > 0; k--)
  {
    mars::BufferEntryType entry(timestamp + mars::Time(1), data, pose_sensor_1_sptr, 1);
    buffer.AddEntrySorted(entry);
  }

  std::cout << "Buffer Length: " << buffer.get_length() << std::endl;
  buffer.PrintBufferEntrys();

  ASSERT_EQ(buffer.get_length(), max_buffer_size);
}

TEST_F(mars_buffer_test, LATEST_ENTRY)
{
  const int num_test_entrys = 10;
  const int max_buffer_size = 100;
  mars::Buffer buffer(max_buffer_size);

  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_1_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_1", core_states_sptr);
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_2_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_2", core_states_sptr);

  mars::Time current_timestamp(0);
  int core_dummy = 13;
  int sensor_dummy = 15;

  for (int k = num_test_entrys; k > 0; k--)
  {
    mars::BufferDataType data(std::make_shared<int>(core_dummy), std::make_shared<int>(sensor_dummy));

    current_timestamp = current_timestamp + mars::Time(1);

    if (k % 2 == 0)
    {
      mars::BufferEntryType entry(current_timestamp, data, pose_sensor_1_sptr, 1);
      buffer.AddEntrySorted(entry);
    }
    else
    {
      mars::BufferEntryType entry(current_timestamp, data, pose_sensor_2_sptr, 4);
      buffer.AddEntrySorted(entry);
    }
  }

  std::cout << buffer.get_length() << std::endl;
  buffer.PrintBufferEntrys();

  mars::BufferEntryType latest_entry;
  buffer.get_latest_entry(&latest_entry);

  std::cout << "Picked:\n" << latest_entry << std::endl;
  EXPECT_EQ(latest_entry.timestamp_, current_timestamp);
}

TEST_F(mars_buffer_test, OLDEST_LATEST_STATE_RETURN)
{
  const int num_test_entrys = 10;
  const int max_buffer_size = 100;
  mars::Buffer buffer(max_buffer_size);

  // Empty buffer return tests
  mars::BufferEntryType latest_state_empty;
  int latest_emtpy_state_return = buffer.get_latest_state(&latest_state_empty);
  ASSERT_EQ(latest_emtpy_state_return, 0);

  mars::BufferEntryType oldest_state_empty;
  int oldest_empty_state_return = buffer.get_oldest_state(&oldest_state_empty);
  ASSERT_EQ(oldest_empty_state_return, 0);

  // Filled buffer return tests
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_1_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_1", core_states_sptr);
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_2_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_2", core_states_sptr);

  mars::Time timestamp(0);
  int core_dummy = 13;
  int sensor_dummy = 15;

  for (int k = num_test_entrys; k > 0; k--)
  {
    mars::BufferDataType data(std::make_shared<int>(core_dummy), std::make_shared<int>(sensor_dummy));

    if (k % 2 == 0)
    {
      mars::BufferEntryType entry(timestamp + mars::Time(1), data, pose_sensor_1_sptr,
                                  mars::BufferMetadataType::sensor_state);
      buffer.AddEntrySorted(entry);
    }
    else
    {
      mars::BufferEntryType entry(timestamp + mars::Time(1), data, pose_sensor_2_sptr,
                                  mars::BufferMetadataType::measurement);
      buffer.AddEntrySorted(entry);
    }
  }

  std::cout << buffer.get_length() << std::endl;
  buffer.PrintBufferEntrys();

  mars::BufferEntryType latest_state_full;
  int latest_filled_state_return = buffer.get_latest_state(&latest_state_full);
  ASSERT_EQ(latest_filled_state_return, 1);
  ASSERT_EQ(latest_state_full.metadata_, mars::BufferMetadataType::sensor_state);
  std::cout << "Picked:\n" << latest_state_full << std::endl;

  mars::BufferEntryType oldest_state_full;
  int oldest_filled_state_return = buffer.get_oldest_state(&oldest_state_full);
  ASSERT_EQ(oldest_filled_state_return, 1);
  ASSERT_EQ(oldest_state_full.metadata_, mars::BufferMetadataType::sensor_state);
  std::cout << "Picked:\n" << oldest_state_full << std::endl;
}

///
/// \brief Test that resetting the buffer removes all entrys
///
TEST_F(mars_buffer_test, RESET_BUFFER)
{
  const int num_test_entrys = 100;
  const int max_buffer_size = 110;
  mars::Buffer buffer(max_buffer_size);

  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_1_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_1", core_states_sptr);
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_2_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_2", core_states_sptr);

  mars::Time timestamp(0);
  int core_dummy = 13;
  int sensor_dummy = 15;

  for (int k = num_test_entrys; k > 0; k--)
  {
    mars::BufferDataType data(std::make_shared<int>(core_dummy), std::make_shared<int>(sensor_dummy));

    if (k % 2 == 0)
    {
      mars::BufferEntryType entry(timestamp + mars::Time(1), data, pose_sensor_1_sptr, 1);
      buffer.AddEntrySorted(entry);
    }
    else
    {
      mars::BufferEntryType entry(timestamp + mars::Time(1), data, pose_sensor_2_sptr, 1);
      buffer.AddEntrySorted(entry);
    }
  }

  std::cout << buffer.get_length() << std::endl;
  buffer.PrintBufferEntrys();
  ASSERT_EQ(buffer.get_length(), num_test_entrys);
  ASSERT_EQ(buffer.IsEmpty(), 0);

  buffer.ResetBufferData();
  std::cout << buffer.get_length() << std::endl;

  // Check if buffer is reset
  ASSERT_EQ(buffer.get_length(), 0);
  ASSERT_EQ(buffer.IsEmpty(), 1);
}

TEST_F(mars_buffer_test, GET_ENTRY_METHODS)
{
  mars::Buffer buffer;

  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_1_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_1", core_states_sptr);
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_2_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_2", core_states_sptr);

  int core_dummy = 13;
  int sensor_dummy = 15;
  mars::BufferDataType data(std::make_shared<int>(core_dummy), std::make_shared<int>(sensor_dummy));

  buffer.AddEntrySorted(mars::BufferEntryType(0, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(1, data, pose_sensor_1_sptr, mars::BufferMetadataType::init_state));
  buffer.AddEntrySorted(mars::BufferEntryType(2, data, pose_sensor_2_sptr, mars::BufferMetadataType::init_state));
  buffer.AddEntrySorted(mars::BufferEntryType(3, data, pose_sensor_1_sptr, mars::BufferMetadataType::core_state));
  buffer.AddEntrySorted(mars::BufferEntryType(4, data, pose_sensor_2_sptr, mars::BufferMetadataType::core_state));
  buffer.AddEntrySorted(mars::BufferEntryType(5, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(6, data, pose_sensor_2_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(7, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(8, data, pose_sensor_2_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(9, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement_ooo));
  buffer.AddEntrySorted(mars::BufferEntryType(10, data, pose_sensor_2_sptr, mars::BufferMetadataType::measurement_ooo));

  mars::BufferEntryType latest_entry_return;
  buffer.get_latest_entry(&latest_entry_return);
  ASSERT_EQ(latest_entry_return.timestamp_, 10);

  mars::BufferEntryType oldest_state_return;
  buffer.get_oldest_state(&oldest_state_return);
  ASSERT_EQ(oldest_state_return.timestamp_, 1);

  mars::BufferEntryType oldest_core_state_return;
  buffer.get_oldest_core_state(&oldest_core_state_return);
  ASSERT_EQ(oldest_core_state_return.timestamp_, 3);

  mars::BufferEntryType latest_init_state_return;
  buffer.get_latest_init_state(&latest_init_state_return);
  ASSERT_EQ(latest_init_state_return.timestamp_, 2);

  mars::BufferEntryType latest_state_return;
  buffer.get_latest_state(&latest_state_return);
  ASSERT_EQ(latest_state_return.timestamp_, 6);

  mars::BufferEntryType latest_sensor1_handle_state_return;
  buffer.get_latest_sensor_handle_state(pose_sensor_1_sptr, &latest_sensor1_handle_state_return);
  ASSERT_EQ(latest_sensor1_handle_state_return.timestamp_, 5);

  int latest_sensor1_handle_state_index;
  buffer.get_latest_sensor_handle_state(pose_sensor_1_sptr, &latest_sensor1_handle_state_return,
                                        &latest_sensor1_handle_state_index);
  ASSERT_EQ(latest_sensor1_handle_state_index, 5);

  mars::BufferEntryType latest_sensor2_handle_state_return;
  buffer.get_latest_sensor_handle_state(pose_sensor_2_sptr, &latest_sensor2_handle_state_return);
  ASSERT_EQ(latest_sensor2_handle_state_return.timestamp_, 6);

  int latest_sensor2_handle_state_index;
  buffer.get_latest_sensor_handle_state(pose_sensor_2_sptr, &latest_sensor2_handle_state_return,
                                        &latest_sensor2_handle_state_index);
  ASSERT_EQ(latest_sensor2_handle_state_index, 6);

  mars::BufferEntryType latest_sensor1_handle_measurement_return;
  buffer.get_latest_sensor_handle_measurement(pose_sensor_1_sptr, &latest_sensor1_handle_measurement_return);
  ASSERT_EQ(latest_sensor1_handle_measurement_return.timestamp_, 9);

  mars::BufferEntryType latest_sensor2_handle_measurement_return;
  buffer.get_latest_sensor_handle_measurement(pose_sensor_2_sptr, &latest_sensor2_handle_measurement_return);
  ASSERT_EQ(latest_sensor2_handle_measurement_return.timestamp_, 10);
}

TEST_F(mars_buffer_test, GET_CLOSEST_STATE)
{
  mars::Buffer buffer;

  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_1_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_1", core_states_sptr);

  int core_dummy = 13;
  int sensor_dummy = 15;
  mars::BufferDataType data(std::make_shared<int>(core_dummy), std::make_shared<int>(sensor_dummy));

  buffer.AddEntrySorted(mars::BufferEntryType(0, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(1, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(2, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(3, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));

  // Buffer is not empty but has no state, needs to return false

  mars::BufferEntryType no_state_entry;
  int no_state_return = buffer.get_closest_state(2, &no_state_entry);
  ASSERT_EQ(no_state_return, 0);

  // With index
  int closest_state_idx_no_state;
  buffer.get_closest_state(2, &no_state_entry, &closest_state_idx_no_state);
  ASSERT_EQ(closest_state_idx_no_state, -1);

  buffer.AddEntrySorted(mars::BufferEntryType(4, data, pose_sensor_1_sptr, mars::BufferMetadataType::init_state));
  buffer.AddEntrySorted(mars::BufferEntryType(5, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(6, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(7, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(8, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(9, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));

  // Equal timestamp
  mars::Time search_timestamp(6);
  mars::BufferEntryType equal_timestamp_return;
  buffer.get_closest_state(search_timestamp, &equal_timestamp_return);
  ASSERT_EQ(equal_timestamp_return.timestamp_, search_timestamp);

  // Equal time distance - needs to return the newer state
  mars::Time search_timestamp_2(8);
  mars::BufferEntryType equal_timestamp_distance_return;
  buffer.get_closest_state(search_timestamp_2, &equal_timestamp_distance_return);
  ASSERT_EQ(equal_timestamp_distance_return.timestamp_, 9);

  // Timestamp closer to older state
  mars::Time search_timestamp_3(6.1);
  mars::BufferEntryType close_to_older_state_return;
  buffer.get_closest_state(search_timestamp_3, &close_to_older_state_return);
  ASSERT_EQ(close_to_older_state_return.timestamp_, 6);

  // Timestamp closer to newer state
  mars::Time search_timestamp_4(6.9);
  mars::BufferEntryType close_to_newer_state_return;
  buffer.get_closest_state(search_timestamp_4, &close_to_newer_state_return);
  ASSERT_EQ(close_to_newer_state_return.timestamp_, 7);

  // Timestamp newer than newer state
  mars::Time search_timestamp_5(10);
  mars::BufferEntryType newer_than_newer_state_return;
  buffer.get_closest_state(search_timestamp_5, &newer_than_newer_state_return);
  ASSERT_EQ(newer_than_newer_state_return.timestamp_, 9);

  // Test if correct entry index is returned
  mars::Time index_timestamp(6);
  int closest_state_idx;
  mars::BufferEntryType dummy_return;
  buffer.get_closest_state(index_timestamp, &dummy_return, &closest_state_idx);
  ASSERT_EQ(closest_state_idx, 6);
}

TEST_F(mars_buffer_test, GET_CLOSEST_STATE_ONLY_ONE_STATE_IN_BUFFER)
{
  mars::Buffer buffer;
  mars::BufferDataType data;
  mars::Time timestamp(0.0);

  // setup propagation sensor
  std::shared_ptr<mars::ImuSensorClass> imu_sensor_sptr = std::make_shared<mars::ImuSensorClass>("IMU");

  mars::BufferEntryType meas_entry(timestamp, data, imu_sensor_sptr, mars::BufferMetadataType::measurement);
  buffer.AddEntrySorted(meas_entry);

  mars::BufferEntryType state_entry(timestamp, data, imu_sensor_sptr, mars::BufferMetadataType::core_state);
  buffer.AddEntrySorted(state_entry);

  mars::BufferEntryType result;
  bool status = buffer.get_closest_state(timestamp, &result);

  EXPECT_TRUE(status);
}

TEST_F(mars_buffer_test, GET_ENTRY_AT_INDEX)
{
  const int max_buffer_size = 10;
  mars::Buffer buffer(max_buffer_size);

  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_1_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_1", core_states_sptr);

  int core_dummy = 13;
  int sensor_dummy = 15;
  mars::BufferDataType data(std::make_shared<int>(core_dummy), std::make_shared<int>(sensor_dummy));

  buffer.AddEntrySorted(mars::BufferEntryType(0, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(1, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(2, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(3, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));

  // test return entry value
  mars::BufferEntryType entry_return;

  for (int k = 0; k < 4; k++)
  {
    buffer.get_entry_at_idx(k, &entry_return);
    ASSERT_EQ(entry_return.timestamp_, k);
  }

  // test return success status

  ASSERT_EQ(buffer.get_entry_at_idx(0, &entry_return), 1);
  ASSERT_EQ(buffer.get_entry_at_idx(3, &entry_return), 1);

  ASSERT_EQ(buffer.get_entry_at_idx(-1, &entry_return), 0);
  ASSERT_EQ(buffer.get_entry_at_idx(4, &entry_return), 0);
}

TEST_F(mars_buffer_test, ADD_SORTED)
{
  const int max_buffer_size = 50;
  mars::Buffer buffer(max_buffer_size);

  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_1_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_1", core_states_sptr);

  int core_dummy = 13;
  int sensor_dummy = 15;
  mars::BufferDataType data(std::make_shared<int>(core_dummy), std::make_shared<int>(sensor_dummy));

  buffer.AddEntrySorted(mars::BufferEntryType(1, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(0, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(3.2, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(4, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(2, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(6, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(5, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));

  buffer.PrintBufferEntrys();
  ASSERT_EQ(buffer.IsSorted(), 1);
  ASSERT_EQ(buffer.get_length(), 7);
}

TEST_F(mars_buffer_test, REMOVE_STATES_STARTING_AT_IDX)
{
  const int max_buffer_size = 10;
  mars::Buffer buffer(max_buffer_size);

  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_1_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_1", core_states_sptr);

  int core_dummy = 13;
  int sensor_dummy = 15;
  mars::BufferDataType data(std::make_shared<int>(core_dummy), std::make_shared<int>(sensor_dummy));

  buffer.AddEntrySorted(mars::BufferEntryType(0, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(1, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(2, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(3, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(4, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(5, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(6, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(7, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(8, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(9, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));

  buffer.PrintBufferEntrys();
  buffer.DeleteStatesStartingAtIdx(4);
  buffer.PrintBufferEntrys();

  ASSERT_EQ(buffer.get_length(), 6);
}

TEST_F(mars_buffer_test, MULTI_SENSOR_TYPE_SETUP)
{
  const int max_buffer_size = 100;
  mars::Buffer buffer(max_buffer_size);

  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_1_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_1", core_states_sptr);
  std::shared_ptr<mars::PositionSensorClass> position_sensor_1_sptr =
      std::make_shared<mars::PositionSensorClass>("Position_1", core_states_sptr);

  int core_dummy = 13;
  int sensor_dummy = 15;
  mars::BufferDataType data(std::make_shared<int>(core_dummy), std::make_shared<int>(sensor_dummy));

  buffer.AddEntrySorted(mars::BufferEntryType(1, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(1, data, pose_sensor_1_sptr, mars::BufferMetadataType::init_state));
  buffer.AddEntrySorted(mars::BufferEntryType(2, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(2, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));

  buffer.AddEntrySorted(mars::BufferEntryType(3, data, position_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(3, data, position_sensor_1_sptr, mars::BufferMetadataType::init_state));
  buffer.AddEntrySorted(mars::BufferEntryType(4, data, position_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(4, data, position_sensor_1_sptr, mars::BufferMetadataType::sensor_state));

  buffer.AddEntrySorted(mars::BufferEntryType(5, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(5, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));

  buffer.AddEntrySorted(mars::BufferEntryType(6, data, position_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(6, data, position_sensor_1_sptr, mars::BufferMetadataType::sensor_state));

  buffer.AddEntrySorted(mars::BufferEntryType(7, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(7, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));

  buffer.PrintBufferEntrys();
  ASSERT_EQ(buffer.IsSorted(), 1);
  ASSERT_EQ(buffer.get_length(), 14);

  // If the new timestamp matches with an existing one, then the new entry is added after the existing one
  mars::BufferEntryType entry_return_0;
  buffer.get_entry_at_idx(0, &entry_return_0);
  ASSERT_EQ(entry_return_0.metadata_, mars::BufferMetadataType::measurement);

  mars::BufferEntryType entry_return_1;
  buffer.get_entry_at_idx(1, &entry_return_1);
  ASSERT_EQ(entry_return_1.metadata_, mars::BufferMetadataType::init_state);
}

TEST_F(mars_buffer_test, INSERT_DATA_IDX_TEST)
{
  mars::Buffer buffer;

  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_1_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_1", core_states_sptr);

  int core_dummy = 13;
  int sensor_dummy = 15;
  mars::BufferDataType data(std::make_shared<int>(core_dummy), std::make_shared<int>(sensor_dummy));

  buffer.AddEntrySorted(mars::BufferEntryType(4, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(5, data, pose_sensor_1_sptr, mars::BufferMetadataType::init_state));
  buffer.AddEntrySorted(mars::BufferEntryType(6, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(7, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));

  // New entry is newer than newest existing entry and needs to be inserted at idx 4
  int idx_newer = buffer.InsertDataAtTimestamp(
      mars::BufferEntryType(8, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  ASSERT_EQ(4, idx_newer);

  // add entry in the middle of the buffer
  int idx_mid = buffer.InsertDataAtTimestamp(
      mars::BufferEntryType(5.3, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  ASSERT_EQ(2, idx_mid);

  idx_mid = buffer.InsertDataAtTimestamp(
      mars::BufferEntryType(5.6, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  ASSERT_EQ(3, idx_mid);

  // New entry is older than oldest existing entry and needs to be inserted at idx 0
  int idx_older = buffer.InsertDataAtTimestamp(
      mars::BufferEntryType(1, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  ASSERT_EQ(int(0), idx_older);
}

TEST_F(mars_buffer_test, INSERT_DATA_IDX_MAX_BUFFER_TEST)
{
  const int buffer_size = 10;
  mars::Buffer buffer(buffer_size);

  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_1_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_1", core_states_sptr);

  int core_dummy = 13;
  int sensor_dummy = 15;
  mars::BufferDataType data(std::make_shared<int>(core_dummy), std::make_shared<int>(sensor_dummy));

  // Fill the buffer until max_buffer_size and compare the returned index
  for (int k = 0; k < buffer_size; k++)
  {
    int idx = buffer.AddEntrySorted(
        mars::BufferEntryType(k, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
    ASSERT_EQ(k, idx);
  }

  // Add further entrys at the beginning of the buffer to trigger the deletion of old states and ensure that the correct
  // index is returned
  for (int k = 0; k < 5; k++)
  {
    int idx = buffer.AddEntrySorted(
        mars::BufferEntryType(buffer_size, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
    ASSERT_EQ(buffer_size - 1, idx);
  }
}

TEST_F(mars_buffer_test, CHECK_LAST_SENSOR_HANDLE)
{
  mars::Buffer buffer(10);
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_1_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_1", core_states_sptr);
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_2_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_2", core_states_sptr);

  mars::BufferDataType data;
  mars::Time timestamp(1);

  mars::BufferEntryType entry(timestamp, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state);
  buffer.AddEntrySorted(entry);
  timestamp = timestamp + mars::Time(1);

  EXPECT_TRUE(buffer.CheckForLastSensorHandle(pose_sensor_1_sptr));

  // Add an entry with another sensor instance
  mars::BufferEntryType entry2(timestamp, data, pose_sensor_2_sptr, mars::BufferMetadataType::sensor_state);
  buffer.AddEntrySorted(entry2);
  timestamp = timestamp + mars::Time(1);

  EXPECT_TRUE(buffer.CheckForLastSensorHandle(pose_sensor_1_sptr));

  // Add an entry for sensor 1
  mars::BufferEntryType entry3(timestamp, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state);
  buffer.AddEntrySorted(entry3);

  // Two entrys of sensor 1 exist
  EXPECT_FALSE(buffer.CheckForLastSensorHandle(pose_sensor_1_sptr));
}

///
/// \brief Tests if getting all measurements from a single sensor from buffer works.
///
/// \author Martin Scheiber <martin.scheiber@ieee.org>
///
TEST_F(mars_buffer_test, GET_SENSOR_MEASUREMENTS)
{
  const int max_buffer_size = 20;
  mars::Buffer buffer(max_buffer_size);

  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_1_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_1", core_states_sptr);
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_2_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_2", core_states_sptr);
  std::shared_ptr<mars::PositionSensorClass> position_sensor_1_sptr =
      std::make_shared<mars::PositionSensorClass>("Position", core_states_sptr);
  std::shared_ptr<mars::ImuSensorClass> imu_sensor_sptr = std::make_shared<mars::ImuSensorClass>("IMU");

  int core_dummy = 13;
  int sensor_dummy = 15;
  mars::BufferDataType data(std::make_shared<int>(core_dummy), std::make_shared<int>(sensor_dummy));

  mars::PoseMeasurementType meas_pose1(Eigen::Vector3d(1, 1, 1), Eigen::Quaterniond::Identity());
  std::shared_ptr<mars::PoseMeasurementType> meas_pose1_ptr = std::make_shared<mars::PoseMeasurementType>(meas_pose1);
  mars::BufferDataType data_pose1;
  data_pose1.set_sensor_data(meas_pose1_ptr);
  mars::PoseMeasurementType meas_pose2(Eigen::Vector3d(5, 4, 3), Eigen::Quaterniond::Identity());
  std::shared_ptr<mars::PoseMeasurementType> meas_pose2_ptr = std::make_shared<mars::PoseMeasurementType>(meas_pose2);
  mars::BufferDataType data_pose2;
  data_pose2.set_sensor_data(meas_pose2_ptr);

  buffer.AddEntrySorted(mars::BufferEntryType(4, data, pose_sensor_1_sptr, mars::BufferMetadataType::init_state));
  buffer.AddEntrySorted(
      mars::BufferEntryType(5, data_pose1, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(6, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(7, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(
      mars::BufferEntryType(8, data_pose1, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(9, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));

  buffer.AddEntrySorted(mars::BufferEntryType(11, data, pose_sensor_2_sptr, mars::BufferMetadataType::init_state));
  buffer.AddEntrySorted(
      mars::BufferEntryType(9, data_pose2, pose_sensor_2_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(8, data, pose_sensor_2_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(7, data, pose_sensor_2_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(
      mars::BufferEntryType(5, data_pose2, pose_sensor_2_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(3, data, pose_sensor_2_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(
      mars::BufferEntryType(1, data_pose2, pose_sensor_2_sptr, mars::BufferMetadataType::measurement));

  buffer.AddEntrySorted(mars::BufferEntryType(3, data, position_sensor_1_sptr, mars::BufferMetadataType::init_state));
  buffer.AddEntrySorted(mars::BufferEntryType(4, data, position_sensor_1_sptr, mars::BufferMetadataType::sensor_state));

  // test return measurements size1
  std::vector<const mars::BufferEntryType*> entries_return;
  buffer.get_sensor_handle_measurements(pose_sensor_1_sptr, entries_return);

  ASSERT_EQ(entries_return.size(), 2);

  // iterate over buffer and test elements1
  int ts = 5;
  for (const auto& it : entries_return)
  {
    mars::PoseMeasurementType meas = *static_cast<mars::PoseMeasurementType*>(it->data_.sensor_.get());
    std::cout << "pose1_meas: " << meas.position_.transpose() << std::endl;
    // value
    ASSERT_EQ((meas.position_ - Eigen::Vector3d(1, 1, 1)).norm(), 0);
    // timestamp
    ASSERT_EQ(it->timestamp_, ts);
    ts += 3;
    // check if pointer is corresponding to correct one
    ASSERT_EQ(it->data_.sensor_, meas_pose1_ptr);
  }

  // test return measurements size2
  buffer.get_sensor_handle_measurements(pose_sensor_2_sptr, entries_return);

  ASSERT_EQ(entries_return.size(), 3);

  // iterate over buffer and test elements2
  ts = 1;
  for (const auto& it : entries_return)
  {
    mars::PoseMeasurementType meas = *static_cast<mars::PoseMeasurementType*>(it->data_.sensor_.get());
    std::cout << "pose2_meas: " << meas.position_.transpose() << std::endl;
    ASSERT_EQ((meas.position_ - Eigen::Vector3d(5, 4, 3)).norm(), 0);
    // timestamp
    ASSERT_EQ(it->timestamp_, ts);
    ts += 4;
    // check if pointer is corresponding to correct one
    ASSERT_EQ(it->data_.sensor_, meas_pose2_ptr);
  }

  // change value one item, retreive values again, should still be the same
  mars::PoseMeasurementType meas = *static_cast<mars::PoseMeasurementType*>(entries_return.at(1)->data_.sensor_.get());
  meas.position_ = Eigen::Vector3d(1, 1, 1);
  ASSERT_EQ((meas.position_ - Eigen::Vector3d(1, 1, 1)).norm(), 0);

  // get all entries again and rerun above test case
  buffer.get_sensor_handle_measurements(pose_sensor_2_sptr, entries_return);
  ASSERT_EQ(entries_return.size(), 3);
  ts = 1;
  for (const auto& it : entries_return)
  {
    mars::PoseMeasurementType meas = *static_cast<mars::PoseMeasurementType*>(it->data_.sensor_.get());
    std::cout << "pose2_meas: " << meas.position_.transpose() << std::endl;
    ASSERT_EQ((meas.position_ - Eigen::Vector3d(5, 4, 3)).norm(), 0);
    ASSERT_EQ(it->timestamp_, ts);
    ts += 4;
  }
  // if this succeeds then the entries are unchangable in the buffer (as they should be)!

  // test return measurements
  buffer.get_sensor_handle_measurements(position_sensor_1_sptr, entries_return);

  ASSERT_EQ(entries_return.size(), 0);
  ASSERT_TRUE(entries_return.empty());

  // test return success status
  ASSERT_EQ(buffer.get_sensor_handle_measurements(pose_sensor_1_sptr, entries_return), 1);
  ASSERT_EQ(buffer.get_sensor_handle_measurements(pose_sensor_2_sptr, entries_return), 1);
  ASSERT_EQ(buffer.get_sensor_handle_measurements(position_sensor_1_sptr, entries_return), 0);
  ASSERT_EQ(buffer.get_sensor_handle_measurements(imu_sensor_sptr, entries_return), 0);
}

TEST_F(mars_buffer_test, REMOVE_OVERFLOW_ENTRIES)
{
  const int max_buffer_size = 5;
  mars::Buffer buffer(max_buffer_size);
  buffer.set_keep_last_sensor_handle(true);

  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_1_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_1", core_states_sptr);
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_2_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_2", core_states_sptr);

  int core_dummy = 13;
  int sensor_dummy = 15;
  mars::BufferDataType data(std::make_shared<int>(core_dummy), std::make_shared<int>(sensor_dummy));

  buffer.AddEntrySorted(mars::BufferEntryType(0, data, pose_sensor_2_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(1, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(1, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(2, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(2, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.PrintBufferEntrys();

  // Trigger overflow removal
  buffer.AddEntrySorted(mars::BufferEntryType(4, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.PrintBufferEntrys();

  // Check that last state entry is still pose sensor 2
  mars::BufferEntryType last_state;
  buffer.get_oldest_state(&last_state);

  ASSERT_EQ(pose_sensor_2_sptr, last_state.sensor_);
}

TEST_F(mars_buffer_test, ADD_AUTOREMOVE_ENTRIES)
{
  const int max_buffer_size = 5;
  mars::Buffer buffer(max_buffer_size);
  buffer.set_keep_last_sensor_handle(true);

  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_1_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_1", core_states_sptr);
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_2_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_2", core_states_sptr);

  int core_dummy = 13;
  int sensor_dummy = 15;
  mars::BufferDataType data(std::make_shared<int>(core_dummy), std::make_shared<int>(sensor_dummy));

  buffer.AddEntrySorted(mars::BufferEntryType(1, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(3, data, pose_sensor_2_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(3, data, pose_sensor_2_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(4, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(4, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));

  buffer.AddEntrySorted(mars::BufferEntryType(2, data, pose_sensor_1_sptr, mars::BufferMetadataType::measurement));

  ASSERT_EQ(buffer.get_length(), 5);

  mars::BufferEntryType oldest_entry_return;
  buffer.get_oldest_state(&oldest_entry_return);
  ASSERT_EQ(oldest_entry_return.timestamp_, 1);
}

///
/// \brief Tests if buffer returning indices works for adding entries
///
/// \author Martin Scheiber <martin.scheiber@ieee.org>
///
TEST_F(mars_buffer_test, ADD_INDEX_TEST)
{
  const int max_buffer_size = 5;
  mars::Buffer buffer(max_buffer_size);
  buffer.set_keep_last_sensor_handle(true);

  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_1_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_1", core_states_sptr);
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_2_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_2", core_states_sptr);
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_3_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_3", core_states_sptr);

  int core_dummy = 13;
  int sensor_dummy = 15;
  mars::BufferDataType data(std::make_shared<int>(core_dummy), std::make_shared<int>(sensor_dummy));

  buffer.AddEntrySorted(mars::BufferEntryType(0, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(3, data, pose_sensor_2_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(3, data, pose_sensor_2_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(4, data, pose_sensor_2_sptr, mars::BufferMetadataType::measurement));
  buffer.AddEntrySorted(mars::BufferEntryType(4, data, pose_sensor_2_sptr, mars::BufferMetadataType::sensor_state));

  int idx =
      buffer.AddEntrySorted(mars::BufferEntryType(2, data, pose_sensor_3_sptr, mars::BufferMetadataType::measurement));

  ASSERT_EQ(buffer.get_length(), 5);

  mars::BufferEntryType entry_01;
  buffer.get_entry_at_idx(1, &entry_01);

  ASSERT_EQ(entry_01.timestamp_, 2);
  ASSERT_EQ(idx, 1);
}

///
/// \brief Tests if given more sensors than buffer size, the buffer will still keep at least one state per sensor
/// if it is the last.
///
/// This requires the buffer to 'grow' larger than its allowed size, which is a desired functionality
///
/// \author Martin Scheiber <martin.scheiber@ieee.org>
///
TEST_F(mars_buffer_test, SIZE_TEST)
{
  const int max_buffer_size = 2;
  mars::Buffer buffer(max_buffer_size);
  buffer.set_keep_last_sensor_handle(true);

  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_1_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_1", core_states_sptr);
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_2_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_2", core_states_sptr);
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_3_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose_3", core_states_sptr);

  int core_dummy = 13;
  int sensor_dummy = 15;
  mars::BufferDataType data(std::make_shared<int>(core_dummy), std::make_shared<int>(sensor_dummy));

  buffer.AddEntrySorted(mars::BufferEntryType(0, data, pose_sensor_1_sptr, mars::BufferMetadataType::sensor_state));
  buffer.AddEntrySorted(mars::BufferEntryType(3, data, pose_sensor_2_sptr, mars::BufferMetadataType::sensor_state));

  buffer.AddEntrySorted(mars::BufferEntryType(2, data, pose_sensor_3_sptr, mars::BufferMetadataType::sensor_state));

  ASSERT_EQ(buffer.get_length(), 3);
  buffer.PrintBufferEntrys();

  // Check that last state entry is still pose sensor 2
  mars::BufferEntryType last_state;
  buffer.get_oldest_state(&last_state);

  ASSERT_EQ(pose_sensor_1_sptr, last_state.sensor_);
}

TEST_F(mars_buffer_test, INSERT_DATA_AT_IDX)
{
  // TODO
}
