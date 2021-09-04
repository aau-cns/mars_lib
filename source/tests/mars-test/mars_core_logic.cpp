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
#include <mars/buffer.h>
#include <mars/core_logic.h>
#include <mars/core_state.h>
#include <mars/sensors/imu/imu_measurement_type.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars/sensors/pose/pose_sensor_class.h>
#include <mars/sensors/sensor_abs_class.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/core_type.h>
#include <Eigen/Dense>
#include <memory>

class mars_core_logic_test : public testing::Test
{
public:
  int test(int a)
  {
    return a;
  }
};

TEST_F(mars_core_logic_test, CTOR)
{
  // Setup the core definition
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();

  // Create the CoreLogic and link the core states
  mars::CoreLogic core_logic(core_states_sptr);
}

TEST_F(mars_core_logic_test, INITIALIZE)
{
  Eigen::Vector3d p_wi_init(0, 0, 5);
  Eigen::Quaterniond q_wi_init = Eigen::Quaterniond::Identity();

  // Setup the core definition
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::ImuSensorClass> imu_sensor_sptr = std::make_shared<mars::ImuSensorClass>("IMU");
  core_states_sptr.get()->set_propagation_sensor(imu_sensor_sptr);

  // Create the CoreLogic and link the core states
  mars::CoreLogic core_logic(core_states_sptr);

  // Call on empty buffer
  ASSERT_FALSE(core_logic.Initialize(p_wi_init, q_wi_init));

  int timestamp = 0;
  mars::IMUMeasurementType imu_meas(Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(4, 5, 6));

  mars::BufferDataType data;
  data.set_sensor_data(std::make_shared<mars::IMUMeasurementType>(imu_meas));

  mars::BufferEntryType buffer_entry(timestamp, data, imu_sensor_sptr, mars::BufferMetadataType::measurement);

  core_logic.buffer_prior_core_init_.AddEntrySorted(buffer_entry);

  // Call on non-empty buffer

  ASSERT_TRUE(core_logic.Initialize(p_wi_init, q_wi_init));
}

TEST_F(mars_core_logic_test, GENERATE_STATE_TRANSITION_BLOCK)
{
  const int test_size = 10;
  const int first_test_idx = 3;
  const int last_test_idx = 7;

  // Setup the core definition
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  std::shared_ptr<mars::ImuSensorClass> imu_sensor_sptr = std::make_shared<mars::ImuSensorClass>("IMU");
  core_states_sptr.get()->set_propagation_sensor(imu_sensor_sptr);

  // Create the CoreLogic and link the core states
  mars::CoreLogic core_logic(core_states_sptr);

  // Generate test data
  std::vector<mars::CoreType> state_transition_test_data(test_size);
  for (int k = 0; k < test_size; k++)
  {
    // Generate test data
    state_transition_test_data[k].state_transition_ = mars::CoreStateMatrix::Random();
    double timestamp = k;
    // Generate buffer entrys
    mars::BufferDataType data;
    data.set_core_data(std::make_shared<mars::CoreType>(state_transition_test_data[k]));

    mars::BufferEntryType buffer_entry(timestamp, data, imu_sensor_sptr, mars::BufferMetadataType::core_state);
    core_logic.buffer_.AddEntrySorted(buffer_entry);
  }

  mars::CoreStateMatrix state_transition_test_result;
  state_transition_test_result = core_logic.GenerateStateTransitionBlock(first_test_idx, last_test_idx);

  mars::CoreStateMatrix state_transition_correct_result(mars::CoreStateMatrix::Identity());
  for (int k = first_test_idx; k <= last_test_idx; k++)
  {
    state_transition_correct_result = state_transition_test_data[k].state_transition_ * state_transition_correct_result;
  }

  ASSERT_TRUE(state_transition_correct_result.isApprox(state_transition_test_result, 1e-16));

  EXPECT_EQ(state_transition_correct_result, state_transition_test_result);

  // Test that negativ idx call an assert and terminate the program
  EXPECT_DEBUG_DEATH(core_logic.GenerateStateTransitionBlock(-1, 2), "");
  EXPECT_DEBUG_DEATH(core_logic.GenerateStateTransitionBlock(1, -2), "");
  EXPECT_DEBUG_DEATH(core_logic.GenerateStateTransitionBlock(-1, -2), "");
}

TEST_F(mars_core_logic_test, REWORK_BUFFER)
{
  // Setup the core definition
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();

  // Create the CoreLogic and link the core states
  mars::CoreLogic core_logic(core_states_sptr);

  // Test that negativ idx call an assert and terminate the program
  EXPECT_DEATH(core_logic.ReworkBufferStartingAtIndex(-1), "");
}

TEST_F(mars_core_logic_test, PERFORM_SENSOR_UPDATE)
{
  // Setup the core definition
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();

  // Create the CoreLogic and link the core states
  mars::CoreLogic core_logic(core_states_sptr);

  // Create a sensor instance
  std::shared_ptr<mars::PoseSensorClass> sensor_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose", core_states_sptr);

  // Prepare Dummy data
  int sensor_dummy = 15;
  mars::BufferDataType data(std::make_shared<mars::CoreType>(), std::make_shared<int>(sensor_dummy));
  std::shared_ptr<mars::BufferDataType> data_sptr = std::make_shared<mars::BufferDataType>(data);

  // Call sensor update with prepared objects
  int sensor_update_status;
  mars::BufferEntryType new_buffer_entry;
  sensor_update_status = core_logic.PerformSensorUpdate(&new_buffer_entry, sensor_sptr, 10, data_sptr);

  ASSERT_EQ(sensor_update_status, 0);
}

TEST_F(mars_core_logic_test, PROPAGATE_SENSOR_CROSS_COV)
{
  constexpr int sensor_cov_dim = 20;
  constexpr int core_cov_dim = mars::CoreStateType::size_error_;
  using SensorCovMatrix = Eigen::Matrix<double, core_cov_dim + sensor_cov_dim, core_cov_dim + sensor_cov_dim>;

  // Setup the core definition
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();

  // Create the CoreLogic and link the core states
  mars::CoreLogic core_logic(core_states_sptr);

  // Generate symmetryc sensor covariance
  SensorCovMatrix prior_sensor_covariance;
  prior_sensor_covariance.setRandom();
  prior_sensor_covariance = (prior_sensor_covariance * prior_sensor_covariance.transpose()) / 2;

  // Generate symmetryc core covariance
  mars::CoreStateMatrix core_cov;
  core_cov.setRandom();
  core_cov = (core_cov * core_cov.transpose()) / 2;

  mars::CoreStateMatrix state_transition(mars::CoreStateMatrix::Random());
  Eigen::MatrixXd cov_result;
  cov_result = core_logic.PropagateSensorCrossCov(prior_sensor_covariance, core_cov, state_transition);

  // Ensure that core elements remain the same
  ASSERT_TRUE(core_cov.isApprox(cov_result.block(0, 0, core_cov_dim, core_cov_dim), 1e-16));

  // Ensure that inter-sensor covariance remain the same
  Eigen::Matrix<double, sensor_cov_dim, sensor_cov_dim> sensor_cov_before, sensor_cov_after;
  sensor_cov_before = prior_sensor_covariance.block(core_cov_dim, core_cov_dim, sensor_cov_dim, sensor_cov_dim);
  sensor_cov_after = cov_result.block(core_cov_dim, core_cov_dim, sensor_cov_dim, sensor_cov_dim);
  ASSERT_TRUE(sensor_cov_before.isApprox(sensor_cov_after, 1e-16));

  // Ensure symmetry
  SensorCovMatrix zero_cov(SensorCovMatrix::Zero());
  ASSERT_TRUE(zero_cov.isApprox(cov_result - cov_result.transpose()));

  // Ensure correct propagation
  Eigen::Matrix<double, core_cov_dim, sensor_cov_dim> sensor_cross_cov_before, sensor_cross_cov_after;
  sensor_cross_cov_before = prior_sensor_covariance.block(0, core_cov_dim, core_cov_dim, sensor_cov_dim);
  sensor_cross_cov_after = cov_result.block(0, core_cov_dim, core_cov_dim, sensor_cov_dim);

  ASSERT_TRUE(sensor_cross_cov_after.isApprox(state_transition * sensor_cross_cov_before, 1e-16));
}
