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
#include <mars/sensors/pose/pose_measurement_type.h>
#include <mars/sensors/pose/pose_sensor_class.h>
#include <mars/sensors/pose/pose_sensor_state_type.h>
#include <mars/type_definitions/base_states.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <Eigen/Dense>

class mars_pose_sensor_test : public testing::Test
{
public:
};

TEST_F(mars_pose_sensor_test, CTOR_POSE_SENSOR)
{
  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  mars::PoseSensorClass pose_sensor("Pose", core_states_sptr);
}

TEST_F(mars_pose_sensor_test, POSE_SENSOR_MEASUREMENT)
{
  Eigen::Vector3d position;               // Position [x y z]
  Eigen::Quaternion<double> orientation;  // Quaternion [W X Y Z]

  position << 1, 2, 3;
  orientation = Eigen::Quaternion<double>(1, 0, 0, 0);
  mars::PoseMeasurementType measurement(position, orientation);

  Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
  std::cout << measurement.position_.format(OctaveFmt) << std::endl;
  std::cout << measurement.orientation_.coeffs().format(OctaveFmt) << std::endl;
}

TEST_F(mars_pose_sensor_test, POSE_SENSOR_INIT)
{
  Eigen::Vector3d position;               // Position [x y z]
  Eigen::Quaternion<double> orientation;  // Quaternion [W X Y Z]

  position << 1, 2, 3;
  orientation = Eigen::Quaternion<double>(1, 0, 0, 0);
  mars::PoseMeasurementType measurement(position, orientation);

  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  mars::PoseSensorClass pose_sensor("Pose", core_states_sptr);

  EXPECT_DEATH(pose_sensor.Initialize(1, std::make_shared<mars::PoseMeasurementType>(measurement),
                                      std::make_shared<mars::CoreType>()),
               "");
}

TEST_F(mars_pose_sensor_test, POSE_UPDATE)
{
  Eigen::Vector3d position;               // Position [x y z]
  Eigen::Quaternion<double> orientation;  // Quaternion [W X Y Z]

  position << 1, 2, 3;
  orientation = Eigen::Quaternion<double>(1, 0, 0, 0);
  mars::PoseMeasurementType measurement(position, orientation);

  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  mars::PoseSensorClass pose_sensor("Pose", core_states_sptr);

  mars::CoreStateType prior_core_state;
  mars::BufferDataType prior_sensor_buffer_data;
  Eigen::Matrix<double, prior_core_state.size_error_ + 6, prior_core_state.size_error_ + 6> prior_cov;
  prior_cov.setIdentity();

  // TODO no update without init
  //  int timestamp = 1;
  //  mars::BufferDataType test =
  //      pose_sensor.CalcUpdate(timestamp, std::make_shared<mars::PoseMeasurementType>(measurement), prior_core_state,
  //                             prior_sensor_buffer_data.sensor_, prior_cov);
}
