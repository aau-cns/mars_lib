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
#include <mars/sensors/position/position_measurement_type.h>
#include <mars/sensors/position/position_sensor_class.h>
#include <mars/type_definitions/base_states.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <Eigen/Dense>

class mars_position_sensor_test : public testing::Test
{
public:
};

TEST_F(mars_position_sensor_test, CTOR_POSITION_SENSOR)
{
  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  mars::PositionSensorClass position_sensor("Position", core_states_sptr);
}

TEST_F(mars_position_sensor_test, POSITION_SENSOR_MEASUREMENT)
{
  Eigen::Vector3d position;  // Position [x y z]

  position << 1, 2, 3;
  mars::PositionMeasurementType b(position);

  Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
  std::cout << b.position_.format(OctaveFmt) << std::endl;
}

TEST_F(mars_position_sensor_test, POSITION_SENSOR_INIT)
{
  Eigen::Vector3d position;  // Position [x y z]

  position << 1, 2, 3;
  mars::PositionMeasurementType measurement(position);

  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  mars::PositionSensorClass position_sensor("Position", core_states_sptr);

  EXPECT_DEATH(position_sensor.Initialize(1, std::make_shared<mars::PositionMeasurementType>(measurement),
                                          std::make_shared<mars::CoreType>()),
               "");
}

TEST_F(mars_position_sensor_test, POSITION_UPDATE)
{
  Eigen::Vector3d position;  // Position [x y z]

  position << 1, 2, 3;
  mars::PositionMeasurementType measurement(position);

  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  mars::PositionSensorClass position_sensor("Position", core_states_sptr);

  mars::CoreStateType prior_core_state;
  mars::BufferDataType prior_sensor_buffer_data;
  Eigen::Matrix<double, prior_core_state.size_error_ + 3, prior_core_state.size_error_ + 3> prior_cov;
  prior_cov.setIdentity();

  // TODO no update without init
  //  int timestamp = 1;
  //  mars::BufferDataType test =
  //      position_sensor.CalcUpdate(timestamp, std::make_shared<mars::PositionMeasurementType>(measurement),
  //                                 prior_core_state, prior_sensor_buffer_data.sensor_, prior_cov);
}
