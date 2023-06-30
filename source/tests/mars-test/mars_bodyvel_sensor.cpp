// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@ieee.org>

#include <gmock/gmock.h>
#include <mars/sensors/bodyvel/bodyvel_measurement_type.h>
#include <mars/sensors/bodyvel/bodyvel_sensor_class.h>
#include <mars/type_definitions/base_states.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <Eigen/Dense>

class mars_bodyvel_sensor_test : public testing::Test
{
public:
};

TEST_F(mars_bodyvel_sensor_test, CTOR_BODYVEL_SENSOR)
{
  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  mars::BodyvelSensorClass bodyvel_sensor("Bodyvel", core_states_sptr);
}

TEST_F(mars_bodyvel_sensor_test, BODYVEL_SENSOR_MEASUREMENT)
{
  Eigen::Vector3d velocity;  // Velocity [x y z]

  velocity << 1, 2, 3;
  mars::BodyvelMeasurementType b(velocity);

  Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
  std::cout << b.velocity_.format(OctaveFmt) << std::endl;
}

TEST_F(mars_bodyvel_sensor_test, BODYVEL_SENSOR_INIT)
{
  Eigen::Vector3d velocity;  // Velocity [x y z]

  velocity << 1, 2, 3;
  mars::BodyvelMeasurementType measurement(velocity);

  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  mars::BodyvelSensorClass bodyvel_sensor("Bodyvel", core_states_sptr);

  EXPECT_DEATH(bodyvel_sensor.Initialize(1, std::make_shared<mars::BodyvelMeasurementType>(measurement),
                                         std::make_shared<mars::CoreType>()),
               "");
}

TEST_F(mars_bodyvel_sensor_test, BODYVEL_UPDATE)
{
  Eigen::Vector3d velocity;  // Velocity [x y z]

  velocity << 1, 2, 3;
  mars::BodyvelMeasurementType b(velocity);

  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  mars::BodyvelSensorClass bodyvel_sensor("Bodyvel", core_states_sptr);

  mars::CoreStateType prior_core_state;
  mars::BufferDataType prior_sensor_buffer_data;
  Eigen::Matrix<double, prior_core_state.size_error_ + 3, prior_core_state.size_error_ + 3> prior_cov;
  prior_cov.setIdentity();

  // TODO no update without init
  //  int timestamp = 1;
  //  mars::BufferDataType test =
  //      position_sensor.CalcUpdate(timestamp, std::make_shared<mars::BodyvelMeasurementType>(measurement),
  //                                 prior_core_state, prior_sensor_buffer_data.sensor_, prior_cov);
}
