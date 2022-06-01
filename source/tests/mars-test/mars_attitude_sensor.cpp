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
#include <mars/sensors/attitude/attitude_measurement_type.h>
#include <mars/sensors/attitude/attitude_sensor_class.h>
#include <mars/sensors/attitude/attitude_sensor_state_type.h>
#include <mars/type_definitions/base_states.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>

#include <eigen3/Eigen/Dense>

class mars_attitude_sensor_test : public testing::Test
{
public:
};

TEST_F(mars_attitude_sensor_test, CTOR_ATTITUDE_SENSOR)
{
  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  mars::AttitudeSensorClass attitude_sensor("Attitude", core_states_sptr);
}

TEST_F(mars_attitude_sensor_test, ATTITUDE_SENSOR_MEASUREMENT)
{
  Eigen::Vector2d rp_vector;  // Attitude [roll pitch]
  Eigen::Matrix3d rot_mat;    // Attitude [rotation matrix]
                              //  Eigen::Quaternion<double> orientation;  // Quaternion [W X Y Z]

  //  rp_vector << M_PI, M_PI_2;
  //  mars::AttitudeMeasurementType measurement(rp_vector);
  rot_mat = Eigen::Matrix3d::Identity();
  mars::AttitudeMeasurementType measurement(rot_mat);

  Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
  //  std::cout << measurement.rp_vector_.format(OctaveFmt) << std::endl;
  std::cout << measurement.attitude_.get_rp().format(OctaveFmt) << std::endl;
}

TEST_F(mars_attitude_sensor_test, ATTITUDE_UPDATE)
{
  Eigen::Vector2d rp_vector;  // Attitude [roll pitch]
  Eigen::Matrix3d rot_mat;    // Attitude [rotation matrix]
                              //  Eigen::Quaternion<double> orientation;  // Quaternion [W X Y Z]

  //  rp_vector << M_PI, M_PI_2;
  //  mars::AttitudeMeasurementType measurement(rp_vector);
  rot_mat = Eigen::Matrix3d::Identity();
  mars::AttitudeMeasurementType measurement(rot_mat);

  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  mars::AttitudeSensorClass attitude_sensor("Attitude", core_states_sptr);

  mars::CoreStateType prior_core_state;
  mars::BufferDataType prior_sensor_buffer_data;
  Eigen::Matrix<double, prior_core_state.size_error_ + 2, prior_core_state.size_error_ + 2> prior_cov;
  prior_cov.setIdentity();

  // TODO no update without init
  //  int timestamp = 1;
  //  mars::BufferDataType test =
  //      pose_sensor.CalcUpdate(timestamp, std::make_shared<mars::PoseMeasurementType>(measurement), prior_core_state,
  //                             prior_sensor_buffer_data.sensor_, prior_cov);
}
