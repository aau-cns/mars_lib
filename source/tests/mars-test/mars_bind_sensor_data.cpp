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
#include <mars/general_functions/utils.h>
#include <mars/sensors/pose/pose_sensor_class.h>
#include <mars/type_definitions/core_state_type.h>
#include <Eigen/Dense>

class mars_bind_sensor_data : public testing::Test
{
public:
};

TEST_F(mars_bind_sensor_data, CTOR)
{
  mars::PoseSensorData sensor_data;

  constexpr int core_state_size = mars::CoreStateType::size_error_;
  int sensor_state_size = sensor_data.state_.cov_size_;
  int full_state_size = core_state_size + sensor_state_size;

  // Check for correct dimensions
  EXPECT_EQ(sensor_data.get_full_cov().size(), full_state_size * full_state_size);

  // Check that all entrys are zero when no data was written
  EXPECT_EQ(sensor_data.get_full_cov(), Eigen::MatrixXd::Zero(full_state_size, full_state_size));

  // Check that the cov is returned correctly, and that the core cov is set to zero
  Eigen::MatrixXd full_cov(full_state_size, full_state_size);
  full_cov.setRandom();
  full_cov = mars::Utils::EnforceMatrixSymmetry(full_cov);

  Eigen::MatrixXd expected_result = full_cov;
  expected_result.block(0, 0, core_state_size, core_state_size) =
      Eigen::Matrix<double, core_state_size, core_state_size>::Zero();

  sensor_data.set_cov(full_cov);
  Eigen::MatrixXd full_cov_return = sensor_data.get_full_cov();

  Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
  std::cout << (full_cov_return - expected_result).format(OctaveFmt) << std::endl;

  EXPECT_EQ(expected_result, full_cov_return);
}
