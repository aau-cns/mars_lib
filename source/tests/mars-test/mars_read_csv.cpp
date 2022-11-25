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
#include <mars/data_utils/read_csv.h>
#include <mars/data_utils/read_pose_data.h>
#include <mars/data_utils/read_sim_data.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars/sensors/pose/pose_sensor_class.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>
#include <vector>
#include "../include_local/test_data_settings.h"

class mars_read_csv_test : public testing::Test
{
public:
};

TEST_F(mars_read_csv_test, READ_CSV)
{
  std::string test_data_path = std::string(MARS_LIB_TEST_DATA_PATH) + "/traj_test_dummy.csv";

  // TODO(CHB) Test needs to be re-written for the new CSV reader
  //  mars::CsvDataType data;
  //  mars::ReadCsv csvread(&data, test_data_path);

  //  // traj.csv has 23 columns, expecting 22 columns should lead to an exit
  //  EXPECT_DEATH(mars::ReadCsv csvread(&data, test_data_path), "");
}

TEST_F(mars_read_csv_test, READ_POSE_CSV)
{
  std::string test_data_path = std::string(MARS_LIB_TEST_DATA_PATH) + "/pose_test_dummy.csv";

  // create sensor
  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose", core_states_sptr);
  std::vector<mars::BufferEntryType> measurement_data;

  // read data
  EXPECT_NO_FATAL_FAILURE(mars::ReadPoseData(&measurement_data, pose_sensor_sptr, test_data_path));
}
