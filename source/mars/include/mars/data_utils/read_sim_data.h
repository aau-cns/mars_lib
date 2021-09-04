// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef READ_SIM_DATA_H
#define READ_SIM_DATA_H

#include <mars/data_utils/read_csv.h>
#include <mars/sensors/imu/imu_measurement_type.h>
#include <mars/time.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <mars/type_definitions/core_state_type.h>
#include <Eigen/Dense>
#include <vector>

namespace mars
{
class ReadSimData
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReadSimData(std::vector<BufferEntryType>* data_out, std::shared_ptr<SensorAbsClass> sensor,
              const std::string& file_path)
  {
    constexpr int expected_columns = 23;
    CsvDataType sim_data;
    ReadCsv(&sim_data, file_path, expected_columns);

    unsigned long number_of_datapoints = sim_data.size();

    CoreStateType core_ground_truth;

    data_out->resize(number_of_datapoints);

    unsigned long current_index = 0;
    for (auto k : sim_data)
    {
      Time time = k[0];

      Eigen::Vector3d w_imu(std::vector<double>(k.begin() + 1, k.begin() + 3 + 1).data());
      Eigen::Vector3d a_imu(std::vector<double>(k.begin() + 4, k.begin() + 6 + 1).data());
      Eigen::Vector3d p(std::vector<double>(k.begin() + 7, k.begin() + 9 + 1).data());
      Eigen::Vector3d v(std::vector<double>(k.begin() + 10, k.begin() + 12 + 1).data());
      Eigen::Quaterniond q(std::vector<double>(k.begin() + 13, k.begin() + 16 + 1).data());
      Eigen::Vector3d bGyr(std::vector<double>(k.begin() + 17, k.begin() + 19 + 1).data());
      Eigen::Vector3d bAcc(std::vector<double>(k.begin() + 20, k.begin() + 22 + 1).data());

      CoreStateType core_ground_truth;
      core_ground_truth.p_wi_ = p;
      core_ground_truth.q_wi_ = q;
      core_ground_truth.v_wi_ = v;
      core_ground_truth.b_w_ = bGyr;
      core_ground_truth.b_a_ = bAcc;

      BufferDataType data;
      data.set_core_data(std::make_shared<CoreStateType>(core_ground_truth));
      data.set_sensor_data(std::make_shared<IMUMeasurementType>(a_imu, w_imu));

      BufferEntryType current_entry(time, data, sensor, BufferMetadataType::measurement);
      data_out->at(current_index) = current_entry;

      ++current_index;
    }
  }
};
}

#endif  // READ_SIM_DATA_H
