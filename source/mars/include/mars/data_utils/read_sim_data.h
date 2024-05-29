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
    std::vector<std::string> expect_entry = {
      "t",   "a_x", "a_y", "a_z", "w_x", "w_y",  "w_z",  "p_x",  "p_y",  "p_z",  "v_x",  "v_y",
      "v_z", "q_w", "q_x", "q_y", "q_z", "ba_x", "ba_y", "ba_z", "bw_x", "bw_y", "bw_z",
    };

    CsvDataType csv_data;
    ReadCsv(&csv_data, file_path);

    unsigned long number_of_datapoints = csv_data["t"].size();
    data_out->resize(number_of_datapoints);

    CoreStateType core_ground_truth;

    for (size_t k = 0; k < number_of_datapoints; k++)
    {
      Time time = csv_data["t"][k];

      Eigen::Vector3d w_imu(csv_data["w_x"][k], csv_data["w_y"][k], csv_data["w_z"][k]);
      Eigen::Vector3d a_imu(csv_data["a_x"][k], csv_data["a_y"][k], csv_data["a_z"][k]);
      Eigen::Vector3d p(csv_data["p_x"][k], csv_data["p_y"][k], csv_data["p_z"][k]);
      Eigen::Vector3d v(csv_data["v_x"][k], csv_data["v_y"][k], csv_data["v_z"][k]);
      Eigen::Quaterniond q(csv_data["q_w"][k], csv_data["q_x"][k], csv_data["q_y"][k], csv_data["q_z"][k]);
      q = Utils::NormalizeQuaternion(q, "sim csv reader");

      Eigen::Vector3d bGyr(csv_data["bw_x"][k], csv_data["bw_y"][k], csv_data["bw_z"][k]);
      Eigen::Vector3d bAcc(csv_data["ba_x"][k], csv_data["ba_y"][k], csv_data["ba_z"][k]);

      CoreStateType core_ground_truth;
      core_ground_truth.p_wi_ = p;
      core_ground_truth.q_wi_ = q;
      core_ground_truth.v_wi_ = v;
      core_ground_truth.b_w_ = bGyr;
      core_ground_truth.b_a_ = bAcc;

      BufferDataType data;
      // TODO
      // data.set_core_state(std::make_shared<CoreStateType>(core_ground_truth));
      data.set_measurement(std::make_shared<IMUMeasurementType>(a_imu, w_imu));

      BufferEntryType current_entry(time, data, sensor);
      data_out->at(k) = current_entry;
    }
  }
};
}  // namespace mars

#endif  // READ_SIM_DATA_H
