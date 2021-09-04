// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef READ_POSE_DATA_H
#define READ_POSE_DATA_H

#include <mars/data_utils/read_csv.h>
#include <mars/sensors/pose/pose_measurement_type.h>
#include <mars/time.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <Eigen/Dense>
#include <vector>

namespace mars
{
class ReadPoseData
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReadPoseData(std::vector<BufferEntryType>* data_out, std::shared_ptr<SensorAbsClass> sensor,
               const std::string& file_path)
  {
    constexpr int expected_columns = 8;
    CsvDataType sim_data;
    ReadCsv(&sim_data, file_path, expected_columns);

    unsigned long number_of_datapoints = sim_data.size();

    CoreStateType core_ground_truth;

    data_out->resize(number_of_datapoints);

    unsigned long current_index = 0;
    for (auto k : sim_data)
    {
      Time time = k[0] + 1e-13;

      Eigen::Vector3d position(k[1], k[2], k[3]);
      Eigen::Quaterniond orientation(k[4], k[5], k[6], k[7]);

      BufferDataType data;
      data.set_sensor_data(std::make_shared<PoseMeasurementType>(position, orientation));

      BufferEntryType current_entry(time, data, sensor, BufferMetadataType::measurement);
      data_out->at(current_index) = current_entry;

      ++current_index;
    }
  }
};
}

#endif  // READ_POSE_DATA_H
