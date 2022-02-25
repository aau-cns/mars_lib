// Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef READ_POSITION_DATA_H
#define READ_POSITION_DATA_H

#include <mars/data_utils/read_csv.h>
#include <mars/sensors/position/position_measurement_type.h>
#include <mars/time.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <Eigen/Dense>
#include <vector>

namespace mars
{
class ReadPositionData
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReadPositionData(std::vector<BufferEntryType>* data_out, std::shared_ptr<SensorAbsClass> sensor,
                   const std::string& file_path, const double& time_offset)
  {
    constexpr int expected_columns = 4;
    CsvDataType sim_data;
    ReadCsv(&sim_data, file_path, expected_columns);

    unsigned long number_of_datapoints = sim_data.size();

    data_out->resize(number_of_datapoints);

    unsigned long current_index = 0;
    for (auto k : sim_data)
    {
      Time time = k[0] + time_offset;

      Eigen::Vector3d position(k[1], k[2], k[3]);

      BufferDataType data;
      data.set_sensor_data(std::make_shared<PositionMeasurementType>(position));

      BufferEntryType current_entry(time, data, sensor, BufferMetadataType::measurement);
      data_out->at(current_index) = current_entry;

      ++current_index;
    }
  }

  ReadPositionData(std::vector<BufferEntryType>* data_out, std::shared_ptr<SensorAbsClass> sensor,
                   const std::string& file_path)
  {
    ReadPositionData(data_out, sensor, file_path, 0);
  }
};
}

#endif  // READ_POSITION_DATA_H
