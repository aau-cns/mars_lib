// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef READ_GPS_W_VEL_DATA_H
#define READ_GPS_W_VEL_DATA_H

#include <mars/data_utils/read_csv.h>
#include <mars/sensors/gps_w_vel/gps_w_vel_measurement_type.h>
#include <mars/time.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <Eigen/Dense>
#include <vector>

namespace mars
{
class ReadGpsWithVelData
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReadGpsWithVelData(std::vector<BufferEntryType>* data_out, std::shared_ptr<SensorAbsClass> sensor,
                     const std::string& file_path, const double& time_offset)
  {
    constexpr int expected_columns = 7;
    CsvDataType sim_data;
    ReadCsv(&sim_data, file_path, expected_columns);

    unsigned long number_of_datapoints = sim_data.size();

    data_out->resize(number_of_datapoints);

    unsigned long current_index = 0;
    for (auto k : sim_data)
    {
      Time time = k[0] + time_offset;

      BufferDataType data;
      data.set_sensor_data(std::make_shared<GpsVelMeasurementType>(k[1], k[2], k[3], k[4], k[5], k[6]));

      BufferEntryType current_entry(time, data, sensor, BufferMetadataType::measurement);
      data_out->at(current_index) = current_entry;

      ++current_index;
    }
  }

  ReadGpsWithVelData(std::vector<BufferEntryType>* data_out, std::shared_ptr<SensorAbsClass> sensor,
                     const std::string& file_path)
  {
    ReadGpsWithVelData(data_out, sensor, file_path, 0);
  }
};
}  // namespace mars

#endif  // READ_GPS_W_VEL_DATA_H
