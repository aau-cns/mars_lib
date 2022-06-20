// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef READ_IMU_DATA_H
#define READ_IMU_DATA_H

#include <mars/data_utils/read_csv.h>
#include <mars/sensors/imu/imu_measurement_type.h>
#include <mars/time.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <Eigen/Dense>
#include <vector>

namespace mars
{
class ReadImuData
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReadImuData(std::vector<BufferEntryType>* data_out, std::shared_ptr<SensorAbsClass> sensor,
              const std::string& file_path, const double& time_offset)
  {
    constexpr int expected_columns = 7;
    CsvDataType imu_data;
    ReadCsv(&imu_data, file_path, expected_columns);

    unsigned long number_of_datapoints = imu_data.size();

    CoreStateType core_ground_truth;
    data_out->resize(number_of_datapoints);

    unsigned long current_index = 0;
    for (auto k : imu_data)
    {
      Time time = k[0] + time_offset;
      Eigen::Vector3d linear_acceleration(k[1], k[2], k[3]);
      Eigen::Vector3d angular_velocity(k[4], k[5], k[6]);

      BufferDataType data;
      data.set_sensor_data(std::make_shared<IMUMeasurementType>(linear_acceleration, angular_velocity));

      BufferEntryType current_entry(time, data, sensor, BufferMetadataType::measurement);
      data_out->at(current_index) = current_entry;

      ++current_index;
    }
  }

  ReadImuData(std::vector<BufferEntryType>* data_out, std::shared_ptr<SensorAbsClass> sensor,
              const std::string& file_path)
  {
    ReadImuData(data_out, sensor, file_path, 0);
  }
};
}

#endif  // READ_IMU_DATA_H
