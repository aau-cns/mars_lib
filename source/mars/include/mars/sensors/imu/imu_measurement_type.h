// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef IMU_MEASUREMENT_TYPE_H
#define IMU_MEASUREMENT_TYPE_H
#include <Eigen/Dense>
#include <utility>

namespace mars
{
class IMUMeasurementType
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d linear_acceleration_{ 0, 0, 0 };
  Eigen::Vector3d angular_velocity_{ 0, 0, 0 };

  IMUMeasurementType() = default;

  IMUMeasurementType(Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity)
    : linear_acceleration_(std::move(linear_acceleration)), angular_velocity_(std::move(angular_velocity))
  {
  }

  bool operator==(const IMUMeasurementType& rhs) const
  {
    return (linear_acceleration_ == rhs.linear_acceleration_ && angular_velocity_ == rhs.angular_velocity_);
  }

  bool operator!=(const IMUMeasurementType& rhs) const
  {
    return !(*this == rhs);
  }

  static std::string get_csv_state_header_string()
  {
    std::stringstream os;
    os << "t, ";
    os << "a_x, a_y, a_z, ";
    os << "w_x, w_y, w_z";

    return os.str();
  }

  std::string to_csv_string(const double& timestamp) const
  {
    std::stringstream os;
    os.precision(17);
    os << timestamp;

    os << ", " << linear_acceleration_(0) << ", " << linear_acceleration_(1) << ", " << linear_acceleration_(2);
    os << ", " << angular_velocity_(0) << ", " << angular_velocity_(1) << ", " << angular_velocity_(2);

    return os.str();
  }
};
}  // namespace mars
#endif  // IMU_MEASUREMENT_TYPE_H
