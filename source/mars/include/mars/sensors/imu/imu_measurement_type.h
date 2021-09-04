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
};
}
#endif  // IMU_MEASUREMENT_TYPE_H
