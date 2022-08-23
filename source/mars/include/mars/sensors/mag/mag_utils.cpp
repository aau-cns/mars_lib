// Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include "mag_utils.h"

namespace mars
{
void MagnetometerInit::AddElement(const Eigen::Vector3d& mag_vector, const Eigen::Vector3d& imu_linear_acc_vector)
{
  rot_init_vec_.push_back(MagImuData(mag_vector, imu_linear_acc_vector));
}

int MagnetometerInit::get_size() const
{
  return static_cast<int>(rot_init_vec_.size());
}

void MagnetometerInit::set_done()
{
  once_ = true;
}

bool MagnetometerInit::IsDone() const
{
  return once_;
}

void MagnetometerInit::Reset()
{
  once_ = false;
  rot_init_vec_.clear();
}

MagnetometerInit::MagImuData MagnetometerInit::get_vec_mean() const
{
  Eigen::Vector3d mag_vec_sum(0, 0, 0);
  Eigen::Vector3d imu_vec_sum(0, 0, 0);
  for (const auto& k : rot_init_vec_)
  {
    mag_vec_sum += k.mag_vec_;
    imu_vec_sum += k.imu_vec_;
  }
  return MagImuData(mag_vec_sum / get_size(), imu_vec_sum / get_size());
}

Eigen::Matrix3d MagnetometerInit::get_rot() const
{
  MagImuData mag_imu = get_vec_mean();

  // Normalize vectors
  Eigen::Vector3d mag = mag_imu.mag_vec_.normalized();
  Eigen::Vector3d imu = mag_imu.imu_vec_.normalized();

  // Generate Frame base vectors
  Eigen::Vector3d x_axis = mag.cross(imu).normalized();
  Eigen::Vector3d y_axis = imu.cross(x_axis).normalized();
  Eigen::Vector3d z_axis = imu;

  // Build rotation matrix
  Eigen::Matrix3d R;
  R << x_axis, y_axis, z_axis;
  return R;
}

Eigen::Quaterniond MagnetometerInit::get_quat() const
{
  return Eigen::Quaterniond(get_rot());
}
}
