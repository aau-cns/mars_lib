// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@ieee.org>

#ifndef ATTITUDE_CONVERSION_H
#define ATTITUDE_CONVERSION_H

#include <mars/general_functions/utils.h>
#include <Eigen/Dense>

namespace mars
{
///
/// \brief The Attitude struct
///
struct Attitude
{
  Attitude() = default;
  Attitude(const Eigen::Quaterniond& quaternion) : quaternion_(quaternion)
  {
  }
  Attitude(const Eigen::Matrix3d& rotation_matrix) : quaternion_(rotation_matrix)
  {
  }
  Attitude(const Eigen::Vector3d& vec, const std::string& order = "XYZ")
  {
    // scoping
    using AAd = Eigen::AngleAxisd;
    using v3d = Eigen::Vector3d;

    /// \todo TODO other orders need to be implemented
    if (order == "XYZ")
    {
      quaternion_ = AAd(vec(0), v3d::UnitX()) * AAd(vec(1), v3d::UnitY()) * AAd(vec(2), v3d::UnitZ());
    }
    else if (order == "ZYX")
    {
      quaternion_ = AAd(vec(2), v3d::UnitZ()) * AAd(vec(1), v3d::UnitY()) * AAd(vec(0), v3d::UnitX());
    }
  }

  Eigen::Quaterniond quaternion_;

  Eigen::Matrix3d get_rotation_matrix()
  {
    return quaternion_.toRotationMatrix();
  }

  Eigen::Vector2d get_rp()
  {
    Eigen::Vector3d rpy = mars::Utils::RPYFromRotMat(quaternion_.toRotationMatrix());
    return { rpy(0), rpy(1) };
  }

  friend std::ostream& operator<<(std::ostream& out, const Attitude& attitude);
};

}  // namespace mars
#endif  // ATTITUDE_CONVERSION_H
