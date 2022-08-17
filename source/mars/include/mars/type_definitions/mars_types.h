// Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef MARSTYPES_H
#define MARSTYPES_H

#include <Eigen/Dense>

namespace mars
{
class Pose
{
public:
  Eigen::Vector3d p{ Eigen::Vector3d::Zero() };
  Eigen::Quaterniond q{ Eigen::Quaterniond::Identity() };

  Pose() = default;
  Pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) : p(position), q(orientation)
  {
  }

  bool operator==(const Pose& rhs) const
  {
    return ((p == rhs.p) && ((q.coeffs() == rhs.q.coeffs())));
  }

  friend std::ostream& operator<<(std::ostream& out, const Pose& data)
  {
    out << "p: " << data.p[0] << ", " << data.p[1] << ", " << data.p[2];
    out << " q: " << data.q.w() << ", " << data.q.x() << ", " << data.q.y() << ", " << data.q.z();

    return out;
  }
};
}  // namespace mars

#endif  // MARSTYPES_H
