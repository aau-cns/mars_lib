// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef POSEMEASUREMENTTYPE_H
#define POSEMEASUREMENTTYPE_H

#include <Eigen/Dense>
#include <utility>

namespace mars
{
class PoseMeasurementType
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d position_;               ///< Position [x y z]
  Eigen::Quaternion<double> orientation_;  ///< Quaternion [w x y z]

  PoseMeasurementType() = default;

  PoseMeasurementType(Eigen::Vector3d position, Eigen::Quaternion<double> orientation)
    : position_(std::move(position)), orientation_(std::move(orientation))
  {
  }
};
}  // namespace mars
#endif  // POSEMEASUREMENTTYPE_H
