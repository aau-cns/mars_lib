// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef MAG_MEASUREMENT_TYPE_H
#define MAG_MEASUREMENT_TYPE_H

#include <Eigen/Dense>
#include <utility>

namespace mars
{
class MagMeasurementType
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d mag_vector_;  ///< Raw magnetometer measurement [x y z]

  MagMeasurementType() = default;

  MagMeasurementType(Eigen::Vector3d mag_vector) : mag_vector_(std::move(mag_vector))
  {
  }
};
}

#endif  // MAG_MEASUREMENT_TYPE_H
