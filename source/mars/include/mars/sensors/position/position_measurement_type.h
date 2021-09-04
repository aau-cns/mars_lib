// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef POSITIONMEASUREMENTTYPE_H
#define POSITIONMEASUREMENTTYPE_H

#include <Eigen/Dense>

namespace mars
{
class PositionMeasurementType
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d position_;  ///< Position [x y z]

  PositionMeasurementType(Eigen::Vector3d position) : position_(position)
  {
  }
};
}
#endif  // POSITIONMEASUREMENTTYPE_H
