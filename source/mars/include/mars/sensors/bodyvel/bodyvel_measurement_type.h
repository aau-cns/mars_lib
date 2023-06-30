// Copyright (C) 2021 Martin Scheiber and Christian Brommer, Control of Networked Systems, University of Klagenfurt,
// Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <martin.scheiber@ieee.org>
// and <christian.brommer@ieee.org>.

#ifndef BODYVELMEASUREMENTTYPE_H
#define BODYVELMEASUREMENTTYPE_H

#include <mars/sensors/measurement_base_class.h>
#include <Eigen/Dense>
#include <utility>

namespace mars
{
class BodyvelMeasurementType : public BaseMeas
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d velocity_;  ///< Velocity [x y z]

  BodyvelMeasurementType() = default;

  BodyvelMeasurementType(Eigen::Vector3d velocity) : velocity_(std::move(velocity))
  {
  }
};
}  // namespace mars
#endif  // BODYVELMEASUREMENTTYPE_H
