// Copyright (C) 2021 Martin Scheiber, Christian Brommer,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <christian.brommer@ieee.org>
// and <martin.scheiber@ieee.org>

#ifndef PRESSUREMEASUREMENTTYPE_H
#define PRESSUREMEASUREMENTTYPE_H

#include <Eigen/Dense>

namespace mars
{
class PressureMeasurementType
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Eigen::Matrix<double, 1, 1> Matrix1d_t;

  Matrix1d_t height_;  ///< height [z]

  PressureMeasurementType(double height)  // : height_(height)
  {
    height_ = Matrix1d_t{ height };
  }
};
}  // namespace mars
#endif  // PRESSUREMEASUREMENTTYPE_H
