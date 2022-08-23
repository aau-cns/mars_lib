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

#ifndef PRESSURESENSORSTATETYPE_H
#define PRESSURESENSORSTATETYPE_H

#include <mars/type_definitions/base_states.h>

#include <Eigen/Dense>

namespace mars
{
class PressureSensorStateType : public BaseStates
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p_ip_;
  // technically also bias and scale here, which are now assumed zero/one

  PressureSensorStateType() : BaseStates(3)  // cov size
  {
    p_ip_.setZero();
  }
};
}  // namespace mars
#endif  // PRESSURESENSORSTATETYPE_H
