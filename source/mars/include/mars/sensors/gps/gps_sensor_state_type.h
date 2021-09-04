// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef GPSSENSORSTATETYPE_H
#define GPSSENSORSTATETYPE_H

#include <mars/type_definitions/base_states.h>
#include <Eigen/Dense>

namespace mars
{
class GpsSensorStateType : public BaseStates
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p_ig_;
  Eigen::Vector3d p_gw_w_;
  Eigen::Quaterniond q_gw_w_;

  GpsSensorStateType() : BaseStates(9)  // cov size
  {
    p_ig_.setZero();
    p_gw_w_.setZero();
    q_gw_w_.setIdentity();
  }
};
}
#endif  // GPSSENSORSTATETYPE_H
