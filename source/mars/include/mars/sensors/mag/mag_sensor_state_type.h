// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef MAG_SENSOR_STATE_TYPE_H
#define MAG_SENSOR_STATE_TYPE_H

#include <mars/type_definitions/base_states.h>
#include <Eigen/Dense>

namespace mars
{
class MagSensorStateType : public BaseStates
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d mag_;
  Eigen::Quaterniond q_im_;

  MagSensorStateType() : BaseStates(6)  // cov size
  {
    mag_.setZero();
    q_im_.setIdentity();
  }
};
}
#endif  // MAG_SENSOR_STATE_TYPE_H
