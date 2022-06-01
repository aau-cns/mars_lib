// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@ieee.org>

#ifndef BODYVELSENSORSTATETYPE_H
#define BODYVELSENSORSTATETYPE_H

#include <mars/type_definitions/base_states.h>

#include <eigen3/Eigen/Dense>

namespace mars
{
class BodyvelSensorStateType : public BaseStates
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p_ib_;
  Eigen::Quaternion<double> q_ib_;

  BodyvelSensorStateType() : BaseStates(6)  // size of covariance
  {
    p_ib_.setZero();
    q_ib_.setIdentity();
  }

  std::string to_csv_string(const double& timestamp) const
  {
    std::stringstream os;
    os << timestamp;

    os << ", " << p_ib_(0) << ", " << p_ib_(1) << ", " << p_ib_(2);

    Eigen::Vector4d q_ib = q_ib_.coeffs();  // x y z w
    os << ", " << q_ib(3) << ", " << q_ib(0) << ", " << q_ib(1) << ", " << q_ib(2);

    return os.str();
  }
};
}  // namespace mars
#endif  // BODYVELSENSORSTATETYPE_H
