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

#ifndef BODYVELSENSORSTATETYPE_H
#define BODYVELSENSORSTATETYPE_H

#include <mars/type_definitions/base_states.h>
#include <Eigen/Dense>

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

  static std::string get_csv_state_header_string()
  {
    std::stringstream os;
    os << "t, ";
    os << "p_ib_x, p_ib_y, p_ib_z, ";
    os << "q_ib_w, q_ib_x, q_ib_y, q_ib_z";

    return os.str();
  }

  std::string to_csv_string(const double& timestamp) const
  {
    std::stringstream os;
    os.precision(17);
    os << timestamp;

    os << ", " << p_ib_(0) << ", " << p_ib_(1) << ", " << p_ib_(2);
    os << ", " << q_ib_.w() << ", " << q_ib_.x() << ", " << q_ib_.y() << ", " << q_ib_.z();

    return os.str();
  }
};
}  // namespace mars
#endif  // BODYVELSENSORSTATETYPE_H
