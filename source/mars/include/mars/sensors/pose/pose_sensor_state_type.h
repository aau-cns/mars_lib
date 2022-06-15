// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef POSESENSORSTATETYPE_H
#define POSESENSORSTATETYPE_H

#include <mars/type_definitions/base_states.h>
#include <Eigen/Dense>

namespace mars
{
class PoseSensorStateType : public BaseStates
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p_ip_;
  Eigen::Quaternion<double> q_ip_;

  PoseSensorStateType() : BaseStates(6)  // size of covariance
  {
    p_ip_.setZero();
    q_ip_.setIdentity();
  }

  static std::string get_csv_state_header_string()
  {
    std::stringstream os;
    os << "t, ";
    os << "p_ip_x, p_ip_y, p_ip_z, ";
    os << "q_ip_w, q_ip_x, q_ip_y, q_ip_z";

    return os.str();
  }

  std::string to_csv_string(const double& timestamp) const
  {
    std::stringstream os;
    os.precision(17);
    os << timestamp;

    os << ", " << p_ip_(0) << ", " << p_ip_(1) << ", " << p_ip_(2);

    Eigen::Vector4d q_ip = q_ip_.coeffs();  // x y z w
    os << ", " << q_ip(3) << ", " << q_ip(0) << ", " << q_ip(1) << ", " << q_ip(2);

    return os.str();
  }
};
}  // namespace mars
#endif  // POSESENSORSTATETYPE_H
