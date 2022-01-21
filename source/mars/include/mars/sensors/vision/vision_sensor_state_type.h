// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef VISIONSENSORSTATETYPE_H
#define VISIONSENSORSTATETYPE_H

#include <mars/type_definitions/base_states.h>
#include <Eigen/Dense>

namespace mars
{
class VisionSensorStateType : public BaseStates
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p_vw_;
  Eigen::Quaternion<double> q_vw_;
  Eigen::Vector3d p_ic_;
  Eigen::Quaternion<double> q_ic_;
  double lambda_;

  VisionSensorStateType() : BaseStates(13)  // size of covariance
  {
    p_vw_.setZero();      // 3
    q_vw_.setIdentity();  // 4-1
    p_ic_.setZero();      // 3
    q_ic_.setIdentity();  // 4-1
    lambda_ = 1;          // 1
  }

  static std::string get_csv_state_header_string()
  {
    std::stringstream os;
    os << "t, ";
    os << "p_vw_x, p_vw_y, p_vw_z, ";
    os << "q_vw_w, q_vw_x, q_vw_y, q_vw_z,";
    os << "p_ic_x, p_ic_y, p_ic_z, ";
    os << "q_ic_w, q_ic_x, q_ic_y, q_ic_z,";
    os << "lambda";

    return os.str();
  }

  std::string to_csv_string(const double& timestamp) const
  {
    std::stringstream os;
    os.precision(17);
    os << timestamp;

    os << ", " << p_vw_(0) << ", " << p_vw_(1) << ", " << p_vw_(2);

    Eigen::Vector4d q_vw = q_vw_.coeffs();  // x y z w
    os << ", " << q_vw(3) << ", " << q_vw(0) << ", " << q_vw(1) << ", " << q_vw(2);

    os << ", " << p_ic_(0) << ", " << p_ic_(1) << ", " << p_ic_(2);

    Eigen::Vector4d q_ic = q_ic_.coeffs();  // x y z w
    os << ", " << q_ic(3) << ", " << q_ic(0) << ", " << q_ic(1) << ", " << q_ic(2);

    os << ", " << lambda_;

    return os.str();
  }
};
}
#endif  // VISIONSENSORSTATETYPE_H
