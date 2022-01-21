// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef GPSVELSENSORSTATETYPE_H
#define GPSVELSENSORSTATETYPE_H

#include <mars/type_definitions/base_states.h>
#include <Eigen/Dense>

namespace mars
{
class GpsVelSensorStateType : public BaseStates
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p_ig_;
  Eigen::Vector3d p_gw_w_;
  Eigen::Quaterniond q_gw_w_;

  GpsVelSensorStateType() : BaseStates(9)  // cov size
  {
    p_ig_.setZero();
    p_gw_w_.setZero();
    q_gw_w_.setIdentity();
  }

  static std::string get_csv_state_header_string()
  {
    std::stringstream os;
    os << "t, ";
    os << "p_ig_x, p_ig_y, p_ig_z, ";
    os << "p_gw_w_x, p_gw_w_y, p_gw_w_z, ";
    os << "q_gw_w_w, q_gw_w_x, q_gw_w_y, q_gw_w_z";

    return os.str();
  }

  ///
  /// \brief to_csv_string export state to single csv string
  /// \param timestamp
  /// \return string format [p_ig p_gw_w q_gw_w]
  ///
  std::string to_csv_string(const double& timestamp) const
  {
    std::stringstream os;
    os.precision(17);
    os << timestamp;

    os << ", " << p_ig_(0) << ", " << p_ig_(1) << ", " << p_ig_(2);
    os << ", " << p_gw_w_(0) << ", " << p_gw_w_(1) << ", " << p_gw_w_(2);
    Eigen::Vector4d q_gw_w = q_gw_w_.coeffs();  // x y z w
    os << ", " << q_gw_w(3) << ", " << q_gw_w(0) << ", " << q_gw_w(1) << ", " << q_gw_w(2);

    return os.str();
  }
};
}
#endif  // GPSVELSENSORSTATETYPE_H
