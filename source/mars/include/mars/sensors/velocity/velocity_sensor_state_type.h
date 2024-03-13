// Copyright (C) 2024 Christian Brommer, Control of Networked Systems, University of Klagenfurt,
// Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef VELOCITYSENSORSTATETYPE_H
#define VELOCITYSENSORSTATETYPE_H

#include <mars/type_definitions/base_states.h>
#include <Eigen/Dense>

namespace mars
{
class VelocitySensorStateType : public BaseStates
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p_iv_;

  VelocitySensorStateType() : BaseStates(3)  // size of covariance
  {
    p_iv_.setZero();
  }

  static std::string get_csv_state_header_string()
  {
    std::stringstream os;
    os << "t, ";
    os << "p_iv_x, p_iv_y, p_iv_z";

    return os.str();
  }

  std::string to_csv_string(const double& timestamp) const
  {
    std::stringstream os;
    os.precision(17);
    os << timestamp;

    os << ", " << p_iv_(0) << ", " << p_iv_(1) << ", " << p_iv_(2);
    return os.str();
  }
};
}  // namespace mars
#endif  // VELOCITYSENSORSTATETYPE_H
