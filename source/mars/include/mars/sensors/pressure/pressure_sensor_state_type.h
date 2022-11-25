// Copyright (C) 2022 Martin Scheiber, Christian Brommer,
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
  double bias_p_;
  // technically also scale here, which is currently assumed one

  PressureSensorStateType() : BaseStates(4)  // cov size
  {
    p_ip_.setZero();
    bias_p_ = 0.0;
  }

  static std::string get_csv_state_header_string()
  {
    std::stringstream os;
    os << "t, ";
    os << "p_ip_x, p_ip_y, p_ip_z, ";
    os << "bias_p";

    return os.str();
  }

  std::string to_csv_string(const double& timestamp) const
  {
    std::stringstream os;
    os.precision(17);
    os << timestamp;

    os << ", " << p_ip_(0) << ", " << p_ip_(1) << ", " << p_ip_(2);
    os << ", " << bias_p_;

    return os.str();
  }
};
}  // namespace mars
#endif  // PRESSURESENSORSTATETYPE_H
