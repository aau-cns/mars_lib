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

#ifndef ATTITUDE_SENSOR_STATE_TYPE_H
#define ATTITUDE_SENSOR_STATE_TYPE_H

#include <mars/type_definitions/base_states.h>
#include <Eigen/Dense>

namespace mars
{
class AttitudeSensorStateType : public BaseStates
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Quaternion<double> q_aw_;  // calibration between world and attitude reference frame
  Eigen::Quaternion<double> q_ib_;  // calibration between IMU and attitude sensor

  AttitudeSensorStateType() : BaseStates(6)  // BaseStates(6)  // cov size
  {
    q_aw_.setIdentity();
    q_ib_.setIdentity();
  }

  static std::string get_csv_state_header_string()
  {
    std::stringstream os;
    os << "t, ";
    os << "q_aw_w, q_aw_x, q_aw_y, q_aw_z";
    os << "q_ib_w, q_ib_x, q_ib_y, q_ib_z";

    return os.str();
  }

  std::string to_csv_string(const double& timestamp) const
  {
    std::stringstream os;
    os.precision(17);
    os << timestamp;

    os << ", " << q_aw_.w() << ", " << q_aw_.x() << ", " << q_aw_.y() << ", " << q_aw_.z();
    os << ", " << q_ib_.w() << ", " << q_ib_.x() << ", " << q_ib_.y() << ", " << q_ib_.z();

    return os.str();
  }
};
}  // namespace mars
#endif  // ATTITUDE_SENSOR_STATE_TYPE_H
