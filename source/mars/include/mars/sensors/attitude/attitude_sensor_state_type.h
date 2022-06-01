// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@ieee.org>

#ifndef ATTITUDE_SENSOR_STATE_TYPE_H
#define ATTITUDE_SENSOR_STATE_TYPE_H

#include <mars/type_definitions/base_states.h>

#include <eigen3/Eigen/Dense>

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

    Eigen::Vector4d q_aw = q_aw_.coeffs();  // x y z w
    Eigen::Vector4d q_ib = q_ib_.coeffs();  // x y z w
    os << ", " << q_aw(3) << ", " << q_aw(0) << ", " << q_aw(1) << ", " << q_aw(2);
    os << ", " << q_ib(3) << ", " << q_ib(0) << ", " << q_ib(1) << ", " << q_ib(2);

    return os.str();
  }
};
}  // namespace mars
#endif  // ATTITUDE_SENSOR_STATE_TYPE_H
