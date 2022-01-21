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

  static std::string get_csv_state_header_string()
  {
    std::stringstream os;
    os << "t, ";
    os << "mag_w_x, mag_w_y, mag_w_z, ";
    os << "q_im_w, q_im_x, q_im_y, q_im_z";

    return os.str();
  }

  ///
  /// \brief to_csv_string export state to single csv string
  /// \param timestamp
  /// \return string format [mag q_wi]
  ///
  std::string to_csv_string(const double& timestamp) const
  {
    std::stringstream os;
    os.precision(17);
    os << timestamp;

    os << ", " << mag_(0) << ", " << mag_(1) << ", " << mag_(2);
    Eigen::Vector4d q_im = q_im_.coeffs();  // x y z w
    os << ", " << q_im(3) << ", " << q_im(0) << ", " << q_im(1) << ", " << q_im(2);

    return os.str();
  }
};
}
#endif  // MAG_SENSOR_STATE_TYPE_H
