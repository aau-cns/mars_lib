// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef POSEMEASUREMENTTYPE_H
#define POSEMEASUREMENTTYPE_H

#include <mars/sensors/measurement_base_class.h>
#include <Eigen/Dense>
#include <utility>

namespace mars
{
class PoseMeasurementType : public BaseMeas
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d position_;               ///< Position [x y z]
  Eigen::Quaternion<double> orientation_;  ///< Quaternion [w x y z]

  PoseMeasurementType() = default;

  PoseMeasurementType(Eigen::Vector3d position, const Eigen::Quaternion<double>& orientation)
    : position_(std::move(position)), orientation_(orientation)
  {
  }

  static std::string get_csv_state_header_string()
  {
    std::stringstream os;
    os << "t, ";
    os << "p_x, p_y, p_z, ";
    os << "q_w, q_x, q_y, q_z";

    return os.str();
  }

  std::string to_csv_string(const double& timestamp) const
  {
    std::stringstream os;
    os.precision(17);
    os << timestamp;

    os << ", " << position_.x() << ", " << position_.y() << ", " << position_.z();
    os << ", " << orientation_.w() << ", " << orientation_.x() << ", " << orientation_.y() << ", " << orientation_.z();

    return os.str();
  }
};
}  // namespace mars
#endif  // POSEMEASUREMENTTYPE_H
