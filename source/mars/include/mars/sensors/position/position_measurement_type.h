// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef POSITIONMEASUREMENTTYPE_H
#define POSITIONMEASUREMENTTYPE_H

#include <mars/sensors/measurement_base_class.h>
#include <Eigen/Dense>
#include <utility>

namespace mars
{
class PositionMeasurementType : public BaseMeas
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d position_;  ///< Position [x y z]

  PositionMeasurementType(Eigen::Vector3d position) : position_(std::move(position))
  {
  }

  static std::string get_csv_state_header_string()
  {
    std::stringstream os;
    os << "t, ";
    os << "p_x, p_y, p_z";

    return os.str();
  }

  std::string to_csv_string(const double& timestamp) const
  {
    std::stringstream os;
    os.precision(17);
    os << timestamp;

    os << ", " << position_.x() << ", " << position_.y() << ", " << position_.z();

    return os.str();
  }
};
}  // namespace mars
#endif  // POSITIONMEASUREMENTTYPE_H
