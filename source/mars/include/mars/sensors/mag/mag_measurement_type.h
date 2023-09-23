// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef MAG_MEASUREMENT_TYPE_H
#define MAG_MEASUREMENT_TYPE_H

#include <mars/sensors/measurement_base_class.h>
#include <Eigen/Dense>
#include <utility>

namespace mars
{
class MagMeasurementType : public BaseMeas
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d mag_vector_;  ///< Raw magnetometer measurement [x y z]

  MagMeasurementType() = default;

  MagMeasurementType(Eigen::Vector3d mag_vector) : mag_vector_(std::move(mag_vector))
  {
  }

  static std::string get_csv_state_header_string()
  {
    std::stringstream os;
    os << "t, ";
    os << "m_x, m_y, m_z";

    return os.str();
  }

  std::string to_csv_string(const double& timestamp) const
  {
    std::stringstream os;
    os.precision(17);
    os << timestamp;

    os << ", " << mag_vector_.x() << ", " << mag_vector_.y() << ", " << mag_vector_.z();

    return os.str();
  }
};
}  // namespace mars

#endif  // MAG_MEASUREMENT_TYPE_H
