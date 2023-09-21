// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef GPSVELMEASUREMENTTYPE_H
#define GPSVELMEASUREMENTTYPE_H

#include <mars/sensors/gps/gps_conversion.h>
#include <mars/sensors/measurement_base_class.h>
#include <Eigen/Dense>

namespace mars
{
class GpsVelMeasurementType : public BaseMeas
{
public:
  GpsCoordinates coordinates_;
  Eigen::Vector3d velocity_;

  GpsVelMeasurementType(double latitude, double longitude, double altitude, double vel_x, double vel_y, double vel_z)
    : coordinates_(std::move(latitude), std::move(longitude), std::move(altitude))
    , velocity_(std::move(vel_x), std::move(vel_y), std::move(vel_z))
  {
  }

  static std::string get_csv_state_header_string()
  {
    std::stringstream os;
    os << "t, ";
    os << "lat, lon, alt, ";
    os << "vel_x, vel_y, vel_z";

    return os.str();
  }

  std::string to_csv_string(const double& timestamp) const
  {
    std::stringstream os;
    os.precision(17);
    os << timestamp;

    os << ", " << coordinates_.latitude_ << ", " << coordinates_.longitude_ << ", " << coordinates_.altitude_;
    os << ", " << velocity_.x() << ", " << velocity_.y() << ", " << velocity_.z();

    return os.str();
  }
};
}  // namespace mars
#endif  // GPSVELMEASUREMENTTYPE_H
