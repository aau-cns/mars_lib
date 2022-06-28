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
#include <Eigen/Dense>

namespace mars
{
class GpsVelMeasurementType
{
public:
  GpsCoordinates coordinates_;
  Eigen::Vector3d velocity_;

  GpsVelMeasurementType(double latitude, double longitude, double altitude, double vel_x, double vel_y, double vel_z)
    : coordinates_(std::move(latitude), std::move(longitude), std::move(altitude))
    , velocity_(std::move(vel_x), std::move(vel_y), std::move(vel_z))
  {
  }
};
}  // namespace mars
#endif  // GPSVELMEASUREMENTTYPE_H
