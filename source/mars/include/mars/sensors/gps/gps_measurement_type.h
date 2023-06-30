// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef GPSMEASUREMENTTYPE_H
#define GPSMEASUREMENTTYPE_H

#include <mars/sensors/gps/gps_conversion.h>
#include <mars/sensors/measurement_base_class.h>
#include <utility>

namespace mars
{
class GpsMeasurementType : public BaseMeas
{
public:
  GpsCoordinates coordinates_;

  GpsMeasurementType(double latitude, double longitude, double altitude)
    : coordinates_(std::move(latitude), std::move(longitude), std::move(altitude))
  {
  }
};
}  // namespace mars
#endif  // GPS_MEASUREMENTTYPE_H
