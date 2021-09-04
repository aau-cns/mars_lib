// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef BASESTATES_H
#define BASESTATES_H

#include <string>

namespace mars
{
///
/// \brief The BaseStates class is used to ensure that all sensor data classes define a covariance size for the
/// 'bind_sensor_data' class
///
class BaseStates
{
public:
  int cov_size_;

  BaseStates(int cov_size) : cov_size_(cov_size)
  {
  }
};
}

#endif  // BASESTATES_H
