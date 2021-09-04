// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef CORETYPE_H
#define CORETYPE_H

#include <mars/type_definitions/core_state_type.h>

namespace mars
{
class CoreType
{
public:
  CoreStateType state_;
  CoreStateMatrix cov_;
  CoreStateMatrix state_transition_;
  // ref_to_nav;

  CoreType() = default;
};
}

#endif  // CORETYPE_H
