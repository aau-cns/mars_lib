// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef PROGRESSINDICATOR_H
#define PROGRESSINDICATOR_H

#include <iomanip>
#include <iostream>

namespace mars
{
class ProgressIndicator
{
public:
  ProgressIndicator(const int& total_iterations, const int& step_width);

  void next_step();

private:
  const double total_iterations_;
  const double step_width_;

  double current_progress_{ 0 };
  int current_iteration_{ 0 };
};
}  // namespace mars

#endif  // PROGRESSINDICATOR_H
