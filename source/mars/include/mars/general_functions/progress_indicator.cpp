// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include "progress_indicator.h"
namespace mars
{
ProgressIndicator::ProgressIndicator(const int& total_iterations, const int& step_width)
  : total_iterations_(total_iterations), step_width_(step_width)
{
}

void ProgressIndicator::next_step()
{
  current_iteration_++;
  const double prog = (current_iteration_ / total_iterations_) * 100;
  if (prog >= current_progress_)
  {
    std::cout << "Progress: [ ";
    std::cout << std::setprecision(2) << current_progress_;
    std::cout << "% ]" << std::endl;
    current_progress_ += step_width_;
  }
}
}  // namespace mars
