// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include <mars/time.h>
#include <cmath>
#include <iostream>

namespace mars
{
Time::Time(const double& seconds) : seconds_(seconds)
{
}

double Time::get_seconds() const
{
  return seconds_;
}

Time Time::abs() const
{
  return { std::abs(seconds_) };
}

Time Time::operator+(const Time& rhs) const
{
  return { seconds_ + rhs.seconds_ };
}

Time Time::operator-(const Time& rhs) const
{
  return { seconds_ - rhs.seconds_ };
}

bool Time::operator==(const Time& rhs) const
{
  return (seconds_ == rhs.seconds_);
}

bool Time::operator<(const Time& rhs) const
{
  return seconds_ < rhs.seconds_;
}

bool Time::operator<=(const Time& rhs) const
{
  return seconds_ <= rhs.seconds_;
}

bool Time::operator>(const Time& rhs) const
{
  return seconds_ > rhs.seconds_;
}

bool Time::operator>=(const Time& rhs) const
{
  return seconds_ >= rhs.seconds_;
}

std::ostream& operator<<(std::ostream& out, const Time& data)
{
  out << data.seconds_ << '\t';

  return out;
}
}
