// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef TIME_H
#define TIME_H

#include <iostream>

namespace mars
{
class Time
{
public:
  Time() = default;
  Time(const double& seconds);

  double get_seconds() const;

  Time abs() const;

  Time operator+(const Time& rhs) const;
  Time operator-(const Time& rhs) const;
  bool operator==(const Time& rhs) const;
  bool operator<(const Time& rhs) const;
  bool operator<=(const Time& rhs) const;
  bool operator>(const Time& rhs) const;
  bool operator>=(const Time& rhs) const;

  friend std::ostream& operator<<(std::ostream& out, const Time& data);

private:
  double seconds_{ 0.0 };
};
}
#endif  // TIME_H
