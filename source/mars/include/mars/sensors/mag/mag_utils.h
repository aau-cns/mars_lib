// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef MAG_UTILS_H
#define MAG_UTILS_H

#include <Eigen/Dense>
#include <deque>

namespace mars
{
class MagnetometerInit
{
public:
  MagnetometerInit() = default;

  double get_yaw();
  int get_size();
  void set_done();
  bool IsDone();
  void Reset();

  void AddElement(const Eigen::Vector3d& mag_vector);
  void AddElement(const double& x, const double& y, const double& z);

  void get_vec_mean(Eigen::Vector3d& mean_res);
  void get_quat(Eigen::Quaterniond& q_mag);

private:
  std::deque<Eigen::Vector3d> m_vec_;
  bool once_{ false };
  double yaw_{ 0 };
};
}

#endif  // MAG_UTILS_H
