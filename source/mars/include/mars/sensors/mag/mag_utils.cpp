// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include "mag_utils.h"

namespace mars
{
double MagnetometerInit::get_yaw()
{
  return yaw_;
}

void MagnetometerInit::AddElement(const Eigen::Vector3d& mag_vector)
{
  m_vec_.push_back(mag_vector);
}

void MagnetometerInit::AddElement(const double& x, const double& y, const double& z)
{
  const Eigen::Vector3d m(x, y, z);
  m_vec_.push_back(m);
}

int MagnetometerInit::get_size()
{
  return int(m_vec_.size());
}

void MagnetometerInit::set_done()
{
  once_ = true;
}

bool MagnetometerInit::IsDone()
{
  return once_;
}

void MagnetometerInit::Reset()
{
  once_ = false;
  m_vec_.clear();
}

void MagnetometerInit::get_vec_mean(Eigen::Vector3d& mean_res)
{
  double x(0), y(0), z(0);
  const unsigned long v_size = m_vec_.size();

  for (unsigned long i = 0; i < v_size; i++)
  {
    x += m_vec_[i](0);
    y += m_vec_[i](1);
    z += m_vec_[i](2);
  }

  mean_res = Eigen::Vector3d(x / v_size, y / v_size, z / v_size);
}

void MagnetometerInit::get_quat(Eigen::Quaterniond& q_mag)
{
  Eigen::Vector3d m;
  get_vec_mean(m);

  // const double r = m.norm();
  const double az = atan2(m(1), m(0));
  // const double el = atan2(m(1), sqrt(m(0) * m(0) + m(1) * m(1)));

  yaw_ = (M_PI / 2) - az;

  Eigen::Matrix3d r_z;
  r_z << cos(yaw_), -sin(yaw_), 0, sin(yaw_), cos(yaw_), 0, 0, 0, 1;

  q_mag = Eigen::Quaterniond(r_z);
}
}
