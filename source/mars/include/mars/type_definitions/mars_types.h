// Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef MARSTYPES_H
#define MARSTYPES_H

#include <Eigen/Dense>

namespace mars
{
class Pose
{
public:
  Eigen::Vector3d p_{ Eigen::Vector3d::Zero() };
  Eigen::Quaterniond q_{ Eigen::Quaterniond::Identity() };

  Eigen::Vector3d n_p_{ Eigen::Vector3d::Ones() * 0.1 };
  Eigen::Vector3d n_r_{ Eigen::Vector3d::Ones() * 0.1 };

  Pose() = default;
  Pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) : p_(position), q_(orientation)
  {
  }

  Pose(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation) : p_(position), q_(rotation)
  {

  }

  void set_meas_noise(Eigen::Vector3d n_p, Eigen::Vector3d n_r)
  {
    n_p_ = n_p;
    n_r_ = n_r;
  }

  bool operator==(const Pose& rhs) const
  {
    return ((p_ == rhs.p_) && ((q_.coeffs() == rhs.q_.coeffs())));
  }

  friend std::ostream& operator<<(std::ostream& out, const Pose& data)
  {
    out << "p: " << data.p_[0] << ", " << data.p_[1] << ", " << data.p_[2];
    out << " q: " << data.q_.w() << ", " << data.q_.x() << ", " << data.q_.y() << ", " << data.q_.z();

    return out;
  }

  Eigen::Matrix<double, 6, 6> get_meas_noise_mat() const
  {
    Eigen::Matrix<double, 6, 1> vec;
    vec << n_p_, n_r_;
    return vec.asDiagonal();
  }

  ///
  /// \brief Return inverse transformation (T_AB -> T_BA) as mars::Pose
  /// \return Pose
  ///
  Pose get_inverse_pose() const
  {
    return Pose(-q_.toRotationMatrix().transpose()*p_, q_.conjugate());
  }

};
}  // namespace mars

#endif  // MARSTYPES_H
