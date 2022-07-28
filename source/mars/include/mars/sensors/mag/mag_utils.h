// Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
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
#include <vector>

namespace mars
{
class MagnetometerInit
{
public:
  MagnetometerInit() = default;

  ///
  /// \brief AddElement Add magnetometer and imu measurement pairs to buffer
  /// IMU and Magnetometer vectors nee to be expressed in the IMU reference frame
  /// \param mag_vector Magnetic vector expressed in the IMU frame
  /// \param imu_linear_acc_vector Linear acceleration vector, assuming static conditions
  ///
  void AddElement(const Eigen::Vector3d& mag_vector, const Eigen::Vector3d& imu_linear_acc_vector);

  ///
  /// \brief get_rot Get the rotation matrix for the vector pairs in the buffer
  /// \return Rotation of the IMU sensor w.r.t. the world frame
  ///
  Eigen::Matrix3d get_rot() const;

  ///
  /// \brief get_quat Same as 'get_rot' but as quaternion
  /// \return Rotation of the IMU sensor w.r.t. the world frame
  ///
  Eigen::Quaterniond get_quat() const;

  ///
  /// \brief get_size Get the current size of the buffer
  /// \return Size of the buffer
  ///
  int get_size() const;

  ///
  /// \brief set_done Set if the rotation initialization was done
  ///
  void set_done();

  ///
  /// \brief IsDone Check if the initialization was done
  /// \return True if initialization was done
  ///
  bool IsDone() const;

  ///
  /// \brief Reset Reset the initialization module
  ///
  /// Reset 'isDone' and clear the buffer
  ///
  void Reset();

  ///
  /// \brief mag_var_ang_to_vec Perform Spherical to Cartesian conversion for a vector in the GNSS world frame
  ///
  /// The GNSS world frame is rotated -90deg w.r.t. the magnetic world frame.
  ///
  /// \param dec Declination or azimuth (in degree) of the magnetic field vector
  /// \param inc Inclination or elevation (in degree) of the magnetic field vector
  /// \param r magnitude of the magnetic field (defaults to 1)
  /// \return Carthesian vector
  ///
  static Eigen::Vector3d mag_var_ang_to_vec(const double& dec, const double& inc, const double& r = 1);

  ///
  /// \brief The MagImuData class keeps a pair of magnetometer and imu linear acceleration measurements
  ///
  class MagImuData
  {
  public:
    ///
    /// \brief MagImuData
    /// \param mag_vec Single mag measurement
    /// \param imu_vec Single imu measurement
    ///
    MagImuData(const Eigen::Vector3d& mag_vec, const Eigen::Vector3d& imu_vec) : mag_vec_(mag_vec), imu_vec_(imu_vec)
    {
    }
    ///
    /// \brief mag_vec_ Single vector of a magnetometer measurement
    ///
    Eigen::Vector3d mag_vec_;

    ///
    /// \brief imu_vec_ Single vector of an IMU linear acceleration measurement
    ///
    Eigen::Vector3d imu_vec_;
  };

  ///
  /// \brief get_vec_mean Get the mean of the Mag and IMU vectors in the measurement buffer
  /// \return Return the mean as 'MagImuData' type
  ///
  MagImuData get_vec_mean() const;

private:
  ///
  /// \brief rot_init_vec_ Measurement buffer
  ///
  std::vector<MagImuData> rot_init_vec_;

  ///
  /// \brief once_ Indicate if the intialization was done
  ///
  bool once_{ false };
};
}

#endif  // MAG_UTILS_H
