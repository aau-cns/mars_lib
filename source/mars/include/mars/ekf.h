// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef EKF_HPP
#define EKF_HPP

#include <Eigen/Dense>
#include <iostream>

#include <boost/math/distributions/chi_squared.hpp>

namespace mars
{
class Chi2
{
public:
  ///
  /// \brief Chi2 Default constructor
  ///
  Chi2();

  ///
  /// \brief Chi2 Constructor
  /// \param dof Degrees of freedom of the measurement
  /// \param chi_value Chi value to determine the confidence interval
  ///
  Chi2(const int& dof, const double& chi_value);

  ///
  /// \brief set_dof Set degree of freedom for the X2 distribution
  /// \param value Degrees of freedom
  ///
  void set_dof(const int& value);

  ///
  /// \brief set_chi_value Set chi value for the calculation of the upper critical value
  /// \param value Chi value
  ///
  void set_chi_value(const double& value);

  ///
  /// \brief CalculateUcv Perform the calculation of the upper critical value
  ///
  void CalculateUcv();

  ///
  /// \brief ActivateTest Enable or disable the X2 test
  /// \param value True for enabeling, false for disabeling
  ///
  void ActivateTest(const bool& value);

  ///
  /// \brief CalculateChi2 Calculate the X2 value and compare it to the upper critical value (UCV)
  /// \param res Residual
  /// \param S Innovation / Variance of the residual
  /// \return True if the test passed, false if it did not pass
  ///
  bool CalculateChi2(const Eigen::MatrixXd& res, const Eigen::MatrixXd& S);

  ///
  /// \brief PrintReport Print a formated report e.g. if the test did not pass
  /// \param name Name of the sensor, used in the print
  ///
  void PrintReport(std::string name);

  boost::math::chi_squared dist_;  /// Chi2 distribution, generated based on the DoF
  int dof_{ 3 };                   /// Degrees of freedom for the setup
  double chi_value_{ 0.05 };       /// Chi value for the confidence intervall (0.05 represents 95% test)
  double ucv_;                     /// Upper critival value
  bool do_test_{ false };          /// Determine if the test is performed or not
  bool passed_{ false };           /// Shows if the test passed or not (true=passed)

private:
  Eigen::MatrixXd last_res_;  /// Last residual, for the report
  double last_X2_;            /// Last X2 value, for the report
};

class Ekf
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ///
  /// \brief Ekf Essential EFK update component
  /// \param H Jacobian
  /// \param R Measurement noise
  /// \param res Residual
  /// \param P State covariance
  ///
  Ekf(const Eigen::Ref<const Eigen::MatrixXd>& H, const Eigen::Ref<const Eigen::MatrixXd>& R,
      const Eigen::Ref<const Eigen::MatrixXd>& res, const Eigen::Ref<const Eigen::MatrixXd>& P)
  {
    this->H_ = H;
    this->R_ = R;
    this->res_ = res;
    this->P_ = P;
  }

  Eigen::MatrixXd H_;    /// Jacobian
  Eigen::MatrixXd R_;    /// Measurement noise
  Eigen::MatrixXd res_;  /// Residual
  Eigen::MatrixXd P_;    /// State covariance
  Eigen::MatrixXd S_;    /// Innovation / variance of the residual
  Eigen::MatrixXd K_;    /// Kalman gain

  ///
  /// \brief CalculateCorrection Calculating the state correction without a post Chi2 test
  /// \return
  ///
  Eigen::MatrixXd CalculateCorrection();
  ///
  /// \brief CalculateCorrection Calculating the state correction with a post Chi2 test
  /// \param chi2 'Chi2' class based on the sensor measurement
  /// \return
  ///
  Eigen::MatrixXd CalculateCorrection(Chi2& chi2);
  ///
  /// \brief CalculateCovUpdate Updating the state covariance after the state update
  /// \return Updated state covariance matrix
  ///
  Eigen::MatrixXd CalculateCovUpdate();

private:
  ///
  /// \brief CalculateStateCorrection Calculation of EKF components, correction, innovation etc.
  /// \return State correction vector
  ///
  Eigen::MatrixXd CalculateStateCorrection();
};
}  // namespace mars

#endif  // EKF_HPP
