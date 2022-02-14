// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include <gmock/gmock.h>
#include <mars/core_logic.h>
#include <mars/core_state.h>
#include <mars/data_utils/read_pose_data.h>
#include <mars/data_utils/read_sim_data.h>
#include <mars/sensors/empty/empty_measurement_type.h>
#include <mars/sensors/empty/empty_sensor_class.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include "../include_local/test_data_settings.h"

///
/// \brief mars_e2e_imu_prop_empty_update End to end test with imu measurements, no noise and empty updates
///
/// This test should ensure that a high number of updates inbetween IMU propagations
/// do not result in a big position error compared to pure IMU integration
///
class mars_e2e_imu_prop_empty_update : public testing::Test
{
public:
  bool read_yaml_vec_3(std::vector<double>* value, const std::string& parameter, YAML::Node config)
  {
    if (config[parameter])
    {
      *value = config[parameter].as<std::vector<double>>();

      std::cout << parameter << ": \t [";
      for (auto const& i : *value)
        std::cout << i << " ";

      std::cout << " ]" << std::endl;
      return true;
    }
    return false;
  }

  std::vector<mars::BufferEntryType> gen_empty_meas_entry(std::shared_ptr<mars::EmptySensorClass> sensor_sptr,
                                                          mars::Time t_begin, mars::Time t_end, int rate)
  {
    const double full_time = t_end.get_seconds() - t_begin.get_seconds();
    const volatile int num_empty_meas = abs(static_cast<int>(full_time * rate));
    const volatile double dt = 1 / double(rate);

    std::vector<mars::BufferEntryType> measurement_data_empty;
    measurement_data_empty.resize(num_empty_meas);

    mars::BufferDataType data;
    data.set_sensor_data(std::make_shared<mars::EmptyMeasurementType>(13));

    for (int k = 0; k < num_empty_meas; k++)
    {
      mars::Time t_sensor(t_begin.get_seconds() + dt * k);
      mars::BufferEntryType current_entry(t_sensor, data, sensor_sptr, mars::BufferMetadataType::measurement);
      measurement_data_empty.at(k) = current_entry;
    }

    return measurement_data_empty;
  }
};

TEST_F(mars_e2e_imu_prop_empty_update, END_2_END_IMU_PROPAGATION)
{
  bool test_with_dummy_imu_only_ = false;

  std::string test_data_path = std::string(MARS_LIB_TEST_DATA_PATH);

  // get config
  YAML::Node config = YAML::LoadFile(test_data_path + "parameter.yaml");

  std::string traj_file_name = config["traj_file_name"].as<std::string>();
  std::cout << "Trajectory File: " << traj_file_name << std::endl;

  std::vector<double> imu_n_w;
  std::vector<double> imu_n_bw;
  std::vector<double> imu_n_a;
  std::vector<double> imu_n_ba;

  std::cout << "IMU Noise Parameter: " << std::endl;
  read_yaml_vec_3(&imu_n_w, "imu_n_w", config);
  read_yaml_vec_3(&imu_n_bw, "imu_n_bw", config);
  read_yaml_vec_3(&imu_n_a, "imu_n_a", config);
  read_yaml_vec_3(&imu_n_ba, "imu_n_ba", config);

  // setup propagation sensor
  std::shared_ptr<mars::ImuSensorClass> imu_sensor_sptr = std::make_shared<mars::ImuSensorClass>("IMU");

  // setup the core definition
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>();
  core_states_sptr.get()->set_propagation_sensor(imu_sensor_sptr);
  core_states_sptr.get()->set_noise_std(Eigen::Vector3d(imu_n_w.data()), Eigen::Vector3d(imu_n_bw.data()),
                                        Eigen::Vector3d(imu_n_a.data()), Eigen::Vector3d(imu_n_ba.data()));

  // Empty sensor
  std::shared_ptr<mars::EmptySensorClass> empty_sensor_sptr =
      std::make_shared<mars::EmptySensorClass>("Empty Sensor", core_states_sptr);
  empty_sensor_sptr->const_ref_to_nav_ = true;

  // Define measurement noise
  Eigen::Matrix<double, 1, 1> empty_meas_std;
  empty_meas_std << 0.02;
  empty_sensor_sptr->R_ = empty_meas_std.cwiseProduct(empty_meas_std);

  // Define initial calibration and covariance
  mars::EmptySensorData empty_init_cal;
  empty_init_cal.state_.value_ = Eigen::Vector3d::Zero();

  // The covariance should enclose the initialization with a 3 Sigma bound
  Eigen::Matrix<double, 3, 1> std;
  std << 0.1, 0.1, 0.1;
  empty_init_cal.sensor_cov_ = std.cwiseProduct(std).asDiagonal();

  empty_sensor_sptr->set_initial_calib(std::make_shared<mars::EmptySensorData>(empty_init_cal));

  // load data
  std::vector<mars::BufferEntryType> measurement_data;

  {  // keep individual measurement data limited to this scope
    std::vector<mars::BufferEntryType> measurement_data_imu;
    mars::ReadSimData(&measurement_data_imu, imu_sensor_sptr, test_data_path + traj_file_name);

    measurement_data.insert(measurement_data.end(), measurement_data_imu.begin(), measurement_data_imu.end());

    // generate interfering measurements to the original imu data
    if (test_with_dummy_imu_only_)
    {
      // Add Original IMU data shifted by dt/2 to confirm errors in intermediate propagation
      std::vector<mars::BufferEntryType> measurement_data_imu_2;
      mars::ReadSimData(&measurement_data_imu_2, imu_sensor_sptr, test_data_path + traj_file_name);

      for (int k = 0; k < measurement_data_imu_2.size(); k++)
      {
        measurement_data_imu_2[k].timestamp_ = measurement_data_imu_2[k].timestamp_ + (1 / 200.0) / 2;
      }

      measurement_data.insert(measurement_data.end(), measurement_data_imu_2.begin(), measurement_data_imu_2.end());
    }
    else
    {  // Add empty measurements to ensure that intermediate updates do not cause incorrect propagation
      std::sort(measurement_data_imu.begin(), measurement_data_imu.end());
      mars::Time t_begin = measurement_data_imu.front().timestamp_;
      mars::Time t_end = measurement_data_imu.back().timestamp_;

      std::vector<mars::BufferEntryType> measurement_data_empty(
          gen_empty_meas_entry(empty_sensor_sptr, t_begin + mars::Time((1 / 200.0) / 2), t_end, 200));

      measurement_data.insert(measurement_data.end(), measurement_data_empty.begin(), measurement_data_empty.end());
    }
  }

  // Sort all measurement errors by time before processing
  std::sort(measurement_data.begin(), measurement_data.end());

  // create the CoreLogic and link the core states
  mars::CoreLogic core_logic(core_states_sptr);

  core_logic.buffer_.set_max_buffer_size(500);

  // process data
  for (auto k : measurement_data)
  {
    core_logic.ProcessMeasurement(k.sensor_, k.timestamp_, k.data_);

    if (!core_logic.core_is_initialized_)
    {
      // Initialize the first time at which the propagation sensor occures
      if (k.sensor_ == core_logic.core_states_->propagation_sensor_)
      {
        // Initialize with ground truth
        Eigen::Vector3d p_wi_init(0, 0, 5);
        Eigen::Quaterniond q_wi_init = Eigen::Quaterniond::Identity();
        core_logic.Initialize(p_wi_init, q_wi_init);
      }
      else
      {
        continue;
      }
    }

    // Stop early
    if (k.timestamp_ >= 240)
    {
      break;
    }
  }

  // Print latest state information
  mars::BufferEntryType latest_result;
  core_logic.buffer_.get_latest_state(&latest_result);

  std::cout << "Timestamp: " << latest_result.timestamp_ << std::endl;

  mars::CoreStateType last_state = static_cast<mars::CoreType*>(latest_result.data_.core_.get())->state_;
  Eigen::MatrixXd last_state_cov = static_cast<mars::CoreType*>(latest_result.data_.core_.get())->cov_;

  std::cout << "Last State:" << std::endl;
  std::cout << last_state << std::endl;

  // Print the covariance matrix
  // Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
  // std::cout << last_state_cov.format(OctaveFmt) << std::endl;

  // The true end values are calculated using the matlab framework with the same simulated dataset
  Eigen::Vector3d true_p_wi(-4078.496717743096, 1122.195944724751, 606.096041679294);
  Eigen::Vector3d true_v_wi(-28.432412398458293, 6.316462668407424, 8.525302373674785);
  Eigen::Quaterniond true_q_wi(0.973861319706999, 0.180269162045436, 0.068066505705610, -0.120266828323550);

  std::cout << "p_wi error [m]: [" << (last_state.p_wi_ - true_p_wi).transpose() << " ]" << std::endl;
  std::cout << "v_wi error [m/s]: [" << (last_state.v_wi_ - true_v_wi).transpose() << " ]" << std::endl;

  Eigen::Quaterniond q_wi_error(last_state.q_wi_.conjugate() * true_q_wi);

  std::cout << "q_wi error [w,x,y,z]: [" << q_wi_error.w() << " " << q_wi_error.vec().transpose() << " ]" << std::endl;
  std::cout << "euler angle error [deg]: ["
            << (q_wi_error.toRotationMatrix().eulerAngles(0, 1, 2) * (180 / M_PI)).transpose() << " ]" << std::endl;

  EXPECT_TRUE(last_state.p_wi_.isApprox(true_p_wi, 1e-5));
  EXPECT_TRUE(last_state.v_wi_.isApprox(true_v_wi, 1e-5));
  EXPECT_TRUE(last_state.q_wi_.coeffs().isApprox(true_q_wi.coeffs(), 1e-5));
}
