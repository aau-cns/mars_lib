// Copyright (C) 2020 Christian Brommer, Control of Networked Systems, Universit?t Klagenfurt, Austria
// You can contact the author at <christian.brommer@ieee.org>
//
// All rights reserved.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#include <mars/core_logic.h>
#include <mars/core_state.h>
#include <mars/data_utils/read_pose_data.h>
#include <mars/data_utils/read_sim_data.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars/sensors/pose/pose_sensor_class.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include "include_local/thl_example_data_settings.h"

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

int main(int /*argc*/, char** /*argv[]*/)
{
  std::string test_data_path = std::string(MARS_LIB_THL_EXAMPLE_DATA_PATH);

  // get config
  YAML::Node config = YAML::LoadFile(test_data_path + "parameter.yaml");

  std::string traj_file_name = config["traj_file_name"].as<std::string>();
  std::cout << "Trajectory File: " << traj_file_name << std::endl;

  std::string pose_file_name = config["pose_file_name"].as<std::string>();
  std::cout << "Pose File: " << pose_file_name << std::endl;

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

  // setup additional sensors
  // Pose sensor
  std::shared_ptr<mars::PoseSensorClass> pose_sensor_sptr =
      std::make_shared<mars::PoseSensorClass>("Pose", core_states_sptr);
  pose_sensor_sptr->const_ref_to_nav_ =
      true;  // TODO is set here for now but will be managed by core logic in later versions

  // Define measurement noise
  Eigen::Matrix<double, 6, 1> pose_meas_std;
  pose_meas_std << 0.02, 0.02, 0.02, 2 * (M_PI / 180), 2 * (M_PI / 180), 2 * (M_PI / 180);
  pose_sensor_sptr->R_ = pose_meas_std.cwiseProduct(pose_meas_std);

  // Define initial calibration and covariance
  mars::PoseSensorData pose_init_cal;
  pose_init_cal.state_.p_ip_ = Eigen::Vector3d::Zero();
  pose_init_cal.state_.q_ip_ = Eigen::Quaterniond::Identity();

  // The covariance should enclose the initialization with a 3 Sigma bound
  Eigen::Matrix<double, 6, 1> std;
  std << 0.1, 0.1, 0.1, (10 * M_PI / 180), (10 * M_PI / 180), (10 * M_PI / 180);
  pose_init_cal.sensor_cov_ = std.cwiseProduct(std).asDiagonal();

  pose_sensor_sptr->set_initial_calib(std::make_shared<mars::PoseSensorData>(pose_init_cal));

  // load data
  std::vector<mars::BufferEntryType> measurement_data;

  {  // keep individual measurement data limited to this scope
    std::vector<mars::BufferEntryType> measurement_data_imu;
    mars::ReadSimData(&measurement_data_imu, imu_sensor_sptr, test_data_path + traj_file_name);

    std::vector<mars::BufferEntryType> measurement_data_pose;
    mars::ReadPoseData(&measurement_data_pose, pose_sensor_sptr, test_data_path + pose_file_name);

    measurement_data.insert(measurement_data.end(), measurement_data_imu.begin(), measurement_data_imu.end());
    measurement_data.insert(measurement_data.end(), measurement_data_pose.begin(), measurement_data_pose.end());
  }

  std::sort(measurement_data.begin(), measurement_data.end());

  // generate a single out of order measurement
  // std::swap(measurement_data[31],measurement_data[32]);

  // create the CoreLogic and link the core states
  mars::CoreLogic core_logic(core_states_sptr);

  // Open file for data export
  std::ofstream ofile_core;
  ofile_core.open("/tmp/mars_core_state.csv", std::ios::out);
  ofile_core << std::setprecision(17);

  std::ofstream ofile_pose;
  ofile_pose.open("/tmp/mars_pose_state.csv", std::ios::out);
  ofile_pose << std::setprecision(17);

  std::ofstream ofile_meas;
  ofile_meas.open("/tmp/mars_pose_meas.csv", std::ios::out);
  ofile_meas << std::setprecision(17);

  // process data
  for (auto k : measurement_data)
  {
    core_logic.ProcessMeasurement(k.sensor_handle_, k.timestamp_, k.data_);

    if (!core_logic.core_is_initialized_)
    {
      // Initialize the first time at which the propagation sensor occures
      if (k.sensor_handle_ == core_logic.core_states_->propagation_sensor_)
      {
        Eigen::Vector3d p_wi_init(0, 0, 5);
        Eigen::Quaterniond q_wi_init = Eigen::Quaterniond::Identity();
        core_logic.Initialize(p_wi_init, q_wi_init);
      }
      else
      {
        continue;
      }
    }

    // Store results in a csv file
    if (k.sensor_handle_ == core_logic.core_states_->propagation_sensor_)
    {
      mars::BufferEntryType latest_result;
      core_logic.buffer_.get_latest_state(&latest_result);
      mars::CoreStateType last_state = static_cast<mars::CoreType*>(latest_result.data_.core_state_.get())->state_;
      ofile_core << last_state.to_csv_string(latest_result.timestamp_.get_seconds()) << std::endl;
    }

    if (k.sensor_handle_ == pose_sensor_sptr)
    {
      // Repropagation after an out of order update can cause the latest state to be different from the current update
      // sensor. Using get_latest_sensor_handle_state is the safest option.
      mars::BufferEntryType latest_result;
      core_logic.buffer_.get_latest_sensor_handle_state(pose_sensor_sptr, &latest_result);
      mars::PoseSensorStateType last_state = pose_sensor_sptr->get_state(latest_result.data_.sensor_state_);
      ofile_pose << last_state.to_csv_string(latest_result.timestamp_.get_seconds()) << std::endl;

      // Retreive the last measurement and write it to the measurement file
      mars::BufferEntryType latest_meas;
      core_logic.buffer_.get_latest_sensor_handle_measurement(pose_sensor_sptr, &latest_meas);
      mars::PoseMeasurementType* last_meas =
          static_cast<mars::PoseMeasurementType*>(latest_meas.data_.measurement_.get());
      ofile_meas << last_meas->to_csv_string(k.timestamp_.get_seconds()) << std::endl;
    }
  }

  ofile_core.close();
  ofile_pose.close();
  ofile_meas.close();

  mars::BufferEntryType latest_result;
  core_logic.buffer_.get_latest_state(&latest_result);
  mars::CoreStateType last_state = static_cast<mars::CoreType*>(latest_result.data_.core_state_.get())->state_;

  std::cout << "Last State:" << std::endl;
  std::cout << last_state << std::endl;

  // Define final ground truth values
  Eigen::Vector3d true_p_wi(-20946.817372738657, -3518.039994126535, 8631.1520460773336);
  Eigen::Vector3d true_v_wi(15.924719563070044, -20.483884216740151, 11.455154466026718);
  Eigen::Quaterniond true_q_wi(0.98996033625708202, 0.048830414166879263, -0.02917972697860232, -0.12939345742158029);

  std::cout << "p_wi error [m]: [" << (last_state.p_wi_ - true_p_wi).transpose() << " ]" << std::endl;
  std::cout << "v_wi error [m/s]: [" << (last_state.v_wi_ - true_v_wi).transpose() << " ]" << std::endl;

  Eigen::Quaterniond q_wi_error(last_state.q_wi_.conjugate() * true_q_wi);

  std::cout << "q_wi error [w,x,y,z]: [" << q_wi_error.w() << " " << q_wi_error.vec().transpose() << " ]" << std::endl;

  return 0;
}
