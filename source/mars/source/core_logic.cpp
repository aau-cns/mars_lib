// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include <mars/core_logic.h>
#include <mars/general_functions/utils.h>
#include <mars/nearest_cov.h>
#include <mars/sensors/imu/imu_measurement_type.h>
#include <mars/sensors/pose/pose_sensor_class.h>
#include <mars/type_definitions/core_type.h>

namespace mars
{
CoreLogic::CoreLogic(std::shared_ptr<CoreState> core_states) : core_states_(move(core_states))
{
  std::cout << "Created: CoreLogic - Using MaRS Version: " << mars::Utils::get_mars_version_string() << std::endl;
}

int CoreLogic::Initialize(const Eigen::Vector3d& p_wi_init, const Eigen::Quaterniond& q_wi_init)
{
  // Warning if the prior buffer is empty and the filter should be initialized.
  if (buffer_prior_core_init_.IsEmpty())
  {
    std::cout << "CoreLogic: "
              << "Warning: No measurements to initialize the filter" << std::endl;
    return false;
  }

  // Get Propagation Sensor Data from Prior Buffer
  // NOTE: Additional modification of the past IMU data such as prefiltering can be done here.
  BufferEntryType latest_prop_sensor_prior_buffer_entry;
  buffer_prior_core_init_.get_latest_sensor_handle_measurement(core_states_->propagation_sensor_,
                                                               &latest_prop_sensor_prior_buffer_entry);

  // Prepare the first main buffer entry by coping the pre buffer entry, clearing possible states and setting the meta
  BufferEntryType init_main_buffer_entry(latest_prop_sensor_prior_buffer_entry);
  init_main_buffer_entry.ClearStates();
  init_main_buffer_entry.metadata_ = BufferMetadataType::init;

  // Initialize the state with the latest IMU data, as well as previously set position and orientation (assuming zero
  // velocity at the moment)

  // Get IMU measurement
  IMUMeasurementType imu_measurement =
      *static_cast<IMUMeasurementType*>(init_main_buffer_entry.data_.measurement_.get());

  // Set core-states
  CoreType initial_core_state;
  initial_core_state.state_ = core_states_->InitializeState(
      imu_measurement.angular_velocity_, imu_measurement.linear_acceleration_, p_wi_init, Eigen::Vector3d::Zero(),
      q_wi_init, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  initial_core_state.cov_ = core_states_->InitializeCovariance();

  // Generate data element for initial state entry and add it to the existing entry
  init_main_buffer_entry.data_.set_core_state(std::make_shared<CoreType>(initial_core_state));

  buffer_.AddEntrySorted(init_main_buffer_entry);

  core_is_initialized_ = true;
  std::cout << "Info: Filter was initialized" << std::endl;

  return true;
}

CoreStateMatrix CoreLogic::GenerateStateTransitionBlock(const int& first_transition_idx, const int& last_transition_idx)
{
  assert(first_transition_idx >= 0);
  assert(last_transition_idx >= 0);

  CoreStateMatrix state_transition(CoreStateMatrix::Identity());

  for (int k = first_transition_idx; k <= last_transition_idx; k++)
  {
    // loop oldest (f_1) to newest(f_x) trough the buffer
    // Example: f_31 = f_3 * f_2 * f_1 * I

    BufferEntryType entry;
    buffer_.get_entry_at_idx(k, &entry);

    if (entry.HasStates() && (entry.sensor_handle_ == core_states_->propagation_sensor_) &&
        (entry.metadata_ != BufferMetadataType::init))
    {
      CoreStateMatrix entry_st = static_cast<CoreType*>(entry.data_.core_state_.get())->state_transition_;
      state_transition = entry_st * state_transition;
    }
  }

  return state_transition;
}

Eigen::MatrixXd CoreLogic::PropagateSensorCrossCov(const Eigen::MatrixXd& sensor_cov, const CoreStateMatrix& core_cov,
                                                   const CoreStateMatrix& state_transition)
{
  // isolate the right sensor-core cross-covariance entrys
  const int full_cov_size = static_cast<int>(sensor_cov.rows());
  const int core_cov_size = core_states_->state.size_error_;
  const int sensor_cov_start_idx = core_cov_size;
  const int sensor_cov_dim_col = full_cov_size - core_cov_size;
  const int sensor_cov_dim_row = core_cov_size;

  Eigen::MatrixXd old_sensor_cross_cov =
      sensor_cov.block(0, sensor_cov_start_idx, sensor_cov_dim_row, sensor_cov_dim_col);

  // Propagate right sensor-core cross-covariance entrys
  Eigen::MatrixXd new_sensor_cross_cov = state_transition * old_sensor_cross_cov;

  // Fill core states
  Eigen::MatrixXd propagated_cov(sensor_cov);
  propagated_cov.block(0, 0, core_cov_size, core_cov_size) = core_cov;

  // Replace sensor cross-covariance with updated values
  propagated_cov.block(0, sensor_cov_start_idx, sensor_cov_dim_row, sensor_cov_dim_col) = new_sensor_cross_cov;
  propagated_cov.block(sensor_cov_start_idx, 0, sensor_cov_dim_col, sensor_cov_dim_row) =
      new_sensor_cross_cov.transpose();

  return propagated_cov;
}

bool CoreLogic::PerformSensorUpdate(std::shared_ptr<SensorAbsClass> sensor, const Time& timestamp,
                                    BufferEntryType* sensor_data)
{
  bool dummy;
  return PerformSensorUpdate(sensor, timestamp, sensor_data, &dummy);
}

bool CoreLogic::PerformSensorUpdate(std::shared_ptr<SensorAbsClass> sensor, const Time& timestamp,
                                    BufferEntryType* sensor_data, bool* added_interm_state)
{
  if (verbose_)
  {
    std::cout << "[CoreLogic]: Perform Sensor Update (" << sensor->name_ << ")" << std::endl;
  }

  // Check if the sensor was initialized
  if (!sensor->is_initialized_)
  {
    // Sensor was not Initialized, initialize now
    BufferEntryType closest_state_entry;

    if (!buffer_.get_closest_state(timestamp, &closest_state_entry))
    {
      std::cout << "Warning: Could not perform Sensor update. No core state in buffer" << std::endl;
      return false;
    }

    CoreType latest_core_data = *static_cast<CoreType*>(closest_state_entry.data_.core_state_.get());

    BufferDataType init_data =
        sensor->Initialize(timestamp, sensor_data->data_.measurement_, std::make_shared<CoreType>(latest_core_data));

    // TODO the init function should directly receive the state entry and return it
    sensor_data->data_.set_states(init_data.core_state_, init_data.sensor_state_);
    sensor_data->metadata_ = BufferMetadataType::init;

    return true;
  }

  BufferEntryType prior_core_state_entry;
  int prior_core_idx;
  if (!buffer_.get_closest_state(timestamp, &prior_core_state_entry, &prior_core_idx))
  {
    std::cout << "Warning: Could not perform Sensor update. No core state in buffer" << std::endl;
    return false;
  }

  BufferEntryType prior_sensor_state_entry;
  int prior_sensor_idx;
  if (!buffer_.get_latest_sensor_handle_state(sensor, &prior_sensor_state_entry, &prior_sensor_idx))
  {
    std::cout << "Warning: Could not perform Sensor update. No corresponding prior sensor state in buffer" << std::endl;
    return false;
  }

  // Since the measurement was not out of order, get latest state is valid
  mars::BufferEntryType latest_state_buffer_entry;
  buffer_.get_latest_state(&latest_state_buffer_entry);

  // Get latest IMU measurement
  CoreType core_prev = *static_cast<CoreType*>(latest_state_buffer_entry.data_.core_state_.get());
  IMUMeasurementType imu_meas_curr(core_prev.state_.a_m_, core_prev.state_.w_m_);
  BufferDataType interm_prop;
  interm_prop.set_measurement(std::make_shared<IMUMeasurementType>(imu_meas_curr));

  // Copy IMU measurement for zero order hold interpolation
  mars::BufferEntryType interm_buffer_entry(timestamp, interm_prop, core_states_->propagation_sensor_,
                                            mars::BufferMetadataType::auto_add);

  PerformCoreStatePropagation(core_states_->propagation_sensor_, timestamp, latest_state_buffer_entry,
                              &interm_buffer_entry);

  // Extract prior information from buffer entries
  CoreType prior_core_data = *static_cast<CoreType*>(interm_buffer_entry.data_.core_state_.get());
  Utils::CheckCov(prior_core_data.cov_, "CoreLogic: Core cov prior");
  Eigen::MatrixXd prior_sensor_covariance = sensor->get_covariance(prior_sensor_state_entry.data_.sensor_state_);
  CoreStateMatrix state_transition;

  if (add_interm_buffer_entries_)
  {
    // Intermediate IMU Measurement and propergated state
    buffer_.AddEntrySorted(interm_buffer_entry, false);
    *added_interm_state = true;

    if (verbose_)
    {
      std::cout << "[CoreLogic]: Intermediate propergated state was written to buffer." << std::endl;
    }

    // Generate state transition block between prior_sensor_idx and prior_core_idx
    BufferEntryType tmp_entry;
    int prior_core_idx_after_interm;
    buffer_.get_closest_state(timestamp, &tmp_entry, &prior_core_idx_after_interm);
    int prior_sensor_idx_after_interm;
    buffer_.get_latest_sensor_handle_state(sensor, &tmp_entry, &prior_sensor_idx_after_interm);

    state_transition = GenerateStateTransitionBlock(prior_sensor_idx_after_interm, prior_core_idx_after_interm);
  }
  else
  {
    state_transition = GenerateStateTransitionBlock(prior_sensor_idx, prior_core_idx);
    state_transition = prior_core_data.state_transition_ * state_transition;
  }

  Eigen::MatrixXd prior_cov = PropagateSensorCrossCov(prior_sensor_covariance, prior_core_data.cov_, state_transition);

  NearestCov correct_cov(prior_cov);
  Eigen::MatrixXd corrected_cov = correct_cov.EigenCorrectionUsingCovariance(NearestCovMethod::abs);

  // Perform the sensor update
  BufferDataType corrected_state_data;
  bool successful_update;
  successful_update =
      sensor->CalcUpdate(timestamp, sensor_data->data_.measurement_, prior_core_data.state_,
                         prior_sensor_state_entry.data_.sensor_state_, corrected_cov, &corrected_state_data);

  // TODO: This should also happen inside the update class or a preset object should be given that already has the
  // measurement

  if (successful_update)
  {
    sensor_data->data_.set_states(corrected_state_data.core_state_, corrected_state_data.sensor_state_);

    if (verbose_)
    {
      std::cout << "[CoreLogic]: Perform Sensor Update (Successfull) - DONE" << std::endl;
    }

    return true;
  }
  else
  {
    if (verbose_)
    {
      std::cout << "[CoreLogic]: Perform Sensor Update (Rejected) - DONE" << std::endl;
    }

    return false;
  }
}

void CoreLogic::PerformCoreStatePropagation(std::shared_ptr<SensorAbsClass> sensor, const Time& timestamp,
                                            const BufferEntryType& prior_state_entry, BufferEntryType* sensor_entry)
{
  if (verbose_)
  {
    std::cout << "[CoreLogic]: Perform Core State Propagation" << std::endl;
  }

  CoreType prior_core_data = *static_cast<CoreType*>(prior_state_entry.data_.core_state_.get());
  IMUMeasurementType meas_system_input = *static_cast<IMUMeasurementType*>(sensor_entry->data_.measurement_.get());

  const Time current_time = timestamp;
  const Time previous_time = prior_state_entry.timestamp_;
  const Time dt = (current_time - previous_time).abs();

  if (dt == 0 && verbose_)
  {
    std::cout << "Warning: dt for propagation is zero" << std::endl;
  }

  CoreType propagated_core_state;
  propagated_core_state = core_states_->PredictProcessCovariance(prior_core_data, meas_system_input, dt.get_seconds());
  propagated_core_state.state_ =
      core_states_->PropagateState(prior_core_data.state_, meas_system_input, dt.get_seconds());

  sensor_entry->data_.set_core_state(std::make_shared<CoreType>(propagated_core_state));

  if (verbose_)
  {
    std::cout << "[CoreLogic]: Perform Core State Propagation - DONE" << std::endl;
  }
}

void CoreLogic::ReworkBufferStartingAtIndex(const int& index)
{
  if (verbose_)
  {
    std::cout << "[CoreLogic]: Rework Buffer Starting At Index " << index << std::endl;
  }

  assert(index >= 0);

  buffer_.ClearStatesStartingAtIdx(index);

  // get running current index
  int current_state_entry_idx = index;

  // The buffer size can change during the reiteration.
  // Thats why we need to compare for each iteration
  while (current_state_entry_idx < buffer_.get_length())
  {
    // All states after the "out of order" index have been deleted
    // that's why getting the latest state returns the latest valid
    // state before the out of order measurement at this point

    BufferEntryType current_measurement_buffer_entry;
    buffer_.get_entry_at_idx(current_state_entry_idx, &current_measurement_buffer_entry);
    std::shared_ptr<SensorAbsClass> sensor_handle = current_measurement_buffer_entry.sensor_handle_;
    Time timestamp = current_measurement_buffer_entry.timestamp_;

    // Propagate State with System Input from Propagation Sensor
    if (sensor_handle == core_states_->propagation_sensor_)
    {
      mars::BufferEntryType latest_state_buffer_entry;
      buffer_.get_latest_state(&latest_state_buffer_entry);

      PerformCoreStatePropagation(sensor_handle, timestamp, latest_state_buffer_entry,
                                  &current_measurement_buffer_entry);

      buffer_.OverwriteDataAtIndex(current_measurement_buffer_entry, current_state_entry_idx);
      current_state_entry_idx++;
    }
    else
    {
      //  Processing update sensors
      bool added_interm_state = false;
      PerformSensorUpdate(sensor_handle, timestamp, &current_measurement_buffer_entry, &added_interm_state);
      if (added_interm_state)
      {
        current_state_entry_idx++;
      }

      buffer_.OverwriteDataAtIndex(current_measurement_buffer_entry, current_state_entry_idx);
      current_state_entry_idx++;
    }
  }

  if (verbose_)
  {
    std::cout << "[CoreLogic]: Rework Buffer Starting At Index - DONE" << std::endl;
  }
}

bool CoreLogic::ProcessMeasurement(std::shared_ptr<SensorAbsClass> sensor, const Time& timestamp,
                                   const BufferDataType& data)
{
  buffer_.RemoveOverflowEntrys();

  if (verbose_)
  {
    std::cout << "[CoreLogic]: Process Measurement (" << sensor->name_ << ")";
    if (!sensor->do_update_)
    {
      std::cout << ". Sensor is deactivated.";
    }
    std::cout << std::endl;
  }

  if (!sensor->do_update_)
  {
    // Do not perform update for this sensor
    return false;
  }

  // Generate buffer entry element for the measurement
  mars::BufferEntryType new_sensor_entry(timestamp, data, sensor);

  // Store measurements prior to the core initialization in the Prior-Buffer
  if (!this->core_is_initialized_)
  {
    if (!core_init_warn_once_)
    {
      std::cout << "Warning: Core is not initialized yet. Measurement is stored but not processed" << std::endl;
      core_init_warn_once_ = true;
    }

    buffer_prior_core_init_.AddEntrySorted(new_sensor_entry);
    return false;
  }

  // Check if the measurement is out of order
  mars::BufferEntryType latest_buffer_entry;
  buffer_.get_latest_entry(&latest_buffer_entry);

  if (timestamp >= latest_buffer_entry.timestamp_)
  {
    // Measurement is newer than the latest entry - proceed normally
    // Main part to process the measurement
    if (sensor == core_states_->propagation_sensor_)
    {
      // Propagate State with System Input from Propagation Sensor

      // Since the measurement was not out of order, get latest state is valid
      mars::BufferEntryType latest_state_buffer_entry;
      buffer_.get_latest_state(&latest_state_buffer_entry);

      PerformCoreStatePropagation(sensor, timestamp, latest_state_buffer_entry, &new_sensor_entry);

      buffer_.AddEntrySorted(new_sensor_entry);
    }
    else
    {
      if (!PerformSensorUpdate(sensor, timestamp, &new_sensor_entry))
      {
        return false;
      }

      buffer_.AddEntrySorted(new_sensor_entry);
    }

    return true;
  }
  else
  {
    // Measurement is out of order, perform additional checks

    // If the measurement is out of order and from a propagation sensor, discard the measurement
    if (discard_ooo_prop_meas_)
    {
      if (sensor.get() == core_states_->propagation_sensor_.get())
      {
        if (verbose_ || verbose_out_of_order_)
        {
          std::cout << "Warning: Propagation Sensor Measurement is out of order. Discarding measurement. "
                    << timestamp - latest_buffer_entry.timestamp_ << " sec. older" << std::endl;
        }

        return false;
      }
    }

    // Get timestamp of the oldest core state. If the measurement is
    // older, then a state for this sensor can not be generated. Such
    // measurements are discarded.
    mars::BufferEntryType oldest_core_state_buffer_entry;
    buffer_.get_oldest_core_state(&oldest_core_state_buffer_entry);

    if (oldest_core_state_buffer_entry.timestamp_ > timestamp)
    {
      std::cout << "Warning: " << sensor.get()->name_
                << " Measurement is older than oldest core state. Discarding measurement. "
                << timestamp - oldest_core_state_buffer_entry.timestamp_ << " sec. older" << std::endl;
      return false;
    }

    // Discard measurement if it is older than its own, oldest state
    BufferEntryType prior_sensor_state_entry;
    if (buffer_.get_oldest_sensor_handle_state(sensor, &prior_sensor_state_entry))
    {
      // If a sensor handle was found then
      if (timestamp < prior_sensor_state_entry.timestamp_)
      {
        std::cout << "Warning: Measurement is older than its own oldest state. Discarting measurement." << std::endl;
        return false;
      }
    }

    // Discard measurement if older than latest init_state, this
    // would cause a reinitialization of the particular sensor which
    // is not desired.
    mars::BufferEntryType latest_init_state_buffer_entry;
    bool found_latest_init_state = buffer_.get_latest_init_state(&latest_init_state_buffer_entry);

    if (found_latest_init_state)
    {
      if (latest_init_state_buffer_entry.timestamp_ > timestamp)
      {
        std::cout << "Warning: " << sensor.get()->name_
                  << " Measurement is older than latest INIT state. Discarding measurement. "
                  << timestamp - latest_init_state_buffer_entry.timestamp_ << " sec. older" << std::endl;
        return false;
      }
    }

    // Use measurement and perform out of order updates
    if (verbose_ || verbose_out_of_order_)
    {
      std::cout << "Warning: " << sensor.get()->name_ << " Measurement is out of order. \n"
                << "dt to latest buffer = " << latest_buffer_entry.timestamp_ - timestamp << "\n"
                << "dt to time now = " << mars::Time::get_time_now() - timestamp << "\n"
                << "Latest stamp: " << latest_buffer_entry.timestamp_ << " Measurement stamp: " << timestamp
                << std::endl;
    }

    // Store Measurement as out of order
    mars::BufferEntryType new_ooo_measurement_buffer_entry(timestamp, data, sensor,
                                                           mars::BufferMetadataType::out_of_order);

    int out_of_order_buffer_idx = buffer_.AddEntrySorted(new_ooo_measurement_buffer_entry);

    // Reworking the buffer starting at out of order buffer index
    ReworkBufferStartingAtIndex(out_of_order_buffer_idx);

    if (verbose_)
    {
      std::cout << "[CoreLogic]: Process Measurement - DONE" << std::endl;
    }

    return true;
  }
}
}  // namespace mars
