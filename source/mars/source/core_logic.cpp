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
#include <mars/type_definitions/core_type.h>

namespace mars
{
CoreLogic::CoreLogic(std::shared_ptr<CoreState> core_states) : core_states_(move(core_states))
{
  std::cout << "Created: CoreLogic" << std::endl;
}

int CoreLogic::Initialize(const Eigen::Vector3d& p_wi_init, const Eigen::Quaterniond& q_wi_init)
{
  if (buffer_prior_core_init_.IsEmpty())
  {
    std::cout << "CoreLogic: "
              << "No measurements to initialize the filter" << std::endl;
    return false;
  }

  // Get Propagation Sensor Data from Prior Buffer
  BufferEntryType latest_prop_sensor_prior_buffer_entry;

  buffer_prior_core_init_.get_latest_sensor_handle_measurement(core_states_->propagation_sensor_,
                                                               &latest_prop_sensor_prior_buffer_entry);
  // NOTE: Additional modification of the past IMU data such as
  // prefiltering can be done here.
  buffer_.AddEntrySorted(latest_prop_sensor_prior_buffer_entry);

  // Get Propagation Sensor Data from Main Buffer
  BufferEntryType latest_prop_sensor_buffer_entry;
  buffer_.get_latest_sensor_handle_measurement(core_states_->propagation_sensor_, &latest_prop_sensor_buffer_entry);

  // Initialize the state with the latest IMU data, as well as previously set position and orientation (assuming zero
  // velocity at the moment)
  IMUMeasurementType imu_measurement =
      *static_cast<IMUMeasurementType*>(latest_prop_sensor_buffer_entry.data_.sensor_.get());

  CoreType initial_core_state;

  initial_core_state.state_ = core_states_->InitializeState(
      imu_measurement.angular_velocity_, imu_measurement.linear_acceleration_, p_wi_init, Eigen::Vector3d::Zero(),
      q_wi_init, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  initial_core_state.cov_ = core_states_->InitializeCovariance();

  BufferDataType init_core_state_data;
  init_core_state_data.set_core_data(std::make_shared<CoreType>(initial_core_state));

  BufferEntryType new_core_state_entry(latest_prop_sensor_buffer_entry.timestamp_, init_core_state_data,
                                       latest_prop_sensor_buffer_entry.sensor_, mars::BufferMetadataType::init_state);

  buffer_.AddEntrySorted(new_core_state_entry);

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

    if (entry.metadata_ == BufferMetadataType::core_state)
    {
      state_transition = static_cast<CoreType*>(entry.data_.core_.get())->state_transition_ * state_transition;
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

bool CoreLogic::PerformSensorUpdate(BufferEntryType* state_buffer_entry_return, std::shared_ptr<SensorAbsClass> sensor,
                                    const Time& timestamp, std::shared_ptr<BufferDataType> sensor_data)
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

    CoreType latest_core_data = *static_cast<CoreType*>(closest_state_entry.data_.core_.get());

    BufferDataType init_data =
        sensor->Initialize(timestamp, sensor_data->sensor_, std::make_shared<CoreType>(latest_core_data));
    *state_buffer_entry_return = BufferEntryType(timestamp, init_data, sensor, BufferMetadataType::init_state);

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

  CoreType core_prev = *static_cast<CoreType*>(latest_state_buffer_entry.data_.core_.get());
  IMUMeasurementType imu_meas_curr(core_prev.state_.a_m_, core_prev.state_.w_m_);
  BufferDataType interm_prop;
  interm_prop.set_sensor_data(std::make_shared<IMUMeasurementType>(imu_meas_curr));

  mars::BufferEntryType new_core_state_entry;
  new_core_state_entry = PerformCoreStatePropagation(latest_state_buffer_entry.sensor_, timestamp,
                                                     std::make_shared<BufferDataType>(interm_prop),
                                                     std::make_shared<BufferEntryType>(latest_state_buffer_entry));

  // Extract prior information from buffer entry
  CoreType prior_core_data = *static_cast<CoreType*>(new_core_state_entry.data_.core_.get());

  Eigen::MatrixXd prior_sensor_covariance = sensor->get_covariance(prior_sensor_state_entry.data_.sensor_);

  Utils::CheckCov(prior_core_data.cov_, "CoreLogic: Core cov prior");

  // Generate state transition block between prior_sensor_idx and prior_core_idx
  CoreStateMatrix state_transition = GenerateStateTransitionBlock(prior_sensor_idx, prior_core_idx);

  Eigen::MatrixXd prior_cov = PropagateSensorCrossCov(prior_sensor_covariance, prior_core_data.cov_, state_transition);

  NearestCov correct_cov(prior_cov);
  Eigen::MatrixXd corrected_cov = correct_cov.EigenCorrectionUsingCovariance(NearestCovMethod::abs);

  // Perform the sensor update
  BufferDataType corrected_state_data;
  bool successful_update;
  successful_update = sensor->CalcUpdate(timestamp, sensor_data->sensor_, prior_core_data.state_,
                                         prior_sensor_state_entry.data_.sensor_, corrected_cov, &corrected_state_data);

  if (verbose_)
  {
    std::cout << "[CoreLogic]: Perform Sensor Update - DONE" << std::endl;
  }

  if (successful_update)
  {
    // Generate buffer entry and return the corrected states
    *state_buffer_entry_return =
        BufferEntryType(timestamp, corrected_state_data, sensor, BufferMetadataType::sensor_state);

    return true;
  }
  else
  {
    return false;
  }
}

BufferEntryType CoreLogic::PerformCoreStatePropagation(std::shared_ptr<SensorAbsClass> sensor, const Time& timestamp,
                                                       const std::shared_ptr<BufferDataType>& data_measurement,
                                                       const std::shared_ptr<BufferEntryType>& prior_state_entry)
{
  if (verbose_)
  {
    std::cout << "[CoreLogic]: Perform Core State Propagation" << std::endl;
  }

  CoreType prior_core_data = *static_cast<CoreType*>(prior_state_entry->data_.core_.get());
  IMUMeasurementType meas_system_input = *static_cast<IMUMeasurementType*>(data_measurement->sensor_.get());

  const Time current_time = timestamp;
  const Time previous_time = prior_state_entry->timestamp_;
  const Time dt = (current_time - previous_time).abs();

  if (dt == 0)
  {
    std::cout << "Warning: dt for propagation is zero" << std::endl;
  }

  CoreType propagated_core_state;
  propagated_core_state = core_states_->PredictProcessCovariance(prior_core_data, meas_system_input, dt.get_seconds());
  propagated_core_state.state_ =
      core_states_->PropagateState(prior_core_data.state_, meas_system_input, dt.get_seconds());

  BufferDataType buffer_data;
  buffer_data.set_core_data(std::make_shared<CoreType>(propagated_core_state));

  BufferEntryType new_core_state_entry(current_time, buffer_data, sensor, mars::BufferMetadataType::core_state);

  if (verbose_)
  {
    std::cout << "[CoreLogic]: Perform Core State Propagation - DONE" << std::endl;
  }

  return new_core_state_entry;
}

bool CoreLogic::ReworkBufferStartingAtIndex(const int& index)
{
  if (verbose_)
  {
    std::cout << "[CoreLogic]: Rework Buffer Starting At Index" << std::endl;
  }

  assert(index >= 0);

  buffer_.DeleteStatesStartingAtIdx(index);

  // The buffer grows by one index for each additional state,
  // index_offset corrects this
  int index_offset = 0;

  // The buffer size can change during the reiteration. The reiteration needs to be done with the initial size
  // (current_buffer_length).
  const int current_buffer_length = buffer_.get_length();
  for (int k = index; k < current_buffer_length; k++)
  {
    // All states after the "out of order" index have been deleted
    // that's why getting the latest state returns the latest valid
    // state before the out of order measurement at this point

    // get next measurement
    int measurement_entry_idx = k + index_offset;
    int state_insertion_idx = measurement_entry_idx + 1;

    BufferEntryType next_measurement_buffer_entry;
    buffer_.get_entry_at_idx(measurement_entry_idx, &next_measurement_buffer_entry);

    std::shared_ptr<SensorAbsClass> sensor_handle = next_measurement_buffer_entry.sensor_;
    Time timestamp = next_measurement_buffer_entry.timestamp_;
    BufferDataType buffer_data = next_measurement_buffer_entry.data_;

    // Propagate State with System Input from Propagation Sensor
    if (sensor_handle == core_states_->propagation_sensor_)
    {
      // Since the measurement was not out of order, get latest state is valid
      mars::BufferEntryType latest_state_buffer_entry;
      buffer_.get_latest_state(&latest_state_buffer_entry);

      // Zero-order hold for IMU measurement. Copy imu measurement from latest state to current sensor state
      mars::BufferEntryType latest_prop_meas_entry;
      buffer_.get_latest_sensor_handle_state(core_states_->propagation_sensor_, &latest_prop_meas_entry);

      latest_state_buffer_entry.data_.sensor_ = latest_prop_meas_entry.data_.sensor_;

      mars::BufferEntryType new_core_state_entry;
      new_core_state_entry =
          PerformCoreStatePropagation(sensor_handle, timestamp, std::make_shared<BufferDataType>(buffer_data),
                                      std::make_shared<BufferEntryType>(latest_state_buffer_entry));

      buffer_.InsertDataAtIndex(new_core_state_entry, state_insertion_idx);
    }
    else
    {
      // Process non-propagation sensor information
      mars::BufferEntryType new_state_buffer_entry;
      if (!PerformSensorUpdate(&new_state_buffer_entry, sensor_handle, timestamp,
                               std::make_shared<BufferDataType>(buffer_data)))
      {
        return false;
      }

      buffer_.InsertDataAtIndex(new_state_buffer_entry, state_insertion_idx);
    }

    index_offset = index_offset + 1;
  }

  // delete last buffer entry if the max. buffer size is reached
  buffer_.RemoveOverflowEntrys();

  if (verbose_)
  {
    std::cout << "[CoreLogic]: Rework Buffer Starting At Index - DONE" << std::endl;
  }

  return true;
}

bool CoreLogic::ProcessMeasurement(std::shared_ptr<SensorAbsClass> sensor, const Time& timestamp,
                                   const BufferDataType& data)
{
  if (verbose_)
  {
    std::cout << "[CoreLogic]: Process Measurement (" << sensor->name_ << ")" << std::endl;
  }

  // Generate buffer entry element for the measurement
  mars::BufferEntryType new_measurement_buffer_entry(timestamp, data, sensor, mars::BufferMetadataType::measurement);

  // Store measurements prior to the core initialization in the Prior-Buffer
  if (!this->core_is_initialized_)
  {
    if (!core_init_warn_once_)
    {
      std::cout << "Warning: Core is not initialized yet. Measurement is stored but not processed" << std::endl;
      core_init_warn_once_ = true;
    }

    buffer_prior_core_init_.AddEntrySorted(new_measurement_buffer_entry);
    return false;
  }

  // Check if the measurement is out of order
  mars::BufferEntryType latest_buffer_entry;
  buffer_.get_latest_entry(&latest_buffer_entry);

  if (timestamp >= latest_buffer_entry.timestamp_)
  {
    // Measurement is newer than the latest entry - proceed normally
    buffer_.AddEntrySorted(new_measurement_buffer_entry);
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
      std::cout << "Warning: " << sensor.get()->name_ << " Measurement is out of order. "
                << "dt = " << latest_buffer_entry.timestamp_ - timestamp << " "
                << "Latest stamp: " << latest_buffer_entry.timestamp_ << " Measurement stamp: " << timestamp
                << std::endl;
    }

    // Store Measurement as out of order
    mars::BufferEntryType new_ooo_measurement_buffer_entry(timestamp, data, sensor,
                                                           mars::BufferMetadataType::measurement_ooo);

    int out_of_order_buffer_idx = buffer_.AddEntrySorted(new_ooo_measurement_buffer_entry);

    // Reworking the buffer starting at out of order buffer index
    ReworkBufferStartingAtIndex(out_of_order_buffer_idx);

    if (verbose_)
    {
      std::cout << "[CoreLogic]: Process Measurement - DONE" << std::endl;
    }

    return true;
  }

  // Main part to process the measurement
  if (sensor == core_states_->propagation_sensor_)
  {
    // Propagate State with System Input from Propagation Sensor

    // Since the measurement was not out of order, get latest state is valid
    mars::BufferEntryType latest_state_buffer_entry;
    buffer_.get_latest_state(&latest_state_buffer_entry);

    mars::BufferEntryType new_core_state_entry;
    new_core_state_entry = PerformCoreStatePropagation(sensor, timestamp, std::make_shared<BufferDataType>(data),
                                                       std::make_shared<BufferEntryType>(latest_state_buffer_entry));

    buffer_.AddEntrySorted(new_core_state_entry);
  }
  else
  {
    // Process non-propagation sensor information
    mars::BufferEntryType new_state_buffer_entry;
    if (!PerformSensorUpdate(&new_state_buffer_entry, sensor, timestamp, std::make_shared<BufferDataType>(data)))
    {
      return false;
    }

    buffer_.AddEntrySorted(new_state_buffer_entry);
  }

  return true;
}
}  // namespace mars
