// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef CORELOGIC_H
#define CORELOGIC_H

#include <mars/buffer.h>
#include <mars/core_state.h>
#include <mars/sensor_manager.h>
#include <mars/type_definitions/core_state_type.h>
#include <Eigen/Dense>
#include <iostream>
#include <memory>

namespace mars
{
///
/// \brief The CoreLogic class represents the high-level logic for the operation of the filter
///
class CoreLogic
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::shared_ptr<CoreState> core_states_{ nullptr };  /// Holds a pointer to the core_states
  Buffer buffer_;                               /// Main buffer of the filter
  Buffer buffer_prior_core_init_;               /// Buffer that holds measurements prior initialization
  SensorManager sensor_manager_;
  bool core_is_initialized_{ false };  /// core_is_initialized_ = true if the core state was initialized, false
                                       /// otherwise
  bool core_init_warn_once_{ false };
  bool verbose_{ false };                    /// Increased output of information
  bool verbose_out_of_order_{ false };       /// Increased output of information for delayed measurements
  bool discard_ooo_prop_meas_{ false };      /// Discard out of order propagation sensor measurements
  bool add_interm_buffer_entries_{ false };  /// Determines if intermediate entries before a sensor update are stored to
                                             /// the buffer

  ///
  /// \brief CoreLogic
  /// \param core_states Core state type used for updates and propagation
  ///
  CoreLogic(std::shared_ptr<CoreState> core_states);
  CoreLogic() = default;

  ///
  /// \brief Initialize the filter with information available in the prior init buffer
  ///
  /// Uses the latest propagation sensor to initialize the core
  /// state and writes the init state to the main buffer.
  /// \note At leased one IMU measurement must exist
  int Initialize(const Eigen::Vector3d& p_wi_init, const Eigen::Quaterniond& q_wi_init);

  ///
  /// \brief GenerateStateTransitionBlock Returns the state transition block between 'first_transition_idx' and
  /// 'last_transition_idx'
  ///
  CoreStateMatrix GenerateStateTransitionBlock(const int& first_transition_idx, const int& last_transition_idx);

  ///
  /// \brief PropagateSensorCrossCov
  ///
  Eigen::MatrixXd PropagateSensorCrossCov(const Eigen::MatrixXd& sensor_cov, const CoreStateMatrix& core_cov,
                                          const CoreStateMatrix& state_transition);

  ///
  /// \brief PerformSensorUpdate Returns new state with corrected state and updated covariance
  ///
  bool PerformSensorUpdate(BufferEntryType* state_buffer_entry_return, std::shared_ptr<SensorAbsClass> sensor,
                           const Time& timestamp, std::shared_ptr<BufferDataType> data);
  ///
  /// \brief PerformCoreStatePropagation Propagates the core state and returns the new state entry
  ///
  /// We know that the current sensor is the input for the
  /// propagation. All that is needed is the past core state
  /// and the input for the system determined by the
  /// propagation sensor handle.
  /// The core_state propagation function needs to be able to
  /// handle the data structure of the propagation sensor.
  ///
  BufferEntryType PerformCoreStatePropagation(std::shared_ptr<SensorAbsClass> sensor, const Time& timestamp,
                                              const std::shared_ptr<BufferDataType>& data_measurement,
                                              const std::shared_ptr<BufferEntryType>& prior_state_entry);
  ///
  /// \brief ReworkBufferStartingAtIndex Reprocesses the buffer after an out of order update,
  /// starting at given 'idx'
  ///
  /// \note It is important that the max buffer size is only
  /// enforced after all states of the current repropagation are
  /// handles. This ensures that the buffer index is valid during
  /// the repropagation. Overflowing old entries are removed after
  /// the repropagation.
  ///
  /// \note The update segment of this method is similar to ProcessMeasurement. However, this method handles buffer
  /// indexes directly and knows exactly where to insert a new state. Thus, this methode can use 'insert state at idx'
  /// and
  /// not 'add state sorted' which increases performance.
  ///
  void ReworkBufferStartingAtIndex(const int& index);

  ///
  /// \brief ProcessMeasurement Processes the sensor input
  ///
  /// \param sensor Pointer to the sensor instance associated with the sensor data
  /// \param timestamp Timestamp associated with the sensor data
  /// \param data Sensor data to process
  ///
  /// This is the primary function that controls the sensor updates.
  /// The function determines filter operations based on the sensor
  /// type (propagation or regular sensor).
  ///
  /// The function generates a measurement handle decides if the
  /// measurement should be used or if it is too old. A check for
  /// out of order measurements is performed, and the measurement is
  /// either processed depending on the sensor handle either
  /// as a propagation sensor or a regular sensor. If the
  /// measurement was out of order; the buffer is reprocessed
  /// starting at the index of the out of order measurement.
  ///
  /// \return True if the processing of the measurement was successful
  ///
  bool ProcessMeasurement(std::shared_ptr<SensorAbsClass> sensor, const Time& timestamp, const BufferDataType& data);
};
}  // namespace mars

#endif  // CORELOGIC_H
