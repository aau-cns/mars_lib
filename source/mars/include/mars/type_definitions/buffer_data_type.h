// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef BUFFERDATATYPE_H
#define BUFFERDATATYPE_H

#include <memory>
#include <utility>

namespace mars
{
/// \brief The BufferDataType binds the core and sensor state in form of a shared void pointer.
///
/// This shared pointer is of type void to allow generalized storage of different sensor and core data types.
/// The need for the versatility of data types is caused by the fact that sensor measurements can not be stored with a
/// common base class since the input and output parameter differ for each class.
/// A high-level class such as the PoseSensorClass needs to keep track of the type such that it can cast the data before
/// it is used.
///
/// An entry always contains a measurement, if has_state is true, then the entry also has a core state.
/// Not all entrys have sensor states.
///
/// \attention Shared pointer should be passed by value to ensure the correct reference count. Further, using the move
/// operator saves two internal atomic copys of the shared pointer and is more efficient.
///
/// \remarks Implementation: Done, Testing: Open
///
class BufferDataType
{
public:
  BufferDataType() = default;

  ///
  /// \brief BufferDataType
  /// \param core this data field holds the core state data
  /// \param sensor this data field holds the sensor state or measurement data
  ///
  /// \note Return/pass smart pointers by value
  ///
  BufferDataType(std::shared_ptr<void> meas)
  {
    set_measurement(meas);
  }

  BufferDataType(std::shared_ptr<void> core, std::shared_ptr<void> sensor)
  {
    set_states(core, sensor);
  }

  inline void set_core_state(std::shared_ptr<void> core)
  {
    core_state_ = core;
    has_core_state_ = true;
  }

  inline void set_sensor_state(std::shared_ptr<void> sensor)
  {
    sensor_state_ = sensor;
    has_sensor_state_ = true;
  }

  inline void set_states(std::shared_ptr<void> core, std::shared_ptr<void> sensor)
  {
    set_core_state(core);
    set_sensor_state(sensor);
  }

  inline void set_measurement(std::shared_ptr<void> meas)
  {
    measurement_ = meas;
  }

  void ClearStates(void)
  {
    core_state_ = nullptr;
    sensor_state_ = nullptr;
    has_core_state_ = false;
    has_sensor_state_ = false;
  }

  inline bool HasCoreStates(void) const
  {
    return has_core_state_;
  }

  inline bool HasSensorStates(void) const
  {
    return has_sensor_state_;
  }

  inline bool HasStates(void) const
  {
    return HasCoreStates() || HasSensorStates();
  }

  std::shared_ptr<void> core_state_{ nullptr };    ///< Core state data
  std::shared_ptr<void> sensor_state_{ nullptr };  ///< Sensor state data
  std::shared_ptr<void> measurement_{ nullptr };   ///< Sensor measurement

private:
  bool has_core_state_ = { false };
  bool has_sensor_state_ = { false };
};
}  // namespace mars
#endif  // BUFFERDATATYPE_H
