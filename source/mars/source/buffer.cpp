// Copyright (C) 2021 Christian Brommer and Martin Scheiber, Control of Networked Systems, University of Klagenfurt,
// Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <christian.brommer@ieee.org>
// and <martin.scheiber@ieee.org>

#include <mars/buffer.h>
#include <utility>

namespace mars
{
Buffer::Buffer(const int& size)
{
  this->set_max_buffer_size(size);
  std::cout << "Created: Buffer (Size=" << max_buffer_size_ << ")" << std::endl;
}

void Buffer::set_max_buffer_size(const int& size)
{
  max_buffer_size_ = std::abs(size);
}

void Buffer::set_keep_last_sensor_handle(const bool& value)
{
  keep_last_sensor_handle_ = value;
}

int Buffer::get_max_buffer_size() const
{
  return max_buffer_size_;
}

void Buffer::ResetBufferData()
{
  data_.erase(data_.begin(), data_.end());
}

bool Buffer::IsEmpty() const
{
  return data_.empty();
}

int Buffer::get_length() const
{
  return static_cast<int>(data_.size());
}

void Buffer::PrintBufferEntries() const
{
  std::cout << "Idx \t Sensor Name \t Timestamp \t Metadata " << std::endl;

  // iterate forwards
  for (size_t k = 0; k < data_.size(); ++k)
  {
    std::cout << k << "\t " << data_[k] << std::endl;
  }
}

bool Buffer::get_latest_entry(BufferEntryType* entry) const
{
  if (this->IsEmpty())
  {
    return false;
  }

  *entry = data_.back();
  return true;
}

bool Buffer::get_latest_state(BufferEntryType* entry) const
{
  if (this->IsEmpty())
  {
    return false;
  }

  // iterate backwards
  for (auto k = data_.rbegin(); k != data_.rend(); ++k)
  {
    if (k->IsState())
    {
      *entry = *k;
      return true;
    }
  }

  return false;
}

bool Buffer::get_oldest_state(BufferEntryType* entry) const
{
  if (this->IsEmpty())
  {
    return false;
  }

  // iterate forwards
  for (const auto& k : data_)
  {
    if (k.IsState())
    {
      *entry = k;
      return true;
    }
  }

  return false;
}

bool Buffer::get_oldest_core_state(BufferEntryType* entry) const
{
  if (this->IsEmpty())
  {
    return false;
  }

  // iterate forwards (oldest to newest)
  for (const auto& k : data_)
  {
    if (k.metadata_ == mars::BufferMetadataType::core_state)
    {
      *entry = k;
      return true;
    }
  }

  return false;
}

bool Buffer::get_latest_init_state(BufferEntryType* entry) const
{
  if (this->IsEmpty())
  {
    return false;
  }

  // iterate backwards (newest to oldest)
  for (auto k = data_.rbegin(); k != data_.rend(); ++k)
  {
    if (k->IsState())
    {
      if (k->metadata_ == BufferMetadataType::init_state)
      {
        *entry = *k;
        return true;
      }
    }
  }

  return false;
}

bool Buffer::get_latest_sensor_handle_state(const std::shared_ptr<SensorAbsClass>& sensor_handle,
                                            BufferEntryType* entry) const
{
  int none;
  return this->get_latest_sensor_handle_state(sensor_handle, entry, &none);
}

bool Buffer::get_latest_sensor_handle_state(const std::shared_ptr<SensorAbsClass>& sensor_handle,
                                            BufferEntryType* entry, int* index) const
{
  if (this->IsEmpty())
  {
    *index = -1;
    return false;
  }

  // iterate backwards
  for (int k = data_.size() - 1; k >= 0; --k)
  {
    if (data_[k].IsState())
    {
      if (data_[k].sensor_.get() == sensor_handle.get())
      {
        *entry = data_[k];
        *index = k;
        return true;
      }
    }
  }

  *index = -1;
  return false;
}

bool Buffer::get_oldest_sensor_handle_state(const std::shared_ptr<SensorAbsClass>& sensor_handle,
                                            BufferEntryType* entry) const
{
  if (this->IsEmpty())
  {
    return false;
  }

  // iterate forwards (oldest to newest)
  for (const auto& k : data_)
  {
    if (k.IsState())
    {
      if (k.sensor_.get() == sensor_handle.get())
      {
        *entry = k;
        return true;
      }
    }
  }

  return false;
}

bool Buffer::get_latest_sensor_handle_measurement(const std::shared_ptr<SensorAbsClass>& sensor_handle,
                                                  BufferEntryType* entry) const
{
  if (this->IsEmpty())
  {
    return false;
  }

  // iterate backwards (newest to oldest)
  for (auto k = data_.rbegin(); k != data_.rend(); ++k)
  {
    if (k->IsMeasurement())
    {
      if (k->sensor_.get() == sensor_handle.get())
      {
        *entry = *k;
        return true;
      }
    }
  }

  return false;
}

bool Buffer::get_sensor_handle_measurements(const std::shared_ptr<SensorAbsClass>& sensor_handle,
                                            std::vector<const BufferEntryType*>* entries) const
{
  if (this->IsEmpty())
  {
    return false;
  }

  // reset return value
  entries->clear();

  // iterate forwards (oldest to newest)
  for (const auto& k : data_)
  {
    if (k.IsMeasurement())
    {
      if (k.sensor_.get() == sensor_handle.get())
      {
        entries->push_back(&k);
      }
    }
  }

  // return false if vector is emtpy
  return !entries->empty();
}

bool Buffer::get_closest_state(const Time& timestamp, BufferEntryType* entry) const
{
  int none;
  return this->get_closest_state(timestamp, entry, &none);
}

bool Buffer::get_closest_state(const Time& timestamp, BufferEntryType* entry, int* index) const
{
  if (this->IsEmpty())
  {
    *index = -1;
    return false;
  }

  int previous_state_index = -1;  // remains -1 if no state was found
  Time time_distance(1e100);

  bool found_state = false;

  // iterate backwards / start with latest entry
  for (int k = data_.size() - 1; k >= 0; --k)
  {
    if (data_[k].IsState())
    {
      found_state = true;

      Time current_distance = (timestamp - data_[k].timestamp_).abs();

      if (current_distance < time_distance)
      {
        time_distance = current_distance;
      }
      else
      {
        continue;
      }
      previous_state_index = k;
    }
  }

  if (found_state)
  {
    *entry = data_[previous_state_index];
    *index = previous_state_index;
    return true;
  }
  else
  {
    *index = previous_state_index;
    return false;
  }
}

bool Buffer::get_entry_at_idx(const int& index, BufferEntryType* entry) const
{
  if (this->IsEmpty())
  {
    return false;
  }

  if (index < 0)
  {
    return false;
  }

  if (index < this->get_length())
  {
    *entry = data_[index];
    return true;
  }

  return false;
}

bool Buffer::RemoveSensorFromBuffer(const std::shared_ptr<SensorAbsClass>& sensor_handle)
{
  if (this->IsEmpty())
  {
    return false;
  }

  for (int k = 0; k < this->get_length(); k++)
  {
    if (data_[k].sensor_ == sensor_handle)
    {
      *data_.erase(data_.begin() + k);
    }
  }

  return true;
}

int Buffer::AddEntrySorted(const BufferEntryType& new_entry)
{
  int index = InsertDataAtTimestamp(new_entry);

  if (this->get_length() > max_buffer_size_)
  {
    int del_idx = RemoveOverflowEntrys();
    if (del_idx < 0)
    {
      index = -1;
    }
    else
    {
      index -= del_idx < index ? 1 : 0;
    }
  }

  return index;
}

int Buffer::FindClosestTimestamp(const Time& /*timestamp*/) const
{
  // TODO(chb)
  return false;
}

bool Buffer::DeleteStatesStartingAtIdx(const int& idx)
{
  if (this->IsEmpty())
  {
    return false;
  }

  // start deleting from the back to keep the running index valid
  if (idx < this->get_length())
  {
    for (int k = get_length() - 1; k >= idx; --k)
    {
      if (data_[k].IsState())
      {
        *data_.erase(data_.begin() + k);
      }
    }
    return true;
  }

  return false;
}

bool Buffer::IsSorted() const
{
  if (this->IsEmpty())
  {
    return false;
  }

  return std::is_sorted(data_.begin(), data_.end());
}

int Buffer::InsertDataAtTimestamp(const BufferEntryType& new_entry)
{
  if (this->IsEmpty())
  {
    data_.push_back(new_entry);
    // entry is added at idx 0, buffer was empty
    return 0;
  }

  BufferEntryType latest_entry;
  this->get_latest_entry(&latest_entry);
  if (latest_entry <= new_entry)
  {
    data_.push_back(new_entry);
    // get index based on iterator
    return static_cast<int>(data_.end() - data_.begin()) - 1;
  }

  Time previous_time_distance(1e100);
  const Time timestamp = new_entry.timestamp_;

  // iterate backwards and start with latest entry
  // find the first entry at which (state_entry_stamp - new_stamp) is >=0
  // the new entry is entered after this index (idx+1)
  for (int k = data_.size() - 1; k >= 0; --k)
  {
    Time current_time_distance = timestamp - data_[k].timestamp_;

    if (current_time_distance.get_seconds() >= 0)
    {
      data_.insert(data_.begin() + k + 1, new_entry);
      return k + 1;  // return entry index
    }
  }

  // If the buffer has only one element and the new entry is older then the existing entry
  data_.push_front(new_entry);
  return 0;  // push front adds element at index 0
}

bool Buffer::InsertDataAtIndex(const BufferEntryType& new_entry, const int& index)
{
  if (this->get_length() - 1 < index)
  {
    // required index is beyond buffersize, append at the end of the buffer
    data_.push_back(new_entry);
    return true;
  }

  data_.insert(data_.begin() + index, new_entry);
  return true;
}

bool Buffer::InsertIntermediateData(const BufferEntryType& measurement, const BufferEntryType& state)
{
  // Ensure the given data has the right meta data
  if (!(measurement.IsMeasurement() && state.IsState()))
  {
    return false;
  }

  // Check of the latest entry is a state or measurement
  if (data_.back().IsState())
  {
    return false;
  }
  else if (data_.back().IsMeasurement())
  {
    // insert set of data one element before latest
    // change entry meta data to auto generated
    BufferEntryType meas_auto(measurement);
    meas_auto.metadata_ = BufferMetadataType::measurement_auto;

    BufferEntryType state_auto(state);
    state_auto.metadata_ = BufferMetadataType::core_state_auto;

    int last_idx = get_length();
    InsertDataAtIndex(meas_auto, last_idx - 1);
    InsertDataAtIndex(state_auto, last_idx);
  }

  return true;
}

bool Buffer::get_intermediate_entry_pair(const std::shared_ptr<SensorAbsClass>& sensor_handle,
                                         BufferEntryType* imu_state, BufferEntryType* sensor_state) const
{
  BufferEntryType found_sensor_state;
  int found_sensor_state_idx;
  get_latest_sensor_handle_state(sensor_handle, &found_sensor_state, &found_sensor_state_idx);

  // The intermediate IMU state will be located before the found sensor state and the corresponding sensor measurement
  // (idx-2). Ensure that we are accessing valid data.
  const int entry_offset = 2;

  if (found_sensor_state_idx < entry_offset)
  {
    return false;
  }

  size_t interm_core_idx = size_t(found_sensor_state_idx - entry_offset);

  // Ensure that the entrie is a state entry, as expected, and has the same timestamp as the sensor state
  if (data_[interm_core_idx].IsState() && data_[interm_core_idx].timestamp_ == found_sensor_state.timestamp_)
  {
    *sensor_state = found_sensor_state;
    *imu_state = data_[interm_core_idx];
    return true;
  }

  return false;
}

int Buffer::RemoveOverflowEntrys()
{
  // Only delete if buffer would overflow
  if (this->get_length() < this->max_buffer_size_)
  {
    return -1;
  }

  int delete_idx = 0;  // 0 is the oldest index

  for (int k = 0; k < this->get_length() - 1; k++)
  {
    if (data_[delete_idx].IsMeasurement())
    {
      if (CheckForLastSensorHandlePair(data_[delete_idx].sensor_))
      {
        // Is last SensorHandle, dont delete
        delete_idx++;
      }
      else
      {
        bool same_timestamp = data_[delete_idx].timestamp_ == data_[delete_idx + 1].timestamp_;
        bool same_handle = data_[delete_idx].sensor_ == data_[delete_idx + 1].sensor_;

        if (same_timestamp && same_handle)
        {
          *data_.erase(data_.begin() + delete_idx + 1);
          *data_.erase(data_.begin() + delete_idx);
        }
        else
        {
          // Error
        }
        return delete_idx;
      }
    }
    else
    {
      delete_idx++;
    }
  }

  return -1;
}  // namespace mars

bool Buffer::CheckForLastSensorHandlePair(const std::shared_ptr<SensorAbsClass>& sensor_handle) const
{
  int num_found_ms_pair = 0;

  for (auto current_it = data_.begin(); current_it != data_.end() - 1; ++current_it)
  {
    auto next_it = current_it + 1;

    if (current_it->sensor_ == sensor_handle && current_it->IsMeasurement() && next_it->IsState())
    {
      // Additional check
      if (next_it->sensor_ != sensor_handle)
      {
        // Error
      }

      num_found_ms_pair++;
    }

    if (num_found_ms_pair > 2)
    {
      return false;
    }
  }

  if (num_found_ms_pair == 0)
  {
    // Cover the case that no entrie existed
    return false;
  }
  else
  {
    return true;
  }
}
}  // namespace mars
