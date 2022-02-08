// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

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

void Buffer::PrintBufferEntrys() const
{
  std::cout << "Idx \t Sensor Name \t Timestamp \t Metadata " << std::endl;

  // iterate forwards
  for (int k = 0; k < data_.size(); ++k)
  {
    std::cout << k << " " << data_[k] << std::endl;
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
  for (int k = data_.size() - 1; k >= 0; --k)
  {
    if (data_[k].IsState())
    {
      *entry = data_[k];
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
  for (int k = 0; k < data_.size(); ++k)
  {
    if (data_[k].IsState())
    {
      *entry = data_[k];
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
  for (int k = 0; k < data_.size(); ++k)
  {
    if (data_[k].metadata_ == mars::BufferMetadataType::core_state)
    {
      *entry = data_[k];
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
  for (int k = data_.size() - 1; k >= 0; --k)
  {
    if (data_[k].IsState())
    {
      if (data_[k].metadata_ == BufferMetadataType::init_state)
      {
        *entry = data_[k];
        return true;
      }
    }
  }

  return false;
}

bool Buffer::get_latest_sensor_handle_state(std::shared_ptr<SensorAbsClass> sensor_handle, BufferEntryType* entry) const
{
  int none;
  return this->get_latest_sensor_handle_state(std::move(sensor_handle), entry, &none);
}

bool Buffer::get_latest_sensor_handle_state(std::shared_ptr<SensorAbsClass> sensor_handle, BufferEntryType* entry,
                                            int* index) const
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

bool Buffer::get_oldest_sensor_handle_state(std::shared_ptr<SensorAbsClass> sensor_handle, BufferEntryType* entry) const
{
  if (this->IsEmpty())
  {
    return false;
  }

  // iterate forwards (oldest to newest)
  for (int k = 0; k < data_.size(); ++k)
  {
    if (data_[k].IsState())
    {
      if (data_[k].sensor_.get() == sensor_handle.get())
      {
        *entry = data_[k];
        return true;
      }
    }
  }

  return false;
}

bool Buffer::get_latest_sensor_handle_measurement(std::shared_ptr<SensorAbsClass> sensor_handle,
                                                  BufferEntryType* entry) const
{
  if (this->IsEmpty())
  {
    return false;
  }

  // iterate backwards (newest to oldest)
  for (int k = data_.size() - 1; k >= 0; --k)
  {
    if (data_[k].IsMeasurement())
    {
      if (data_[k].sensor_.get() == sensor_handle.get())
      {
        *entry = data_[k];
        return true;
      }
    }
  }

  return false;
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

int Buffer::AddEntrySorted(const BufferEntryType& new_entry)
{
  int index = InsertDataAtTimestamp(new_entry);

  if (this->get_length() > max_buffer_size_)
  {
    RemoveOverflowEntrys();
    index = index - 1;
  }

  return index;
}

int Buffer::FindClosestTimestamp(const Time& timestamp) const
{
  // TODO
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

void Buffer::RemoveOverflowEntrys()
{
  if (this->get_length() > this->max_buffer_size_)
  {
    int delete_idx = 0;  // 0 is the oldest index

    // This only keeps sensor states, not measurements or core states
    if (this->keep_last_sensor_handle_ && (data_[delete_idx].metadata_ == BufferMetadataType::sensor_state))
    {
      for (int k = 0; k < this->get_length(); k++)
      {
        if (CheckForLastSensorHandle(data_[delete_idx].sensor_))
        {
          delete_idx++;
        }
        else
        {
          *data_.erase(data_.begin() + delete_idx);
          break;
        }
      }
    }
    else
    {
      *data_.erase(data_.begin() + delete_idx);
    }
  }
}

bool Buffer::CheckForLastSensorHandle(const std::shared_ptr<SensorAbsClass>& sensor_handle)
{
  int num_found_handle = 0;
  int num_found_meas = 0;

  for (int k = 0; k < this->get_length(); k++)
  {
    if (data_[k].sensor_ == sensor_handle)
    {
      if (data_[k].metadata_ == BufferMetadataType::measurement)
      {
        num_found_meas++;

        if (num_found_meas > 1)
        {
          return true;
        }
      }
      else
      {
        if (data_[k].metadata_ == BufferMetadataType::sensor_state)
        {
          num_found_handle++;

          if (num_found_handle > 1 || num_found_meas > 0)
          {
            return false;
          }
        }
      }
    }
  }

  return true;
}
}
