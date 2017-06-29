/*
 * Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <iomanip>
#include <string>
#include <sstream>

#include <utils/List.h>
#include <utils/Mutex.h>
#include <utils/KeyedVector.h>
#include <utils/Condition.h>
#include <utils/Log.h>
#include <system/graphics.h>
#include <system/window.h>

#include "common/qmmf_log.h"
#include "qmmf-sdk/qmmf_codec.h"

namespace qmmf {

using namespace android;

const nsecs_t kWaitDelay = 500000000; // 0.5s

struct StreamBuffer {
  CameraBufferMetaData info;
  int64_t timestamp;
  int64_t frame_number;
  uint32_t camera_id;
  uint32_t stream_id;
  android_dataspace data_space;
  buffer_handle_t handle;
  int32_t fd;
  uint32_t size;
  void *data;
  uint32_t flags;
  uint32_t filled_length;
  uint32_t frame_length;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "camera[" << camera_id << "] ";
    stream << "stream[" << stream_id << "] ";
    stream << "data[" << data << "] ";
    stream << "fd[" << fd << "] ";
    stream << "size[" << size << "] ";
    stream << "timestamp[" << timestamp << "] ";
    stream << "flags[" << ::std::setbase(16) << flags << ::std::setbase(10)
           << "]";
    return stream.str();
  }
};

// Thread safe Queue
template <class T>
class TSQueue {
 public:
  typedef typename List<T>::iterator iterator;

  iterator Begin() {
    Mutex::Autolock autoLock(lock_);
    return queue_.begin();
  }

  void PushBack(const T& item) {
    Mutex::Autolock autoLock(lock_);
    queue_.push_back(item);
  }

  int32_t Size() {
    Mutex::Autolock autoLock(lock_);
    return queue_.size();
  }

  bool Empty() {
   Mutex::Autolock autoLock(lock_);
   return queue_.empty();
  }

  iterator End() {
    Mutex::Autolock autoLock(lock_);
    return queue_.end();
  }

  void Erase(iterator it) {
    Mutex::Autolock autoLock(lock_);
    queue_.erase(it);
  }

  void Clear() {
    Mutex::Autolock autoLock(lock_);
    queue_.clear();
  }

 private:
  List<T> queue_;
  Mutex lock_;
};

template <class T>
class SignalQueue {
 public:
  SignalQueue(uint32_t size):cmd_queue_size(size) {
    QMMF_INFO("%s: Enter",__func__);
    QMMF_INFO("%s: Exit",__func__);
  }

  ~SignalQueue() {
    QMMF_INFO("%s: Enter",__func__);
    cmd_queue_size = -1;
    QMMF_INFO("%s: Exit",__func__);
  }

  T Pop() {
    void* item = NULL;
    status_t ret = 0;
    uint32_t size;

    {
      Mutex::Autolock l(cmd_queue_mutex_);
      size = cmd_queue_.size();
    }
    if (size == 0) {
      // wait for signal or for data to come into queue
      Mutex::Autolock l(lock_);
      while (size == 0) {
        ret = wait_for_cmd_.waitRelative(lock_, kWaitDelay);
        if (TIMED_OUT == ret) {
            QMMF_WARN("%s: Wait for cmd.. timed out", __func__);
            {
              Mutex::Autolock l(cmd_queue_mutex_);
              size = cmd_queue_.size();
            }
            continue;
        } else {
            break;
        }
      }
    }
    if (ret == 0) {
      {
        Mutex::Autolock l(cmd_queue_mutex_);
        item = *cmd_queue_.begin();
        cmd_queue_.erase(cmd_queue_.begin());
      }
    }
    return item;
  }

  status_t Push(void* item) {
    uint32_t size;
    {
      Mutex::Autolock l(cmd_queue_mutex_);
      size = cmd_queue_.size();
    }
    if (cmd_queue_size < size) {
      QMMF_ERROR("%s: command queue size full", __func__);
      return -1;
    }
    {
      Mutex::Autolock l(cmd_queue_mutex_);
      cmd_queue_.push_back(item);
    }
    Mutex::Autolock autoLock(lock_);
    wait_for_cmd_.signal();
    return NO_ERROR;
  }

  void Clear() {
    QMMF_INFO("%s: Enter",__func__);
    Mutex::Autolock l(cmd_queue_mutex_);
    cmd_queue_.clear();
    QMMF_INFO("%s: Exit",__func__);
  }

 private:
  Mutex      lock_;
  Condition  wait_for_cmd_;
  Mutex      cmd_queue_mutex_;
  List<T>    cmd_queue_;
  uint32_t   cmd_queue_size;

}; //SignalQueue

// Thread safe KeyedVector
template <class T1, class T2>
class TSKeyedVector {
 public:
  void Add(StreamBuffer& buffer) {
      Mutex::Autolock autoLock(lock_);
      map_.add(buffer.handle, 1);
  }

  uint32_t ValueFor(StreamBuffer& buffer) {
      Mutex::Autolock autoLock(lock_);
      return map_.valueFor(buffer.handle);
  }

  void RemoveItem(StreamBuffer& buffer) {
      Mutex::Autolock autoLock(lock_);
      map_.removeItem(buffer.handle);
  }

  int32_t Size() {
      Mutex::Autolock autoLock(lock_);
      return map_.size();
  }

  bool IsEmpty() {
       Mutex::Autolock autoLock(lock_);
       return map_.isEmpty();
  }

  void ReplaceValueFor(StreamBuffer& buffer, uint32_t value) {
      Mutex::Autolock autoLock(lock_);
      map_.replaceValueFor(buffer.handle, value);
  }

  void Clear() {
      Mutex::Autolock autoLock(lock_);
      map_.clear();
  }

 private:
  DefaultKeyedVector<T1, T2> map_;
  Mutex lock_;
};

}; //namespace qmmf.
