/*
* Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace qmmf {

enum class BufferFlags {
  kFlagEOS = (1 << 1),
  kFlagCodecConfig = (1 << 2)
};

struct BufferDescriptor {
  void*    data;
  int32_t  fd;
  uint32_t buf_id;
  uint32_t size;
  uint32_t capacity;
  uint32_t offset;
  uint64_t timestamp;
  uint32_t flag;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "data[" << data << "] ";
    stream << "fd[" << fd << "] ";
    stream << "buf_id[" << buf_id << "] ";
    stream << "size[" << size << "] ";
    stream << "capacity[" << capacity << "] ";
    stream << "offset[" << offset << "] ";
    stream << "timestamp[" << timestamp << "] ";
    stream << "flag[" << ::std::setbase(16) << flag << ::std::setbase(10)
           << "]";
    return stream.str();
  }
};

}; // namespace qmmf
