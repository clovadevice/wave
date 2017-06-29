/*
* Copyright (c) 2017, The Linux Foundation. All rights reserved.
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

#include <fcntl.h>
#include <dirent.h>
#include <functional>
#include <gtest/gtest.h>
#include <vector>
#include <map>

#include <qmmf-sdk/qmmf_recorder.h>
#include <qmmf-sdk/qmmf_recorder_params.h>

using namespace qmmf;
using namespace recorder;
using namespace android;

class Recorder360Gtest : public ::testing::Test {
 public:
  Recorder360Gtest() : recorder_() {};

  ~Recorder360Gtest() {};

 protected:
  const ::testing::TestInfo* test_info_;

  void SetUp() override;

  void TearDown() override;

  int32_t Init();

  int32_t DeInit();

  void ClearSessions();

  void RecorderCallbackHandler(EventType event_type, void *event_data,
                               size_t event_data_size);

  void SessionCallbackHandler(EventType event_type,
                              void *event_data,
                              size_t event_data_size);

  void CameraResultCallbackHandler(uint32_t camera_id,
                                   const CameraMetadata &result);

  void VideoTrackYUVDataCb(uint32_t track_id, std::vector<BufferDescriptor>
                           buffers, std::vector<MetaData> meta_buffers);

  void VideoTrackOneEncDataCb(uint32_t track_id, std::vector<BufferDescriptor>
                              buffers, std::vector<MetaData> meta_buffers);

  void VideoTrackTwoEncDataCb(uint32_t track_id, std::vector<BufferDescriptor>
                              buffers, std::vector<MetaData> meta_buffers);

  void VideoTrackThreeEncDataCb(uint32_t track_id, std::vector<BufferDescriptor>
                                buffers, std::vector<MetaData> meta_buffers);

  void VideoTrackEventCb(uint32_t track_id, EventType event_type,
                         void *event_data, size_t event_data_size);

  void SnapshotCb(uint32_t camera_id, uint32_t image_sequence_count,
                  BufferDescriptor buffer, MetaData meta_data);

  status_t DumpBitStream(std::vector<BufferDescriptor>& buffers,
                     int32_t file_fd);

  Recorder              recorder_;
  uint32_t              multi_camera_id_;
  uint32_t              multi_camera_type_;
  uint32_t              iteration_count_;
  std::vector<uint32_t> camera_ids_;
  CameraStartParam      multicam_start_params_;
  RecorderCb            recorder_status_cb_;
  int32_t               track1_bitstream_filefd_;
  int32_t               track2_bitstream_filefd_;
  int32_t               track3_bitstream_filefd_;
  std::map <uint32_t , std::vector<uint32_t> > sessions_;
};

