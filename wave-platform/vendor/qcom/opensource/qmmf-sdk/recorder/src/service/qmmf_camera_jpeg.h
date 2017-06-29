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

#include <utils/KeyedVector.h>
#include <utils/Log.h>
#include <libgralloc/gralloc_priv.h>

#include "qmmf-sdk/qmmf_recorder_params.h"
#include "recorder/src/service/qmmf_recorder_common.h"
#include "recorder/src/service/qmmf_camera_reprocess.h"
#include "common/cameraadaptor/qmmf_camera3_device_client.h"

#include "qmmf_jpeg_encoder.h"


namespace qmmf {

namespace recorder {

using namespace jpegencoder;

class CameraJpeg : public Camera3Thread , public ICameraPostProcess {

 public:

  CameraJpeg();

  ~CameraJpeg();

  status_t Create(const int32_t stream_id,
                  const PostProcParam& input,
                  const PostProcParam& output,
                  const uint32_t frame_rate,
                  const uint32_t num_images,
                  const void* static_meta,
                  const PostProcCb& cb,
                  const void* context) override;
  status_t Delete() override;

  void Process(StreamBuffer& in_buffer, StreamBuffer& out_buffer);

  void AddBuff(StreamBuffer in_buffer, StreamBuffer out_buff) override;

  void AddResult(const void* result) override;

  status_t ReturnBuff(StreamBuffer buffer) override;

  status_t GetCapabilities(ReprocCaps *caps) override;

  status_t Start() override;

  status_t Stop();

 private:

  struct Buff {
    StreamBuffer in;
    StreamBuffer out;
  };

  status_t FillMetaInfo(const PostProcParam& input, CameraBufferMetaData* info);

  bool ThreadLoop() override;

  int32_t                input_stream_id_;

  bool                   reprocess_flag_;
  bool                   ready_to_start_;
  uint32_t               num_images_;

  JpegEncoder*           jpeg_encoder_;
  PostProcCb             capture_client_cb_;

  Condition              wait_for_buffer_;
  Mutex                  wait_lock_;
  List<Buff>             input_buffer_;

  static const nsecs_t kFrameTimeout  = 50000000;  // 50 ms.
};

}; //namespace recorder

}; //namespace qmmf
