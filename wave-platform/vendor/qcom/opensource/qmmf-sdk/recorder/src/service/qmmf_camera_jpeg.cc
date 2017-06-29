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

#define TAG "RecorderCameraJpeg"

#include <algorithm>
#include <fcntl.h>
#include <sys/mman.h>

#include "recorder/src/service/qmmf_recorder_utils.h"

#include "qmmf_camera_jpeg.h"

namespace qmmf {

namespace recorder {

CameraJpeg::CameraJpeg()
    : reprocess_flag_(false),
      ready_to_start_(false),
      jpeg_encoder_(nullptr) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

CameraJpeg::~CameraJpeg() {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

int32_t CameraJpeg::Create(const int32_t stream_id,
                           const PostProcParam& input,
                           const PostProcParam& output,
                           const uint32_t frame_rate,
                           const uint32_t num_images,
                           const void* static_meta,
                           const PostProcCb& cb,
                           const void* context) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  if (ready_to_start_) {
    QMMF_ERROR("%s:%s: Failed: Already configured.", TAG, __func__);
    return BAD_VALUE;
  }

  if (reprocess_flag_) {
    QMMF_ERROR("%s:%s: Failed: Wrong state.", TAG, __func__);
    return BAD_VALUE;
  }
  CameraBufferMetaData meta_info;
  memset(&meta_info, 0, sizeof(CameraBufferMetaData));
  if (FillMetaInfo(input, &meta_info) != NO_ERROR) {
    return BAD_VALUE;
  }

  jpeg_encoder_ = JpegEncoder::getInstance();

  capture_client_cb_ = cb;
  input_stream_id_ = stream_id;
  num_images_ = num_images;

  ready_to_start_ = true;

  Run("Camera Jpeg");

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return 55; //TODO use reprocess ID
}

status_t CameraJpeg::GetCapabilities(ReprocCaps *caps) {
  caps->internal_buff = 1;
  // TODO
  return NO_ERROR;
}

status_t CameraJpeg::Start() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  if (!ready_to_start_) {
    return BAD_VALUE;
  }
  reprocess_flag_ = true;

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t CameraJpeg::Stop() {
  return NO_ERROR;
}

status_t CameraJpeg::Delete() {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);

  RequestExitAndWait();

  JpegEncoder::releaseInstance();
  jpeg_encoder_ = nullptr;

  reprocess_flag_ = false;
  ready_to_start_ = false;

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

void CameraJpeg::Process(StreamBuffer& in_buffer,
                         StreamBuffer& out_buffer) {
  QMMF_INFO("%s:%s: %d: Enter ", TAG, __func__, __LINE__);

  void *buf_vaaddr = mmap(nullptr, in_buffer.size, PROT_READ  | PROT_WRITE,
      MAP_SHARED, in_buffer.fd, 0);
  void *out_vaaddr = mmap(nullptr, out_buffer.size, PROT_READ  | PROT_WRITE,
      MAP_SHARED, out_buffer.fd, 0);
  

  if (buf_vaaddr != MAP_FAILED || out_vaaddr != MAP_FAILED) {
    size_t jpeg_size = 0;
    snapshot_info img_buffer;
    img_buffer.img_data[0] = (uint8_t*)buf_vaaddr;
    img_buffer.out_data[0] = (uint8_t*)out_vaaddr;
    img_buffer.source_info = in_buffer.info;
    auto buf_vaddr = jpeg_encoder_->Encode(img_buffer, &jpeg_size);

    memcpy(buf_vaaddr, buf_vaddr, jpeg_size);
    munmap(buf_vaaddr, in_buffer.size);
    munmap(out_vaaddr, out_buffer.size);
    out_buffer.info.plane_info[0].width = jpeg_size;
    out_buffer.data = nullptr;
    out_buffer.filled_length = jpeg_size;
  } else {
    QMMF_INFO("%s:%s: SKIPP JPEG", TAG, __func__);
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

void CameraJpeg::AddBuff(StreamBuffer in_buff, StreamBuffer out_buff) {
  Mutex::Autolock lock(wait_lock_);
  Buff buff;
  buff.in = in_buff;
  buff.out = out_buff;
  input_buffer_.push_back(buff);
  wait_for_buffer_.signal();
}

void CameraJpeg::AddResult(const void* result) {
}

status_t CameraJpeg::ReturnBuff(StreamBuffer buffer) {
  QMMF_INFO("%s:%s: StreamBuffer(0x%p) ts: %lld", TAG,
       __func__, buffer.handle, buffer.timestamp);
  return NO_ERROR;
}


bool CameraJpeg::ThreadLoop() {
  status_t ret = NO_ERROR;
  Buff buffer;
  {
    Mutex::Autolock lock(wait_lock_);
    if (input_buffer_.empty()) {
      ret = wait_for_buffer_.waitRelative(wait_lock_, kFrameTimeout);
      if (ret == TIMED_OUT) {
        QMMF_DEBUG("%s:%s: Wait for frame available timed out", TAG, __func__);
        return true;
      }
    }
    auto iter = input_buffer_.begin();
    buffer = *iter;
    input_buffer_.erase(iter);
  }
  Process(buffer.in, buffer.out);
  capture_client_cb_(buffer.in, buffer.out);
  return true;
}

status_t CameraJpeg::FillMetaInfo(const PostProcParam& input,
                                  CameraBufferMetaData* info) {
  int alignedW = input.width; //todo
  int alignedH = input.height; //todo

  switch (input.format) {
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
    case HAL_PIXEL_FORMAT_YCbCr_420_888:
      info->format = BufferFormat::kNV12;
      info->num_planes = 2;
      info->plane_info[0].width = input.width;
      info->plane_info[0].height = input.height;
      info->plane_info[0].stride = alignedW;
      info->plane_info[0].scanline = alignedH;
      info->plane_info[1].width = input.width;
      info->plane_info[1].height = input.height/2;
      info->plane_info[1].stride = alignedW;
      info->plane_info[1].scanline = alignedH/2;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
      info->format = BufferFormat::kNV12UBWC;
      info->num_planes = 2;
      info->plane_info[0].width = input.width;
      info->plane_info[0].height = input.height;
      info->plane_info[0].stride = alignedW;
      info->plane_info[0].scanline = alignedH;
      info->plane_info[1].width = input.width;
      info->plane_info[1].height = input.height/2;
      info->plane_info[1].stride = alignedW;
      info->plane_info[1].scanline = alignedH/2;
      break;
    case HAL_PIXEL_FORMAT_NV21_ZSL:
      info->format = BufferFormat::kNV21;
      info->num_planes = 2;
      info->plane_info[0].width = input.width;
      info->plane_info[0].height = input.height;
      info->plane_info[0].stride = alignedW;
      info->plane_info[0].scanline = alignedH;
      info->plane_info[1].width = input.width;
      info->plane_info[1].height = input.height/2;
      info->plane_info[1].stride = alignedW;
      info->plane_info[1].scanline = alignedH/2;
      break;
    default:
      QMMF_ERROR("%s:%s: Unsupported format: 0x%x", TAG, __func__,
                 input.format);
      return NAME_NOT_FOUND;
  }

  return NO_ERROR;
}

}; // namespace recoder

}; // namespace qmmf

