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

#include <inttypes.h>
#include <utils/Errors.h>
#include <utils/String8.h>
#include "recorder/src/service/qmmf_recorder_common.h"
#include "qmmf_time_lapse.h"

using std::vector;

namespace qmmf {

namespace timelapse {

const uint32_t  TimeLapse::kPreviewTrackId = 1;

int32_t TimeLapse::Run() {
  int32_t status;
  uint32_t count = params_.count;

  auto ret = Init();
  if (NO_ERROR != ret) {
    printf("%s: Initialization failed: %d\n", __func__, ret);
    goto EXIT;
  }

  //One initial capture so that snapshot stream is configured
  //and streaming otherwise the first initial call will get
  //delayed significantly and measurements will be skewed.
  ret = CaptureImage(false);
  if (NO_ERROR != ret) {
    printf("%s: Image capture failed: %d\n", __func__, ret);
    goto EXIT;
  } else {
    Mutex::Autolock l(snapshot_lock_);
    snapshot_cond_.wait(snapshot_lock_);
  }

  ret = CreateSession();
  if (NO_ERROR != ret) {
    printf("%s: Session create failed: %d\n", __func__, ret);
    goto DEINIT;
  }

  ret = AddPreviewTrack();
  if (NO_ERROR != ret) {
    printf("%s: Preview track init failed: %d\n", __func__, ret);
    goto DELETE_SESSION;
  }

  ret = StartSession();
  if (NO_ERROR != ret) {
    printf("%s: Session start failed: %d\n", __func__, ret);
    goto DELETE_PREVIEW;
  }

  while (0 < count) {
    Mutex::Autolock l(lapse_lock_);
    lapse_cond_.wait(lapse_lock_);
    ret = CaptureImage();
    if (NO_ERROR != ret) {
      printf("%s: Image capture failed: %d\n", __func__, ret);
      break;
    }
    count--;
  }
  if (NO_ERROR == ret) {
    Mutex::Autolock l(snapshot_lock_);
    while (snapshot_count_ < params_.count) {
      snapshot_cond_.wait(snapshot_lock_);
    }
  }

  status = StopSession();
  if (NO_ERROR != status) {
    printf("%s: Session stop failed: %d\n", __func__, status);
  }

DELETE_PREVIEW:

  status = DeletePreviewTrack();
  if (NO_ERROR != status) {
    printf("%s: Preview track delete failed: %d\n", __func__, status);
  }

DELETE_SESSION:

  status = DeleteSession();
  if (NO_ERROR != status) {
    printf("%s: Session delete failed: %d\n", __func__, status);
  }

DEINIT:

  status = DeInit();
  if (NO_ERROR != status) {
    printf("%s: DeInit failed: %d\n", __func__, status);
  }

EXIT:

  return ret;
}

int32_t TimeLapse::Init() {
  RecorderCb recorder_status = {[] (EventType event_type, void *event_data,
      size_t event_data_size) {}};

  auto ret = recorder_.Connect(recorder_status);
  if (NO_ERROR != ret) {
    printf("%s Connect Failed: %d!!", __func__, ret);
    return ret;
  }

  CameraStartParam camera_start_params;
  memset(&camera_start_params, 0x0, sizeof camera_start_params);
  camera_start_params.zsl_mode         = false;
  camera_start_params.frame_rate       = 30;

  ret = recorder_.StartCamera(params_.camera_id, camera_start_params);
  if (NO_ERROR != ret) {
    printf("%s StartCamera Failed: %d!!", __func__, ret);
    return ret;
  }

  ret = recorder_.GetDefaultCaptureParam(params_.camera_id, static_info_);
  if (NO_ERROR != ret) {
    printf("%s Unable to query default capture parameters!\n", __func__);
  }

  camera_metadata_entry_t entry;
  bool preview_size_supported = false;
  bool snapshot_size_supported = false;
  if (static_info_.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
    entry = static_info_.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
    uint32_t w, h;
    for (uint32_t i = 0 ; i < entry.count; i += 4) {
      if (HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED == entry.data.i32[i]) {
        if (ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
            entry.data.i32[i+3]) {
          w = static_cast<decltype(w)> (entry.data.i32[i+1]);
          h = static_cast<decltype(h)> (entry.data.i32[i+2]);
          if ((params_.preview_width == w) && (params_.preview_height == h)) {
            preview_size_supported = true;
            continue;
          }
        }
      } else if (HAL_PIXEL_FORMAT_BLOB == entry.data.i32[i]) {
        if (ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
            entry.data.i32[i+3]) {
          w = static_cast<decltype(w)> (entry.data.i32[i+1]);
          h = static_cast<decltype(h)> (entry.data.i32[i+2]);
          if ((params_.snapshot_width == w) &&
              (params_.snapshot_height == h)) {
            snapshot_size_supported = true;
            continue;
          }
        }
      }

      if (preview_size_supported && snapshot_size_supported) {
        break;
      }
    }
  }

  if (!preview_size_supported) {
    printf("%s: Preview size %dx%d not supported!\n", __func__,
               params_.preview_width, params_.preview_height);
    return BAD_VALUE;
  }

  if (!snapshot_size_supported) {
    printf("%s: Snapshot size %dx%d not supported!\n", __func__,
               params_.snapshot_width, params_.snapshot_height);
    return BAD_VALUE;
  }

  return NO_ERROR;
}

int32_t TimeLapse::DeInit() {
  auto ret = recorder_.StopCamera(params_.camera_id);
  if (NO_ERROR != ret) {
    printf("%s Camera stop failed: %d!!", __func__, ret);
    return ret;
  }

  return recorder_.Disconnect();
}

void TimeLapse::PreviewTrackHandler(uint32_t track_id,
                                    vector<BufferDescriptor> buffers,
                                    vector<MetaData> meta_buffers) {
  if (!buffers.empty()) {
    if (0 < last_capture_ts_) {
      uint64_t delta = buffers[0].timestamp - last_capture_ts_;
      assert(0 < delta);
      delta = ns2ms(delta);
      if (delta >= params_.period) {
        Mutex::Autolock l(lapse_lock_);
        lapse_cond_.signal();
        last_capture_ts_ = buffers[0].timestamp;
      }
    } else {
      last_capture_ts_ = buffers[0].timestamp;
    }
  }

  recorder_.ReturnTrackBuffer(session_id_, kPreviewTrackId, buffers);
}

int32_t TimeLapse::CreateSession() {
  SessionCb session_status_cb;
  session_status_cb.event_cb = {[] ( EventType event_type, void *event_data,
      size_t event_data_size) {} };

  auto ret = recorder_.CreateSession(session_status_cb, &session_id_);
  assert(session_id_ > 0);

  return ret;
}

int32_t TimeLapse::DeleteSession() {
  auto ret = recorder_.DeleteSession(session_id_);
  session_id_ = 0;
  return ret;
}

int32_t TimeLapse::StartSession() {
  return recorder_.StartSession(session_id_);
}

int32_t TimeLapse::StopSession() {
  return recorder_.StopSession(session_id_, true);
}

int32_t TimeLapse::AddPreviewTrack() {
  VideoTrackCreateParam video_track_param;
  memset(&video_track_param, 0x0, sizeof video_track_param);

  video_track_param.camera_id   = params_.camera_id;
  video_track_param.width       = params_.preview_width;
  video_track_param.height      = params_.preview_height;
  video_track_param.frame_rate  = 30;
  video_track_param.format_type = VideoFormat::kYUV;
  video_track_param.out_device  = 0x1;

  TrackCb video_track_cb;
  video_track_cb.data_cb = { [&] (uint32_t track_id,
      std::vector<BufferDescriptor> buffers,
      std::vector<MetaData> meta_buffers) {
      PreviewTrackHandler(track_id, buffers, meta_buffers);
  } };

  video_track_cb.event_cb = { [] (uint32_t track_id, EventType event_type,
      void *event_data, size_t event_data_size) { } };

  return recorder_.CreateVideoTrack(session_id_, kPreviewTrackId,
                                    video_track_param, video_track_cb);
}

int32_t TimeLapse::DeletePreviewTrack() {
  return recorder_.DeleteVideoTrack(session_id_, kPreviewTrackId);
}

int32_t TimeLapse::CaptureImage(bool store) {
  ImageParam image_param;
  memset(&image_param, 0x0, sizeof image_param);
  image_param.width         = params_.snapshot_width;
  image_param.height        = params_.snapshot_height;
  image_param.image_format  = ImageFormat::kJPEG;
  image_param.image_quality = 95;

  std::vector<CameraMetadata> meta_array;
  ImageCaptureCb cb;
  if (store) {
    cb = {[&] (uint32_t camera_id, uint32_t image_count,
                              BufferDescriptor buffer,
                              MetaData meta_data)
      { SnapshotCb(camera_id, image_count, buffer, meta_data); } };
  } else {
    cb = {[&] (uint32_t camera_id, uint32_t image_count,
                              BufferDescriptor buffer,
                              MetaData meta_data)
      { recorder_.ReturnImageCaptureBuffer(camera_id, buffer);
        Mutex::Autolock l(snapshot_lock_);
        snapshot_cond_.signal();
      } };
  }

  return recorder_.CaptureImage(params_.camera_id, image_param, 1, meta_array,
                                cb);
}

void TimeLapse::SnapshotCb(uint32_t camera_id,
                           uint32_t image_sequence_count,
                           BufferDescriptor buffer, MetaData meta_data) {

  String8 file_path;
  size_t written_len;
  Mutex::Autolock l(snapshot_lock_);

  file_path.appendFormat("/data/time_lapse_%llu.jpg", snapshot_count_);
  FILE *file = fopen(file_path.string(), "w+");
  if (!file) {
    printf("%s: Unable to open file(%s)", __func__,
               file_path.string());
    goto FAIL;
  }

  written_len = fwrite(buffer.data, sizeof(uint8_t), buffer.size, file);
  if (buffer.size != written_len) {
    printf("%s: Bad Write error (%d):(%s)\n", __func__, errno,
               strerror(errno));
    goto FAIL;
  }
  snapshot_count_++;
  snapshot_cond_.signal();

FAIL:
  if (file != NULL) {
    fclose(file);
  }

  // Return buffer back to recorder service.
  recorder_.ReturnImageCaptureBuffer(camera_id, buffer);
}

} //namespace timelapse ends here
} //namespace qmmf ends here

