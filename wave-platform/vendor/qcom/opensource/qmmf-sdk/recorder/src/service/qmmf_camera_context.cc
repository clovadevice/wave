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

#define TAG "RecorderCameraContext"

#include <algorithm>
#include <fcntl.h>
#include <sys/mman.h>
#include <QCamera3VendorTags.h>
#include <chrono>

#include "recorder/src/service/qmmf_camera_context.h"
#include "recorder/src/service/qmmf_recorder_utils.h"

using namespace qcamera;

namespace qmmf {

namespace recorder {

//Framerate after which we need to run in constrained mode.
uint32_t CameraContext::kConstrainedModeThreshold = 30;
//Framerate at which batch requests are needed.
uint32_t CameraContext::kHFRBatchModeThreshold = 120;

const nsecs_t CameraContext::kSyncFrameWaitDuration = 500000000; // 500 ms.

CameraContext::CameraContext()
    : camera_id_(-1),
      streaming_request_id_(-1),
      previous_streaming_request_id_(-1),
      snapshot_request_id_(-1),
      snapshot_param_{0, 0, 0, ImageFormat::kJPEG},
      sequence_cnt_(1),
      burst_cnt_(0),
      reprocess_enable_(false),
      result_cb_(nullptr),
      hfr_supported_(false) {
  memset(&camera_start_params_, 0x0, sizeof(camera_start_params_));
}

CameraContext::~CameraContext() {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  if(camera_device_.get()) {
    camera_device_.clear();
    camera_device_ = nullptr;
  }
  //TODO: check all active ports
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

void CameraContext::InitSupportedFPS(const CameraMetadata &static_meta) {
  if (static_meta.exists(ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)) {
    camera_metadata_ro_entry_t entry = static_meta.find(
        ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
    for (size_t i = 0 ; i < entry.count; i += 2) {
      if (entry.data.i32[i] == entry.data.i32[i+1]) {
        supported_fps_.add(entry.data.i32[i]);
      }
    }
  } else {
    QMMF_INFO("%s:%s: Tag ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES "
        " doesn't exist in static metadata", TAG, __func__);
  }
}

bool CameraContext::IsInputSupported(const CameraMetadata &static_meta) {
  if (static_meta.exists(ANDROID_REQUEST_MAX_NUM_INPUT_STREAMS)) {
    camera_metadata_ro_entry entry = static_meta.find(
        ANDROID_REQUEST_MAX_NUM_INPUT_STREAMS);
    if (0 < entry.data.i32[0]) {
      return true;
    }
  }

  return false;
}

status_t CameraContext::CreateSnapshotStream(const ImageParam &param) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  int32_t stream_id = -1;
  int32_t ret = NO_ERROR;

  if (!snapshot_request_.streamIds.isEmpty()) {
    if (1 < snapshot_request_.streamIds.size()) {
      QMMF_ERROR("%s: Several non-zsl snapshot streams present!\n",
                 __func__);
      return BAD_VALUE;
    }
    QMMF_INFO("%s:%s: Deleting Existing Snapshot Stream!!", TAG, __func__);
    ret = DeleteDeviceStream(snapshot_request_.streamIds[0], true);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s: Failed to delete non-zsl snapshot stream: %d\n",
                 __func__, ret);
      return ret;
    }
    snapshot_request_.streamIds.clear();

    if(camera_reprocess_.get() != nullptr) {
      camera_reprocess_->Delete();
    }
  }

  CameraStreamParameters stream_param;
  memset(&stream_param, 0x0, sizeof(stream_param));

  ret = ValidateResolution(param.image_format, param.width, param.height);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: format(0x%x),width(%d):height(%d) Not supported!",
               TAG, __func__, param.image_format, param.width, param.height);
    return ret;
  }

  reprocess_enable_ = IsReprocessNeed(param);

  stream_param.format       = ImageToHalFormat(param.image_format);
  stream_param.width        = param.width;
  stream_param.height       = param.height;
  stream_param.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
  stream_param.cb           = GetStreamCb(param);
  if (reprocess_enable_) {
    stream_param.bufferCount  = sequence_cnt_;
    stream_param.format       = HAL_PIXEL_FORMAT_YCbCr_420_888;
  }

  QMMF_INFO("%s:%s: W(%d) & H(%d)", TAG, __func__, stream_param.width,
            stream_param.height);

  ret = CreateDeviceStream(stream_param, camera_start_params_.frame_rate,
                           &stream_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: Failed creating snapshot stream: %d!",
               TAG, __func__, ret);
    return ret;
  }
  QMMF_INFO("%s:%s Snapshot stream_id(%d)", TAG, __func__, stream_id);
  snapshot_param_ = param;
  snapshot_request_.streamIds.add(stream_id);

  if (reprocess_enable_) {
    CameraMetadata static_meta;
    ReprocParam in, out;
    ReprocessCb cb = [this] (int32_t stream_id, StreamBuffer buffer) -> void
        {SnapshotCaptureCallback(stream_id, buffer);};

    ret = camera_device_->GetCameraInfo(camera_id_, &static_meta);
    assert(ret == NO_ERROR);

    in.width = param.width;
    in.height = param.height;
    in.format = stream_param.format;
    out.width = param.width;
    out.height = param.height;
    out.format = ImageToHalFormat(param.image_format);
    ret = camera_reprocess_->Create(stream_id, in, out,
                                    camera_start_params_.frame_rate,
                                    sequence_cnt_, static_meta, cb);
    assert(ret == NO_ERROR);
  }
  return ret;
}

status_t CameraContext::OpenCamera(const uint32_t camera_id,
                                   const CameraStartParam &param,
                                   const ResultCb &cb) {

  uint32_t ret = NO_ERROR;
  bool match_camera_id = false;
  uint32_t num_camera = 0;
  CameraMetadata static_meta;

  //Setup Camera3DeviceClient callbacks.
  memset(&camera_callbacks_, 0x0, sizeof camera_callbacks_);
  camera_callbacks_.errorCb = [&] (CameraErrorCode error_code,
      const CaptureResultExtras &extras) { CameraErrorCb(error_code, extras);};

  camera_callbacks_.idleCb = [&] () { CameraIdleCb(); };

  camera_callbacks_.peparedCb = [&] (int32_t id) { CameraPreparedCb(id); };

  camera_callbacks_.shutterCb = [&] (const CaptureResultExtras &extras,
      int64_t ts) { CameraShutterCb(extras, ts); };

  camera_callbacks_.resultCb = [&] (const CaptureResult &result)
      { CameraResultCb(result); };

  camera_device_ = new Camera3DeviceClient(camera_callbacks_);
  if(!camera_device_.get()) {
    QMMF_ERROR("%s:%s: Can't Instantiate Camera3DeviceClient", TAG, __func__);
    return NO_MEMORY;
  }
  camera_start_params_ = param;

  camera_reprocess_ = new CameraReprocess(this);
  assert(camera_reprocess_.get() != nullptr);

  ret = camera_device_->Initialize();
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s:%s Unable to Initialize Camera3DeviceClient %d", TAG,
               __func__, ret);
    goto FAIL;
  }

  num_camera = camera_device_->GetNumberOfCameras();
  for(uint32_t i = 0; i < num_camera; i++) {
    if(i == camera_id) {
      match_camera_id = true;
      break;
    }
  }
  if(!match_camera_id) {
    QMMF_ERROR("%s:%s: Invalid Camera Id (%d)", TAG, __func__, camera_id);
    ret = BAD_VALUE;
    goto FAIL;
  }

  ret = camera_device_->OpenCamera(camera_id);
  assert(ret == NO_ERROR);
  camera_id_ = camera_id;

  ret = camera_device_->GetCameraInfo(camera_id, &static_meta);
  assert(ret == NO_ERROR);
  InitSupportedFPS(static_meta);
  assert(!supported_fps_.isEmpty());
  InitHFRModes(static_meta);

  ret = CreateCaptureRequest(snapshot_request_,
                             CAMERA3_TEMPLATE_STILL_CAPTURE);
  assert(ret == NO_ERROR);
  QMMF_INFO("%s:%s: Non-zsl snapshot capture request created successfully!",
      TAG, __func__);

  if (param.zsl_mode) {

    if (!IsInputSupported(static_meta)) {
      QMMF_ERROR("%s:%s: Camera doesn't support input streams!",
                 TAG, __func__);
      ret = BAD_VALUE;
      goto FAIL;
    }

    //The snapshot stream is fixed and matches the ZSL stream
    //size. We cannot re-configure streams dynamically during
    //re-processing as this could have impact on the already
    //cached ZSL buffers and they may fail re-process.
    ImageParam image_param;
    memset(&image_param, 0, sizeof(image_param));
    image_param.width = param.zsl_width;
    image_param.height = param.zsl_height;
    image_param.image_format = ImageFormat::kJPEG;

    ret = CreateSnapshotStream(image_param);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s Failed during snapshot stream setup",
                 TAG, __func__);
      return ret;
    }

    if (streaming_active_requests_.isEmpty()) {
      streaming_active_requests_.push();
    }

    ret = CreateCaptureRequest(streaming_active_requests_.editItemAt(0),
                               CAMERA3_TEMPLATE_ZERO_SHUTTER_LAG);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s: Capture request for ZSL failed!", TAG, __func__);
      return ret;
    }

    int32_t fps_range[2];
    fps_range[0] = param.frame_rate;
    fps_range[1] = param.frame_rate;

    streaming_active_requests_.editItemAt(0).metadata.update(
        ANDROID_CONTROL_AE_TARGET_FPS_RANGE, fps_range, 2);

    CameraStreamParam zsl_param = {};
    zsl_param.cam_stream_dim.width  = param.zsl_width;
    zsl_param.cam_stream_dim.height = param.zsl_height;
    zsl_param.frame_rate            = param.frame_rate;
    zsl_param.low_power_mode        = false;
    zsl_port_ = new ZslPort(const_cast<CameraStreamParam&>(zsl_param), 1,
        CameraPortType::kZSL, this);
    assert(zsl_port_.get() != nullptr);

    auto ret = zsl_port_->Init();
    if(ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: CameraPort is not initialized in ZSL mode!", TAG,
          __func__);
      zsl_port_.clear();
      return BAD_VALUE;
    }

    active_ports_.push_back(zsl_port_);

    ret = zsl_port_->Start(0, nullptr);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: zsl port start failed!", TAG, __func__);
      return ret;
    }
    QMMF_INFO("%s:%s: Number of Active ports=%d", TAG, __func__,
        active_ports_.size());
  }

  result_cb_ = cb;
  sensor_vendor_mode_ = param.getSensorVendorMode();

  return ret;
FAIL:
  camera_device_.clear();
  camera_device_= nullptr;
  return ret;
}

void CameraContext::InitHFRModes(CameraMetadata &static_meta) {
  uint32_t width_offset = 0;
  uint32_t height_offset = 1;
  uint32_t min_fps_offset = 2;
  uint32_t max_fps_offset = 3;
  uint32_t batch_size_offset = 4;
  uint32_t hfr_size = 5;

  camera_metadata_entry meta_entry =
      static_meta.find(ANDROID_REQUEST_AVAILABLE_CAPABILITIES);
  for (uint32_t i = 0; i < meta_entry.count; ++i) {
    uint8_t caps = meta_entry.data.u8[i];
    if (ANDROID_REQUEST_AVAILABLE_CAPABILITIES_CONSTRAINED_HIGH_SPEED_VIDEO ==
        caps) {
      hfr_supported_ = true;
      break;
    }
  }
  if (!hfr_supported_) {
    return;
  }

  meta_entry = static_meta.find(
      ANDROID_CONTROL_AVAILABLE_HIGH_SPEED_VIDEO_CONFIGURATIONS);
  for (uint32_t i = 0; i < meta_entry.count; i += hfr_size) {
    uint32_t width = meta_entry.data.i32[i + width_offset];
    uint32_t height = meta_entry.data.i32[i + height_offset];
    uint32_t min_fps = meta_entry.data.i32[i + min_fps_offset];
    uint32_t max_fps = meta_entry.data.i32[i + max_fps_offset];
    uint32_t batch = meta_entry.data.i32[i + batch_size_offset];
    if (min_fps == max_fps) { //Only constant framerates are supported
      HFRMode_t mode = {width, height, batch, min_fps};
      hfr_batch_modes_list_.add(mode);
    }
  }
}

status_t CameraContext::CloseCamera(const uint32_t camera_id) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  int32_t ret = NO_ERROR;
  assert(camera_id_ == camera_id);
  assert(camera_device_.get() != nullptr);

  if (camera_start_params_.zsl_mode && zsl_port_.get() != nullptr) {
    ZslPort* port = static_cast<ZslPort*>(zsl_port_.get());
    auto ret = port->PauseAndFlushZSLQueue();
    if (ret != NO_ERROR) {
      QMMF_WARN("%s:%s: ZSL queue is not flashed!", TAG, __func__);
      // Even it is not flushed still give a try to Stop it.
    }
    ret = zsl_port_->Stop(0);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s ZSL port stop failed!", TAG, __func__);
      return ret;
    }
  }
  if(camera_reprocess_.get() != nullptr) {
    camera_reprocess_->Delete();
  }

  if (streaming_request_id_ > 0) {
    QMMF_ERROR("%s:%s: Streaming Request still running! delete all tracks "
    "before closing camera", TAG, __func__);
    return INVALID_OPERATION;
  }

  ret = camera_device_->WaitUntilIdle();
  assert(ret == NO_ERROR);

  camera_device_.clear();
  camera_device_ = nullptr;

  QMMF_INFO("%s:%s: Camera Closed Succussfully!", TAG, __func__);
  return ret;
}

int32_t CameraContext::ImageToHalFormat(ImageFormat image_format) {
      int32_t format;
  switch (image_format) {
    case ImageFormat::kJPEG:
      format = HAL_PIXEL_FORMAT_BLOB;
      break;
    case ImageFormat::kNV12:
      format = HAL_PIXEL_FORMAT_YCbCr_420_888;
      break;
    case ImageFormat::kBayerRDI:
      format = HAL_PIXEL_FORMAT_RAW10;
      break;
    case ImageFormat::kBayerIdeal:
      // Not supported.
      QMMF_ERROR("%s:%s ImageFormat::kBayerIdeal is Not supported!", TAG,
          __func__);
      return BAD_VALUE;
      break;
    default:
      format = HAL_PIXEL_FORMAT_BLOB;
      break;
  }
  return format;
}

bool CameraContext::IsReprocessNeed(const ImageParam &param) {
  return ((sequence_cnt_ > 1) && (param.image_format == ImageFormat::kJPEG));
}

void CameraContext::ReprocessCaptureCallback(int32_t stream_id,
                                          StreamBuffer buffer) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  camera_reprocess_->AddBuff(stream_id, buffer);
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

std::function<void(int32_t stream_id, StreamBuffer buffer)>
    CameraContext::GetStreamCb(const ImageParam &param) {
  if (reprocess_enable_) {
    return [=](int32_t stream_id, StreamBuffer buffer)
        { ReprocessCaptureCallback (stream_id, buffer); };
  } else {
    return [=](int32_t stream_id, StreamBuffer buffer)
        { SnapshotCaptureCallback (stream_id, buffer); };
  }
}

status_t CameraContext::CaptureImage(const ImageParam &param,
                                     const uint32_t num_images,
                                     const std::vector<CameraMetadata> &meta,
                                     const StreamSnapshotCb& cb) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  int32_t ret = NO_ERROR;
  client_snapshot_cb_ = cb;
  burst_cnt_ = 0;
  if (!camera_start_params_.zsl_mode) {

    bool reconfigure_needed = (snapshot_param_.width !=
        param.width) ||
        (snapshot_param_.height != param.height) ||
        snapshot_request_.streamIds.isEmpty() ||
        (reprocess_enable_ != IsReprocessNeed(param));

    sequence_cnt_ = num_images;

    QMMF_INFO("%s:%s: reconfigure_needed=%d", TAG, __func__,
        reconfigure_needed);

    if (reconfigure_needed) {
      ret = CreateSnapshotStream(param);
      if (NO_ERROR != ret) {
        QMMF_ERROR("%s:%s Failed during snapshot re-configure",
                   TAG, __func__);
        return ret;
      }
    }

    {
      Mutex::Autolock lock(device_access_lock_);
      int64_t last_frame_mumber;
      uint8_t jpeg_quality = snapshot_param_.image_quality;
      List<Camera3Request> requests;
      std::vector<CameraMetadata>::const_iterator it = meta.begin();
      for (uint32_t i = 0; i < sequence_cnt_; i++) {
        if (it != meta.end()) {
          snapshot_request_.metadata.clear();
          snapshot_request_.metadata.append(*it++);
        }
        snapshot_request_.metadata.update(ANDROID_JPEG_QUALITY, &jpeg_quality,
                                          1);
        requests.push_back(snapshot_request_);
      }
      auto request_id = camera_device_->SubmitRequestList(requests,
                                              false,
                                              &last_frame_mumber);
      assert(request_id >= 0);
      snapshot_request_id_ = request_id;
    }
    QMMF_INFO("%s:%s: Request for non-zsl submitted successfully"
      " request_id(%d)", TAG, __func__, snapshot_request_id_);
  } else {
    ret = CaptureZSLImage(param);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s:CaptureImage Failed in ZSL mode!", TAG, __func__);
      return ret;
    }
  }
  QMMF_VERBOSE("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t CameraContext::CancelCaptureImage() {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  status_t ret = NO_ERROR;

  if (!snapshot_request_.streamIds.isEmpty() && snapshot_request_id_ > -1) {

    std::unique_lock<std::mutex> lock(capture_count_lock_);
    {
      cancel_capture_ = true;
      if (sequence_cnt_ > 0) {
        // Single or Burst capture is not complete yet, wait till pending
        // buffers (for pending count) are returned.
        QMMF_INFO("%s:%s Cancel request with pending buffer(%d)!", TAG,
            __func__, sequence_cnt_);
        int32_t wait_time = sequence_cnt_ * (kSyncFrameWaitDuration/1000000);
        if (capture_count_signal_.wait_for(lock,
            std::chrono::milliseconds(wait_time)) == std::cv_status::timeout) {
          QMMF_ERROR("%s:%s Timed out on Wait", TAG, __func__);
          return UNKNOWN_ERROR;
        }
      }
      assert(sequence_cnt_ == 0);
    }

    ret = DeleteDeviceStream(snapshot_request_.streamIds[0], reprocess_enable_ ?
                             true : false);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s: Failed to delete non-zsl snapshot stream: %d\n",
          __func__, ret);
      return ret;
    }
    snapshot_request_.streamIds.clear();

    if (camera_reprocess_.get() != nullptr) {
      camera_reprocess_->Delete();
    }
  }
  cancel_capture_ = false;
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t CameraContext::CreateStream(const CameraStreamParam& param) {

  QMMF_VERBOSE("%s:%s: Enter", TAG, __func__);
  // 1. Check if streaming request already is going on, if yes then cancel it
  //    and reconfigure it with adding new request.
  // 2. Check for available port where consumer can be attached, if not then
  //    Create new one.
  // 3. Create camera adaptor stream.
  // 4. Create port and link it with adaptor stream.
  // 5. Create producer interface in port and link consumer.

  assert(camera_device_.get() != nullptr);
  assert(param.id != 0);
  size_t batch = 1;

  if ((kConstrainedModeThreshold < param.frame_rate) && (!hfr_supported_)) {
    QMMF_ERROR("%s:%s: Stream tries to enable HFR which is not supported!",
               TAG, __func__);
    return BAD_VALUE;
  }

  if ((kConstrainedModeThreshold < param.frame_rate) &&
      (camera_start_params_.zsl_mode)) {
    QMMF_ERROR("%s:%s: HFR and ZSL are mutually exclusive!",
               TAG, __func__);
    return BAD_VALUE;
  }

  if (kHFRBatchModeThreshold <= param.frame_rate) {
    bool supported = false;
    for (size_t i = 0; i < hfr_batch_modes_list_.size(); i++) {
      if ((param.cam_stream_dim.width == hfr_batch_modes_list_[i].width) &&
          (param.cam_stream_dim.height == hfr_batch_modes_list_[i].height) &&
          (param.frame_rate == hfr_batch_modes_list_[i].framerate)) {
        batch = hfr_batch_modes_list_[i].batch_size;
        supported = true;
        break;
      }
    }

    if (!supported) {
      QMMF_ERROR("%s:%s: HFR stream with size %dx%d fps: %d is not supported!",
                 TAG, __func__, param.cam_stream_dim.width,
                 param.cam_stream_dim.height,
                 param.frame_rate);
      return BAD_VALUE;
    }
  }

  sp<CameraPort> port;
  if (param.low_power_mode) {
    port = new CameraPort(param, batch, CameraPortType::kPreview, this);
  } else {
    port = new CameraPort(param, batch, CameraPortType::kVideo, this);
  }
  assert(port.get() != nullptr);

  auto ret = port->Init();
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CameraPort Can't be Created!", TAG, __func__);
    return BAD_VALUE;
  }

  // Create global streaming capture request, this capture request would be
  // Common to all video/preview and zsl snapshot stream. non zsl snapshot
  // will have separate capture request.
  if (streaming_active_requests_.isEmpty()) {
    streaming_active_requests_.push();
    ret = CreateCaptureRequest(streaming_active_requests_.editItemAt(0),
                               CAMERA3_TEMPLATE_VIDEO_RECORD);
    assert(ret == NO_ERROR);
    QMMF_INFO("%s:%s: Global Streaming Capture request created successfully!",
        TAG, __func__);
  }

  streaming_active_requests_.editItemAt(0).metadata.update(
      QCAMERA3_VENDOR_SENSOR_MODE, &sensor_vendor_mode_, 1);

  // Add port to list of active ports.
  active_ports_.push_back(port);

  QMMF_INFO("%s:%s: Number of Active ports=%d", TAG, __func__,
      active_ports_.size());

  QMMF_VERBOSE("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t CameraContext::DeleteStream(const uint32_t track_id) {

  auto port = GetPort(track_id);
  if (!port) {
    QMMF_ERROR("%s:%s: Invalid track_id = %d", TAG, __func__, track_id);
    return BAD_VALUE;
  }
  assert(port != nullptr);
  if (port->GetNumConsumers() > 0) {
    // Port still being used by another consumer, eventually this port would be
    // deleted once consumers count would become zero.
    return NO_ERROR;
  }

  auto ret = port->DeInit();
  assert(ret == NO_ERROR);

  DeletePort(track_id);

  QMMF_INFO("%s:%s: Camera Port for track_id(%d) deleted", TAG, __func__,
      track_id);

  return ret;
}

status_t CameraContext::StartStream(const uint32_t track_id,
                                    sp<IBufferConsumer>& consumer) {

  auto port = GetPort(track_id);
  if (!port) {
    QMMF_ERROR("%s:%s: Invalid track_id = %d", TAG, __func__, track_id);
    return BAD_VALUE;
  }
  assert(port != nullptr);

  auto ret = port->Start(track_id, consumer);
  assert(ret == NO_ERROR);
  QMMF_INFO("%s:%s: track_id(%d) started on port(0x%p)", TAG, __func__,
      track_id, port);
  return ret;
}

status_t CameraContext::StopStream(const uint32_t track_id) {

  auto port = GetPort(track_id);
  if (!port) {
    QMMF_ERROR("%s:%s: Invalid track_id = %d", TAG, __func__, track_id);
    return BAD_VALUE;
  }
  assert(port != nullptr);

  auto ret = port->Stop(track_id);
  assert(ret == NO_ERROR);
  return ret;
}

status_t CameraContext::SetCameraParam(const CameraMetadata &meta) {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);

  Mutex::Autolock lock(device_access_lock_);
  if ((!streaming_active_requests_.isEmpty()) &&
      (!streaming_active_requests_[0].metadata.isEmpty())) {
    int64_t last_frame_mumber;
    List<Camera3Request> request_list;
    for (size_t i = 0; i < streaming_active_requests_.size(); i++) {
      Camera3Request &req = streaming_active_requests_.editItemAt(i);
      req.metadata.clear();
      req.metadata.append(meta);
      request_list.push_back(req);
    }
    // Submit request with updated camera meta data only if streaming is
    // started, if not then just update default meta data and leave it to
    // startSession -> startStream to submit request.
    if (streaming_request_id_ >= 0) {
      auto ret = camera_device_->SubmitRequestList(request_list, true,
                                                   &last_frame_mumber);
      assert(ret >= 0);
      previous_streaming_request_id_ = streaming_request_id_;
      streaming_request_id_ = ret;
    }
  } else {
    QMMF_ERROR("%s: No active requests present!\n", __func__);
    return NO_INIT;
  }
  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t CameraContext::GetCameraParam(CameraMetadata &meta) {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);
  meta.clear();
  int32_t ret = NO_ERROR;
  if ((!streaming_active_requests_.isEmpty()) &&
      (!streaming_active_requests_[0].metadata.isEmpty())) {
    meta.append(streaming_active_requests_[0].metadata);
  } else {
    QMMF_ERROR("%s:%s No active requests present!\n", TAG, __func__);
    return NO_INIT;
  }
  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t CameraContext::GetDefaultCaptureParam(CameraMetadata &meta) {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);
  CameraMetadata static_meta;
  auto ret = camera_device_->GetCameraInfo(camera_id_, &static_meta);
  assert(ret == NO_ERROR);
  if (!snapshot_request_.metadata.isEmpty()) {
    meta.clear();
    // Append static meta data.
    meta.append(static_meta);
    // Append default snapshot meta data.
    meta.append(snapshot_request_.metadata);
  } else {
    QMMF_WARN("%s:%s Camera is not started Or it is started in zsl mode!\n",
        TAG, __func__);
    ret = NO_INIT;
  }
  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t CameraContext::ReturnImageCaptureBuffer(const uint32_t camera_id,
                                                 const int32_t buffer_id) {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);
  if (snapshot_buffer_list_.indexOfKey(buffer_id) < 0) {
    QMMF_ERROR("%s:%s: buffer_id(%u) is not valid!!", TAG, __func__, buffer_id);
    return BAD_VALUE;
  }

  StreamBuffer buffer = snapshot_buffer_list_.valueFor(buffer_id);
  assert(buffer.fd == buffer_id);


  if (snapshot_buffer_stream_list_.indexOfKey(buffer_id) < 0) {
    QMMF_ERROR("%s:%s: buffer_id(%u) is not valid!!", TAG, __func__, buffer_id);
    return BAD_VALUE;
  }
  int32_t stream_id = snapshot_buffer_stream_list_.valueFor(buffer_id);

  QMMF_DEBUG("%s:%s: stream_id(%d):stream_buffer(0x%p):ion_fd(%d)"
      " returned back!", TAG, __func__, stream_id, buffer.handle, buffer_id);

  auto ret = ReturnStreamBuffer(stream_id, buffer);

  QMMF_DEBUG("%s:%s: ret %d", TAG, __func__, ret);
  assert(ret == NO_ERROR);

  snapshot_buffer_list_.removeItem(buffer_id);
  snapshot_buffer_stream_list_.removeItem(buffer_id);

  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return ret;
}

CameraStartParam& CameraContext::GetCameraStartParam() {

  return camera_start_params_;
}

Vector<int32_t>& CameraContext::GetSupportedFps() {

  return supported_fps_;
}

status_t CameraContext::CreateDeviceStream(CameraStreamParameters& params,
                                           uint32_t frame_rate,
                                           int32_t* stream_id) {

  Mutex::Autolock lock(device_access_lock_);
  QMMF_VERBOSE("%s:%s: Enter", TAG, __func__);

  int32_t ret = NO_ERROR;
  assert(camera_device_.get() != nullptr);

  if (camera_start_params_.zsl_mode && zsl_port_.get() != nullptr) {
    ZslPort* zsl_port = static_cast<ZslPort*>(zsl_port_.get());
    if (zsl_port->IsRunning()) {
      QMMF_INFO("%s:%s: ZSL is running, pause and flush queue!", TAG,
          __func__);
      ret = zsl_port->PauseAndFlushZSLQueue();
      if (ret != NO_ERROR) {
        QMMF_ERROR("%s:%s: zsl queue is not flashed!", TAG, __func__);
        return ret;
      }
    }
  }

  // Configure is required only once, if streaming request is already submitted
  // then BeginConfigure is not required to be called, stream can be created
  // without calling it.
  if (streaming_request_id_ < 0) {
    ret = camera_device_->BeginConfigure();
    assert(ret == NO_ERROR);
  }

  int32_t id;
  id = camera_device_->CreateStream(params);
  if (id < 0) {
    QMMF_INFO("%s:%s: createStream failed!!", TAG, __func__);
    return BAD_VALUE;
  }
  *stream_id = id;

  // At this point stream is created but it is not added to request, it will be
  // added once corresponding port will get the start cmd from it's consumer.
  if (streaming_request_id_ < 0) {
    bool is_constrained_mode = false;
    if (hfr_supported_) {
      for (auto iter : active_ports_) {
        if (kConstrainedModeThreshold < iter->GetPortFramerate()) {
          is_constrained_mode = true;
          break;
        }
      }
      if (!is_constrained_mode && (kConstrainedModeThreshold < frame_rate)) {
        is_constrained_mode = true;
      }
    }
    QMMF_INFO("%s:%s: is_constrained_mode(%d)", TAG, __func__,
        is_constrained_mode);
    ret = camera_device_->EndConfigure(is_constrained_mode);
    assert(ret == NO_ERROR);
  }

  if (camera_start_params_.zsl_mode && zsl_port_.get() != nullptr) {
    ZslPort* zsl_port = static_cast<ZslPort*>(zsl_port_.get());
    QMMF_INFO("%s:%s: Resume ZSL!", TAG, __func__);
    zsl_port->ResumeZSL();
  }

  QMMF_VERBOSE("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t CameraContext::CreateDeviceInputStream(
    CameraInputStreamParameters& params, int32_t* stream_id) {
  Mutex::Autolock lock(device_access_lock_);
  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  int32_t ret = NO_ERROR;
  assert(camera_device_.get() != nullptr);

  // Configure is required only once, if streaming request is already submitted
  // then BeginConfigure is not required to be called, stream can be created
  // without calling it.
  if (streaming_request_id_ < 0) {
    ret = camera_device_->BeginConfigure();
    assert(ret == NO_ERROR);
  }

  int32_t id;
  id = camera_device_->CreateInputStream(params);
  if (id < 0) {
    QMMF_INFO("%s:%s: createStream failed!!", TAG, __func__);
    return BAD_VALUE;
  }
  *stream_id = id;

  // At this point stream is created but it is not added to request, it will be
  // added once corresponding port will get the start cmd from it's consumer.
  if (streaming_request_id_ < 0) {
    ret = camera_device_->EndConfigure();
    assert(ret == NO_ERROR);
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t CameraContext::DeleteDeviceStream(int32_t stream_id, bool cache) {

  QMMF_VERBOSE("%s:%s: Enter", TAG, __func__);
  status_t ret = NO_ERROR;
  int64_t last_frame_mumber;
  assert(camera_device_.get() != nullptr);

  bool resume_streaming = false;
  if (camera_start_params_.zsl_mode && zsl_port_.get() != nullptr
      && (0 <= streaming_request_id_)) {

    ZslPort* zsl_port = static_cast<ZslPort*>(zsl_port_.get());
    if (zsl_port->IsRunning()) {
      QMMF_INFO("%s:%s: ZSL is running, pause and flush queue!", TAG,
        __func__);
      auto ret = zsl_port->PauseAndFlushZSLQueue();
      if (ret != NO_ERROR) {
        QMMF_ERROR("%s:%s: zsl queue is not flashed!", TAG, __func__);
        return ret;
      }
      QMMF_INFO("%s:%s: Cancelling Request!!", TAG, __func__);
      ret = CancelRequest();
      if (NO_ERROR != ret) {
        QMMF_ERROR("%s:%s Cancel request failed:%d", TAG, __func__, ret);
        return ret;
      }
      resume_streaming = true;
    }
  }

  Mutex::Autolock lock(device_access_lock_);

  if (camera_start_params_.zsl_mode && zsl_port_.get() != nullptr) {
    ret = camera_device_->BeginConfigure();
    assert(ret == NO_ERROR);
  }

  ret = camera_device_->DeleteStream(stream_id, cache);
  assert(ret == NO_ERROR);
  QMMF_INFO("%s:%s: Camera Device Stream(%d) deleted successfully!", TAG,
      __func__, stream_id);

  if (camera_start_params_.zsl_mode && zsl_port_.get() != nullptr) {
    ret = camera_device_->EndConfigure();
    assert(ret == NO_ERROR);

    ZslPort* zsl_port = static_cast<ZslPort*>(zsl_port_.get());
    zsl_port->ResumeZSL();

    if (resume_streaming) {
      ret = camera_device_->SubmitRequest(streaming_active_requests_[0], true,
                                          &last_frame_mumber);
      assert(ret >= 0);
      if (streaming_request_id_ > -1) {
        previous_streaming_request_id_ = streaming_request_id_;
      }
      streaming_request_id_ = ret;
      ret = NO_ERROR;
    }
  }

  QMMF_VERBOSE("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t CameraContext::CreateCaptureRequest(Camera3Request& request,
                                             camera3_request_template_t
                                             template_type) {

  Mutex::Autolock lock(device_access_lock_);

  auto ret = camera_device_->CreateDefaultRequest(template_type,
      &request.metadata);
  assert(ret == NO_ERROR);
  return ret;
}

status_t CameraContext::UpdateRequest(bool is_streaming) {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);
  int32_t ret = NO_ERROR;
  uint32_t max_fps = 0;
  Vector<int32_t> removed_streams;

  //Get all camera stream ids from all active ports which are ready to start.
  size_t size = active_ports_.size();
  QMMF_INFO("%s:%s: Number of active_ports(%d)", TAG, __func__, size);

  for (size_t i = 0; i < size; i++) {
    sp<CameraPort> port = active_ports_[i];
    assert(port != nullptr);

    int32_t cam_stream_id = port->GetCameraStreamId();
    size_t batch_size = port->GetPortBatchSize();
    QMMF_INFO("%s:%s: cam_stream_id(%d)", TAG, __func__, cam_stream_id);
    if (port->getPortState() == PortState::PORT_READYTOSTART) {

      QMMF_INFO("%s:%s: CameraPort(0x%p):camera_stream_id(%d) is ready to"
          " start!", TAG, __func__, port.get(), cam_stream_id);
      if (max_fps < port->GetPortFramerate()) {
        max_fps = port->GetPortFramerate();
      }
      if (batch_size > streaming_active_requests_.size()) {
        streaming_active_requests_.resize(batch_size);
      }
      for (size_t i = 0; i < batch_size; i++) {
        streaming_active_requests_.editItemAt(i).streamIds.add(cam_stream_id);
        if ((1 < i) && (streaming_active_requests_[i].metadata.isEmpty())) {
          assert(!streaming_active_requests_[0].metadata.isEmpty());
          streaming_active_requests_.editItemAt(i).metadata.append(
              streaming_active_requests_[0].metadata);
        }
      }
    } else if (port->getPortState() == PortState::PORT_READYTOSTOP) {

      QMMF_INFO("%s:%s: CameraPort(0x%p):camera_stream_id(%d) is stopped ",
          TAG, __func__, port.get(), cam_stream_id);
      // Check if camera stream is already part of request, if yes then remove
      // it from request list. if not then it means stream is created but its
      // corresponding port is not started yet.
      QMMF_INFO("%s:%s: streaming_active_requests_.size(%d)", TAG, __func__,
          streaming_active_requests_.size());
      for (size_t j = 0; j < streaming_active_requests_.size(); j++) {
        Camera3Request &req = streaming_active_requests_.editItemAt(j);
        bool match = false;
        size_t idx = -1;
        QMMF_INFO("%s:%s: req.streamIds.size(%d)", TAG, __func__,
            req.streamIds.size());
        for (size_t i = 0; i < req.streamIds.size(); i++) {
          if (cam_stream_id == req.streamIds[i]) {
            match = true;
            idx = i;
            break;
          }
        }
        if(match == true) {
          req.streamIds.removeAt(idx);
          QMMF_INFO("%s:%s: cam_stream_id(%d) removed from Request!", TAG,
                      __func__, cam_stream_id);
          bool is_present = false;
          for (size_t j = 0; j < removed_streams.size(); j++) {
            if (removed_streams[j] == cam_stream_id) {
              is_present = true;
              break;
            }
          }
          if (!is_present) {
            removed_streams.add(cam_stream_id);
          }
          QMMF_INFO("%s:%s: removed_streams.size(%d)", TAG, __func__,
              removed_streams.size());
        }
      }
    } else if (port->getPortState() == PortState::PORT_STARTED) {
      if (max_fps < port->GetPortFramerate()) {
        max_fps = port->GetPortFramerate();
      }
    }
  }

  bool stale_batches_present = false;
  ssize_t stale_idx = -1;
  size_t stale_count = 0;
  //Check for any stale batch requests and remove if present
  for (size_t i = 1; i < streaming_active_requests_.size(); i++) {
    if(streaming_active_requests_[i].streamIds.isEmpty()) {
      if (!stale_batches_present) {
        stale_batches_present = true;
        stale_idx = i;
      }
      stale_count++;
    } else {
      assert(!stale_batches_present);
    }
  }

  if (stale_batches_present) {
    streaming_active_requests_.removeItemsAt(stale_idx, stale_count);
  }
  size = streaming_active_requests_[0].streamIds.size();
  QMMF_INFO("%s:%s: Number of streams(%d) to start", TAG, __func__, size);
  if (size == 0) {
    QMMF_INFO("%s:%s:Cancelling the request, no pending stream!", TAG, __func__);
    ret = CancelRequest();
    assert (ret == NO_ERROR);
    removed_streams.clear();
    return ret;
  }

  {
    Mutex::Autolock lock(device_access_lock_);
    if (0 < max_fps) {
      int32_t fpsRange[2];
      fpsRange[0] = max_fps;
      fpsRange[1] = max_fps;

      for (size_t i = 0; i < streaming_active_requests_.size(); i++) {
        streaming_active_requests_.editItemAt(i).metadata.update(
            ANDROID_CONTROL_AE_TARGET_FPS_RANGE, fpsRange, 2);
      }
    }
    List<Camera3Request> request_list;
    for (size_t i = 0; i < streaming_active_requests_.size(); i++) {
      request_list.push_back(streaming_active_requests_[i]);
      assert(!streaming_active_requests_[i].metadata.isEmpty());
    }
    Mutex::Autolock sync_lock(sync_frame_lock_);
    if (!removed_streams.isEmpty()) {
      sync_frame_.stream_ids.clear();
      sync_frame_.stream_ids.appendVector(removed_streams);
    }
    auto ret = camera_device_->SubmitRequestList(request_list, is_streaming,
                                                 &sync_frame_.last_frame_id);
    assert(ret >= 0);
    if (streaming_request_id_ > -1) {
      previous_streaming_request_id_ = streaming_request_id_;
    }
    streaming_request_id_ = ret;
    while (!sync_frame_.stream_ids.isEmpty()) {
      auto stat = sync_frame_cond_.waitRelative(sync_frame_lock_,
                                                kSyncFrameWaitDuration);
      if (NO_ERROR == ret) {
          QMMF_ERROR("%s:%s: Sync frame condition failed: %d\n",
                     TAG, __func__, stat);
      }
    }
  }
  QMMF_INFO("%s:%s: SubmitRequest for Num streams(%d) is successfull"
      " request_id(%d) batches: %d", TAG, __func__, size, streaming_request_id_,
      streaming_active_requests_.size());

  return ret;
}

int32_t CameraContext::SubmitRequest(Camera3Request request,
                                     bool is_streaming,
                                     int64_t *lastFrameNumber) {
  Mutex::Autolock lock(device_access_lock_);

  int32_t ret = NO_ERROR;
  ret = camera_device_->SubmitRequest(request, is_streaming,
                                      lastFrameNumber);
  assert(ret >= 0);
  return ret;
}

status_t CameraContext::CancelRequest() {

  Mutex::Autolock lock(device_access_lock_);

  int64_t last_frame_mumber;
  assert(streaming_request_id_ >= 0);

  QMMF_INFO("%s:%s: Issuing CancelRequest!", TAG, __func__);
  auto ret = camera_device_->CancelRequest(streaming_request_id_,
                                           &last_frame_mumber);
  assert(ret == NO_ERROR);
  QMMF_INFO("%s:%s: last_frame_mumber(%lld) after CancelRequest", TAG, __func__,
      last_frame_mumber);

  ret = camera_device_->WaitUntilIdle();
  assert(ret == NO_ERROR);

  streaming_request_id_ = -1;
  previous_streaming_request_id_ = -1;
  QMMF_INFO("%s:%s: Request cancelled last frame number: %lld\n", TAG,
      __func__, last_frame_mumber);
  return ret;
}

status_t CameraContext::ReturnStreamBuffer(int32_t stream_id,
                                           StreamBuffer buffer) {
  QMMF_DEBUG("%s:%s: camera_stream_id: %d, buffer: 0x%p ts: %lld\n", TAG,
      __func__, stream_id, buffer.handle, buffer.timestamp);

  auto ret = camera_device_->ReturnStreamBuffer(stream_id, buffer);
  assert(ret == NO_ERROR);

  Mutex::Autolock lock(sync_frame_lock_);
  if (sync_frame_.last_frame_id == buffer.frame_number) {
    if (!sync_frame_.stream_ids.isEmpty()) {
      ssize_t idx = -1;
      size_t count = sync_frame_.stream_ids.size();
      for (size_t i = 0; i < count; i++) {
        if (sync_frame_.stream_ids[i] == stream_id) {
          idx = i;
          break;
        }
      }
      if (0 <= idx) {
        sync_frame_.stream_ids.removeAt(idx);
        sync_frame_cond_.signal();
      }
    }
  }

  return ret;
}

void CameraContext::SnapshotCaptureCallback(int32_t stream_id,
                                            StreamBuffer buffer) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  QMMF_DEBUG("%s:%s format(0x%x):num_planes(%d) ", TAG, __func__,
      buffer.info.format, buffer.info.num_planes);
  for (uint32_t i = 0; i < buffer.info.num_planes; ++i) {
    QMMF_DEBUG("%s:%s plane_info[%d].stride=%d", TAG, __func__, i,
        buffer.info.plane_info[i].stride);
    QMMF_DEBUG("%s:%s plane_info[%d].scanline=%d", TAG, __func__, i,
        buffer.info.plane_info[i].scanline);
    QMMF_DEBUG("%s:%s plane_info[%d].width=%d", TAG, __func__, i,
        buffer.info.plane_info[i].width);
    QMMF_DEBUG("%s:%s plane_info[%d].height=%d", TAG, __func__, i,
        buffer.info.plane_info[i].height);
  }
  QMMF_DEBUG("%s:%s fd(0x%x):size(%d) ", TAG, __func__, buffer.fd, buffer.size);

  std::lock_guard<std::mutex> lock(capture_count_lock_);
  {
    --sequence_cnt_;
    if (cancel_capture_) {
      camera_device_->ReturnStreamBuffer(stream_id, buffer);
      if (sequence_cnt_ == 0) {
        QMMF_INFO("%s:%s CancelCapture: Count is zero!", TAG, __func__);
        capture_count_signal_.notify_one();
      }
      return;
    }
  }

  buffer.camera_id = camera_id_;
  snapshot_buffer_list_.add(buffer.fd, buffer);
  snapshot_buffer_stream_list_.add(buffer.fd, stream_id);

  assert(client_snapshot_cb_ != nullptr);
  client_snapshot_cb_(burst_cnt_, buffer);
  ++burst_cnt_;

  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

status_t CameraContext::ValidateResolution(const ImageFormat format,
                                           const uint32_t width,
                                           const uint32_t height) {

  QMMF_VERBOSE("%s:%s Enter ", TAG, __func__);

  CameraMetadata static_meta;
  camera_metadata_entry_t entry;
  auto ret = camera_device_->GetCameraInfo(camera_id_, &static_meta);
  assert(ret == NO_ERROR);

  bool supported = false;
  uint32_t w, h;
  switch (format) {
    case ImageFormat::kJPEG:
    //TODO: ANDROID_SCALER_AVAILABLE_JPEG_SIZES tag is not available in static
    // meta.
    if (static_meta.exists(ANDROID_SCALER_AVAILABLE_JPEG_SIZES)) {
      entry = static_meta.find(ANDROID_SCALER_AVAILABLE_JPEG_SIZES);
      for (uint32_t i = 0 ; i < entry.count; i += 2) {
        w = entry.data.i32[i+0];
        h = entry.data.i32[i+1];
        QMMF_INFO("%s:%s:(%d) Supported Jpeg:(%d)x(%d)",TAG, __func__, i, w, h);
        if(w == width && h == height) {
          supported = true;
          break;
        }
      }
    }
    supported = true;
    break;
    case ImageFormat::kNV12:
    if (static_meta.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
      entry = static_meta.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
      for (uint32_t i = 0 ; i < entry.count; i += 4) {
        if (HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED == entry.data.i32[i]) {
          if (ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
              entry.data.i32[i+3]) {
            w = entry.data.i32[i+1];
            h = entry.data.i32[i+2];
            QMMF_DEBUG("%s:%s:(%d) Supported Raw YUV:(%d)x(%d)",TAG, __func__,
                i, w, h);
            if(w == width && h == height) {
              supported = true;
              break;
            }
          }
        }
      }
    }
    break;
    case ImageFormat::kBayerRDI:
    if (static_meta.exists(ANDROID_SCALER_AVAILABLE_RAW_SIZES)) {
      entry = static_meta.find(ANDROID_SCALER_AVAILABLE_RAW_SIZES);
      for (uint32_t i = 0 ; i < entry.count; i += 2) {
        w = entry.data.i32[i+0];
        h = entry.data.i32[i+1];
        QMMF_INFO("%s:%s: (%d) Supported RAW RDI W(%d):H(%d)", TAG, __func__, i,
            width, height);
        if(w == width && h == height) {
          supported = true;
          break;
        }
      }
    }
    break;
    default:
    break;
  }
  if (!supported) {
    QMMF_ERROR("%s:%s: format(0x%x):width(%d):height(%d) not supported!", TAG,
        __func__, format, width, height);
    return BAD_VALUE;
  }
  QMMF_VERBOSE("%s:%s Exit ", TAG, __func__);
  return NO_ERROR;
}

status_t CameraContext::CaptureZSLImage(const ImageParam &param) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  status_t ret = NO_ERROR;

  bool regular_snapshot = false;
  assert(!snapshot_request_.streamIds.isEmpty());

  if (ImageFormat::kJPEG != param.image_format) {
    QMMF_ERROR("%s:%s ZSL capture supports only Jpeg as output!",
               TAG, __func__);
    return BAD_VALUE;
  }

  if ((param.width != camera_start_params_.zsl_width) ||
      (param.height != camera_start_params_.zsl_height)) {
    QMMF_ERROR("%s:%s ZSL stream size %dx%d doesn't match image size %dx%d!",
               TAG, __func__, camera_start_params_.zsl_width,
               camera_start_params_.zsl_height, param.width, param.height);
    return BAD_VALUE;
  }

  assert(zsl_port_.get() != nullptr);
  ZslPort* zsl_port = static_cast<ZslPort*>(zsl_port_.get());
  auto stat = zsl_port->PickZSLBuffer();
  if (NO_ERROR != stat) {
    QMMF_ERROR("%s:%s Failed to find a good ZSL input buffer: %d", TAG,
        __func__, stat);
    QMMF_ERROR("%s:%s Switching to regular snapshot!", TAG, __func__);
    regular_snapshot = true;
  }

  Mutex::Autolock lock(device_access_lock_);
  uint8_t jpeg_quality = snapshot_param_.image_quality;
  int64_t last_frame_mumber;

  if (!regular_snapshot) {
    Camera3Request reprocess_request;
    reprocess_request.streamIds.add(zsl_port->GetInputStreamId());
    reprocess_request.streamIds.add(snapshot_request_.streamIds[0]);
    reprocess_request.metadata = zsl_port->GetInputBuffer().result;
    reprocess_request.metadata.update(ANDROID_JPEG_QUALITY, &jpeg_quality,
                                      1);
    QMMF_INFO("%s:%s: Submit ZSL reprocess request!!", TAG, __func__);
    auto id = camera_device_->SubmitRequest(reprocess_request, false,
                                            &last_frame_mumber);
    if (0 > id) {
      QMMF_ERROR("%s:%s Failed to submit ZSL reprocess request: %d",
                 TAG, __func__, id);
      ret = UNKNOWN_ERROR;
    }
  } else {
    QMMF_INFO("%s:%s: Submit Reguar snapshot request!", TAG, __func__);
    snapshot_request_.metadata.update(ANDROID_JPEG_QUALITY, &jpeg_quality, 1);
    auto id = camera_device_->SubmitRequest(snapshot_request_,
                                            false,
                                            &last_frame_mumber);
    if (0 > id) {
      QMMF_ERROR("%s:%s Failed to submit reguar snapshot request: %d",
                 TAG, __func__, id);
      ret = UNKNOWN_ERROR;
    }
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

//Camera device callbacks
void CameraContext::CameraErrorCb(CameraErrorCode error_code,
                                  const CaptureResultExtras &result) {

  QMMF_WARN("%s:%s: Camera Client: error_code: %d\n", TAG, __func__,
            error_code);
}

void CameraContext::CameraIdleCb() {
  QMMF_WARN("%s:%s: Camera is in Idle State!!", TAG, __func__);
}

void CameraContext::CameraShutterCb(const CaptureResultExtras &result,
                                    int64_t time_stamp) {

}

void CameraContext::CameraPreparedCb(int32_t) {

}

void CameraContext::CameraResultCb(const CaptureResult &result) {

  if (((streaming_request_id_ == result.resultExtras.requestId) ||
      (previous_streaming_request_id_ == result.resultExtras.requestId)) &&
      (nullptr != result_cb_)) {
    result_cb_(camera_id_, result.metadata);
  }
  if (camera_start_params_.zsl_mode) {
    assert(zsl_port_.get() != nullptr);
    ZslPort* zsl_port = static_cast<ZslPort*>(zsl_port_.get());
    zsl_port->HandleZSLCaptureResult(result);
  }
  if (sequence_cnt_ > 1 && burst_cnt_ < sequence_cnt_ &&
      snapshot_request_id_ == result.resultExtras.requestId) {
    if(camera_reprocess_.get() != nullptr) {
      camera_reprocess_->AddResult(result);
    }
  }
}

CameraPort* CameraContext::GetPort(const uint32_t track_id) {

  CameraPort* port = nullptr;
  for (auto iter : active_ports_) {
    auto type = iter->GetPortType();
    if (track_id == iter->GetConsumerId() && (type != CameraPortType::kZSL)) {
      QMMF_INFO("%s:%s: Found the port for id(%d)", TAG, __func__, track_id);
      port = iter.get();
      break;
    }
  }
  if (!port) {
    QMMF_ERROR("%s:%s: No port belongs to consumer(%d)", TAG, __func__,
        track_id);
  }
  return port;
}

void CameraContext::DeletePort(const uint32_t track_id) {

  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, track_id);
  auto iter = active_ports_.begin();
  while (iter != active_ports_.end()) {
    auto type = (*iter)->GetPortType();
    if (track_id == (*iter)->GetConsumerId()
        && (type != CameraPortType::kZSL)) {
      QMMF_INFO("%s:%s: Found the port for id(%d)", TAG, __func__, track_id);
      iter = active_ports_.erase(iter);
      break;
    } else {
      ++iter;
    }
  }
  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, track_id);
}

CameraPort::CameraPort(const CameraStreamParam& param, size_t batch,
                       CameraPortType port_type, CameraContext* context)
    : port_type_(port_type),
      context_(context),
      camera_stream_id_(-1),
      params_(param),
      ready_to_start_(false),
      batch_size_(batch),
      consumer_id_(param.id) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  BufferProducerImpl<CameraPort> *producer_impl;
  producer_impl = new BufferProducerImpl<CameraPort>(this);
  buffer_producer_impl_ = producer_impl;

  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

CameraPort::~CameraPort() {

  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  buffer_producer_impl_.clear();
  buffer_producer_impl_ = nullptr;
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

status_t CameraPort::Init() {

  memset(&cam_stream_params_, 0, sizeof(cam_stream_params_));
  if (params_.cam_stream_format != CameraStreamFormat::kRAW10) {
    cam_stream_params_.format       = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  } else {
    cam_stream_params_.format       = HAL_PIXEL_FORMAT_RAW10;
  }
  cam_stream_params_.width        = params_.cam_stream_dim.width;
  cam_stream_params_.height       = params_.cam_stream_dim.height;
  cam_stream_params_.grallocFlags = GRALLOC_USAGE_HW_FB;

  if (params_.low_power_mode) {
      cam_stream_params_.bufferCount  = PREVIEW_STREAM_BUFFER_COUNT;
  } else {
    cam_stream_params_.grallocFlags |= private_handle_t::
        PRIV_FLAGS_VIDEO_ENCODER;
    cam_stream_params_.bufferCount = VIDEO_STREAM_BUFFER_COUNT;
    if (params_.cam_stream_dim.width == 3840
        && params_.cam_stream_dim.height == 2160) {
      cam_stream_params_.bufferCount += EXTRA_DCVS_BUFFERS;
    }
  }

  cam_stream_params_.cb = [&] (int32_t stream_id, StreamBuffer buffer)
      { StreamCallback (stream_id, buffer); };

  assert(context_ != nullptr);
  int32_t stream_id;
  auto ret = context_->CreateDeviceStream(cam_stream_params_,
                                          params_.frame_rate, &stream_id);
  if (ret != NO_ERROR || stream_id < 0) {
    QMMF_ERROR("%s:%s: CreateDeviceStream failed!!", TAG, __func__);
    return BAD_VALUE;
  }
  camera_stream_id_ = stream_id;

  port_state_ = PortState::PORT_CREATED;

  QMMF_INFO("%s:%s: Camera Device Stream(%d) is created Succussfully!", TAG,
      __func__, camera_stream_id_);
  QMMF_INFO("%s:%s: track_id(%d) is mapped to camera stream_id(%d)", TAG,
      __func__, params_.id, camera_stream_id_);

  return NO_ERROR;
}

status_t CameraPort::DeInit() {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(ready_to_start_ == false);
  assert(context_ != nullptr);

  auto ret = context_->DeleteDeviceStream(camera_stream_id_, true);
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: DeleteDeviceStream failed!!", TAG, __func__);
    return BAD_VALUE;
  }
  consumer_map_.clear();
  QMMF_DEBUG("%s:%s: CameraPort(0x%p) deinitialized successfully! ", TAG,
      __func__, this);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t CameraPort::Start(const uint32_t consumer_id,
                           const sp<IBufferConsumer>& consumer) {

  // ZSL port can start and stop without having any consumer connected.
  if (port_type_ != CameraPortType::kZSL) {
    assert(consumer.get() != nullptr);
    // Establish buffer communication link between consumer (TrackSource) and
    // Buffer Producer interface of camera port.
    AddConsumer(consumer_id, consumer);
  }

  //TODO: protect it with lock.
  ready_to_start_ = true;
  port_state_ = PortState::PORT_READYTOSTART;

  QMMF_INFO("%s:%s: track_id(%d):camera stream(%d) to start!", TAG, __func__,
      consumer_id, camera_stream_id_);

  auto ret = context_->UpdateRequest(true);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CameraPort:Start:UpdateRequest failed! for track_id = %d"
        , TAG, __func__, consumer_id);
  }
  port_state_ = PortState::PORT_STARTED;
  return ret;
}

status_t CameraPort::Stop(const uint32_t consumer_id) {

  // ZSL port can start and stop without having any consumer connected.
  if (port_type_ != CameraPortType::kZSL) {

    if (!IsConsumerIdValid(consumer_id)) {
      QMMF_ERROR("%s:%s: consumer_id(%d) is not valid!", TAG, __func__,
          consumer_id);
      return BAD_VALUE;
    }

    // Break buffer communication link between consumer and camera port.
    RemoveConsumer(consumer_id);

    size_t size = consumer_map_.size();
    QMMF_INFO("%s:%s: Number of Consumer left = %d", TAG, __func__, size);
    if (size > 0) {
      // Camera port is still getting used by some other consumer, don't delete
      // camera device stream, eventaully it would be deleted when number
      // of consumer becomes zero.
      return NO_ERROR;
    }
  }
  //TODO: protect it with lock.
  ready_to_start_ = false;
  port_state_ = PortState::PORT_READYTOSTOP;

  // Stop basically removes the stream from current running capture request,
  // it doen't delete the stream.
  auto ret = context_->UpdateRequest(true);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CameraPort:Start:UpdateRequest failed! for track_id = %d"
        , TAG, __func__, consumer_id);
  }
  QMMF_INFO("%s:%s: track_id(%d):Port(0x%p) Stopped Succussfully!", TAG,
      __func__, consumer_id, this);

  port_state_ = PortState::PORT_STOPPED;

  return ret;
}

status_t CameraPort::AddConsumer(const uint32_t consumer_id,
                                 const sp<IBufferConsumer>& consumer) {

  Mutex::Autolock lock(consumer_lock_);

  consumer_map_.add(consumer_id, consumer);
  // Add consumer to port's producer interface.
  assert(buffer_producer_impl_.get() != nullptr);
  buffer_producer_impl_->AddConsumer(consumer);
  consumer->SetProducerHandle(buffer_producer_impl_);
  QMMF_DEBUG("%s:%s: ConsumerId(%d):(0x%p) has been added to CameraPort(0x%p)."
      "Total number of consumer =%d", TAG, __func__, consumer_id, consumer.get()
      , this, consumer_map_.size());
  return NO_ERROR;
}

status_t CameraPort::RemoveConsumer(const uint32_t consumer_id) {

  Mutex::Autolock lock(consumer_lock_);

  sp<IBufferConsumer> consumer = consumer_map_.valueFor(consumer_id);
  assert(consumer.get() != nullptr);
  // Remove consumer from port's producer interface.
  assert(buffer_producer_impl_.get() != nullptr);
  buffer_producer_impl_->RemoveConsumer(consumer);

  consumer_map_.removeItem(consumer_id);
  QMMF_DEBUG("%s:%s: ConsumerId(%d):(0x%p) has been Remved CameraPort(0x%p)."
      "Total number of consumer =%d", TAG, __func__,consumer_id, consumer.get(),
      this, consumer_map_.size());
  return NO_ERROR;
}

void CameraPort::NotifyBufferReturned(const StreamBuffer& buffer) {

  QMMF_VERBOSE("%s:%s: StreamBuffer(0x%p) Cameback to CameraPort", TAG,
       __func__, buffer.handle);
  //TODO: protect this with lock, would be required once multiple camera ports
  // are enabled.
  context_->ReturnStreamBuffer(camera_stream_id_, buffer);
}

int32_t CameraPort::GetNumConsumers() {
  Mutex::Autolock lock(consumer_lock_);
  return consumer_map_.size();
}

bool CameraPort::IsReadyToStart() {
  //TODO: protect it with lock.
  return ready_to_start_;
}

PortState& CameraPort::getPortState() {
  return port_state_;
}

bool CameraPort::IsConsumerIdValid(const uint32_t id) {

  bool valid = false;
  size_t size = consumer_map_.size();
  for(size_t i = 0; i < size; i++) {
    if (id == consumer_map_.keyAt(i)) {
        valid = true;
        break;
    }
  }
  return valid;
}

void CameraPort::StreamCallback(int32_t stream_id, StreamBuffer stream_buffer) {

  QMMF_VERBOSE("%s:%s: Enter stream_id(%d)", TAG, __func__, stream_id);
  assert(stream_id == camera_stream_id_);
  assert(buffer_producer_impl_.get() != nullptr);

  QMMF_VERBOSE("%s:%s: camera stream_id: %d, buffer: 0x%p ts: %lld\n", TAG,
      __func__, stream_id, stream_buffer.handle, stream_buffer.timestamp);

  if(buffer_producer_impl_->GetNumConsumer() > 0) {
    stream_buffer.camera_id = context_->camera_id_;
    buffer_producer_impl_->NotifyBuffer(stream_buffer);
  } else {
    // Return the buffer back to camera.
    QMMF_VERBOSE("%s:%s: No consumer, simply return buffer back to camera!",
        TAG, __func__);
    context_->ReturnStreamBuffer(stream_id, stream_buffer);
  }
  QMMF_VERBOSE("%s:%s: Exit ", TAG, __func__);
}

ZslPort::ZslPort(const CameraStreamParam& param, size_t batch_size,
                 CameraPortType port_type, CameraContext *context)
    : CameraPort(param, batch_size, port_type, context) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  zsl_input_buffer_.timestamp = -1;
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

ZslPort::~ZslPort() {

  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  if (!zsl_queue_.empty()) {
    PauseAndFlushZSLQueue();
  }
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

status_t ZslPort::Init() {

  assert(port_type_ == CameraPortType::kZSL);
  auto ret = SetUpZSL();
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s SetUpZSL failed!", TAG, __func__);
    return ret;
  }
  QMMF_INFO("%s:%s: ZslPort Initialized!", TAG, __func__);
  return ret;
}

status_t ZslPort::PauseAndFlushZSLQueue() {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);
  int32_t ret = NO_ERROR;
  Mutex::Autolock l(zsl_queue_lock_);
  zsl_running_ = false;

  if (!zsl_queue_.empty()) {
    List<ZSLEntry>::iterator it = zsl_queue_.begin();
    List<ZSLEntry>::iterator end = zsl_queue_.end();
    while (it != end) {
      if (it->timestamp == it->buffer.timestamp) {
        assert(context_ != nullptr);
        auto stat = context_->ReturnStreamBuffer(camera_stream_id_, it->buffer);
        if (NO_ERROR != ret) {
          QMMF_ERROR("%s:%s Failed to flush ZSL buffer: %d",
                     TAG, __func__, ret);
          ret = stat;
        }
      }
      it++;
    }
    zsl_queue_.clear();
  }
  QMMF_INFO("%s:%s: Zsl queue flush is: %s", TAG, __func__,
      ret == NO_ERROR ? "Successful!" : "Failed!");

  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return ret;
}

void ZslPort::ResumeZSL() {
  Mutex::Autolock l(zsl_queue_lock_);
  zsl_running_ = true;
}

bool ZslPort::IsRunning() {
  Mutex::Autolock l(zsl_queue_lock_);
  return zsl_running_ && (camera_stream_id_ > 0);
}

status_t ZslPort::PickZSLBuffer() {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  Mutex::Autolock l(zsl_queue_lock_);
  auto ret = NO_ERROR;

  if (zsl_queue_.empty()) {
    QMMF_ERROR("%s:%s ZSL queue is empty!\n", TAG, __func__);
    return NO_INIT;
  }

  if (0 <= zsl_input_buffer_.timestamp) {
    QMMF_ERROR("%s:%s Previous ZSL input still processing!", TAG, __func__);
    return -EBUSY;
  }

  List<ZSLEntry>::iterator good_entry;
  List<ZSLEntry>::iterator it = zsl_queue_.begin();
  List<ZSLEntry>::iterator end = zsl_queue_.end();
  bool found = false;
  while (it != end) {
    if ((it->timestamp == it->buffer.timestamp) && (!it->result.isEmpty())) {
      camera_metadata_entry_t entry;
      entry = it->result.find(ANDROID_CONTROL_AE_STATE);
      if (0 < entry.count) {
        if ((entry.data.u8[0] == ANDROID_CONTROL_AE_STATE_CONVERGED) ||
            (entry.data.u8[0] == ANDROID_CONTROL_AE_STATE_LOCKED)) {
          good_entry = it;
          found = true;
        }
      }
    }
    it++;
  }

  if (found) {
    zsl_input_buffer_ = *good_entry;
    zsl_queue_.erase(good_entry);
    QMMF_INFO("%s:%s: Found Good ZSL buffer!!", TAG, __func__);
  } else {
    QMMF_ERROR("%s:%s: No appropriate ZSL buffer found!", TAG, __func__);
    ret = NAME_NOT_FOUND;
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

void ZslPort::HandleZSLCaptureResult(const CaptureResult &result) {

  QMMF_VERBOSE("%s:%s Enter ", TAG, __func__);

  if (0 <= camera_stream_id_) {
    int64_t timestamp;
    if (result.metadata.exists(ANDROID_SENSOR_TIMESTAMP)) {
      timestamp = result.metadata.find(ANDROID_SENSOR_TIMESTAMP).data.i64[0];
    } else {
      QMMF_ERROR("%s:%s Sensor timestamp tag missing in result!\n",
        TAG, __func__);
      return;
    }

    {
      ZSLEntry entry;
      entry.timestamp = -1;
      memset(&entry.buffer, 0, sizeof(entry.buffer));

      Mutex::Autolock l(zsl_queue_lock_);
      if (zsl_running_) {
        bool append = true;
        if (!zsl_queue_.empty()) {
          List<ZSLEntry>::iterator it = zsl_queue_.begin();
          List<ZSLEntry>::iterator end = zsl_queue_.end();
          while (it != end) {
            if (it->timestamp == timestamp) {
              it->result.append(result.metadata);
              append = false;
              break;
            }
            it++;
          }
        }

        if (append) {
          //Buffer is missing append to queue directly
          ZSLEntry new_entry;
          new_entry.result.append(result.metadata);
          new_entry.timestamp = timestamp;
          memset(&new_entry.buffer, 0, sizeof(new_entry.buffer));
          zsl_queue_.push_back(new_entry);
        }

        if (zsl_queue_.size() >= (zsl_queue_depth_ + 1)) {
          entry = *zsl_queue_.begin(); //return oldest buffer
          zsl_queue_.erase(zsl_queue_.begin());
        }
      }

      if (entry.timestamp == entry.buffer.timestamp) {
        QMMF_DEBUG("%s:%s Return Buffer from ZSl Queue back to camera!", TAG,
            __func__);
        assert (context_ != nullptr);
        auto ret = context_->ReturnStreamBuffer(camera_stream_id_, entry.buffer);
        if (NO_ERROR != ret) {
          QMMF_ERROR("%s:%s Failed to return ZSL buffer to camera: %d",
                     TAG, __func__, ret);
        }
      }
    }
  }
  QMMF_VERBOSE("%s:%s Exit ", TAG, __func__);
}

status_t ZslPort::SetUpZSL() {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);
  bool is_fps_supported = false;

  CameraStartParam cam_start_param = context_->GetCameraStartParam();

  zsl_queue_depth_ = cam_start_param.zsl_queue_depth;
  if (0 == zsl_queue_depth_) {
    QMMF_ERROR("%s:%s: Invalid ZSL queue depth size!", TAG, __func__);
    return BAD_VALUE;
  }

  for (auto &iter : context_->GetSupportedFps()) {
    if (iter == static_cast<int32_t>(cam_start_param.frame_rate)) {
      is_fps_supported = true;
      break;
    }
  }
  if (!is_fps_supported) {
    QMMF_ERROR("%s:%s: Framerate: %d not supported by camera!",
        TAG, __func__, cam_start_param.frame_rate);
    return BAD_VALUE;
  }
  QMMF_INFO("%s:%s zsl width(%d):height(%d), queue_depth=%d", TAG, __func__,
    cam_start_param.zsl_width, cam_start_param.zsl_height,
    cam_start_param.zsl_queue_depth);

  auto ret = context_->ValidateResolution(ImageFormat::kNV12,
                                          cam_start_param.zsl_width,
                                          cam_start_param.zsl_height);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: ZSL width(%d):height(%d) Not supported!",
        TAG, __func__, cam_start_param.zsl_width, cam_start_param.zsl_height);
    return ret;
  }

  CameraStreamParameters zsl_stream_params;
  memset(&zsl_stream_params, 0x0, sizeof(zsl_stream_params));
  zsl_stream_params.bufferCount = cam_start_param.zsl_queue_depth +
                                  VIDEO_STREAM_BUFFER_COUNT;
  zsl_stream_params.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  zsl_stream_params.width  = cam_start_param.zsl_width;
  zsl_stream_params.height = cam_start_param.zsl_height;
  zsl_stream_params.grallocFlags = GRALLOC_USAGE_HW_FB
                                   |GRALLOC_USAGE_HW_CAMERA_ZSL;
  zsl_stream_params.cb = [&](int32_t stream_id, StreamBuffer buffer)
                        { ZSLCaptureCallback(stream_id, buffer); };

  ret = context_->CreateDeviceStream(zsl_stream_params,
                                     cam_start_param.frame_rate,
                                     &camera_stream_id_);
  if (NO_ERROR != ret || camera_stream_id_ < 0) {
    QMMF_ERROR("%s:%s: CreateDeviceStream failed!", TAG, __func__);
    return ret;
  }

  // Create Input stream for reprocess.
  CameraInputStreamParameters input_stream_params;
  memset(&input_stream_params, 0x0, sizeof input_stream_params);
  input_stream_params.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  input_stream_params.width  = cam_start_param.zsl_width;
  input_stream_params.height = cam_start_param.zsl_height;
  input_stream_params.get_input_buffer = [&] (StreamBuffer& buffer)
      { GetZSLInputBuffer(buffer); };
  input_stream_params.return_input_buffer  = [&] (StreamBuffer& buffer)
      { ReturnZSLInputBuffer(buffer); };

  int32_t stream_id;
  ret = context_->CreateDeviceInputStream(input_stream_params, &stream_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s Failed to create input reprocess stream: %d", TAG,
               __func__, ret);
    return ret;
  }
  assert(stream_id >= 0);
  input_stream_id_ = stream_id;
  QMMF_INFO("%s:%s: zsl input_stream_id_(%d)", TAG, __func__, input_stream_id_);

  zsl_running_ = true;
  QMMF_INFO("%s:%s: zsl port configured with stream id(%d)", TAG, __func__,
      camera_stream_id_);

  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return ret;
}

void ZslPort::ZSLCaptureCallback(int32_t stream_id, StreamBuffer buffer) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  ZSLEntry entry;
  entry.timestamp = -1;
  {
    Mutex::Autolock l(zsl_queue_lock_);
    if (zsl_running_) {
      bool append = true;
      if (!zsl_queue_.empty()) {
        List<ZSLEntry>::iterator it = zsl_queue_.begin();
        List<ZSLEntry>::iterator end = zsl_queue_.end();
        while (it != end) {
          if (it->timestamp == buffer.timestamp) {
            it->buffer = buffer;
            append = false;
            break;
          }
          it++;
        }
      }

      if (append) {
        //Result is missing append to queue directly
        ZSLEntry new_entry;
        new_entry.buffer = buffer;
        new_entry.timestamp = buffer.timestamp;
        new_entry.result.clear();
        zsl_queue_.push_back(new_entry);
      }

      if (zsl_queue_.size() >= (zsl_queue_depth_ + 1)) {
        entry = *zsl_queue_.begin(); //return oldest buffer
        zsl_queue_.erase(zsl_queue_.begin());
      }
    } else {
      entry.buffer = buffer;
      entry.timestamp = buffer.timestamp;
    }
  }

  if (entry.timestamp == entry.buffer.timestamp) {
    QMMF_DEBUG("%s:%s Return Buffer from ZSl Queue back to camera!", TAG,
        __func__);
    assert (context_ != nullptr);
    auto ret = context_->ReturnStreamBuffer(stream_id, entry.buffer);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s Failed to return ZSL buffer to camera: %d",
                 TAG, __func__, ret);
    }
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

void ZslPort::GetZSLInputBuffer(StreamBuffer& buffer) {
  buffer = zsl_input_buffer_.buffer;
  QMMF_INFO("%s:%s buffer(%d) submitted for reprocess!", TAG, __func__,
    buffer.fd);
}

void ZslPort::ReturnZSLInputBuffer(StreamBuffer& buffer) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  if (buffer.handle == zsl_input_buffer_.buffer.handle) {
    assert (context_ != nullptr);
    QMMF_INFO("%s:%s buffer(%d) returned from reprocess!", TAG, __func__,
        buffer.fd);
    auto ret = context_->ReturnStreamBuffer(camera_stream_id_,
                                            zsl_input_buffer_.buffer);
    if (NO_ERROR == ret) {
      zsl_input_buffer_.timestamp = -1;
    } else {
      QMMF_ERROR("%s:%s Failed to return input buffer: %d\n", TAG, __func__,
          ret);
    }
  } else {
    QMMF_ERROR("%s:%s: Buffer handle of returned buffer: %p doesn't match with"
        "expected handle: %p\n", TAG, __func__, buffer.handle,
        zsl_input_buffer_.buffer.handle);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

CameraReprocess::CameraReprocess(CameraContext* context)
    : context_(context),
      reprocess_flag_(false),
      ready_to_start_(false),
      burst_cnt_(0) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

CameraReprocess::~CameraReprocess() {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}


void CameraReprocess::GetInputBuffer(StreamBuffer &buffer) {
  Mutex::Autolock lock(burst_queue_lock_);
  auto iter = input_buffer_.begin();
  input_buffer_done_.push_back((*iter));
  buffer = (*iter);
  input_buffer_.erase(iter);
}

void CameraReprocess::ReturnInputBuffer(StreamBuffer &buffer) {
  Mutex::Autolock lock(burst_queue_lock_);
  auto iter = input_buffer_done_.begin();
  for (; iter != input_buffer_done_.end(); iter++) {
    if ((*iter).handle ==  buffer.handle) {
      auto ret = context_->ReturnStreamBuffer(input_stream_id_, (*iter));
      if (NO_ERROR != ret) {
        QMMF_ERROR("%s: Failed to return input buffer: %d\n", __func__, ret);
      }
      input_buffer_done_.erase(iter);
      break;
    }
  }
}

void CameraReprocess::ReturnAllInputBuffers() {
  Mutex::Autolock lock(burst_queue_lock_);
  auto iter = input_buffer_done_.begin();
  for (; iter != input_buffer_done_.end(); iter++) {
    auto ret = context_->ReturnStreamBuffer(input_stream_id_, (*iter));
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s: Failed to return input buffer: %d\n", __func__, ret);
    }
  }
  input_buffer_done_.clear();
  input_buffer_.clear();
  burst_queue_.clear();
  input_burst_queue_.clear();
}

void CameraReprocess::ReprocessCallback(int32_t stream_id,
                                        StreamBuffer buffer) {
  capture_client_cb_(stream_id, buffer);
  //start next frame reprocess
  if (Start() != NO_ERROR) {
    QMMF_ERROR("%s: Failed: Wrong state. Reprocess is not started.\n",
        __func__);
  }

  // last frame
  if (++burst_cnt_ == num_images_) {
    ReturnAllInputBuffers();
    burst_cnt_= 0;
    reprocess_flag_ = false;
  }
}

status_t CameraReprocess::Create(const int32_t stream_id,
                                 const ReprocParam& input,
                                 const ReprocParam& output,
                                 const uint32_t frame_rate,
                                 const uint32_t num_images,
                                 const CameraMetadata& static_meta,
                                 const ReprocessCb& cb) {
  int32_t ret = NO_ERROR;
  CameraInputStreamParameters inputStreamParams;
  CameraStreamParameters streamParams;
  int32_t stream_id_p;

  assert(context_ != nullptr);

  if (ready_to_start_) {
    QMMF_ERROR("%s:%s: Failed: Already configured.", TAG, __func__);
    return BAD_VALUE;
  }

  if (reprocess_flag_) {
    QMMF_ERROR("%s:%s: Failed: Wrong state.", TAG, __func__);
    return BAD_VALUE;
  }

  if (NO_ERROR != ValidateInput(static_meta, input, output)){
    QMMF_ERROR("%s:%s: Failed: Wrong input parameters.", TAG, __func__);
    return BAD_VALUE;
  }

  Mutex::Autolock lock(reprocess_lock_);

  input_stream_id_ = stream_id;
  num_images_ = num_images;
  burst_cnt_ = 0;
  capture_client_cb_ = cb;

  memset(&inputStreamParams, 0, sizeof(inputStreamParams));
  inputStreamParams.format = input.format;
  inputStreamParams.width = input.width;
  inputStreamParams.height = input.height;
  inputStreamParams.get_input_buffer = [&] (StreamBuffer &buffer)
      { GetInputBuffer(buffer); };
  inputStreamParams.return_input_buffer  = [&] (StreamBuffer &buffer)
      { ReturnInputBuffer(buffer); };
  ret = context_->CreateDeviceInputStream(inputStreamParams, &stream_id_p);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to create input reprocess stream: %d\n",
               __func__, ret);
    return ret;
  }
  assert(stream_id_p >= 0);
  reprocess_request_.streamIds.add(stream_id_p);
  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = num_images;
  streamParams.format = output.format;
  streamParams.width = output.width;
  streamParams.height = output.height;
  streamParams.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
  streamParams.cb = [&](int32_t streamId, StreamBuffer buffer)
                        { ReprocessCallback(streamId, buffer); };
  ret = context_->CreateDeviceStream(streamParams, frame_rate, &stream_id_p);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to create output reprocess stream: %d\n",
               __func__, ret);
    return ret;
  }
  assert(stream_id_p >= 0);
  reprocess_request_.streamIds.add(stream_id_p);
  ready_to_start_ = true;
  return NO_ERROR;
}

status_t CameraReprocess::Start() {
  if (!ready_to_start_) {
    return BAD_VALUE;
  }

  assert(context_ != nullptr);

  Mutex::Autolock lock(burst_queue_lock_);

  reprocess_flag_ = true;

  if (!input_burst_queue_.empty()) {
    List<BurstData>::iterator it = input_burst_queue_.begin();
    input_buffer_.push_back(it->buffer);
    reprocess_request_.metadata.clear();
    reprocess_request_.metadata.append(it->result);
    input_burst_queue_.erase(it);
  } else {
    QMMF_ERROR("%s:%s: Buffer is not ready or no more buffers", TAG, __func__);
    return NO_ERROR;
  }
  int64_t last_frame_mumber;
  auto ret = context_->SubmitRequest(reprocess_request_,
                                     false,
                                     &last_frame_mumber);
  if (ret < 0) {
    QMMF_ERROR("%s:%s: Failed to submit reprocess request.", TAG, __func__);
    return BAD_VALUE;
  }
  return NO_ERROR;
}

status_t CameraReprocess::Delete() {
  QMMF_VERBOSE("%s:%s: Enter ", TAG, __func__);

  assert(context_ != nullptr);

  Mutex::Autolock lock(reprocess_lock_);

  ReturnAllInputBuffers();

  if (!reprocess_request_.streamIds.isEmpty()) {
    for (auto streamId : reprocess_request_.streamIds) {
      if (NO_ERROR != context_->DeleteDeviceStream(streamId, true)) {
        QMMF_ERROR("%s: Failed to delete non-zsl snapshot stream",
                   __func__);
        return BAD_VALUE;
      }
    }
    reprocess_request_.streamIds.clear();
  }
  reprocess_flag_ = false;
  ready_to_start_ = false;
  return NO_ERROR;
}

void CameraReprocess::AddBuff(int32_t streamId, StreamBuffer buffer) {
  {
    Mutex::Autolock lock(burst_queue_lock_);
    bool append = true;
    if (!burst_queue_.empty()) {
      List<BurstData>::iterator it = burst_queue_.begin();
      List<BurstData>::iterator end = burst_queue_.end();
      while (it != end) {
        if (it->timestamp == buffer.timestamp) {
          it->buffer = buffer;
          input_burst_queue_.push_back(*it);
          burst_queue_.erase(it);
          append = false;
          break;
        }
        it++;
      }
    }

    if (append) {
      //Result is missing append to queue directly
      BurstData new_entry;
      new_entry.buffer = buffer;
      new_entry.timestamp = buffer.timestamp;
      new_entry.result.clear();
      burst_queue_.push_back(new_entry);
    }
  }

  if (input_burst_queue_.size() >= num_images_) {
    if (Start() != NO_ERROR) {
      QMMF_ERROR("%s: Failed: Wrong state. Reprocess is not started.\n",
          __func__);
    }
  }
}

void CameraReprocess::AddResult(const CaptureResult &result) {
  int64_t timestamp;
  if (result.metadata.exists(ANDROID_SENSOR_TIMESTAMP)) {
    timestamp = result.metadata.find(ANDROID_SENSOR_TIMESTAMP).data.i64[0];
  } else {
    QMMF_ERROR("%s:%s Sensor timestamp tag missing in result!\n",
        TAG, __func__);
    return;
  }

  {
    Mutex::Autolock lock(burst_queue_lock_);
    bool append = true;
    if (!burst_queue_.empty()) {
      List<BurstData>::iterator it = burst_queue_.begin();
      List<BurstData>::iterator end = burst_queue_.end();
      while (it != end) {
        if (it->timestamp == timestamp) {
          it->result.append(result.metadata);
          input_burst_queue_.push_back(*it);
          burst_queue_.erase(it);
          append = false;
          break;
        }
        it++;
      }
    }

    if (append) {
      //Buffer is missing append to queue directly
      BurstData new_entry;
      new_entry.result.append(result.metadata);
      new_entry.timestamp = timestamp;
      memset(&new_entry.buffer, 0, sizeof(new_entry.buffer));
      burst_queue_.push_back(new_entry);
    }
  }

  if (input_burst_queue_.size() >= num_images_) {
    if (Start() != NO_ERROR) {
      QMMF_ERROR("%s: Failed: Wrong state. Reprocess is not started.\n",
          __func__);
    }
  }
}

status_t CameraReprocess::ValidateInput(const CameraMetadata& static_meta,
                                        const ReprocParam& input,
                                        const ReprocParam& output) {
  camera_metadata_ro_entry_t entry;
  int32_t in_format, num_output_formats;

  if (static_meta.exists(ANDROID_SCALER_AVAILABLE_INPUT_OUTPUT_FORMATS_MAP)) {
    entry = static_meta.find(ANDROID_SCALER_AVAILABLE_INPUT_OUTPUT_FORMATS_MAP);
    for (uint32_t i = 0 ; i < entry.count; i++) {
      in_format = entry.data.i32[i++];
      num_output_formats = entry.data.i32[i++];
      if (in_format != input.format) {
        i +=  (num_output_formats - 1);
        continue;
      }
      for (int32_t f = 0; f < num_output_formats; f++) {
        i += f;
        if (output.format == entry.data.i32[i])
          return NO_ERROR;
      }
    }
  } else {
    QMMF_ERROR("%s: Failed ANDROID_SCALER_AVAILABLE_INPUT_OUTPUT_FORMATS_MAP\n",
        __func__);
  }

  QMMF_ERROR("%s: Failed: input format: 0x%x out format 0x%x\n",  __func__,
      input.format, output.format);

  return BAD_VALUE;
}

}; // namespace recoder

}; // namespace qmmf
