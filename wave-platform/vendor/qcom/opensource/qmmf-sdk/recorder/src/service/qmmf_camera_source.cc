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

#define TAG "RecorderCameraSource"

#include <memory>

#include <sys/time.h>
#include <math.h>
#include <fcntl.h>
#include <dirent.h>
#include <sys/mman.h>

#include "recorder/src/service/qmmf_camera_source.h"
#include "recorder/src/service/qmmf_recorder_common.h"
#include "recorder/src/service/qmmf_recorder_utils.h"
#ifdef ENABLE_360
#include "recorder/src/service/qmmf_multicamera_manager.h"
#endif

namespace qmmf {

namespace recorder {

using ::std::make_shared;
using ::std::shared_ptr;

static const nsecs_t kWaitDuration = 2000000000; // 2 s.

CameraSource* CameraSource::instance_ = nullptr;

CameraSource* CameraSource::CreateCameraSource() {

  if (!instance_) {
    instance_ = new CameraSource;
    if (!instance_) {
      QMMF_ERROR("%s:%s: Can't Create CameraSource Instance", TAG, __func__);
      //return nullptr;
    }
  }
  QMMF_INFO("%s:%s: CameraSource Instance Created Successfully(0x%p)", TAG,
      __func__, instance_);
  return instance_;
}

CameraSource::CameraSource() {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

CameraSource::~CameraSource() {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  if (!camera_map_.isEmpty()) {
    camera_map_.clear();
  }
  instance_ = nullptr;
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

status_t CameraSource::StartCamera(const uint32_t camera_id,
                                   const CameraStartParam &param,
                                   const ResultCb &cb) {

  QMMF_INFO("%s:%s: Camera Id(%u) to open!", TAG, __func__, camera_id);
  bool is_virtual_camera_id = false;

#ifdef ENABLE_360
  is_virtual_camera_id = (kVirtualCameraIdOffset <= camera_id);
#endif

  sp<CameraInterface> camera;

  if (is_virtual_camera_id) {
    if (NAME_NOT_FOUND == camera_map_.indexOfKey(camera_id)) {
      QMMF_ERROR("%s:%s: Invalid Virtual Camera Id(%u)!", TAG, __func__,
                 camera_id);
      return BAD_VALUE;
    }
    camera = camera_map_.valueFor(camera_id);
  } else {
    if (camera_map_.indexOfKey(camera_id) >= 0) {
      QMMF_ERROR("%s:%s: Camera Id(%u) is already open!", TAG, __func__,
          camera_id);
      return BAD_VALUE;
    }
    camera = new CameraContext();
    if (!camera.get()) {
      QMMF_ERROR("%s:%s: Can't Instantiate CameraDevice(%d)!!", TAG,
          __func__, camera_id);
      return NO_MEMORY;
    }
    // Add contexts to map when in regular camera case.
    camera_map_.add(camera_id, camera);
  }

  auto ret = camera->OpenCamera(camera_id, param, cb);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CameraDevice:OpenCamera(%d)failed!", TAG, __func__,
        camera_id);
    if (!is_virtual_camera_id) {
      camera.clear();
      camera_map_.removeItem(camera_id);
    }
    return ret;
  }
  QMMF_INFO("%s:%s: Camera(%d) Open is Successfull!", TAG, __func__, camera_id);
  return ret;
}

status_t CameraSource::StopCamera(const uint32_t camera_id) {

  int32_t ret = NO_ERROR;
  QMMF_INFO("%s:%s: CameraId(%u) to close!", TAG, __func__, camera_id);

  //TODO: check if streams are still active, flush them before closing camera.

  bool match = false;
  for (uint32_t i = 0; i < camera_map_.size(); ++i) {
    if (camera_id == camera_map_.keyAt(i)) {
      match = true;
      sp<CameraInterface> camera = camera_map_.valueFor(camera_id);
      ret = camera->CloseCamera(camera_id);
      assert(ret == NO_ERROR);
      camera_map_.removeItem(camera_id);
      QMMF_INFO("%s:%s: Camera(%d) is Closed Successfull!", TAG, __func__,
          camera_id);
      break;
    }
  }
  if (!match) {
    QMMF_ERROR("%s:%s: Invalid Camera Id(%d)", TAG, __func__, camera_id);
    return BAD_VALUE;
  }
  return ret;
}

status_t CameraSource::CreateMultiCamera(const std::vector<uint32_t> camera_ids,
                                         uint32_t *virtual_camera_id) {

  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
#ifdef ENABLE_360
  sp<CameraInterface> multi_camera = new MultiCameraManager();
  if (!multi_camera.get()) {
    QMMF_ERROR("%s:%s: Can't Instantiate MultiCameraDevice!!", TAG, __func__);
    return NO_MEMORY;
  }

  MultiCameraManager *camera_mgr =
      static_cast<MultiCameraManager*>(multi_camera.get());

  auto ret = camera_mgr->CreateMultiCamera(camera_ids, virtual_camera_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CreateMultiCamera Failed!", TAG, __func__);
    multi_camera.clear();
    return NO_INIT;
  }
  // Adds only virtual cameras. Virtual camera is a camera used
  // for 360 camera case.
  camera_map_.add(*virtual_camera_id, multi_camera);
#endif
  QMMF_INFO("%s:%s: Exit ", TAG, __func__);
  return NO_ERROR;
}

status_t CameraSource::ConfigureMultiCamera(const uint32_t virtual_camera_id,
                                            const uint32_t type,
                                            const void *param,
                                            const uint32_t param_size) {
  // TODO:
  return NO_ERROR;
}

status_t CameraSource::CaptureImage(const uint32_t camera_id,
                                    const ImageParam &param,
                                    const uint32_t num_images,
                                    const std::vector<CameraMetadata> &meta,
                                    const SnapshotCb& cb) {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);

  bool match = false;
  sp<CameraInterface> camera;
  for (uint8_t i = 0; i < camera_map_.size(); i++) {
    if (camera_id == camera_map_.keyAt(i)) {
        match = true;
        camera = camera_map_.valueAt(i);
        break;
    }
  }
  if (!match) {
    QMMF_ERROR("%s:%s: Invalid Camera Id, It is different then camera is open"
        "with", TAG, __func__);
    return BAD_VALUE;
  }
  assert(camera.get() != nullptr);

  client_snapshot_cb_ = cb;
  StreamSnapshotCb stream_cb = [&] (uint32_t count, StreamBuffer& buf) {
    SnapshotCallback(count, buf);
  };
  auto ret = camera->CaptureImage(param, num_images, meta, stream_cb);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CaptureImage Failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t CameraSource::CancelCaptureImage(const uint32_t camera_id) {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);

  bool match = false;
  sp<CameraInterface> camera;
  for (uint8_t i = 0; i < camera_map_.size(); i++) {
    if (camera_id == camera_map_.keyAt(i)) {
      match = true;
      camera = camera_map_.valueAt(i);
    }
  }
  if (!match) {
    QMMF_ERROR("%s:%s: Invalid Camera Id(%d)!", TAG, __func__, camera_id);
    return BAD_VALUE;
  }

  assert(camera.get() != nullptr);
  auto ret = camera->CancelCaptureImage();
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CancelCaptureImage Failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t CameraSource::ReturnImageCaptureBuffer(const uint32_t camera_id,
                                                const int32_t buffer_id) {
  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);

  bool match = false;
  sp<CameraInterface> camera;
  for (uint8_t i = 0; i < camera_map_.size(); i++) {
    if (camera_id == camera_map_.keyAt(i)) {
      match = true;
      camera = camera_map_.valueAt(i);
      break;
    }
  }
  if (!match) {
    QMMF_ERROR("%s:%s: Invalid Camera Id!", TAG, __func__);
    return BAD_VALUE;
  }
  assert(camera.get() != nullptr);
  auto ret = camera->ReturnImageCaptureBuffer(camera_id, buffer_id);

  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t CameraSource::CreateTrackSource(const uint32_t track_id,
                                         const VideoTrackParams& track_params) {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);

  // Find out the camera context corresponding to camera id where track has to
  // be created.
  bool match = false;
  sp<CameraInterface> camera;
  for (uint8_t i = 0; i < camera_map_.size(); i++) {
    if (track_params.params.camera_id == camera_map_.keyAt(i)) {
      match = true;
      camera = camera_map_.valueAt(i);
      break;
    }
  }
  if (!match) {
    QMMF_ERROR("%s:%s: Invalid Camera Id, It is different then camera is open"
        "with", TAG, __func__);
    return BAD_VALUE;
  }

  // Create TrackSource and give it to CameraInterface, CameraConext in turn would
  // Map it to its one of port.
  shared_ptr<TrackSource> track_source = make_shared<TrackSource>(track_params,
                                                                  camera);
  if (!track_source.get()) {
    QMMF_ERROR("%s:%s: Can't create TrackSource Instance", TAG, __func__);
    return NO_MEMORY;
  }

  auto ret = track_source->Init();
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: track_id(%d) TrackSource Init failed!", TAG, __func__,
        track_id);
    goto FAIL;
  }
  track_sources_.add(track_id, track_source);

  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return ret;
FAIL:
  track_source = nullptr;
  return ret;
}

status_t CameraSource::DeleteTrackSource(const uint32_t track_id) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  auto ret = track->DeInit();
  assert(ret == NO_ERROR);

  track_sources_.removeItem(track_id);

  QMMF_INFO("%s:%s: track_id(%d) Deleted Successfully!", TAG, __func__,
      track_id);
  return ret;
}

status_t CameraSource::StartTrackSource(const uint32_t track_id) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  auto ret = track->StartTrack();
  assert(ret == NO_ERROR);

  QMMF_VERBOSE("%s:%s: TrackSource id(%d) Started Succesffuly!", TAG, __func__,
      track_id);
  return ret;
}

status_t CameraSource::StopTrackSource(const uint32_t track_id) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  auto ret = track->StopTrack();
  assert(ret == NO_ERROR);

  QMMF_VERBOSE("%s:%s: TrackSource id(%d) Stopped Succesffuly!", TAG, __func__,
      track_id);
  return ret;
}

status_t CameraSource::PauseTrackSource(const uint32_t track_id) {
  // Not Implemented
  return NO_ERROR;
}

status_t CameraSource::ResumeTrackSource(const uint32_t track_id) {
  // Not Implemented
  return NO_ERROR;
}

status_t CameraSource::ReturnTrackBuffer(const uint32_t track_id,
                                         std::vector<BnBuffer> &buffers) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }

  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);
  auto ret = track->ReturnTrackBuffer(buffers);
  assert(ret == NO_ERROR);
  return ret;
}

status_t CameraSource::SetCameraParam(const uint32_t camera_id,
                                      const CameraMetadata &meta) {

  sp<CameraInterface> camera = camera_map_.valueFor(camera_id);
  assert(camera.get() != nullptr);

  return camera->SetCameraParam(meta);
}

status_t CameraSource::GetCameraParam(const uint32_t camera_id,
                                      CameraMetadata &meta) {

  sp<CameraInterface> camera = camera_map_.valueFor(camera_id);
  assert(camera.get() != nullptr);

  return camera->GetCameraParam(meta);
}

status_t CameraSource::GetDefaultCaptureParam(const uint32_t camera_id,
                                              CameraMetadata &meta) {

  sp<CameraInterface> camera = camera_map_.valueFor(camera_id);
  assert(camera.get() != nullptr);

  return camera->GetDefaultCaptureParam(meta);
}

status_t CameraSource::UpdateTrackFrameRate(const uint32_t track_id,
                                            const uint32_t frame_rate) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  track->UpdateFrameRate(frame_rate);

  return NO_ERROR;
}

status_t CameraSource::CreateOverlayObject(const uint32_t track_id,
                                           OverlayParam *param,
                                           uint32_t *overlay_id) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  auto ret = track->CreateOverlayObject(param, overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CreateOverlayObject failed!", TAG, __func__);
    return BAD_VALUE;
  }
  return ret;
}

status_t CameraSource::DeleteOverlayObject(const uint32_t track_id,
                                           const uint32_t overlay_id) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  auto ret = track->DeleteOverlayObject(overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: DeleteOverlayObject failed!", TAG, __func__);
    return BAD_VALUE;
  }
  return ret;
}

status_t CameraSource::GetOverlayObjectParams(const uint32_t track_id,
                                              const uint32_t overlay_id,
                                              OverlayParam &param) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  auto ret = track->GetOverlayObjectParams(overlay_id, param);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: GetOverlayObjectParams failed!", TAG, __func__);
    return BAD_VALUE;
  }
  return ret;
}

status_t CameraSource::UpdateOverlayObjectParams(const uint32_t track_id,
                                                 const uint32_t overlay_id,
                                                 OverlayParam *param) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  auto ret = track->UpdateOverlayObjectParams(overlay_id, param);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: UpdateOverlayObjectParams failed!", TAG, __func__);
    return BAD_VALUE;
  }
  return ret;
}

status_t CameraSource::SetOverlayObject(const uint32_t track_id,
                                        const uint32_t overlay_id) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  auto ret = track->SetOverlayObject(overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: SetOverlayObject failed!", TAG, __func__);
    return BAD_VALUE;
  }
  return ret;
}

status_t CameraSource::RemoveOverlayObject(const uint32_t track_id,
                                           const uint32_t overlay_id) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  auto ret = track->RemoveOverlayObject(overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: RemoveOverlayObject failed!", TAG, __func__);
    return BAD_VALUE;
  }
  return ret;
}

const shared_ptr<TrackSource>& CameraSource::GetTrackSource(uint32_t track_id) {

  int32_t idx = track_sources_.indexOfKey(track_id);
  assert(idx >= 0);
  return track_sources_.valueFor(track_id);
}

bool CameraSource::IsTrackIdValid(const uint32_t track_id) {

  bool valid = false;
  size_t size = track_sources_.size();
  QMMF_DEBUG("%s: Number of Tracks exist = %d",__func__, size);
  for (size_t i = 0; i < size; i++) {
    if (track_id == track_sources_.keyAt(i)) {
        valid = true;
        break;
    }
  }
  return valid;
}

uint32_t CameraSource::GetJpegSize(uint8_t *blobBuffer, uint32_t width) {

  uint32_t ret = width;
  uint32_t blob_size = sizeof(struct camera3_jpeg_blob);

  if (width > blob_size) {
    size_t offset = width - blob_size;
    uint8_t *footer = blobBuffer + offset;
    struct camera3_jpeg_blob *jpegBlob = (struct camera3_jpeg_blob *)footer;

    if (CAMERA3_JPEG_BLOB_ID == jpegBlob->jpeg_blob_id) {
      ret = jpegBlob->jpeg_size;
    } else {
      QMMF_ERROR("%s:%s Jpeg Blob structure missing!\n", TAG, __func__);
    }
  } else {
    QMMF_ERROR("%s:%s Buffer width: %u equal or smaller than Blob size: %u\n",
        TAG, __func__, width, blob_size);
  }
  return ret;
}

void CameraSource::SnapshotCallback(uint32_t count, StreamBuffer& buffer) {

  uint32_t content_size;
  int32_t width = -1, height = -1;
  void* vaddr = nullptr;
  switch (buffer.info.format) {
    case BufferFormat::kNV12:
    case BufferFormat::kNV21:
    case BufferFormat::kRAW10:
    case BufferFormat::kRAW16:
      width  = buffer.info.plane_info[0].width;
      height = buffer.info.plane_info[0].height;
      content_size = buffer.size;
      break;
    case BufferFormat::kBLOB:
      vaddr = mmap(nullptr, buffer.size, PROT_READ | PROT_WRITE, MAP_SHARED,
          buffer.fd, 0);
      assert(vaddr != nullptr);
      assert(0 < buffer.info.num_planes);
      content_size = GetJpegSize((uint8_t*) vaddr,
                                 buffer.info.plane_info[0].width);
      QMMF_INFO("%s:%s: jpeg buffer size(%d)", TAG, __func__, content_size);
      assert(0 < content_size);
      if (vaddr) {
        munmap(vaddr, buffer.size);
        vaddr = nullptr;
      }
      width  = -1;
      height = -1;
    break;
    default:
      QMMF_ERROR("%s:%s format(%d) not supported", TAG, __func__,
          buffer.info.format);
      assert(0);
    break;
  }

  BnBuffer bn_buffer;
  memset(&bn_buffer, 0x0, sizeof bn_buffer);
  bn_buffer.ion_fd    = buffer.fd;
  bn_buffer.size      = content_size;
  bn_buffer.timestamp = buffer.timestamp;
  bn_buffer.width     = width;
  bn_buffer.height    = height;
  bn_buffer.buffer_id = buffer.fd;
  bn_buffer.capacity  = buffer.size;

  MetaData meta_data;
  memset(&meta_data, 0x0, sizeof meta_data);
  meta_data.meta_flag = static_cast<uint32_t>(MetaParamType::kCamBufMetaData);
  meta_data.cam_buffer_meta_data = buffer.info;
  client_snapshot_cb_(buffer.camera_id, count, bn_buffer, meta_data);
}

TrackSource::TrackSource(const VideoTrackParams& params,
                         const sp<CameraInterface>& camera_intf)
    : track_params_(params),
      is_stop_(false),
      eos_acked_(false),
      enable_overlay_(false),
      display_started_(0) {

  BufferConsumerImpl<TrackSource> *impl;
  impl = new BufferConsumerImpl<TrackSource>(this);
  buffer_consumer_impl_ = impl;
  assert(camera_intf.get() != nullptr);
  camera_interface_ = camera_intf;

  input_frame_rate_ = camera_intf->GetCameraStartParam().frame_rate;
  QMMF_INFO("%s:%s camera_frame_rate =%f", TAG, __func__, input_frame_rate_);
  input_frame_interval_  = 1000000.0 / input_frame_rate_;
  output_frame_interval_ = 1000000.0 / track_params_.params.frame_rate;
  remaining_frame_skip_time_ = output_frame_interval_;
  QMMF_INFO("%s:%s: input_frame_interval_(%f) & output_frame_interval_(%f) & "
      "remaining_frame_skip_time_(%f)", TAG, __func__, input_frame_interval_,
      output_frame_interval_, remaining_frame_skip_time_);

  count_ = 0;
  QMMF_INFO("%s:%s: TrackSource (0x%p)", TAG, __func__, this);
}

TrackSource::~TrackSource() {

  QMMF_INFO("%s:%s: Enter ", TAG, __func__);

  QMMF_INFO("%s:%s: Exit(0x%p) ", TAG, __func__, this);
}

status_t TrackSource::Init() {

  QMMF_DEBUG("%s:%s Enter track_id(%d)", TAG, __func__, TrackId());

  CameraStreamParam stream_param;
  memset(&stream_param, 0x0, sizeof stream_param);
  stream_param.cam_stream_dim.width  = track_params_.params.width;
  stream_param.cam_stream_dim.height = track_params_.params.height;
  if (track_params_.params.format_type == VideoFormat::kBayerRDI) {
    stream_param.cam_stream_format     = CameraStreamFormat::kRAW10;
  } else {
    stream_param.cam_stream_format     = CameraStreamFormat::kNV21;
  }
  stream_param.cam_stream_type  = track_params_.camera_stream_type;
  stream_param.frame_rate       = track_params_.params.frame_rate;
  stream_param.id               = track_params_.track_id;
  stream_param.low_power_mode   = track_params_.params.low_power_mode;

  assert(camera_interface_.get() != nullptr);
  auto ret = camera_interface_->CreateStream(stream_param);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CreateStream failed!!", TAG, __func__);
    return BAD_VALUE;
  }

  QMMF_INFO("%s:%s: TrackSource(0x%p)(%dx%d) and Camera Device Stream "
      " Created Succesffuly for track_id(%d)", TAG, __func__, this,
      track_params_.params.width, track_params_.params.height, TrackId());
  //TODO: Add mechanism to query the stream format from adaptor.
  ret = overlay_.Init(TargetBufferFormat::kYUVNV12);
  assert(ret == NO_ERROR);

  if (track_params_.camera_stream_type == CameraStreamType::kPreview) {
    ret = CreateDisplayPreview(display::DisplayType::kPrimary,
        track_params_);
    if (ret != 0) {
      QMMF_ERROR("%s:%s CreateDisplayPreview Failed!!", TAG, __func__);
      return ret;
    }
  }

  QMMF_DEBUG("%s:%s Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;
}

status_t TrackSource::DeInit() {

  QMMF_DEBUG("%s:%s Enter track_id(%d)", TAG, __func__, TrackId());
  assert(camera_interface_.get() != nullptr);

  auto ret = DeleteDisplayPreview(display::DisplayType::kPrimary);
  assert(ret == NO_ERROR);

  ret = camera_interface_->DeleteStream(TrackId());
  assert(ret == NO_ERROR);

  QMMF_DEBUG("%s:%s Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;
}

status_t TrackSource::StartTrack() {

  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());

  assert(camera_interface_.get() != nullptr);

  Mutex::Autolock lock(stop_lock_);
  is_stop_ = false;
  eos_acked_ = false;

  sp<IBufferConsumer> consumer;
  consumer = GetConsumerIntf();
  assert(consumer.get() != nullptr);

  auto ret = camera_interface_->StartStream(TrackId(), consumer);
  assert(ret == NO_ERROR);

  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return NO_ERROR;
}

status_t TrackSource::StopTrack() {

  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  {
    Mutex::Autolock lock(stop_lock_);
    is_stop_ = true;
  }
  // Stop sequence when encoder is involved.
  // 1. Send EOS to encoder with last valid buffer. If frames_received_ queue is
  //    empty and read thread is waiting for buffers then wait till next buffer
  //    is available then send EOS to encoder. once EOS is notified encoder will
  //    stop calling read method.
  // 2. Once EOS is acknowledged by encoder stop the camera port, which in turn
  //    will break port's connection with TrackSource.
  // 3. Return all the buffers back to camera port from frames_received_ queue
  //    if any. there are very less chances frames_received_ list wil have
  //    buffers after we send EOS to encoder and before we break the connection
  //    between CameraPort and TrackSource, it is very important to check
  //    otherwise camera adaptor will never go in idle state and as a side
  //    effect delete camera stream would fail.
  // 4. Once all buffers are returned at input port of encoder it will notify
  //    the status:kPortIdle, and at this point client's stop method can be
  //    returned.

  bool wait = true;
  if (track_params_.params.format_type == VideoFormat::kYUV ||
      track_params_.params.format_type == VideoFormat::kBayerRDI ||
      track_params_.params.format_type == VideoFormat::kBayerIdeal) {

    //Encoder is not involved in this case.
    assert(camera_interface_.get() != nullptr);
    auto ret = camera_interface_->StopStream(TrackId());
    assert(ret == NO_ERROR);

    Mutex::Autolock autoLock(buffer_list_lock_);
    {
      QMMF_DEBUG("%s:%s: track_id(%d) buffer_list_.size(%d)", TAG, __func__,
          TrackId(), buffer_list_.size());
      if (buffer_list_.size() == 0) {
        wait = false;
      }
    }
  } else {
      QMMF_DEBUG("%s:%s: track_id(%d), Wait for Encoder to return being encoded"
          " buffers!", TAG, __func__, TrackId());
  }
  if (wait) {
    auto ret = wait_for_idle_.waitRelative(idle_lock_, kWaitDuration);
    if (ret == TIMED_OUT) {
        QMMF_ERROR("%s:%s: track_id(%d) StopTrack Timed out happend! Encoder"
        "failed to go in Idle state!", TAG, __func__, TrackId());
      return ret;
    }
  }
  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return NO_ERROR;
}

status_t TrackSource::NotifyPortEvent(PortEventType event_type,
                                      void* event_data) {

  QMMF_DEBUG("%s:%s Enter track_id(%d)", TAG, __func__, TrackId());
  if (event_type == PortEventType::kPortStatus) {
    CodecPortStatus status = *(static_cast<CodecPortStatus*>(event_data));
    if (status == CodecPortStatus::kPortStop) {
      // Encoder Received the EOS with valid last buffer successfully, stop the
      // camera stream and clear the received buffer queue.
      QMMF_INFO("%s:%s: track_id(%d) EOS acknowledged by Encoder!!", TAG,
          __func__, TrackId());
      ClearInputQueue();
      Mutex::Autolock lock(eos_lock_);
      eos_acked_ = true;
    } else if (status == CodecPortStatus::kPortIdle) {
      ClearInputQueue();
      assert(camera_interface_.get() != nullptr);
      auto ret = camera_interface_->StopStream(TrackId());
      assert(ret == NO_ERROR);
      // All input port buffers from encoder are returned, Being encoded queue
      // should be zero at this point.
      assert(frames_being_encoded_.Size() == 0);
      QMMF_INFO("%s:%s: track_id(%d) All queued buffers are returned from"
          " encoder!!", TAG, __func__, TrackId());
      // wait_for_idle_ will not be needed once we make stop api as async.
      Mutex::Autolock lock(idle_lock_);
      wait_for_idle_.signal();
    }
  }

  QMMF_DEBUG("%s:%s Exit track_id(%d)", TAG, __func__, TrackId());
  return NO_ERROR;
}

status_t TrackSource::GetBuffer(BufferDescriptor& buffer,
                                void* client_data) {

  QMMF_DEBUG("%s:%s Enter track_id(%d)", TAG, __func__, TrackId());
  bool timeout = false;
  {
    Mutex::Autolock lock(lock_);
    if (frames_received_.Size() == 0) {
      QMMF_DEBUG("%s:%s: track_id(%d) Wait for bufferr!!", TAG, __func__,
          TrackId());
      auto ret = wait_for_frame_.waitRelative(lock_, kWaitDuration);
      if (ret == TIMED_OUT) {
          QMMF_ERROR("%s:%s: track_id(%d) Buffer Timed out happend! No buffers"
              "from Camera", TAG, __func__, TrackId());
          timeout = true;
      }
    }
    assert(timeout == false);

    QMMF_VERBOSE("%s:%s: track_id(%d) frames_received_.size(%d)", TAG, __func__,
        TrackId(), frames_received_.Size());

    StreamBuffer stream_buffer = *frames_received_.Begin();
    buffer.data =
        const_cast<void*>(reinterpret_cast<const void*>(stream_buffer.handle));
    buffer.fd = stream_buffer.fd;
    buffer.capacity = stream_buffer.frame_length;
    buffer.size = stream_buffer.filled_length;
    buffer.timestamp = stream_buffer.timestamp;
    buffer.flag = stream_buffer.flags;
    frames_being_encoded_.PushBack(stream_buffer);
    frames_received_.Erase(frames_received_.Begin());
  }

  if (IsStop()) {
    QMMF_DEBUG("%s:%s: track_id(%d) Send EOS to Encoder!", TAG, __func__,
        TrackId());
    // TODO defile EOS flag in AVCodec to delete EOS.
    return -1;
  }
  QMMF_DEBUG("%s:%s Exit track_id(%d)", TAG, __func__, TrackId());
  return NO_ERROR;
}

status_t TrackSource::ReturnBuffer(BufferDescriptor& buffer,
                                   void* client_data) {

  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());

  QMMF_VERBOSE("%s:%s: track_id(%d) frames_being_encoded_.size(%d)", TAG,
      __func__, TrackId(), frames_being_encoded_.Size());

  bool found = false;
  auto iter = frames_being_encoded_.Begin();
  for (; iter != frames_being_encoded_.End(); ++iter) {
    if ((*iter).handle ==  buffer.data) {
      QMMF_VERBOSE("%s:%s: Buffer found in frames_being_encoded_ list!", TAG,
          __func__);
      ReturnBufferToProducer((*iter));
      frames_being_encoded_.Erase(iter);
      found = true;
      break;
    }
  }
  assert(found == true);
  QMMF_VERBOSE("%s:%s: frames_being_encoded_.Size(%d)", TAG, __func__,
      frames_being_encoded_.Size());

  QMMF_DEBUG("%s:%s Exit track_id(%d)", TAG, __func__, TrackId());
  return NO_ERROR;
}

void TrackSource::OnFrameAvailable(StreamBuffer& buffer) {

  QMMF_VERBOSE("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());

#ifdef NO_FRAME_PROCESS
  ReturnBufferToProducer(buffer);
  return;
#endif

  {
    Mutex::Autolock lock(eos_lock_);
    if (eos_acked_ && IsStop()) {
      auto track_format = track_params_.params.format_type;
      if (track_format == VideoFormat::kAVC
          || track_format == VideoFormat::kHEVC) {
        // Return buffer if track is stoped and EOS is acknowledged by AVCodec.
        QMMF_INFO("%s:%s: Track(%d) Stoped and eos is acked!", TAG, __func__,
          TrackId());
        ReturnBufferToProducer(buffer);
        return;
      }
    }
  }

  // Return buffer back to camera if frameskip is valid for this frame
  // and is NOT a stop condition. In STOP condition, frame skip logic
  // is bypassed as the buffer consumer may wait for the last buffer
  // as part of stop processing. Skipping frames may result in timeouts
  // in the consumer.
  if ((!IsStop()) && IsFrameSkip()) {
    // Skip frame to adjust fps.
    ReturnBufferToProducer(buffer);
    return;
  }

  // Dynamic FPS measurement
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  uint64_t time_diff = (uint64_t)((tv.tv_sec * 1000000 + tv.tv_usec) -
                       (prevtv_.tv_sec * 1000000 + prevtv_.tv_usec));
  count_++;
  if (time_diff >= FPS_TIME_INTERVAL) {
    float framerate = (count_ * 1000000) / (float)time_diff;
    bool is_first_time = (framerate <= 1.0);

    // Re-calculate input and output frame intervals if input framerate
    // is different from its previous value
    if (!(is_first_time) &&
        (fabs(input_frame_rate_ - framerate) >= FPS_CHANGE_THRESHOLD)) {
      Mutex::Autolock autoLock(frame_skip_lock_);
      QMMF_INFO("%s:%s: track_id(%d) adjusting fps from (%0.2f) to (%0.2f)",
                TAG, __func__, TrackId(), input_frame_rate_, framerate);
      input_frame_rate_ = framerate;
      input_frame_interval_  = 1000000.0 / input_frame_rate_;
      // Output fps cannot be more than input fps.
      if (input_frame_rate_ < track_params_.params.frame_rate)
        output_frame_interval_ = 1000000.0 / input_frame_rate_;
      else
        output_frame_interval_ = 1000000.0 / track_params_.params.frame_rate;
      remaining_frame_skip_time_ = output_frame_interval_;
    }
#ifdef DEBUG_TRACK_FPS
    QMMF_INFO("%s:%s: track_id(%d):fps: = %0.2f", TAG, __func__,
              TrackId(), framerate);
#endif
    prevtv_ = tv;
    count_ = 0;
  }

  QMMF_VERBOSE("%s:%s: track_id(%d) numInts = %d", TAG, __func__, TrackId(),
      buffer.handle->numInts);
  for (int32_t i = 0; i < buffer.handle->numInts; i++) {
    QMMF_VERBOSE("%s:%s: track_id(%d) data[%d] =%d", TAG, __func__, TrackId(),
        i , buffer.handle->data[i]);
  }

  QMMF_VERBOSE("%s:%s: track_id(%d) ion_fd = %d", TAG, __func__, TrackId(),
      buffer.fd);
  QMMF_VERBOSE("%s:%s: track_id(%d) size = %d", TAG, __func__, TrackId(),
      buffer.size);

  if (enable_overlay_) {
    OverlayTargetBuffer overlay_buf;
    //TODO: get format from streamBuffer.
    overlay_buf.format    = TargetBufferFormat::kYUVNV12;
    overlay_buf.width     = buffer.info.plane_info[0].width;
    overlay_buf.height    = buffer.info.plane_info[0].height;
    overlay_buf.ion_fd    = buffer.fd;
    overlay_buf.frame_len = buffer.size;
    overlay_.ApplyOverlay(overlay_buf);
  }
#ifdef ENABLE_FRAME_DUMP
  DumpYUV(buffer);
#endif
  if (track_params_.camera_stream_type == CameraStreamType::kPreview) {
    PushFrameToDisplay(buffer);
    ReturnBufferToProducer(buffer);
  } else {
    // If format type is YUV or BAYER then give callback from this point, do not
    // feed buffer to Encoder.
    if (track_params_.params.format_type == VideoFormat::kYUV ||
        track_params_.params.format_type == VideoFormat::kBayerRDI ||
        track_params_.params.format_type == VideoFormat::kBayerIdeal) {

      if (IsStop()) {
        QMMF_DEBUG("%s:%s: track_id(%d) Stop is triggred, Stop giving raw buffer"
            " to client!", TAG, __func__, TrackId());
        ReturnBufferToProducer(buffer);
        return;
      }

      BnBuffer bn_buffer;
      memset(&bn_buffer, 0x0, sizeof bn_buffer);
      bn_buffer.ion_fd         = buffer.fd;
      bn_buffer.size           = buffer.size;
      bn_buffer.timestamp      = buffer.timestamp;
      bn_buffer.width          = buffer.info.plane_info[0].width;
      bn_buffer.height         = buffer.info.plane_info[0].height;
      bn_buffer.buffer_id      = buffer.fd;
      bn_buffer.flag           = 0x10;
      bn_buffer.capacity       = buffer.size;

      // Buffers from this list used for YUV callback.
      {
        Mutex::Autolock autoLock(buffer_list_lock_);
        buffer_list_.add(buffer.fd, buffer);
      }
      std::vector<BnBuffer> bn_buffers;
      bn_buffers.push_back(bn_buffer);

      MetaData meta_data;
      memset(&meta_data, 0x0, sizeof meta_data);
      meta_data.meta_flag = static_cast<uint32_t>(MetaParamType::kCamBufMetaData);
      meta_data.cam_buffer_meta_data = buffer.info;

      std::vector<MetaData> meta_buffers;
      meta_buffers.push_back(meta_data);

      track_params_.data_cb(TrackId(), bn_buffers, meta_buffers);
    } else {
      // Push buffers into encoder queue.
      PushFrameToQueue(buffer);
    }
  }
}

status_t TrackSource::ReturnTrackBuffer(std::vector<BnBuffer>& bn_buffers) {

  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  assert(bn_buffers.size() > 0);
  assert(buffer_consumer_impl_ != nullptr);

  for (size_t i = 0; i < bn_buffers.size(); ++i) {
    QMMF_VERBOSE("%s:%s: track_id(%d) bn_buffers[%d].ion_fd=%d", TAG, __func__,
        TrackId(), i, bn_buffers[i].ion_fd);
    {
      Mutex::Autolock autoLock(buffer_list_lock_);
      int32_t idx = buffer_list_.indexOfKey(bn_buffers[i].ion_fd);
      QMMF_DEBUG("%s:%s: track_id(%d) Buffer fd(%d) found in list", TAG,
          __func__, TrackId(), bn_buffers[i].ion_fd);
      assert(idx >= 0);
      StreamBuffer buffer = buffer_list_.valueFor(bn_buffers[i].ion_fd);
      ReturnBufferToProducer(buffer);
      buffer_list_.removeItem(bn_buffers[i].ion_fd);
    }
  }
  if (IsStop()) {
    if (buffer_list_.size() > 0) {
      QMMF_INFO("%s:%s: track_id(%d) Stop is triggered, but still num raw "
          "buffers(%d) are with client!", TAG, __func__, TrackId(),
          buffer_list_.size());
    } else {
      // wait_for_idle_ will not be needed once we make stop api as async.
      QMMF_INFO("%s:%s: track_id(%d) Stop is triggered, all raw buffers are"
          " returned from client!", TAG, __func__, TrackId());
      Mutex::Autolock lock(idle_lock_);
      wait_for_idle_.signal();
    }
  }
  QMMF_VERBOSE("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return NO_ERROR;
}

void TrackSource::PushFrameToQueue(StreamBuffer& buffer) {

  QMMF_VERBOSE("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());

  Mutex::Autolock lock(lock_);
  frames_received_.PushBack(buffer);
  QMMF_DEBUG("%s:%s: track_id(%d) frames_received.size(%d)", TAG, __func__,
      TrackId(), frames_received_.Size());
  wait_for_frame_.signal();

  QMMF_VERBOSE("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
}

bool TrackSource::IsStop() {

  QMMF_VERBOSE("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  Mutex::Autolock lock(stop_lock_);
  QMMF_VERBOSE("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return is_stop_;
}

void TrackSource::ClearInputQueue() {

  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  // Once connection is broken b/w port and trackSoure there is no chance to
  // get new buffers in frames_received_ queue.
  uint32_t size = frames_received_.Size();
  QMMF_INFO("%s:%s: track_id(%d): (%d) buffers to return from frames_received_",
      TAG, __func__, TrackId(), size);
  assert(buffer_consumer_impl_->GetProducerHandle().get() != nullptr);
  auto iter = frames_received_.Begin();
  for (; iter != frames_received_.End(); ++iter) {
    ReturnBufferToProducer((*iter));
  }
  frames_received_.Clear();
  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
}

status_t TrackSource::CreateOverlayObject(OverlayParam *param,
                                          uint32_t *overlay_id) {

  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  uint32_t id;
  auto ret = overlay_.CreateOverlayItem(*param, &id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: createOverlayItem failed!", TAG, __func__);
    return BAD_VALUE;
  }
  *overlay_id = id;
  QMMF_INFO("%s:%s: OverlayItem of type(%d) created! id(%d)", TAG, __func__,
      param->type, id);
  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;
}

status_t TrackSource::DeleteOverlayObject(const uint32_t overlay_id) {

  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  auto ret = overlay_.DeleteOverlayItem(overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: deleteOverlayItem failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;
}

status_t TrackSource::GetOverlayObjectParams(const uint32_t overlay_id,
                                             OverlayParam &param) {

  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  auto ret = overlay_.GetOverlayParams(overlay_id, param);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: getOverlayItemParams failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;
}

status_t TrackSource::UpdateOverlayObjectParams(const uint32_t overlay_id,
                                                OverlayParam *param) {

  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  auto ret = overlay_.UpdateOverlayParams(overlay_id, *param);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: updateOverlayParams failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;
}

status_t TrackSource::SetOverlayObject(const uint32_t overlay_id) {

  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  auto ret = overlay_.EnableOverlayItem(overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: enableOverlayItem failed!", TAG, __func__);
    return BAD_VALUE;
  }
  enable_overlay_ = true;
  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;
}

status_t TrackSource::RemoveOverlayObject(const uint32_t overlay_id) {

  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  auto ret = overlay_.DisableOverlayItem(overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: disableOverlayItem failed!", TAG, __func__);
    return BAD_VALUE;
  }
  enable_overlay_ = false;
  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;
}

void TrackSource::UpdateFrameRate(const uint32_t frame_rate) {

  Mutex::Autolock autoLock(frame_skip_lock_);
  assert(frame_rate > 0);

  if (track_params_.params.frame_rate != frame_rate) {
      QMMF_INFO("%s:%s: track_id(%d) Track fps changed from (%d) to (%d)", TAG,
          __func__, TrackId(), track_params_.params.frame_rate, frame_rate);
    track_params_.params.frame_rate = frame_rate;
    output_frame_interval_ = 1000000.0 / frame_rate;
    remaining_frame_skip_time_ = output_frame_interval_;
    QMMF_INFO("%s:%s: remaining_frame_skip_time_(%f)", TAG, __func__,
        remaining_frame_skip_time_);
  }
}

bool TrackSource::IsFrameSkip() {

  Mutex::Autolock autoLock(frame_skip_lock_);
  bool skip;
  remaining_frame_skip_time_ -= input_frame_interval_;
  if (0 >= remaining_frame_skip_time_) {
    skip = false;
    remaining_frame_skip_time_ += output_frame_interval_;
  } else {
    skip = true;
  }
  return skip;
}

status_t TrackSource::CreateDisplayPreview(display::DisplayType display_type,
    const VideoTrackParams& track_param) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  int32_t res = 0;
  DisplayCb  display_status_cb;
  SurfaceConfig surface_config;

  display_= new Display();
  assert(display_ != nullptr);
  res = display_->Connect();
  if (res != 0) {
    QMMF_ERROR("%s:%s Display Connect Failed!!", TAG, __func__);
    delete display_;
    display_ = nullptr;
    return res;
  }

  display_status_cb.EventCb = [&] ( DisplayEventType event_type,
      void *event_data, size_t event_data_size) { DisplayCallbackHandler
      (event_type, event_data, event_data_size); };

  display_status_cb.VSyncCb = [&] ( int64_t time_stamp)
      { DisplayVSyncHandler(time_stamp); };

  res = display_->CreateDisplay(display_type, display_status_cb);
  if (res != 0) {
    QMMF_ERROR("%s:%s CreateDisplay Failed!!", TAG, __func__);
    display_->Disconnect();
    delete display_;
    display_ = nullptr;
    return res;
  }

  memset(&surface_config, 0x0, sizeof surface_config);

  surface_config.width = track_param.params.width;
  surface_config.height = track_param.params.height;
  surface_config.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
  surface_config.buffer_count = 1;
  surface_config.cache = 0;
  surface_config.use_buffer = 1;
  res = display_->CreateSurface(surface_config, &surface_id_);
  if (res != 0) {
    QMMF_ERROR("%s:%s CreateSurface Failed!!", TAG, __func__);
    DeleteDisplayPreview(display_type);
    return res;
  }
  display_started_ = 1;

  surface_param_.src_rect = { 0.0, 0.0, (float)track_param.params.width,
      (float)track_param.params.height };
  surface_param_.dst_rect = { 0.0, 0.0, (float)track_param.params.width,
      (float)track_param.params.height };
  surface_param_.surface_blending =
      SurfaceBlending::kBlendingCoverage;
  surface_param_.surface_flags.cursor = 0;
  surface_param_.frame_rate = track_param.params.frame_rate;
  surface_param_.z_order = 0;
  surface_param_.solid_fill_color = 0;
  surface_param_.surface_transform.rotation = 0.0f;
  surface_param_.surface_transform.flip_horizontal = 0;
  surface_param_.surface_transform.flip_vertical = 0;

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return res;

}

status_t TrackSource::DeleteDisplayPreview(display::DisplayType display_type) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  int32_t res = 0;
  if (display_started_ == 1) {
    display_started_ = 0;
    res = display_->DestroySurface(surface_id_);
    if (res != 0) {
      QMMF_ERROR("%s:%s DestroySurface Failed!!", TAG, __func__);
    }

    res = display_->DestroyDisplay(display_type);
    if (res != 0) {
      QMMF_ERROR("%s:%s DestroyDisplay Failed!!", TAG, __func__);
    }

    res = display_->Disconnect();

    if (display_ != nullptr) {
      QMMF_INFO("%s:%s: DELETE display_:%p", TAG, __func__, display_);
      delete display_;
      display_ = nullptr;
    }
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return res;
}

void TrackSource::DisplayCallbackHandler(DisplayEventType event_type,
    void *event_data, size_t event_data_size) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

void TrackSource::DisplayVSyncHandler(int64_t time_stamp)
{
  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);
  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
}

status_t TrackSource::PushFrameToDisplay(StreamBuffer& buffer) {
  int32_t ret = 0;
  void *buf_vaaddr = mmap(nullptr, buffer.size, PROT_READ  | PROT_WRITE,
                          MAP_SHARED, buffer.fd, 0);
  assert(buf_vaaddr != nullptr);

  if (display_started_) {
    surface_buffer_.plane_info[0].ion_fd = buffer.fd;
    surface_buffer_.buf_id = 0;
    surface_buffer_.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
    surface_buffer_.plane_info[0].stride = buffer.info.plane_info[0].stride;
    surface_buffer_.plane_info[0].size = buffer.frame_length;
    surface_buffer_.plane_info[0].width = buffer.info.plane_info[0].width;
    surface_buffer_.plane_info[0].height = buffer.info.plane_info[0].height;
    surface_buffer_.plane_info[0].offset = 0;
    surface_buffer_.plane_info[0].buf = (void*)buf_vaaddr;

    ret = display_->QueueSurfaceBuffer(surface_id_, surface_buffer_,
        surface_param_);
    if (buf_vaaddr != nullptr) {
      munmap(buf_vaaddr, buffer.size);
      buf_vaaddr = nullptr;
    }
    if (ret != 0) {
      QMMF_ERROR("%s:%s QueueSurfaceBuffer Failed!!", TAG, __func__);
      return ret;
    }

    ret = display_->DequeueSurfaceBuffer(surface_id_, surface_buffer_);
    if (ret != 0) {
      QMMF_ERROR("%s:%s DequeueSurfaceBuffer Failed!!", TAG, __func__);
      return ret;
    }
  }
  return ret;
}

void TrackSource::ReturnBufferToProducer(StreamBuffer& buffer) {
  assert(buffer_consumer_impl_ != nullptr);
  buffer_consumer_impl_->GetProducerHandle()->NotifyBufferReturned(buffer);
}

#ifdef ENABLE_FRAME_DUMP
status_t TrackSource::DumpYUV(StreamBuffer& buffer) {

  static uint32_t id;
  ++id;
  // Dump every 100th frame.
  if (id == 100) {

    void *buf_vaaddr = mmap(nullptr, buffer.size, PROT_READ  | PROT_WRITE,
                            MAP_SHARED, buffer.fd, 0);
    assert(buf_vaaddr != nullptr);

    String8 file_path;
    size_t written_len;
    file_path.appendFormat(FRAME_DUMP_PATH"/track_%d_%lld.yuv",
        TrackId(), buffer.timestamp);

    FILE *file = fopen(file_path.string(), "w+");
    if (!file) {
      QMMF_ERROR("%s:%s: Unable to open file(%s)", TAG, __func__,
          file_path.string());
      goto FAIL;
    }
    written_len = fwrite(buf_vaaddr, sizeof(uint8_t), buffer.size,
        file);
    QMMF_INFO("%s:%s: written_len =%d", TAG, __func__, written_len);

    if (buffer.size != written_len) {
      QMMF_ERROR("%s:%s: Bad Write error (%d):(%s)\n", TAG, __func__, errno,
          strerror(errno));
        goto FAIL;
    }
    QMMF_INFO("%s:%s: Buffer(0x%p) Size(%u) Stored(%s)\n",__func__,
        buf_vaaddr, written_len, file_path.string());

FAIL:
    if (file != nullptr) {
      fclose(file);
    }
    if (buf_vaaddr != nullptr) {
      munmap(buf_vaaddr, buffer.size);
      buf_vaaddr = nullptr;
    }
    id = 0;
  }
}
#endif

}; //namespace recorder

}; //namespace qmmf
