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

#define TAG "RecorderMultiCameraManager"

#include <algorithm>
#include <fcntl.h>
#include <dlfcn.h>
#include <inttypes.h>
#include <cstdlib>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <hardware/hardware.h>
#include <QCamera3VendorTags.h>
#include <cutils/properties.h>

#include "recorder/src/service/qmmf_multicamera_manager.h"
#include "recorder/src/service/qmmf_camera_context.h"
#include "recorder/src/service/qmmf_recorder_utils.h"

namespace qmmf {

namespace recorder {

static const char *kStitchLib = "/vendor/lib/libqmmf_alg_polaris_stitch.so";
static const char *kStitchCalibFile = "";

MultiCameraManager::MultiCameraManager()
  : virtual_camera_id_(kVirtualCameraIdOffset),
    multicam_start_params_{},
    snapshot_param_{0, 0, 0, ImageFormat::kJPEG},
    sequence_cnt_(0),
    postprocess_enable_(false),
    client_snapshot_cb_(nullptr),
    pproc_memory_pool_(nullptr) {}

MultiCameraManager::~MultiCameraManager() {}

status_t MultiCameraManager::CreateMultiCamera(const std::vector<uint32_t>
                                               camera_ids,
                                               uint32_t* virtual_camera_id) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  if (camera_ids.size() < 2) {
    return BAD_VALUE;
  }
  QMMF_INFO("%s:%s: Number of camera to be used(%d)", TAG, __func__,
    camera_ids.size());

  Vector<uint32_t> ids;
  for (uint32_t i = 0; i < camera_ids.size(); ++i) {
    QMMF_INFO("%s:%s camera id=%d", TAG, __func__, camera_ids[i]);
    ids.push_back(camera_ids[i]);
  }
  ++virtual_camera_id_;
  virtual_camera_map_.add(virtual_camera_id_, ids);
  *virtual_camera_id = virtual_camera_id_;

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

// TODO: define MultiCameraConfigTypes in qmmf_recorder_param.h
status_t MultiCameraManager::ConfigureMultiCamera(uint32_t virtual_camera_id,
                                                  /*MultiCameraConfigTypes*/
                                                  uint32_t type,
                                                  void *param,
                                                  size_t param_size) {
  //TODO:
  return NO_ERROR;
}

status_t MultiCameraManager::OpenCamera(const uint32_t virtual_camera_id,
                                        const CameraStartParam &param,
                                        const ResultCb &cb) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  status_t ret = NO_ERROR;

  ssize_t idx = virtual_camera_map_.indexOfKey(virtual_camera_id);
  if (NAME_NOT_FOUND == idx) {
    QMMF_ERROR("%s:%s: Invalid virtual camera ID!", TAG, __func__);
    return BAD_VALUE;
  }
  Vector<uint32_t> camera_ids = virtual_camera_map_.valueFor(virtual_camera_id);
  QMMF_INFO("%s:%s: Total Number of cameras to be open(%d)", TAG, __func__,
      camera_ids.size());

  for (auto const& cam_id : camera_ids) {
    if (camera_contexts_.indexOfKey(cam_id) >= 0) {
      QMMF_WARN("%s:%s: Camera Id(%u) is already open, skipping!", TAG,
          __func__, cam_id);
      continue;
    }
    QMMF_INFO("%s:%s camera id(%d) to be open", TAG, __func__, cam_id);

    sp<CameraContext> camera_context = new CameraContext();
    ret = camera_context->OpenCamera(cam_id, param);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: OpenCamera(%d) failed!", TAG, __func__, cam_id);
      camera_context.clear();
      return NO_INIT;
    }
    camera_contexts_.add(cam_id, camera_context);
  }

  multicam_start_params_ = param;
  supported_fps_ = camera_contexts_.valueAt(0)->GetSupportedFps();

  StitchingBase::InitParams algo_param {};
  algo_param.virtual_camera_id = virtual_camera_id_;
  algo_param.camera_ids = virtual_camera_map_.valueFor(virtual_camera_id_);

  snapshot_stitch_algo_ = new SnapshotStitching(algo_param, camera_contexts_);
  ret = snapshot_stitch_algo_->Initialize();
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Failed to initialize stitching algo!", TAG, __func__);
    return ret;
  }

  multi_camera_pproc_ = new CameraJpeg();
  pproc_memory_pool_ = new GrallocMemory();
  ret = pproc_memory_pool_->Initialize();
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: Post-processing memory pool initialization failed!",
               TAG, __func__);
    return NO_INIT;
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t MultiCameraManager::CloseCamera(const uint32_t virtual_camera_id) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  status_t ret = NO_ERROR;
  bool closing_failed = false;

  ssize_t idx = virtual_camera_map_.indexOfKey(virtual_camera_id);
  if (NAME_NOT_FOUND == idx) {
    QMMF_ERROR("%s:%s: Invalid virtual camera ID!", TAG, __func__);
    return BAD_VALUE;
  }
  Vector<uint32_t> camera_ids = virtual_camera_map_.valueFor(virtual_camera_id);
  QMMF_INFO("%s:%s: Total Number of cameras to be closed(%d)", TAG, __func__,
      camera_ids.size());

  snapshot_stitch_algo_->RequestExitAndWait();
  snapshot_stitch_algo_.clear();

  for (auto const& cam_id : camera_ids) {
    QMMF_INFO("%s:%s camera id(%d) to be closed", TAG, __func__, cam_id);

    sp<CameraContext> camera_context = camera_contexts_.valueFor(cam_id);
    ret = camera_context->CloseCamera(cam_id);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: CloseCamera(%d) failed!", TAG, __func__, cam_id);
      closing_failed = true;
      camera_context.clear();
    }
    camera_contexts_.removeItem(cam_id);
  }

  multi_camera_pproc_->Delete();

  delete pproc_memory_pool_;

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return closing_failed ? UNKNOWN_ERROR : NO_ERROR;
}

status_t MultiCameraManager::CaptureImage(const ImageParam &param,
                                          const uint32_t num_images,
                                          const std::vector<CameraMetadata>
                                          &meta, const StreamSnapshotCb& cb) {
  Mutex::Autolock lock(lock_);
  status_t ret = NO_ERROR;

  if (multicam_start_params_.zsl_mode) {
    QMMF_ERROR("%s:%s: ZSL not supported!", TAG, __func__);
    return BAD_VALUE;
  }

  postprocess_enable_ = false;
  client_snapshot_cb_ = nullptr;
  ImageFormat image_format = param.image_format;
  if (param.image_format == ImageFormat::kJPEG) {
    postprocess_enable_ = true;
    image_format = ImageFormat::kNV12;
  }

  bool reconfigure_needed = (snapshot_param_.width != param.width) ||
      (snapshot_param_.height != param.height) ||
      (sequence_cnt_ != num_images);

  if (reconfigure_needed) {
    snapshot_stitch_algo_->RequestExitAndWait();

    if (postprocess_enable_) {
      multi_camera_pproc_->Delete();
      SetPostProcess(param, image_format, multicam_start_params_.frame_rate);
    }

    // Set buffer params for stitching.
    GrallocMemory::BufferParams buffer_param {};
    buffer_param.format           = ImageToHalFormat(image_format);
    buffer_param.width            = param.width;
    buffer_param.height           = param.height;
    buffer_param.gralloc_flags    = GRALLOC_USAGE_SW_WRITE_OFTEN;
    buffer_param.max_size         = 0;
    buffer_param.max_buffer_count = num_images;

    QMMF_INFO("%s:%s: W(%d) & H(%d)", TAG, __func__, buffer_param.width,
        buffer_param.height);
    ret = snapshot_stitch_algo_->Configure(buffer_param);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s: Failed to configure buffer params!", TAG, __func__);
      return ret;
    }

    // Set buffer params for post-processing.
    GrallocMemory::BufferParams gbuffer_param{};
    gbuffer_param.format           = HAL_PIXEL_FORMAT_BLOB;
    gbuffer_param.width            = param.width;
    gbuffer_param.height           = param.height;
    gbuffer_param.gralloc_flags    = GRALLOC_USAGE_SW_WRITE_OFTEN;
    // TODO: Need to revisit the calculation of max_size.
    //       Width and height need to be extracted from metadata.
    gbuffer_param.max_size         = (param.width * param.height) * 2;
    gbuffer_param.max_buffer_count = num_images;
    ret = pproc_memory_pool_->Configure(gbuffer_param);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s: Failed to configure buffer params!", TAG, __func__);
      return ret;
    }

    snapshot_param_ = param;
    sequence_cnt_   = num_images;
    snapshot_stitch_algo_->Run();
  }

  ImageParam cam_param (param);
  cam_param.image_format = image_format;
  ReCalculateWidth(cam_param.width);

  if (postprocess_enable_) {
    StreamSnapshotCb pproc_cb = [&] (uint32_t count, StreamBuffer& buf) {
      PostprocessCaptureCallback(buf);
    };
    snapshot_stitch_algo_->SetClientCallback(pproc_cb);
    client_snapshot_cb_ = cb;
  } else {
    snapshot_stitch_algo_->SetClientCallback(cb);
  }

  StreamSnapshotCb stream_cb = [&] (uint32_t count, StreamBuffer& buf) {
    snapshot_stitch_algo_->FrameAvailableCb(count, buf);
  };
  for (size_t i = 0; i < camera_contexts_.size(); ++i) {
    sp<CameraContext> camera_context = camera_contexts_.valueAt(i);
    ret = camera_context->CaptureImage(cam_param, num_images, meta, stream_cb);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: CaptureImage Failed!", TAG, __func__);
      return ret;
    }
  }
  return ret;
}

status_t MultiCameraManager::CancelCaptureImage() {
  //TODO
  return NO_ERROR;
}

status_t MultiCameraManager::CreateStream(const CameraStreamParam& param) {

  status_t ret = NO_ERROR;
  CameraStreamParam context_param (param);
  ReCalculateWidth(context_param.cam_stream_dim.width);

  for (size_t i = 0; i < camera_contexts_.size(); ++i) {
    sp<CameraContext> camera_context = camera_contexts_.valueAt(i);
    assert(camera_context.get() != nullptr);
    ret = camera_context->CreateStream(context_param);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: CameraContext CreateStream Failed!", TAG, __func__);
      return ret;
    }
  }

  StitchingBase::InitParams algo_param {};
  algo_param.virtual_camera_id = virtual_camera_id_;
  algo_param.camera_ids = virtual_camera_map_.valueFor(virtual_camera_id_);

  GrallocMemory::BufferParams buf_param {};
  if (param.cam_stream_format != CameraStreamFormat::kRAW10) {
    buf_param.format      = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  } else {
    buf_param.format      = HAL_PIXEL_FORMAT_RAW10;
  }
  buf_param.width         = param.cam_stream_dim.width;
  buf_param.height        = param.cam_stream_dim.height;
  buf_param.gralloc_flags = GRALLOC_USAGE_SW_WRITE_OFTEN;
  buf_param.max_size      = 0;

  buf_param.max_buffer_count = VIDEO_STREAM_BUFFER_COUNT;
  if (param.cam_stream_dim.width == kWidth4K &&
      param.cam_stream_dim.height == kHeight4K) {
    buf_param.max_buffer_count += EXTRA_DCVS_BUFFERS;
  }
  buf_param.gralloc_flags |= private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;

  sp<StreamStitching> stitching_algo = new StreamStitching(algo_param);
  ret = stitching_algo->Initialize();
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Failed to initialize stitching algo!", TAG, __func__);
    goto FAIL;
  }
  ret = stitching_algo->Configure(buf_param);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Failed to configure stitching algo!", TAG, __func__);
    goto FAIL;
  }
  stream_stitch_algos_.add(param.id, stitching_algo);
  return ret;

FAIL:
  stitching_algo.clear();
  for (size_t i = 0; i < camera_contexts_.size(); i++) {
    camera_contexts_.valueAt(i)->DeleteStream(context_param.id);
  }
  return ret;
}

status_t MultiCameraManager::DeleteStream(const uint32_t track_id) {

  status_t ret = NO_ERROR;
  for (size_t i = 0; i < camera_contexts_.size(); ++i) {
    sp<CameraContext> camera_context = camera_contexts_.valueAt(i);
    assert(camera_context.get() != nullptr);
    ret = camera_context->DeleteStream(track_id);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: DeleteStream Failed!", TAG, __func__);
      return ret;
    }
  }

  stream_stitch_algos_.editValueFor(track_id).clear();
  stream_stitch_algos_.removeItem(track_id);
  return ret;
}

status_t MultiCameraManager::StartStream(const uint32_t track_id,
                                         sp<IBufferConsumer>& consumer) {

  status_t ret = NO_ERROR;

  sp<StreamStitching> stitching_algo = stream_stitch_algos_.valueFor(track_id);
  assert(stitching_algo.get() != nullptr);

  stitching_algo->AddConsumer(consumer);
  stitching_algo->Run();

  for (uint32_t i = 0; i < camera_contexts_.size(); ++i) {
    sp<CameraContext> camera_context = camera_contexts_.valueAt(i);
    assert(camera_context.get() != nullptr);

    sp<IBufferConsumer> consumer =
        stitching_algo->GetConsumerIntf(camera_contexts_.keyAt(i));
    assert(consumer.get() != nullptr);

    ret = camera_context->StartStream(track_id, consumer);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: StartStream Failed!", TAG, __func__);
      return ret;
    }
  }
  return ret;
}

status_t MultiCameraManager::StopStream(const uint32_t track_id) {

  status_t ret = NO_ERROR;
  sp<StreamStitching> stitching_algo = stream_stitch_algos_.valueFor(track_id);
  assert(stitching_algo.get() != nullptr);

  stitching_algo->RequestExitAndWait();
  stitching_algo->RemoveConsumer();

  for (size_t i = 0; i < camera_contexts_.size(); ++i) {
    sp<CameraContext> camera_context = camera_contexts_.valueAt(i);
    assert(camera_context.get() != nullptr);

    ret = camera_context->StopStream(track_id);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: StopStream Failed!", TAG, __func__);
      return ret;
    }
  }
  return ret;
}

status_t MultiCameraManager::SetCameraParam(const CameraMetadata &meta) {

  //One of the cameras will be master cam that's why we need
  //to set the params only for one camera.
  sp<CameraContext> camera_context = camera_contexts_.valueAt(0);
  assert(camera_context.get() != nullptr);
  status_t ret = camera_context->SetCameraParam(meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: SetCameraParam Failed!", TAG, __func__);
  }
  return ret;
}

status_t MultiCameraManager::GetCameraParam(CameraMetadata &meta) {

  sp<CameraContext> camera_context = camera_contexts_.valueAt(0);
  assert(camera_context.get() != nullptr);
  status_t ret = camera_context->GetCameraParam(meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: GetCameraParam Failed!", TAG, __func__);
  }
  return ret;
}

status_t MultiCameraManager::GetDefaultCaptureParam(CameraMetadata &meta) {

  sp<CameraContext> camera_context = camera_contexts_.valueAt(0);
  assert(camera_context.get() != nullptr);
  status_t ret = camera_context->GetDefaultCaptureParam(meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: GetDefaultCaptureParam Failed!", TAG, __func__);
  }
  return ret;
}

status_t MultiCameraManager::ReturnImageCaptureBuffer(const uint32_t camera_id,
                                                      const int32_t buffer_id) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  Mutex::Autolock lock(pproc_lock_);
  ssize_t idx = virtual_camera_map_.indexOfKey(camera_id);
  if (idx == NAME_NOT_FOUND) {
    QMMF_ERROR("%s:%s: Invalid virtual camera ID!", TAG, __func__);
    return BAD_VALUE;
  }

  // Return the post-processed buffer back to gralloc memory pool.
  idx = pproc_buffer_list_.indexOfKey(buffer_id);
  if (idx == NAME_NOT_FOUND) {
    QMMF_ERROR("%s:%s: buffer_id(%u) is not valid!", TAG, __func__, buffer_id);
    return BAD_VALUE;
  }

  StreamBuffer buffer = pproc_buffer_list_.valueFor(buffer_id);
  QMMF_VERBOSE("%s:%s: Post processed buffer(handle %p, fd %d) returned", TAG,
               __func__, buffer.handle, buffer.fd);
  status_t ret = pproc_memory_pool_->ReturnBuffer(buffer.handle);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Unable to return post-processed buffer!", TAG, __func__);
    return ret;
  }

  pproc_buffer_list_.removeItem(buffer_id);

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

CameraStartParam& MultiCameraManager::GetCameraStartParam() {

  return multicam_start_params_;
}

Vector<int32_t>& MultiCameraManager::GetSupportedFps() {

  return supported_fps_;
}

void MultiCameraManager::ReCalculateWidth(uint32_t &width) {

  // Divide the width of the stitched output on the number of cameras.
  width /= camera_contexts_.size();
}

int32_t MultiCameraManager::ImageToHalFormat(const ImageFormat &image) {

  int32_t format;
  switch (image) {
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

void MultiCameraManager::SetPostProcess(const ImageParam &param,
                                        const ImageFormat &input_format,
                                        uint32_t frame_rate) {
  PostProcParam in, out;
  PostProcCb cb_reprocess =
      [this] (StreamBuffer in_buffer, StreamBuffer out_buffer) -> void
      { ClientCaptureCallback(in_buffer, out_buffer); };

  in.width = param.width;
  in.height = param.height;
  in.format = ImageToHalFormat(input_format);
  out.width = param.width;
  out.height = param.height;
  out.format = ImageToHalFormat(param.image_format);

  status_t ret = multi_camera_pproc_->Create(0, in, out, frame_rate, 1,
                                             nullptr, cb_reprocess, nullptr);
  assert(ret >= NO_ERROR);
  if (ret < NO_ERROR) {
    QMMF_ERROR("%s: Error with creating reporcess: %d\n", __func__, ret);
  }
}

void MultiCameraManager::PostprocessCaptureCallback(StreamBuffer buffer) {
  QMMF_INFO("%s:%s Enter ", TAG, __func__);

  StreamBuffer output_buffer {};

  status_t ret = pproc_memory_pool_->GetBuffer(output_buffer.handle);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Unable to retrieve gralloc buffer", TAG, __func__);
    return;
  }

  ret = pproc_memory_pool_->PopulateMetaInfo(output_buffer.info,
                                               output_buffer.handle);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Failed to populate buffer meta info", TAG, __func__);
    return;
  }
  struct private_handle_t *priv_handle =
      (struct private_handle_t *) output_buffer.handle;
  output_buffer.fd           = priv_handle->fd;
  output_buffer.size         = priv_handle->size;
  output_buffer.frame_number = buffer.frame_number;
  output_buffer.timestamp    = buffer.timestamp;
  output_buffer.camera_id    = buffer.camera_id;

  multi_camera_pproc_->AddBuff(buffer, output_buffer);
  QMMF_INFO("%s:%s Exit ", TAG, __func__);
}

void MultiCameraManager::ClientCaptureCallback(StreamBuffer in_buffer,
                                               StreamBuffer out_buffer) {

  // Return input(stitched) buffer back to SnapshotStitching.
  status_t ret = snapshot_stitch_algo_->ImageBufferReturned(in_buffer.fd);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Unable to return stitch image buffer!", TAG, __func__);
    return;
  }
  // Map output(post-processed) buffer's fd to StreamBuffer. This is needed to
  // return post-processed buffers to their owners on ReturnImageCaptureBuffer.
  {
    Mutex::Autolock lock(pproc_lock_);
    pproc_buffer_list_.add(out_buffer.fd, out_buffer);
  }

  // Send the post-processed buffer to the client.
  assert(client_snapshot_cb_ != nullptr);
  if(client_snapshot_cb_ == nullptr) {
    QMMF_ERROR("%s:%s: Unable to send post-processed image buffer to client!",
               TAG, __func__);
    return;
  }
  client_snapshot_cb_(1, out_buffer);
  return;
}

SnapshotStitching::SnapshotStitching(
    InitParams &param, KeyedVector<uint32_t, sp<CameraContext> > &contexts)
    : StitchingBase(param),
      client_snapshot_cb_(nullptr) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  work_thread_name_ = new String8("SnapshotStitching");
  camera_contexts_ = contexts;
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

SnapshotStitching::~SnapshotStitching() {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  camera_contexts_.clear();
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

void SnapshotStitching::FrameAvailableCb(uint32_t count,
                                         StreamBuffer &buffer) {

  Mutex::Autolock lock(frame_lock_);
  QMMF_DEBUG("%s:%s: Camera %u: Snapshot Frame %" PRId64 " is available", TAG,
      __func__, buffer.camera_id, buffer.frame_number);

  // Handling input buffers from camera contexts.
  if (stop_frame_sync_) {
    ReturnBufferToCamera(buffer);
  } else {
    FrameSync(buffer);
  }
}

status_t SnapshotStitching::ImageBufferReturned(const int32_t buffer_id) {

  Mutex::Autolock lock(snapshot_lock);
  ssize_t idx = snapshot_buffer_list_.indexOfKey(buffer_id);
  if (idx == NAME_NOT_FOUND) {
    QMMF_ERROR("%s:%s: buffer_id(%u) is not valid!", TAG, __func__, buffer_id);
    return BAD_VALUE;
  }

  StreamBuffer buffer = snapshot_buffer_list_.valueFor(buffer_id);
  QMMF_DEBUG("%s:%s: Image capture buffer(handle %p, fd %d) returned", TAG,
      __func__, buffer.handle, buffer.fd);

  // Handling return buffer from camera source.
  status_t ret = ReturnBufferToBufferPool(buffer);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Unable to return image buffer!", TAG, __func__);
    return ret;
  }
  snapshot_buffer_list_.removeItem(buffer_id);
  return ret;
}

status_t SnapshotStitching::NotifyBufferToClient(StreamBuffer &buffer) {

  status_t ret = NO_ERROR;
  if(nullptr != client_snapshot_cb_) {
    {
      Mutex::Autolock lock(snapshot_lock);
      snapshot_buffer_list_.add(buffer.fd, buffer);
    }
    client_snapshot_cb_(1, buffer);
  } else {
    QMMF_VERBOSE("%s:%s: No client callback, simply return buffer back to"
        " memory pool!", TAG, __func__);
    ret = ReturnBufferToBufferPool(buffer);
  }
  return ret;
}

status_t SnapshotStitching::ReturnBufferToCamera(StreamBuffer &buffer) {

  status_t ret = NO_ERROR;
  ssize_t idx = camera_contexts_.indexOfKey(buffer.camera_id);
  if (NAME_NOT_FOUND == idx) {
    QMMF_ERROR("%s:%s: Invalid camera ID(%d)", TAG, __func__, buffer.camera_id);
    return BAD_VALUE;
  }

  sp<CameraContext> camera = camera_contexts_.valueFor(buffer.camera_id);

  ret = camera->ReturnImageCaptureBuffer(buffer.camera_id, buffer.fd);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Failed to return buffer to camera(%d)", TAG,
        __func__, buffer.camera_id);
  }
  camera.clear();
  return ret;
}

StreamStitching::StreamStitching(InitParams &param)
    : StitchingBase(param) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  use_frame_sync_timeout = true;
  work_thread_name_ = new String8("StreamStitching");

  // Create consumers for the physical cameras.
  for (auto const& camera_id : params_.camera_ids) {
    BufferConsumerImpl<StreamStitching> *impl;
    impl = new BufferConsumerImpl<StreamStitching>(this);
    camera_consumers_map_.add(camera_id, impl);
  }

  BufferProducerImpl<StreamStitching> *producer_impl;
  producer_impl = new BufferProducerImpl<StreamStitching>(this);
  buffer_producer_impl_ = producer_impl;

  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

StreamStitching::~StreamStitching() {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  buffer_producer_impl_.clear();
  camera_consumers_map_.clear();
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

status_t StreamStitching::AddConsumer(const sp<IBufferConsumer>& consumer) {

  if (nullptr != buffer_consumer_impl_.get()) {
    QMMF_ERROR("%s:%s: Consumer already set", TAG, __func__);
    return INVALID_OPERATION;
  }

  if (consumer == nullptr) {
    QMMF_ERROR("%s:%s: Input consumer is NULL", TAG, __func__);
    return BAD_VALUE;
  }

  buffer_consumer_impl_ = consumer;
  buffer_producer_impl_->AddConsumer(consumer);
  consumer->SetProducerHandle(buffer_producer_impl_);
  QMMF_VERBOSE("%s:%s: Consumer(%p) has been added.", TAG, __func__,
      consumer.get());

  return NO_ERROR;
}

status_t StreamStitching::RemoveConsumer() {

  if(buffer_producer_impl_->GetNumConsumer() == 0) {
    QMMF_ERROR("%s:%s: There are no connected consumers!", TAG, __func__);
    return INVALID_OPERATION;
  }
  buffer_producer_impl_->RemoveConsumer(buffer_consumer_impl_);
  buffer_consumer_impl_.clear();
  return NO_ERROR;
}

sp<IBufferConsumer>& StreamStitching::GetConsumerIntf(uint32_t camera_id) {

  return camera_consumers_map_.editValueFor(camera_id);
}

void StreamStitching::OnFrameAvailable(StreamBuffer& buffer) {

  Mutex::Autolock lock(frame_lock_);
  QMMF_VERBOSE("%s:%s: Camera %u: Frame %" PRId64 " is available", TAG,
      __func__, buffer.camera_id, buffer.frame_number);

  if (stop_frame_sync_) {
    ReturnBufferToCamera(buffer);
  } else {
    FrameSync(buffer);
  }
}

void StreamStitching::NotifyBufferReturned(const StreamBuffer& buffer) {

  QMMF_VERBOSE("%s:%s: Stream buffer(handle %p) returned", TAG, __func__,
      buffer.handle);
  ReturnBufferToBufferPool(buffer);
}

status_t StreamStitching::NotifyBufferToClient(StreamBuffer &buffer) {

  status_t ret = NO_ERROR;
  if(buffer_producer_impl_->GetNumConsumer() > 0) {
    buffer_producer_impl_->NotifyBuffer(buffer);
  } else {
    QMMF_VERBOSE("%s:%s: No consumer, simply return buffer back to"
        " memory pool!", TAG, __func__);
    ret = ReturnBufferToBufferPool(buffer);
  }
  return ret;
}

status_t StreamStitching::ReturnBufferToCamera(StreamBuffer &buffer) {

  const sp<IBufferConsumer> consumer = GetConsumerIntf(buffer.camera_id);
  if (consumer.get() == nullptr) {
    QMMF_ERROR("%s:%s: Failed to retrieve buffer consumer for camera(%d)!",
               TAG, __func__, buffer.camera_id);
    return BAD_VALUE;
  }
  consumer->GetProducerHandle()->NotifyBufferReturned(buffer);
  return NO_ERROR;
}

StitchingBase::StitchingBase(InitParams &param)
    : params_(param),
      stop_frame_sync_(false),
      use_frame_sync_timeout(false),
      work_thread_name_(nullptr) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  memset(&stitch_lib_, 0x0, sizeof(stitch_lib_));

  // Initialize the buffer map with unsynchronized buffers.
  for (auto const& camera_id : params_.camera_ids) {
    Vector<StreamBuffer> empty_buffers;
    unsynced_buffer_map_.add(camera_id, empty_buffers);
  }
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

StitchingBase::~StitchingBase() {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  DeInitLibrary();
  unsynced_buffer_map_.clear();
  process_buffers_map_.clear();
  registered_buffers_.clear();
  memory_pool_.clear();
  delete work_thread_name_;

  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}


status_t StitchingBase::Initialize() {

  status_t ret = NO_ERROR;
  if (nullptr != memory_pool_.get()) {
    QMMF_WARN("%s:%s: Memory pool already initialized", TAG, __func__);
    return ret;
  }
  memory_pool_ = new GrallocMemory();

  ret = memory_pool_->Initialize();
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Unable to create memory pool!", TAG, __func__);
    return ret;
  }

  ret = InitLibrary();
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Failed to open algorithm library!", TAG, __func__);
  }
  return ret;
}

status_t StitchingBase::Configure(GrallocMemory::BufferParams &param) {

  return memory_pool_->Configure(param);
}

int32_t StitchingBase::Run() {

  Mutex::Autolock lock(frame_lock_);
  stop_frame_sync_ = false;
  return Camera3Thread::Run(work_thread_name_->string());
}

void StitchingBase::RequestExit() {

  Mutex::Autolock lock(frame_lock_);
  Camera3Thread::RequestExit();
  StopFrameSync();
}

void StitchingBase::RequestExitAndWait() {

  Mutex::Autolock lock(frame_lock_);
  Camera3Thread::RequestExitAndWait();
  StopFrameSync();
}

bool StitchingBase::ThreadLoop() {

  status_t ret = NO_ERROR;
  Vector<StreamBuffer> input_buffers, output_buffers;

  {
    Mutex::Autolock lock(sync_lock_);
    // If there aren't any pending synchronized buffers waiting to go through
    // stitch processing, wait until such buffer becomes available.
    if (synced_buffer_queue_.empty()) {
      if (use_frame_sync_timeout) {
        ret = wait_for_sync_frames_.waitRelative(sync_lock_, kFrameSyncTimeout);
      } else {
        ret = wait_for_sync_frames_.wait(sync_lock_);
      }
      if (NO_ERROR != ret) {
        QMMF_ERROR("%s:%s: Wait for frame available failed, ret(%d)", TAG,
            __func__, ret);
        return true;
      }
    }

    for (auto const& id : params_.camera_ids) {
      input_buffers.push_back(synced_buffer_queue_.front().valueFor(id));
    }
    synced_buffer_queue_.pop();
  }

  // TODO: add some logic for more than 1 output buffer
  StreamBuffer b {};

  ret = memory_pool_->GetBuffer(b.handle);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Unable to retrieve gralloc buffer", TAG, __func__);
    return true;
  }
  ret = memory_pool_->PopulateMetaInfo(b.info, b.handle);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Failed to populate buffer meta info", TAG, __func__);
    return ret;
  }
  const struct private_handle_t *priv_handle =
      static_cast<const private_handle_t *>(b.handle);
  b.fd           = priv_handle->fd;
  b.size         = priv_handle->size;
  b.frame_number = input_buffers.itemAt(0).frame_number;
  b.timestamp    = input_buffers.itemAt(0).timestamp;
  b.camera_id    = params_.virtual_camera_id;
  output_buffers.push_back(b);

  if (!stitch_lib_.configured) {
    ret = Configlibrary(input_buffers, output_buffers);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s: Failed to configure library", TAG, __func__);
      DeInitLibrary();
      return true;
    }
    stitch_lib_.configured = true;
  }

  {
    Mutex::Autolock lock(process_buffers_lock_);
    for (auto const& buffer : input_buffers) {
      std::pair<buffer_handle_t, StreamBuffer> pair (buffer.handle, buffer);
      process_buffers_map_.insert(pair);
    }
    for (auto const& buffer : output_buffers) {
      std::pair<buffer_handle_t, StreamBuffer> pair (buffer.handle, buffer);
      process_buffers_map_.insert(pair);
    }
  }

  ret = ProcessBuffers(input_buffers, output_buffers);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Failed to process images", TAG, __func__);
  }

  return true;
}

status_t StitchingBase::FrameSync(StreamBuffer& buffer) {

  bool match_found;
  int32_t timestamp_delta;
  uint32_t num_matched_frames = 1;
  Vector<StreamBuffer> *unsynced_buffers;
  // Map of camera id and index of the matched buffer from
  // the unsynced_buffers queue for that camera id.
  KeyedVector<uint32_t, uint32_t> matched_buffers;

  // Each matched buffer for given camera will be added to the
  // synced_frames vector and identified by it's camera id.
  KeyedVector<uint32_t, StreamBuffer> synced_frames;
  synced_frames.add(buffer.camera_id, buffer);

  // Iterate through the unsynced buffers for each camera, except current one.
  for (auto const& camera_id : params_.camera_ids) {
    if (camera_id == buffer.camera_id) {
      continue;
    }
    match_found = false;

    // Retrieve a list with unsynced buffers for each of the other cameras.
    unsynced_buffers = &unsynced_buffer_map_.editValueFor(camera_id);

    // Backward search, as the latest buffers are at the back.
    for (int32_t idx = (unsynced_buffers->size() - 1); idx >= 0; --idx) {
      const StreamBuffer &unsynced_frame = unsynced_buffers->itemAt(idx);
      timestamp_delta = buffer.timestamp - unsynced_frame.timestamp;

      if (std::abs(timestamp_delta) < kTimestampMaxDelta) {
        synced_frames.add(camera_id, unsynced_frame);
        matched_buffers.add(camera_id, idx);
        ++num_matched_frames;
        match_found = true;
        break;
      } else if (timestamp_delta > 0) {
        // No need to check the rest of the buffers in the queue for
        // this camera_id, as they will be with a lower timestamp.
        break;
      }
    }
    // If a matched frame wasn't found there is no need to check
    // all other remaining cameras (if any).
    if (!match_found) {
      break;
    }
  }

  // Matched number of frames is not the same as the number of cameras.
  if (num_matched_frames != params_.camera_ids.size()) {
    QMMF_DEBUG("%s:%s: Camera %u: No matching buffers found", TAG, __func__,
        buffer.camera_id);

    // Push the buffer in the unsynced buffer queue for its camera id.
    unsynced_buffer_map_.editValueFor(buffer.camera_id).push_back(buffer);

    // Check if the queue of current buffer camera_id has reached max size.
    unsynced_buffers = &unsynced_buffer_map_.editValueFor(buffer.camera_id);
    int32_t excess_buffers = unsynced_buffers->size() - kUnsyncedQueueMaxSize;

    if (excess_buffers > 0) {
      QMMF_DEBUG("%s:%s: Camera %u: Unsynced buffer queue reached max "
          "size: %d", TAG, __func__, buffer.camera_id, kUnsyncedQueueMaxSize);

      // Remove older excess buffers from the queue.
      for (int32_t i = 0; i < excess_buffers; ++i) {
        StreamBuffer &buf = unsynced_buffers->editItemAt(i);
        ReturnBufferToCamera(buf);
      }
      unsynced_buffers->removeItemsAt(0, excess_buffers);
    }
    return FAILED_TRANSACTION;
  }

  // A matched frame(s) have been found, return all unsynced buffers and clear
  // the queue of the camera_id from which the synchronization buffer came.
  ReturnUnsyncedBuffers(buffer.camera_id);

  // Clear the obsolete unsynced buffers from queue of the matched cameras,
  // starting from beginning to the latest matched buffer and return them
  // back to their corresponding producers.
  for (size_t idx = 0; idx < matched_buffers.size(); ++idx) {
    uint32_t camera_id = matched_buffers.keyAt(idx);
    uint32_t match_idx = matched_buffers.valueAt(idx);
    unsynced_buffers = &unsynced_buffer_map_.editValueFor(camera_id);
    unsynced_buffers->removeAt(match_idx);
    for (uint32_t i = 0; i < match_idx; ++i) {
      StreamBuffer &buf = unsynced_buffers->editItemAt(i);
      ReturnBufferToCamera(buf);
    }
    unsynced_buffers->removeItemsAt(0, match_idx);
  }

  Mutex::Autolock lock(sync_lock_);
  synced_buffer_queue_.push(synced_frames);
  wait_for_sync_frames_.signal();

  return NO_ERROR;
}

status_t StitchingBase::ReturnBufferToBufferPool(const StreamBuffer &buffer) {

  status_t ret = memory_pool_->ReturnBuffer(buffer.handle);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Failed to return buffer to memory pool", TAG, __func__);
  }
  return ret;
}

void StitchingBase::StopFrameSync() {

  // Return all unsynced buffers back to the camera contexts
  for (auto const& camera_id : params_.camera_ids) {
    status_t ret = ReturnUnsyncedBuffers(camera_id);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: Failed to return some of the unsynchronized buffers"
          " for camera %d!", TAG, __func__, camera_id);
    }
  }
  // Flush all pending buffers from the library
  if (nullptr != stitch_lib_.handle) {
    stitch_lib_.flush(stitch_lib_.context);
  }
  stop_frame_sync_ = true;
}

status_t StitchingBase::ReturnProcessedBuffer(buffer_handle_t &handle) {

  status_t ret = NO_ERROR;
  Mutex::Autolock lock(process_buffers_lock_);
  if (process_buffers_map_.find(handle) == process_buffers_map_.end()) {
    QMMF_ERROR("%s:%s: Buffer %p not registered", TAG, __func__, handle);
    return BAD_VALUE;
  }
  StreamBuffer &buffer = process_buffers_map_.at(handle);
  QMMF_DEBUG("%s:%s: Got buffer(%p), camera id %d", TAG, __func__, handle,
      buffer.camera_id);

  if (buffer.camera_id == params_.virtual_camera_id) {
    ret = NotifyBufferToClient(buffer);
  } else {
    ret = ReturnBufferToCamera(buffer);
  }
  process_buffers_map_.erase(handle);

  return ret;
}

status_t StitchingBase::ReturnUnsyncedBuffers(uint32_t camera_id) {

  Vector<StreamBuffer> &buffers =
      unsynced_buffer_map_.editValueFor(camera_id);

  status_t ret = NO_ERROR;
  while (!buffers.isEmpty()) {
    StreamBuffer &buf = buffers.editTop();
    ret = ReturnBufferToCamera(buf);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s: Failed to return buffer %p for camera %d", TAG,
          __func__, buf.handle, camera_id);
      return ret;
    }
    buffers.pop();
  }
  return ret;
}

status_t StitchingBase::InitLibrary() {

  status_t ret = NO_ERROR;
  char prop[PROPERTY_VALUE_MAX];
  int32_t use_calib_file;

  if (nullptr != stitch_lib_.handle) {
    QMMF_WARN("%s:%s: Stitch library already initialized", TAG, __func__);
    return ret;
  }

  void* handle = dlopen(kStitchLib, RTLD_NOW);
  if (nullptr == handle) {
    QMMF_ERROR("%s:%s: Failed to open %s, error: %s", TAG, __func__,
        kStitchLib, dlerror());
    return BAD_VALUE;
  }

  stitch_lib_.handle = handle;

  *(void **) &stitch_lib_.init       = dlsym(handle, "qmmf_alg_init");
  *(void **) &stitch_lib_.deinit     = dlsym(handle, "qmmf_alg_deinit");
  *(void **) &stitch_lib_.get_caps   = dlsym(handle, "qmmf_alg_get_caps");
  *(void **) &stitch_lib_.set_tuning = dlsym(handle, "qmmf_alg_set_tuning");
  *(void **) &stitch_lib_.config     = dlsym(handle, "qmmf_alg_config");
  *(void **) &stitch_lib_.flush      = dlsym(handle, "qmmf_alg_flush");
  *(void **) &stitch_lib_.process    = dlsym(handle, "qmmf_alg_process");
  *(void **) &stitch_lib_.register_bufs =
      dlsym(handle, "qmmf_alg_register_bufs");
  *(void **) &stitch_lib_.unregister_bufs =
      dlsym(handle, "qmmf_alg_unregister_bufs");
  *(void **) &stitch_lib_.get_debug_info_log =
      dlsym(handle, "qmmf_alg_get_debug_info_log");

  if (!stitch_lib_.init || !stitch_lib_.deinit || !stitch_lib_.get_caps ||
      !stitch_lib_.set_tuning || !stitch_lib_.get_debug_info_log ||
      !stitch_lib_.register_bufs || !stitch_lib_.unregister_bufs ||
      !stitch_lib_.flush || !stitch_lib_.process || !stitch_lib_.config) {
    QMMF_ERROR("%s:%s: Unable to link all symbols", TAG, __func__);
    QMMF_ERROR("%s:%s: qmmf_alg_init %p", TAG, __func__, stitch_lib_.init);
    QMMF_ERROR("%s:%s: qmmf_alg_deinit %p", TAG, __func__, stitch_lib_.deinit);
    QMMF_ERROR("%s:%s: qmmf_alg_get_caps %p", TAG, __func__,
        stitch_lib_.get_caps);
    QMMF_ERROR("%s:%s: qmmf_alg_set_tuning %p", TAG, __func__,
        stitch_lib_.set_tuning);
    QMMF_ERROR("%s:%s: qmmf_alg_config %p", TAG, __func__, stitch_lib_.config);
    QMMF_ERROR("%s:%s: qmmf_alg_register_bufs %p", TAG, __func__,
        stitch_lib_.register_bufs);
    QMMF_ERROR("%s:%s: qmmf_alg_unregister_bufs %p", TAG, __func__,
        stitch_lib_.unregister_bufs);
    QMMF_ERROR("%s:%s: qmmf_alg_flush %p", TAG, __func__, stitch_lib_.flush);
    QMMF_ERROR("%s:%s: qmmf_alg_process %p", TAG, __func__,
        stitch_lib_.process);
    QMMF_ERROR("%s:%s: qmmf_alg_get_debug_info_log %p", TAG, __func__,
        stitch_lib_.get_debug_info_log);
    ret = NAME_NOT_FOUND;
    goto FAIL;
  }

  property_get("persist.qmmf.stitch.calibfile", prop, "0");
  use_calib_file = atoi(prop);

  if (use_calib_file) {
    qmmf_alg_blob_t calibation_blob {};
    ret = ParseCalibFile(&calibation_blob.data, calibation_blob.size);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: Failed to parse config file", TAG, __func__);
      goto FAIL;
    }

    ret = stitch_lib_.init(&stitch_lib_.context, &calibation_blob);
    free(calibation_blob.data);
  } else {
    ret = stitch_lib_.init(&stitch_lib_.context, nullptr);
  }

  if (QMMF_ALG_SUCCESS != ret) {
    QMMF_ERROR("%s:%s: Failed to initialize library, ret(%d)", TAG,
        __func__, ret);
    goto FAIL;
  }
  return ret;

FAIL:
  dlclose(handle);
  memset(&stitch_lib_, 0x0, sizeof(stitch_lib_));
  return ret;
}

status_t StitchingBase::DeInitLibrary() {

  status_t ret = NO_ERROR;

  if (nullptr != stitch_lib_.handle) {
    stitch_lib_.deinit(stitch_lib_.context);
    ret = dlclose(stitch_lib_.handle);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s: Failed to close %s, error: %s", TAG, __func__,
          kStitchLib, dlerror());
    }
    memset(&stitch_lib_, 0x0, sizeof(stitch_lib_));
  }
  return ret;
}

status_t StitchingBase::Configlibrary(Vector<StreamBuffer> &input_buffers,
                                      Vector<StreamBuffer> &output_buffers) {

  status_t ret = NO_ERROR;

  if (nullptr == stitch_lib_.handle) {
    QMMF_ERROR("%s:%s: Invalid IL lib handle!", TAG, __func__);
    return BAD_VALUE;
  }

  qmmf_alg_config_t config {};

  config.input.cnt  = input_buffers.size();
  config.output.cnt = output_buffers.size();

  config.input.fmts = static_cast<qmmf_alg_format_t*>(
      calloc(config.input.cnt, sizeof(*config.input.fmts)));

  if (nullptr == config.input.fmts) {
    QMMF_ERROR("%s:%s: Failed to allocate memory for input format list",
        TAG, __func__);
    ret = NO_MEMORY;
    goto EXIT;
  }

  config.output.fmts = static_cast<qmmf_alg_format_t*>(
      calloc(config.output.cnt, sizeof(*config.output.fmts)));

  if (nullptr == config.output.fmts) {
    QMMF_ERROR("%s:%s: Failed to allocate memory for output format list",
        TAG, __func__);
    ret = NO_MEMORY;
    goto EXIT;
  }

  for (uint32_t idx = 0; idx < config.input.cnt; ++idx) {
    const StreamBuffer *buf = &input_buffers.itemAt(idx);
    ret = PopulateImageFormat(config.input.fmts[idx], buf);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s: Failed to set input image format", TAG, __func__);
      goto EXIT;
    }
  }

  for (uint32_t idx = 0; idx < config.output.cnt; ++idx) {
    const StreamBuffer *buf = &output_buffers.itemAt(idx);
    ret = PopulateImageFormat(config.output.fmts[idx], buf);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s: Failed to set output image format", TAG, __func__);
      goto EXIT;
    }
  }

  ret = stitch_lib_.config(stitch_lib_.context, &config);
  if (QMMF_ALG_SUCCESS != ret) {
    QMMF_ERROR("%s:%s: Failed to configure algo library, error(%d)",
        TAG, __func__, ret);
  }

EXIT:
  free(config.input.fmts);
  free(config.output.fmts);
  return ret;
}

status_t StitchingBase::ProcessBuffers(Vector<StreamBuffer> &input_buffers,
                                       Vector<StreamBuffer> &output_buffers) {

  status_t ret = NO_ERROR;
  const StreamBuffer *buffer = nullptr;

  if (nullptr == stitch_lib_.handle) {
    QMMF_ERROR("%s:%s: Invalid IL lib handle!", TAG, __func__);
    return BAD_VALUE;
  }

  qmmf_alg_process_data_t proc_data {};
  qmmf_alg_buf_list_t reg_buf_list {};

  proc_data.input.cnt  = input_buffers.size();
  proc_data.output.cnt = output_buffers.size();
  proc_data.user_data  = this;

  qmmf_alg_buffer_t input_buffer_list[proc_data.input.cnt];
  memset(&input_buffer_list, 0x0, sizeof(input_buffer_list));
  qmmf_alg_buffer_t output_buffer_list[proc_data.output.cnt];
  memset(&output_buffer_list, 0x0, sizeof(output_buffer_list));

  proc_data.input.bufs = input_buffer_list;
  proc_data.output.bufs = output_buffer_list;

  for (uint32_t idx = 0; idx < proc_data.input.cnt; ++idx) {
    buffer = &input_buffers.itemAt(idx);
    ret = PrepareBuffer(reg_buf_list, proc_data.input.bufs[idx], buffer);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s: Failed to prepare input buffer", TAG, __func__);
      goto EXIT;
    }
  }

  for (uint32_t idx = 0; idx < proc_data.output.cnt; ++idx) {
    buffer = &output_buffers.itemAt(idx);
    ret = PrepareBuffer(reg_buf_list, proc_data.output.bufs[idx], buffer);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s: Failed to prepare output buffer", TAG, __func__);
      goto EXIT;
    }
  }

  if (reg_buf_list.cnt > 0) {
    ret = stitch_lib_.register_bufs(stitch_lib_.context, reg_buf_list);
    if (QMMF_ALG_SUCCESS != ret) {
      // Remove the failed buffers from the list with registered buffers.
      for (uint32_t idx = 0; idx < reg_buf_list.cnt; ++idx) {
        registered_buffers_.erase(reg_buf_list.bufs[idx].handle);
      }
      QMMF_ERROR("%s:%s: Register buffers failed, err(%d)", TAG, __func__, ret);
      goto EXIT;
    }
  }

  proc_data.complete = &StitchingBase::ProcessCallback;
  ret = stitch_lib_.process(stitch_lib_.context, &proc_data);
  if (QMMF_ALG_SUCCESS != ret) {
    QMMF_ERROR("%s:%s: Failed to process images, err(%d)", TAG, __func__, ret);
  }

EXIT:
  free(reg_buf_list.bufs);
  return ret;
}

status_t StitchingBase::ParseCalibFile(void **data, uint32_t &size) {

  struct stat st;
  size_t objects_read;

  status_t ret = stat(kStitchCalibFile, &st);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: Get file status failed (%s)", TAG, __func__,
        strerror(errno));
    return ret;
  }

  void *calibration_blob = calloc(1, st.st_size + 1);
  if (nullptr == calibration_blob) {
    QMMF_ERROR("%s:%s: Failed to allocate memory with size %ld", TAG,
        __func__, (long int) st.st_size);
    return NO_MEMORY;
  }

  FILE *file = fopen(kStitchCalibFile, "rb");
  if (nullptr == file) {
    QMMF_ERROR("%s:%s: Unable to open (%s)", TAG, __func__, kStitchCalibFile);
    ret = UNKNOWN_ERROR;
    goto FAIL;
  }

  objects_read = fread(calibration_blob, st.st_size, 1, file);
  if (objects_read != 1) {
    QMMF_ERROR("%s:%s: Reading error", TAG, __func__);
    ret = UNKNOWN_ERROR;
    fclose(file);
    goto FAIL;
  }

  *data = calibration_blob;
  size = st.st_size + 1;

  fclose(file);
  return NO_ERROR;

FAIL:
  free(calibration_blob);
  return ret;
}

status_t StitchingBase::PopulateImageFormat(qmmf_alg_format_t &fmt,
                                            const StreamBuffer *buffer) {

  struct private_handle_t *priv_handle = (struct private_handle_t *)
      buffer->handle;
  if (nullptr == priv_handle) {
    QMMF_ERROR("%s:%s: Invalid private handle!", TAG, __func__);
    return BAD_VALUE;
  }

  switch (priv_handle->format) {
    case HAL_PIXEL_FORMAT_BLOB:
      fmt.pix_fmt = QMMF_ALG_PIXFMT_JPEG;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
      fmt.pix_fmt = QMMF_ALG_PIXFMT_NV12;
      break;
    case HAL_PIXEL_FORMAT_NV21_ZSL:
      fmt.pix_fmt = QMMF_ALG_PIXFMT_NV21;
      break;
    case HAL_PIXEL_FORMAT_RAW10:
      fmt.pix_fmt = QMMF_ALG_PIXFMT_RAW_RGGB10;
      break;
    default:
      QMMF_ERROR("%s:%s: Unsupported format: 0x%x", TAG, __func__,
          priv_handle->format);
      return NAME_NOT_FOUND;
  }
  fmt.width      = priv_handle->width;
  fmt.height     = priv_handle->height;
  fmt.num_planes = buffer->info.num_planes;

  for (uint32_t i = 0; i < buffer->info.num_planes; ++i) {
    fmt.plane[i].stride = buffer->info.plane_info[i].stride;
    fmt.plane[i].offset = 0;
    fmt.plane[i].length = buffer->info.plane_info[i].scanline;
  }

  return NO_ERROR;
}

status_t StitchingBase::PrepareBuffer(qmmf_alg_buf_list_t &reg_buf_list,
                                      qmmf_alg_buffer_t &img_buffer,
                                      const StreamBuffer *buffer) {

  status_t ret = PopulateImageFormat(img_buffer.fmt, buffer);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Failed to set output image format", TAG, __func__);
    return ret;
  }

  img_buffer.vaddr  = 0;
  img_buffer.fd     = buffer->fd;
  img_buffer.size   = buffer->size;
  img_buffer.handle = buffer->handle;

  if (registered_buffers_.find(buffer->handle) == registered_buffers_.end()) {
    // Add buffer to the list, later it will be removed in case register fails.
    registered_buffers_.insert(buffer->handle);

    // Increment the count of the buffers that need to be registered.
    ++reg_buf_list.cnt;

    // reallocate memory of the new qmmf_alg_buffer_t structure
    uint64_t new_size = reg_buf_list.cnt * sizeof(*reg_buf_list.bufs);
    qmmf_alg_buffer_t *new_buffer_ptr =
        static_cast<qmmf_alg_buffer_t*>(realloc(reg_buf_list.bufs, new_size));
    if (nullptr == new_buffer_ptr) {
      QMMF_ERROR("%s:%s: Failed to realloc buffer memory", TAG, __func__);
      return NO_MEMORY;
    }
    reg_buf_list.bufs = new_buffer_ptr;

    // Get a pointer to the last buffer in the newly allocated structure
    // and copy the data from the previously filled image buffer.
    qmmf_alg_buffer_t *buf = &reg_buf_list.bufs[reg_buf_list.cnt - 1];
    memcpy(buf, &img_buffer, sizeof(img_buffer));
  }
  return NO_ERROR;
}


void StitchingBase::ProcessCallback(qmmf_alg_cb_t *cb_data) {

  QMMF_DEBUG("%s:%s: Return status (%d)", TAG, __func__, cb_data->status);

  if (QMMF_ALG_SUCCESS == cb_data->status) {
    StitchingBase *algo = static_cast<StitchingBase *> (cb_data->user_data);
    algo->ReturnProcessedBuffer(cb_data->buf->handle);
  }
}

GrallocMemory::GrallocMemory(alloc_device_t *gralloc_device)
    : gralloc_device_(gralloc_device),
      gralloc_slots_(nullptr),
      buffers_allocated_(0),
      pending_buffer_count_(0) {

  QMMF_INFO("%s: Enter", __func__);

  if (nullptr != gralloc_device_) {
    QMMF_INFO("%s: Gralloc Module author: %s, version: %d name: %s", __func__,
        gralloc_device_->common.module->author,
        gralloc_device_->common.module->hal_api_version,
        gralloc_device_->common.module->name);
  }
  QMMF_INFO("%s: Exit (%p)", __func__, this);
}

GrallocMemory::~GrallocMemory() {

  QMMF_INFO("%s: Enter", __func__);

  delete[] gralloc_slots_;
  gralloc_slots_ = nullptr;

  if (!gralloc_buffers_.isEmpty()) {
    for (uint32_t i = 0; i < gralloc_buffers_.size(); ++i) {
      FreeGrallocBuffer(gralloc_buffers_.keyAt(i));
    }
    gralloc_buffers_.clear();
  }

  if (nullptr != gralloc_device_) {
    gralloc_device_->common.close(&gralloc_device_->common);
  }
  QMMF_INFO("%s: Exit (%p)", __func__, this);
}

status_t GrallocMemory::Initialize() {

  status_t ret = NO_ERROR;
  hw_module_t const *module = nullptr;

  if (nullptr != gralloc_device_) {
    QMMF_WARN("%s: Gralloc allocator already created", __func__);
    return ret;
  }

  ret = hw_get_module(GRALLOC_HARDWARE_MODULE_ID, &module);
  if ((NO_ERROR != ret) || (nullptr == module)) {
    QMMF_ERROR("%s: Unable to load Gralloc module: %d", __func__, ret);
    return ret;
  }

  ret = module->methods->open(module, GRALLOC_HARDWARE_GPU0,
                              (struct hw_device_t **)&gralloc_device_);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Could not open Gralloc module: %s (%d)", __func__,
        strerror(-ret), ret);
    dlclose(module->dso);
    return ret;
  }

  QMMF_INFO("%s: Gralloc Module author: %s, version: %d name: %s", __func__,
      gralloc_device_->common.module->author,
      gralloc_device_->common.module->hal_api_version,
      gralloc_device_->common.module->name);

  return ret;
}

status_t GrallocMemory::Configure(BufferParams &params) {

  if (gralloc_device_ == nullptr) {
    QMMF_ERROR("%s: Gralloc allocator not created", __func__);
    return INVALID_OPERATION;
  }

  delete[] gralloc_slots_;
  gralloc_slots_ = nullptr;

  if (!gralloc_buffers_.isEmpty()) {
    for (uint32_t i = 0; i < gralloc_buffers_.size(); ++i) {
      FreeGrallocBuffer(gralloc_buffers_.keyAt(i));
    }
    gralloc_buffers_.clear();
  }

  params_ = params;

  gralloc_slots_ = new buffer_handle_t[params_.max_buffer_count];
  if (nullptr == gralloc_slots_) {
    QMMF_ERROR("%s: Unable to allocate buffer handles!\n", __func__);
    return NO_MEMORY;
  }

  return NO_ERROR;
}

status_t GrallocMemory::GetBuffer(buffer_handle_t &buffer) {

  status_t ret = NO_ERROR;
  Mutex::Autolock lock(buffer_lock_);

  if (pending_buffer_count_ == params_.max_buffer_count) {
    QMMF_VERBOSE("%s: Already retrieved maximum buffers (%d), waiting"
        " on a free one", __func__, params_.max_buffer_count);

    ret = wait_for_buffer_.waitRelative(buffer_lock_, kBufferWaitTimeout);
    if (ret == TIMED_OUT) {
      QMMF_ERROR("%s: Wait for output buffer return timed out", __func__);
      return ret;
    }
  }
  ret = GetBufferLocked(buffer);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to retrieve output buffer", __func__);
  }

  return ret;
}

status_t GrallocMemory::ReturnBuffer(const buffer_handle_t &buffer) {

  Mutex::Autolock lock(buffer_lock_);
  QMMF_DEBUG("%s: Buffer(%p) returned to memory pool", __func__, buffer);

  status_t ret = ReturnBufferLocked(buffer);
  if (ret == NO_ERROR) {
    wait_for_buffer_.signal();
  }
  return ret;
}

status_t GrallocMemory::GetBufferLocked(buffer_handle_t &buffer) {

  status_t ret = NO_ERROR;
  int32_t idx = -1;
  buffer_handle_t handle = nullptr;

  //Only pre-allocate buffers in case no valid streamBuffer
  //is passed as an argument.
  for (uint32_t i = 0; i < gralloc_buffers_.size(); ++i) {
    if (gralloc_buffers_.valueAt(i)) {
      handle = gralloc_buffers_.keyAt(i);
      gralloc_buffers_.replaceValueAt(i, false);
      break;
    }
  }
  // Find the slot of the available gralloc buffer.
  if (nullptr != handle) {
    for (uint32_t i = 0; i < buffers_allocated_; ++i) {
      if (gralloc_slots_[i] == handle) {
        idx = i;
        break;
      }
    }
  } else if ((nullptr == handle) &&
             (buffers_allocated_ < params_.max_buffer_count)) {
    ret = AllocGrallocBuffer(&handle);
    if (NO_ERROR != ret) {
      return ret;
    }
    idx = buffers_allocated_;
    gralloc_slots_[idx] = handle;
    gralloc_buffers_.add(gralloc_slots_[idx], false);
    ++buffers_allocated_;
  }

  if ((nullptr == handle) || (0 > idx)) {
    QMMF_ERROR("%s: Unable to allocate or find a free buffer!", __func__);
    return INVALID_OPERATION;
  }

  buffer = gralloc_slots_[idx];
  ++pending_buffer_count_;

  return ret;
}


status_t GrallocMemory::ReturnBufferLocked(const buffer_handle_t &buffer) {

  if (pending_buffer_count_ == 0) {
    QMMF_ERROR("%s: Not expecting any buffers!", __func__);
    return INVALID_OPERATION;
  }

  int32_t idx = gralloc_buffers_.indexOfKey(buffer);
  if (NAME_NOT_FOUND == idx) {
    QMMF_ERROR("%s: Buffer %p returned that wasn't allocated by this"
        " Memory Pool!", __func__, buffer);
    return BAD_VALUE;
  }

  gralloc_buffers_.replaceValueFor(buffer, true);
  --pending_buffer_count_;

  return NO_ERROR;
}

status_t GrallocMemory::PopulateMetaInfo(CameraBufferMetaData &info,
                                             buffer_handle_t &buffer) {

  if (nullptr == buffer) {
    QMMF_ERROR("%s: Invalid buffer handle!\n", __func__);
    return BAD_VALUE;
  }

  {
    Mutex::Autolock lock(buffer_lock_);
    bool is_valid_handle = false;
    for (uint32_t i = 0; i < buffers_allocated_; ++i) {
      if (gralloc_slots_[i] == buffer) {
        is_valid_handle = true;
        break;
      }
    }
    if (!is_valid_handle) {
      QMMF_ERROR("%s: Buffer handle wasn't allocated by this Gralloc"
          " Memory Pool!", __func__);
      return BAD_VALUE;
    }
  }

  struct private_handle_t *priv_handle = (struct private_handle_t *) buffer;

  int aligned_width, aligned_height;
  gralloc_module_t const *mapper = reinterpret_cast<gralloc_module_t const *>(
          gralloc_device_->common.module);
  status_t ret = mapper->perform(
      mapper,
      GRALLOC_MODULE_PERFORM_GET_CUSTOM_STRIDE_AND_HEIGHT_FROM_HANDLE,
      priv_handle, &aligned_width, &aligned_height);
  if (0 != ret) {
    QMMF_ERROR("%s: Unable to query stride&scanline: %d\n", __func__, ret);
    return ret;
  }

  switch (priv_handle->format) {
    case HAL_PIXEL_FORMAT_BLOB:
      info.format = BufferFormat::kBLOB;
      info.num_planes = 1;
      info.plane_info[0].width = params_.max_size;
      info.plane_info[0].height = 1;
      info.plane_info[0].stride = aligned_width;
      info.plane_info[0].scanline = aligned_height;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
      info.format = BufferFormat::kNV12;
      info.num_planes = 2;
      info.plane_info[0].width = params_.width;
      info.plane_info[0].height = params_.height;
      info.plane_info[0].stride = aligned_width;
      info.plane_info[0].scanline = aligned_height;
      info.plane_info[1].width = params_.width;
      info.plane_info[1].height = params_.height/2;
      info.plane_info[1].stride = aligned_width;
      info.plane_info[1].scanline = aligned_height/2;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
      info.format = BufferFormat::kNV12UBWC;
      info.num_planes = 2;
      info.plane_info[0].width = params_.width;
      info.plane_info[0].height = params_.height;
      info.plane_info[0].stride = aligned_width;
      info.plane_info[0].scanline = aligned_height;
      info.plane_info[1].width = params_.width;
      info.plane_info[1].height = params_.height/2;
      info.plane_info[1].stride = aligned_width;
      info.plane_info[1].scanline = aligned_height/2;
      break;
    case HAL_PIXEL_FORMAT_NV21_ZSL:
      info.format = BufferFormat::kNV21;
      info.num_planes = 2;
      info.plane_info[0].width = params_.width;
      info.plane_info[0].height = params_.height;
      info.plane_info[0].stride = aligned_width;
      info.plane_info[0].scanline = aligned_height;
      info.plane_info[1].width = params_.width;
      info.plane_info[1].height = params_.height/2;
      info.plane_info[1].stride = aligned_width;
      info.plane_info[1].scanline = aligned_height/2;
      break;
    case HAL_PIXEL_FORMAT_RAW10:
      info.format = BufferFormat::kRAW10;
      info.num_planes = 1;
      info.plane_info[0].width = params_.width;
      info.plane_info[0].height = params_.height;
      info.plane_info[0].stride = aligned_width;
      info.plane_info[0].scanline = aligned_height;
      break;
    case HAL_PIXEL_FORMAT_RAW16:
      info.format = BufferFormat::kRAW16;
      info.num_planes = 1;
      info.plane_info[0].width = params_.width;
      info.plane_info[0].height = params_.height;
      info.plane_info[0].stride = aligned_width;
      info.plane_info[0].scanline = aligned_height;
      break;
    default:
      QMMF_ERROR("%s: Unsupported format: %d", __func__,
          priv_handle->format);
      return NAME_NOT_FOUND;
  }

  return NO_ERROR;
}

status_t GrallocMemory::AllocGrallocBuffer(buffer_handle_t *buf) {

  if (gralloc_device_ == nullptr) {
    QMMF_ERROR("%s: Gralloc allocator not created", __func__);
    return INVALID_OPERATION;
  }

  status_t ret      = NO_ERROR;
  uint32_t width    = params_.width;
  uint32_t height   = params_.height;
  int32_t  format   = params_.format;
  int32_t  usage    = params_.gralloc_flags;
  uint32_t max_size = params_.max_size;

  // Filter out any usage bits that shouldn't be passed to the gralloc module.
  usage &= GRALLOC_USAGE_ALLOC_MASK;

  if (!width || !height) {
    width = height = 1;
  }

  int stride = 0;
  if (0 < max_size) {
    // Blob buffers are expected to get allocated with width equal to blob
    // max size and height equal to 1.
    ret = gralloc_device_->alloc(gralloc_device_, static_cast<int>(max_size),
                                 static_cast<int>(1), format,
                                 static_cast<int>(usage), buf, &stride);
  } else {
    ret = gralloc_device_->alloc(gralloc_device_, static_cast<int>(width),
                                 static_cast<int>(height), format,
                                 static_cast<int>(usage), buf, &stride);
  }
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to allocate gralloc buffer", __func__);
  }

  return ret;
}

status_t GrallocMemory::FreeGrallocBuffer(buffer_handle_t buf) {

  if (gralloc_device_ == nullptr) {
    QMMF_ERROR("%s: Gralloc allocator not created", __func__);
    return INVALID_OPERATION;
  }
  return gralloc_device_->free(gralloc_device_, buf);
}

}; //namespace recorder.

}; //namespace qmmf.
