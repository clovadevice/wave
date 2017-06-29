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

#define TAG "RecorderClient"

#include <binder/Parcel.h>
#include <binder/ProcessState.h>
#include <binder/IPCThreadState.h>
#include <linux/msm_ion.h>
#include <fcntl.h>
#include <dirent.h>
#include <dlfcn.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <map>
#include <type_traits>

#include "recorder/src/client/qmmf_recorder_client.h"
#include "recorder/src/client/qmmf_recorder_client_ion.h"
#include "recorder/src/client/qmmf_recorder_params_internal.h"
#include "recorder/src/service/qmmf_recorder_common.h"

#ifdef LOG_LEVEL_KPI
volatile uint32_t kpi_debug_mask = KPI_DISABLE;
#endif

namespace qmmf {

namespace recorder {

//
// This file has implementation of following classes:
//
// - RecorderClient    : Delegation to binder proxy <IRecorderService>
//                       and implementation of binder CB.
// - BpRecorderService : Binder proxy implementation.
// - BpRecorderServiceCallback : Binder CB proxy implementation.
// - BnRecorderServiceCallback : Binder CB stub implementation.
//

using namespace android;
using ::std::underlying_type;

RecorderClient::RecorderClient()
                : camera_module_(NULL)
                , recorder_service_(nullptr)
                , death_notifier_(nullptr)
                , ion_device_(-1)
                , metadata_cb_(nullptr) {
  QMMF_KPI_GET_MASK();
  QMMF_INFO("%s:%s Enter ", TAG, __func__);
  sp<ProcessState> proc(ProcessState::self());
  proc->startThreadPool();
  QMMF_INFO("%s:%s Exit (0x%p)", TAG, __func__, this);
}

RecorderClient::~RecorderClient() {

  QMMF_INFO("%s:%s Enter ", TAG, __func__);

  if (recorder_service_ != nullptr) {
    recorder_service_.clear();
    recorder_service_ = nullptr;
  }

  if (NULL != camera_module_) {
    dlclose(camera_module_->common.dso);
  }
  camera_module_ = NULL;

  QMMF_INFO("%s:%s Exit 0x%p", TAG, __func__, this);
}

extern "C" {
extern int set_camera_metadata_vendor_ops(const vendor_tag_ops_t *query_ops);
}

status_t RecorderClient::Connect(const RecorderCb& cb) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  QMMF_KPI_BEGIN("Connect");
  Mutex::Autolock lock(lock_);

  if (CheckServiceStatus()) {
    QMMF_WARN("%s:%s Client is already connected to service!", TAG, __func__);
    return NO_ERROR;
  }

  //TODO: close in disconnect.
  ion_device_ = open("/dev/ion", O_RDONLY);
  if (ion_device_ < 0) {
    QMMF_ERROR("%s:%s: Can't open Ion device!", TAG, __func__);
    return NO_INIT;
  }

  recorder_cb_ = cb;

  death_notifier_ = new DeathNotifier(this);
  if (nullptr == death_notifier_.get()) {
    QMMF_ERROR("%s:%s Unable to allocate death notifier!", TAG, __func__);
    return NO_MEMORY;
  }

  sp<IBinder> service_handle;
  sp<IServiceManager> service_manager = defaultServiceManager();

  service_handle = service_manager->
      getService(String16(QMMF_RECORDER_SERVICE_NAME));
  if (service_handle.get() == nullptr) {
    QMMF_ERROR("%s:%s Can't get (%s) service", __func__, TAG,
                     QMMF_RECORDER_SERVICE_NAME);
    return NO_INIT;
  }

  recorder_service_ = interface_cast<IRecorderService>(service_handle);
  IInterface::asBinder(recorder_service_)->linkToDeath(death_notifier_);

  sp<ServiceCallbackHandler> handler = new ServiceCallbackHandler(this);
  auto ret = recorder_service_->Connect(handler);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s Can't connect to (%s) service", __func__, TAG,
        QMMF_RECORDER_SERVICE_NAME);
  }

  if (!session_cb_list_.isEmpty()) {
    session_cb_list_.clear();
  }

  if (!track_cb_list_.isEmpty()) {
    track_cb_list_.clear();
  }

  if (NULL == camera_module_) {
    //TODO: Instead of quering vendor tag ops directly from HAL module
    //      devise a mechanism to share them from service side.
    auto res = Camera3DeviceClient::LoadHWModule(CAMERA_HARDWARE_MODULE_ID,
                                                 (const hw_module_t **)&camera_module_);

    if ((0 != res) || (NULL == camera_module_)) {
      QMMF_ERROR("%s: Unable to load Hal module: %d\n", __func__, res);
      return res;
    }

    if (camera_module_->get_vendor_tag_ops) {
      vendor_tag_ops_ = vendor_tag_ops_t();
      camera_module_->get_vendor_tag_ops(&vendor_tag_ops_);

      res = set_camera_metadata_vendor_ops(&vendor_tag_ops_);
      if (0 != res) {
        QMMF_ERROR(
            "%s: Could not set vendor tag descriptor, "
            "received error %s (%d). \n",
            __func__, strerror(-res), res);
        return res;
      }
    }
  }
  QMMF_KPI_END();
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::Disconnect() {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  QMMF_KPI_BEGIN("Disconnect");
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  auto ret = recorder_service_->Disconnect();
  if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s Disconnect failed!", TAG, __func__);
  }

  recorder_service_->asBinder(recorder_service_)->
      unlinkToDeath(death_notifier_);

  recorder_service_.clear();
  recorder_service_ = NULL;

  death_notifier_.clear();
  death_notifier_ = NULL;

  if (!session_cb_list_.isEmpty()) {
    session_cb_list_.clear();
  }
  if (!track_cb_list_.isEmpty()) {
    track_cb_list_.clear();
  }
  if (!sessions_.isEmpty()) {
    sessions_.clear();
  }
  QMMF_KPI_END();
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::StartCamera(const uint32_t camera_id,
                                     const CameraStartParam &param,
                                     const CameraResultCb &result_cb) {
  bool enable_result_cb = false;
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  QMMF_KPI_BEGIN("StartCamera");
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  if (nullptr != result_cb) {
    metadata_cb_ = result_cb;
    enable_result_cb = true;
  }
  auto ret = recorder_service_->StartCamera(camera_id,
                                        const_cast<CameraStartParam&>(param),
                                        enable_result_cb);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s StartCamera failed!", TAG, __func__);
  }
  QMMF_KPI_END();
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::StopCamera(const uint32_t camera_id) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  auto ret = recorder_service_->StopCamera(camera_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s StopCamera failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::CreateSession(const SessionCb& cb,
                                       uint32_t* session_id) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  auto ret = recorder_service_->CreateSession(session_id);
  if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s CreateSession failed!", TAG, __func__);
  }
  assert(*session_id != 0);
  session_cb_list_.add(*session_id, cb);

  Vector<uint32_t> tracks;
  sessions_.add(*session_id, tracks);
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::DeleteSession(const uint32_t session_id) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  if (sessions_.indexOfKey(session_id) < 0) {
    QMMF_ERROR("%s:%s: session id is not valid!", TAG, __func__);
    return BAD_VALUE;
  }

  Vector<uint32_t> tracks;
  tracks = sessions_.valueFor(session_id);
  if (tracks.size() > 0) {
    QMMF_ERROR("%s:%s: Delete tracks first before deleting Session(%d)", TAG,
        __func__, session_id);
    return INVALID_OPERATION;
  }

  auto ret = recorder_service_->DeleteSession(session_id);
  if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s DeleteSession failed!", TAG, __func__);
  }

  if (session_cb_list_.indexOfKey(session_id) >= 0) {
      session_cb_list_.removeItem(session_id);
  }

  sessions_.removeItem(session_id);
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::StartSession(const uint32_t session_id) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  QMMF_KPI_BEGIN("StartSession");
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  Vector<uint32_t> tracks = sessions_.valueFor(session_id);
  for (size_t i = 0; i < tracks.size(); i++)
    buffer_ion_.Release(tracks[i]);

  auto ret = recorder_service_->StartSession(session_id);
  if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s StartSession failed!", TAG, __func__);
  } else {
    for (size_t i = 0; i < tracks.size(); i++) {
      QMMF_KPI_ASYNC_BEGIN("CapVideo", tracks[i]);
    }
  }
  QMMF_KPI_END();
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::StopSession(const uint32_t session_id,
                                     bool do_flush) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  auto ret = recorder_service_->StopSession(session_id, do_flush);
  if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s StopSession failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::PauseSession(const uint32_t session_id) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  QMMF_KPI_BEGIN("PauseSession");
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  auto ret = recorder_service_->PauseSession(session_id);
  if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s PauseSession failed!", TAG, __func__);
  }
  QMMF_KPI_END();
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::ResumeSession(const uint32_t session_id)
{
    QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
    QMMF_KPI_BEGIN("ResumeSession");
    Mutex::Autolock lock(lock_);

    if (!CheckServiceStatus()) {
      return NO_INIT;
    }

    auto ret = recorder_service_->ResumeSession(session_id);
    if (NO_ERROR != ret) {
        QMMF_ERROR("%s:%s ResumeSession failed!", TAG, __func__);
    }
    QMMF_KPI_END();
    QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
    return ret;
}

status_t RecorderClient::CreateAudioTrack(const uint32_t session_id,
                                          const uint32_t track_id,
                                          const AudioTrackCreateParam& param,
                                          const TrackCb& cb) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  QMMF_VERBOSE("%s:%s INPARAM: session_id[%u]", TAG, __func__, session_id);
  QMMF_VERBOSE("%s:%s INPARAM: track_id[%u]", TAG, __func__, track_id);
  QMMF_VERBOSE("%s:%s INPARAM: param[%s]", TAG, __func__,
               param.ToString().c_str());
  QMMF_KPI_BEGIN("CreateAudioTrack");

  Mutex::Autolock lock(lock_);
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }
  assert(session_id != 0);
  assert(track_id != 0);

  if (sessions_.indexOfKey(session_id) < 0) {
    QMMF_ERROR("%s:%s: session id is not valid!", TAG, __func__);
    return BAD_VALUE;
  }

  auto result = recorder_service_->CreateAudioTrack(session_id, track_id,
                                                    param);
  if (result != NO_ERROR)
      QMMF_ERROR("%s:%s CreateAudioTrack failed: %d", TAG, __func__, result);

  track_cb_list_.add(track_id, cb);

  UpdateSessionTopology(session_id, track_id, true /*add*/);

  QMMF_KPI_END();
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return result;
}

status_t RecorderClient::CreateVideoTrack(const uint32_t session_id,
                                          const uint32_t track_id,
                                          const VideoTrackCreateParam& param,
                                          const TrackCb& cb) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  QMMF_KPI_BEGIN("CreateVideoTrack");
  Mutex::Autolock lock(lock_);
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  assert(session_id != 0);
  assert(track_id != 0);

  if (sessions_.indexOfKey(session_id) < 0) {
    QMMF_ERROR("%s:%s: session_id(%d) is not valid!", TAG, __func__,
        session_id);
    return BAD_VALUE;
  }

  auto ret = recorder_service_->CreateVideoTrack(session_id, track_id,
                                     const_cast<VideoTrackCreateParam&>(param));
  if (NO_ERROR != ret) {
     QMMF_ERROR("%s:%s CreateVideoTrack failed!", TAG, __func__);
  }

  track_cb_list_.add(track_id, cb);

  UpdateSessionTopology(session_id, track_id, true /*add*/);

  QMMF_KPI_END();
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::ReturnTrackBuffer(const uint32_t session_id,
                                           const uint32_t track_id,
                                           std::vector<BufferDescriptor>
                                              &buffers) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  QMMF_VERBOSE("%s:%s INPARAM: session_id[%u]", TAG, __func__, session_id);
  QMMF_VERBOSE("%s:%s INPARAM: track_id[%u]", TAG, __func__, track_id);
  for (const BufferDescriptor& buffer : buffers)
    QMMF_VERBOSE("%s:%s INPARAM: buffer[%s]", TAG, __func__,
                 buffer.ToString().c_str());
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  QMMF_KPI_ASYNC_END("VideoAppCB", track_id);
  uint32_t ret = NO_ERROR;
  std::vector<BnBuffer> bn_buffers_ret;

  for (const BufferDescriptor& buffer : buffers) {
    BnBuffer bn_buffer = {
      buffer.buf_id,    // ion_fd
      buffer.size,      // size
      buffer.timestamp, // timestamp
      0,                // width
      0,                // height
      buffer.buf_id,    // buffer_id
      buffer.flag,      // flag
      buffer.capacity   // capacity
    };
    bn_buffers_ret.push_back(bn_buffer);
  }

  ret = recorder_service_->ReturnTrackBuffer(session_id, track_id,
                                             bn_buffers_ret);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s ReturnTrackBuffer failed: %d", TAG, __func__, ret);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::SetAudioTrackParam(const uint32_t session_id,
                                            const uint32_t track_id,
                                            CodecParamType type,
                                            const void *param,
                                            size_t param_size) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  Mutex::Autolock lock(lock_);
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  auto ret = recorder_service_->SetAudioTrackParam(session_id, track_id,
      type, const_cast<void*>(param), param_size);
  if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s SetAudioTrackParam failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::SetVideoTrackParam(const uint32_t session_id,
                                            const uint32_t track_id,
                                            CodecParamType type,
                                            const void *param,
                                            size_t param_size) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  Mutex::Autolock lock(lock_);
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  auto ret = recorder_service_->SetVideoTrackParam(session_id, track_id,
                                  type, const_cast<void*>(param), param_size);
  if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s SetVideoTrackParam failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::DeleteAudioTrack(const uint32_t session_id,
                                          const uint32_t track_id) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  QMMF_VERBOSE("%s:%s INPARAM: session_id[%u]", TAG, __func__, session_id);
  QMMF_VERBOSE("%s:%s INPARAM: track_id[%u]", TAG, __func__, track_id);

  Mutex::Autolock lock(lock_);
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  if (sessions_.indexOfKey(session_id) < 0) {
    QMMF_ERROR("%s:%s: session_id(%d) is not valid!", TAG, __func__,
        session_id);
    return BAD_VALUE;
  }

  buffer_ion_.Release(track_id);

  auto ret = recorder_service_->DeleteAudioTrack(session_id, track_id);
  if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s DeleteAudioTrack failed: %d", TAG, __func__, ret);
  }

  if (track_cb_list_.indexOfKey(track_id) >= 0) {
    track_cb_list_.removeItem(track_id);
  }

  UpdateSessionTopology(session_id, track_id, false /*remove*/);

  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::DeleteVideoTrack(const uint32_t session_id,
                                          const uint32_t track_id) {

  QMMF_DEBUG("%s:%s Enter track_id(%d)", TAG, __func__, track_id);
  int32_t ret = NO_ERROR;

  Mutex::Autolock lock(lock_);
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  if (track_buf_map_.indexOfKey(track_id) >= 0) {

    buf_info_map info_map = track_buf_map_.valueFor(track_id);
    for (size_t j = 0; j < info_map.size(); j++) {
      BufInfo buf_info = info_map.valueAt(j);
      QMMF_INFO("%s:%s: track_id(%d):buf_info.ion_fd(%d) to close", TAG,
          __func__, track_id, buf_info.ion_fd);
      QMMF_INFO("%s:%s: track_id(%d):buf_info.pointer=0x%p and frame_len=%d",
          TAG, __func__, track_id, buf_info.pointer, buf_info.frame_len);
      if (buf_info.pointer != NULL) {
        struct ion_handle_data ion_handle;
        memset(&ion_handle, 0, sizeof(ion_handle));
        ion_handle.handle = buf_info.ion_handle;
        if (ioctl(ion_device_, ION_IOC_FREE, &ion_handle) < 0) {
          QMMF_ERROR("%s:%s ION free failed: %d", TAG, __func__, -errno);
        }

        auto stat = munmap(buf_info.pointer, buf_info.frame_len);
        if (0 != stat) {
          QMMF_ERROR("%s: Failed to unmap buffer: %p : %d", __func__,
                     buf_info.pointer, -errno);
        }
        buf_info.pointer = NULL;
      }

      if (buf_info.ion_fd > 0) {
        auto stat = close(buf_info.ion_fd);
        if (0 != stat) {
          QMMF_ERROR("%s:%s Failed to close ION fd: %d : %d", TAG, __func__,
                     buf_info.ion_fd, -errno);
        }
      }
      //TODO: check owner ship of buffers, make sure application returned all
      // the buffers after calling stop on track.
    }
    track_buf_map_.removeItem(track_id);
  }

  ret = recorder_service_->DeleteVideoTrack(session_id, track_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: track_id(%d) DeleteVideoTrack failed!", TAG, __func__,
        track_id);
    ret = BAD_VALUE;
  }

  if (track_cb_list_.indexOfKey(track_id) >= 0) {
    track_cb_list_.removeItem(track_id);
  }

  UpdateSessionTopology(session_id, track_id, false /*remove*/);

  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::CaptureImage(const uint32_t camera_id,
                                      const ImageParam &param,
                                      const uint32_t num_images,
                                      const std::vector<CameraMetadata> &meta,
                                      const ImageCaptureCb& cb) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  QMMF_KPI_ASYNC_BEGIN("FirstCapImg", camera_id);

  Mutex::Autolock lock(lock_);
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }
  image_capture_cb_ = cb;
  auto ret = recorder_service_->CaptureImage(camera_id, param, num_images,
                                             meta);
  if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s CaptureImage failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::ConfigImageCapture(const uint32_t camera_id,
                                            const ImageCaptureConfig &config) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  Mutex::Autolock lock(lock_);
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }
  auto ret = recorder_service_->ConfigImageCapture(camera_id, config);
  if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s ConfigImageCapture failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::CancelCaptureImage(const uint32_t camera_id) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }
  auto ret = recorder_service_->CancelCaptureImage(camera_id);
  if(NO_ERROR != ret) {
      QMMF_ERROR("%s:%s CancelCaptureImage failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::ReturnImageCaptureBuffer(const uint32_t camera_id,
                                                  const BufferDescriptor
                                                      &buffer) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  Mutex::Autolock lock(lock_);
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }
  // Unmap buffer from client process, and close duped ION fd.
  if (buffer.data) {
    munmap(buffer.data, buffer.capacity);
    close(buffer.fd);
    QMMF_DEBUG("%s:%s fd(%d) closed!", TAG, __func__, buffer.fd);
  }
  QMMF_DEBUG("%s:%s Returning buf_id(%d) back to service!", TAG, __func__,
      buffer.buf_id);
  auto ret = recorder_service_->ReturnImageCaptureBuffer(camera_id,
                                                         buffer.buf_id);
  if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s ReturnImageCaptureBuffer failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::SetCameraParam(const uint32_t camera_id,
                                        const CameraMetadata &meta) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  Mutex::Autolock lock(lock_);
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  auto ret = recorder_service_->SetCameraParam(camera_id, meta);
  if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s SetCameraParam failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::GetCameraParam(const uint32_t camera_id,
                                        CameraMetadata &meta) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  Mutex::Autolock lock(lock_);
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }
  auto ret = recorder_service_->GetCameraParam(camera_id, meta);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s GetCameraParam failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::GetDefaultCaptureParam(const uint32_t camera_id,
                                                CameraMetadata &meta) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  Mutex::Autolock lock(lock_);
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }
  auto ret = recorder_service_->GetDefaultCaptureParam(camera_id, meta);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s GetDefaultCaptureParam failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::CreateOverlayObject(const uint32_t track_id,
                                             const OverlayParam &param,
                                             uint32_t *overlay_id) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  Mutex::Autolock lock(lock_);
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }
  auto ret = recorder_service_->CreateOverlayObject(track_id,
                                                    const_cast<OverlayParam*>
                                                    (&param), overlay_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s CreateOverlayObject failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::DeleteOverlayObject(const uint32_t track_id,
                                             const uint32_t overlay_id) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  Mutex::Autolock lock(lock_);
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }
  auto ret = recorder_service_->DeleteOverlayObject(track_id, overlay_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s DeleteOverlayObject failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::GetOverlayObjectParams(const uint32_t track_id,
                                                const uint32_t overlay_id,
                                                OverlayParam &param) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  Mutex::Autolock lock(lock_);
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }
  auto ret = recorder_service_->GetOverlayObjectParams(track_id, overlay_id,
                                                       param);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s GetOverlayObjectParams failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::UpdateOverlayObjectParams(const uint32_t track_id,
                                                   const uint32_t overlay_id,
                                                   const OverlayParam &param) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  Mutex::Autolock lock(lock_);
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }
  auto ret = recorder_service_->UpdateOverlayObjectParams(track_id, overlay_id,
                                             const_cast<OverlayParam*>(&param));
  if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s UpdateOverlayObjectParams failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::SetOverlay(const uint32_t track_id,
                                    const uint32_t overlay_id) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  Mutex::Autolock lock(lock_);
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }
  auto ret = recorder_service_->SetOverlayObject(track_id, overlay_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s SetOverlay failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::RemoveOverlay(const uint32_t track_id,
                                       const uint32_t overlay_id) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  Mutex::Autolock lock(lock_);
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }
  auto ret = recorder_service_->RemoveOverlayObject(track_id, overlay_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s RemoveOverlay failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::CreateMultiCamera(const std::vector<uint32_t>
                                           camera_ids,
                                           uint32_t *virtual_camera_id) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  Mutex::Autolock lock(lock_);
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }
  auto ret = recorder_service_->CreateMultiCamera(camera_ids,
                                                  virtual_camera_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s CreateMultiCamera failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t RecorderClient::ConfigureMultiCamera(const uint32_t virtual_camera_id,
                                              const uint32_t type,
                                              const void *param,
                                              const uint32_t param_size) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  Mutex::Autolock lock(lock_);
  if (!CheckServiceStatus()) {
    return NO_INIT;
  }
  auto ret = recorder_service_->ConfigureMultiCamera(virtual_camera_id, type,
                                                     param, param_size);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s ConfigureMultiCamera failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return ret;
}

bool RecorderClient::CheckServiceStatus() {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  bool connected = true;
  if (NULL == recorder_service_.get()) {
    QMMF_WARN("%s:%s Not connected to Recorder service!", TAG, __func__);
    connected = false;
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  return connected;
}

void RecorderClient::UpdateSessionTopology(const uint32_t session_id,
                                           const uint32_t track_id, bool add) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  Vector<uint32_t> tracks;
  tracks = sessions_.valueFor(session_id);
  if (!add) {
    for (size_t i = 0; i < tracks.size(); i++) {
      if (tracks[i] == track_id) {
        tracks.removeAt(i);
        break;
      }
    }
  } else {
    tracks.push_back(track_id);
  }
  sessions_.replaceValueFor(session_id, tracks);
  size_t size = sessions_.size();
  for (size_t i = 0; i < size; i++) {
    Vector<uint32_t> track_ids;
    track_ids = sessions_.valueFor(session_id);
    for (size_t j = 0; j < track_ids.size(); j++) {
      QMMF_INFO("%s:%s: session_id(%d):track_id(%d)", TAG, __func__, session_id,
          track_ids[j]);
    }
  }
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

void RecorderClient::NotifyRecorderEvent(EventType event_type, void *event_data,
                                         size_t event_data_size) {
    QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
    QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

void RecorderClient::NotifySessionEvent(EventType event_type, void *event_data,
                                        size_t event_data_size) {
    QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
    QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

void RecorderClient::NotifySnapshotData(uint32_t camera_id,
                                        uint32_t image_sequence_count,
                                        BnBuffer& buffer, MetaData& meta_data) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  assert(image_capture_cb_ != nullptr);
  assert(ion_device_ > 0);
  struct ion_fd_data ion_info_fd;
  memset(&ion_info_fd, 0x0, sizeof(ion_info_fd));

  assert(buffer.ion_fd > 0);
  assert(buffer.buffer_id > 0);

  ion_info_fd.fd = buffer.ion_fd;
  auto ret = ioctl(ion_device_, ION_IOC_IMPORT, &ion_info_fd);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: ION_IOC_IMPORT failed for fd(%d)", TAG, __func__,
        ion_info_fd.fd);
    return;
  }
  QMMF_VERBOSE("%s:%s: ion_fd(%d)", TAG, __func__, ion_info_fd.fd);
  void* vaddr = mmap(NULL, buffer.capacity, PROT_READ | PROT_WRITE,
                     MAP_SHARED, ion_info_fd.fd, 0);
  assert(vaddr != NULL);

  BufferDescriptor snapshot_buf;
  memset(&snapshot_buf, 0x0, sizeof snapshot_buf);
  snapshot_buf.data      = vaddr;
  snapshot_buf.size      = buffer.size;
  snapshot_buf.timestamp = buffer.timestamp;
  snapshot_buf.flag      = buffer.flag;
  snapshot_buf.capacity  = buffer.capacity;
  snapshot_buf.buf_id    = buffer.buffer_id;
  snapshot_buf.fd        = buffer.ion_fd;

  if(image_sequence_count == 0) {
    QMMF_KPI_ASYNC_END("FirstCapImg", camera_id);
  } else {
    QMMF_KPI_ASYNC_END("SnapShot-Shot", camera_id);
  }

  QMMF_KPI_ASYNC_BEGIN("SnapShot-Shot", camera_id);

  image_capture_cb_(camera_id, image_sequence_count, snapshot_buf, meta_data);
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

void RecorderClient::NotifyVideoTrackData(uint32_t track_id,
                                          std::vector<BnBuffer> &bn_buffers,
                                          std::vector<MetaData> &meta_buffers) {

  QMMF_VERBOSE("%s:%s Enter ", TAG, __func__);
  QMMF_VERBOSE("%s:%s track_id = %d ", TAG, __func__, track_id);
  std::vector<BufferDescriptor> track_buffers;
  for (size_t i = 0; i < bn_buffers.size(); i++) {

    bool is_mapped = false;
    BufInfo buf_info;
    memset(&buf_info, 0x0, sizeof buf_info);

    // Check if ION buffer is already imported and mapped, if it is then get
    // buffer info from map.
    if (!track_buf_map_.isEmpty()) {

      DefaultKeyedVector<uint32_t, BufInfo> buf_map;
      int32_t map_idx = track_buf_map_.indexOfKey(track_id);
      if (map_idx >= 0) {
        buf_map = track_buf_map_.valueFor(track_id);
        int32_t buf_idx = buf_map.indexOfKey(bn_buffers[i].buffer_id);

        if (buf_idx >= 0) {
          buf_info = buf_map.valueFor(bn_buffers[i].buffer_id);
          assert(buf_info.pointer != NULL);
          assert(buf_info.ion_fd > 0);
          is_mapped = true;
          QMMF_VERBOSE("%s:%s: Buf is already mapped! buffer_id(%d):ion_fd(%d):"
            "vaddr(0x%p)", TAG, __func__, bn_buffers[i].buffer_id,
            buf_info.ion_fd, buf_info.pointer);
        }
      }
    }
    if (!is_mapped) {
      // Map Ion Fd to client address space.
      assert(ion_device_ > 0);
      struct ion_fd_data ion_info_fd;
      memset(&ion_info_fd, 0x0, sizeof(ion_info_fd));

      assert(bn_buffers[i].ion_fd > 0);
      assert(bn_buffers[i].buffer_id > 0);

      ion_info_fd.fd = bn_buffers[i].ion_fd;
      auto ret = ioctl(ion_device_, ION_IOC_IMPORT, &ion_info_fd);
      if (ret != NO_ERROR) {
        QMMF_ERROR("%s:%s: ION_IOC_IMPORT failed for fd(%d)", TAG, __func__,
            ion_info_fd.fd);
      }
      QMMF_VERBOSE("%s:%s: ion_info_fd.fd =%d", TAG, __func__, ion_info_fd.fd);
      void* vaddr = mmap(NULL, bn_buffers[i].capacity, PROT_READ | PROT_WRITE,
                        MAP_SHARED, ion_info_fd.fd, 0);
      assert(vaddr != NULL);

      buf_info.pointer   = vaddr;
      buf_info.ion_fd    = ion_info_fd.fd;
      buf_info.frame_len = bn_buffers[i].capacity;
      buf_info.ion_handle = ion_info_fd.handle;

      DefaultKeyedVector<uint32_t, BufInfo> buffer_map;
      if (track_buf_map_.isEmpty()) {
        buffer_map.add(bn_buffers[i].buffer_id, buf_info);
      } else {
        buffer_map = track_buf_map_.valueFor(track_id);
        buffer_map.add(bn_buffers[i].buffer_id, buf_info);
      }
      // Update existing entry or add new one.
      // replaceValueFor() performs as add if entry doesn't exist.
      track_buf_map_.replaceValueFor(track_id, buffer_map);

      QMMF_VERBOSE("%s:%s: track_buf_map_.size = %d", TAG, __func__,
          track_buf_map_.size());

      for (uint32_t i = 0; i < track_buf_map_.size(); i++) {
        buffer_map = track_buf_map_[i];
        QMMF_VERBOSE("%s:%s: buffer_map.size=%d", TAG, __func__,
            buffer_map.size());

        for(uint32_t j = 0; j < buffer_map.size(); j++) {
          QMMF_VERBOSE("%s:%s: buffer_map:idx(%d) :key(%d) :ion_fd:%d :pointer:"
              "0x%p", TAG, __func__, j, buffer_map.keyAt(j),
              buffer_map[j].ion_fd, buffer_map[j].pointer);
        }
      }
    }

    BufferDescriptor buffer;
    memset(&buffer, 0x0, sizeof buffer);
    buffer.data      = buf_info.pointer;
    buffer.size      = bn_buffers[i].size;
    buffer.timestamp = bn_buffers[i].timestamp;
    buffer.flag      = bn_buffers[i].flag;
    buffer.buf_id    = bn_buffers[i].buffer_id;
    buffer.capacity  = bn_buffers[i].capacity;
    buffer.fd        = bn_buffers[i].ion_fd;
    track_buffers.push_back(buffer);

  }

  //Get the handle to client callback.
  TrackCb callback = track_cb_list_.valueFor(track_id);
  QMMF_KPI_ASYNC_END("CapVideo", track_id);
  QMMF_KPI_ASYNC_BEGIN("VideoAppCB", track_id);
  callback.data_cb(track_id, track_buffers, meta_buffers);
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

void RecorderClient::NotifyVideoTrackEvent(uint32_t track_id,
                                           EventType event_type,
                                           void *event_data,
                                           size_t event_data_size) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

void RecorderClient::NotifyAudioTrackData(uint32_t track_id,
                                          const std::vector<BnBuffer>&
                                          bn_buffers,
                                          const std::vector<MetaData>&
                                          meta_buffers) {

  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  QMMF_VERBOSE("%s:%s INPARAM: track_id[%u]", TAG, __func__, track_id);
  for (const BnBuffer& bn_buffer : bn_buffers)
    QMMF_VERBOSE("%s:%s INPARAM: bn_buffer[%s]", TAG, __func__,
                 bn_buffer.ToString().c_str());

  std::vector<BufferDescriptor> track_buffers;
  for (const BnBuffer& bn_buffer : bn_buffers) {
    BufferDescriptor buffer;
    buffer_ion_.Associate(track_id, bn_buffer, &buffer);
    track_buffers.push_back(buffer);
  }

  // Get the handle to client callback.
  TrackCb callback = track_cb_list_.valueFor(track_id);
  callback.data_cb(track_id, track_buffers, meta_buffers);
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

void RecorderClient::NotifyAudioTrackEvent(uint32_t track_id,
                                           EventType event_type,
                                           void *event_data,
                                           size_t event_data_size) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  QMMF_VERBOSE("%s:%s INPARAM: track_id[%u]", TAG, __func__, track_id);
  QMMF_VERBOSE("%s:%s INPARAM: event_type[%d]", TAG, __func__,
               static_cast<underlying_type<EventType>::type>(event_type));

  // get the handle to client callback.
  TrackCb callback = track_cb_list_.valueFor(track_id);
  callback.event_cb(track_id, event_type, event_data, event_data_size);

  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

void RecorderClient::NotifyCameraResult(uint32_t camera_id,
                                        const CameraMetadata &result) {
  if (nullptr != metadata_cb_) {
    metadata_cb_(camera_id, result);
  } else {
    QMMF_ERROR("%s:%s No client registered result callback!\n",
               TAG, __func__);
  }
}

//Binder Proxy implementation of IRecoderService.
class BpRecorderService: public BpInterface<IRecorderService> {
 public:
  BpRecorderService(const sp<IBinder>& impl)
  : BpInterface<IRecorderService>(impl) {}

  status_t Connect(const sp<IRecorderServiceCallback>& service_cb) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    //Register service callback to get callbacks from recorder service.
    //eg : JPEG buffer, Tracks elementry buffers, Recorder/Session status
    //callbacks etc.
    data.writeStrongBinder(IInterface::asBinder(service_cb));
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_CONNECT), data, &reply);
    return reply.readInt32();
  }

  status_t Disconnect() {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_DISCONNECT), data, &reply);
    return reply.readInt32();
  }

  status_t StartCamera(const uint32_t camera_id,
                       const CameraStartParam &param,
                       bool enable_result_cb) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(camera_id);
    data.writeUint32(enable_result_cb ? 1 : 0);
    uint32_t param_size = sizeof param;
    data.writeUint32(param_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(param_size, false, &blob);
    memset(blob.data(), 0x0, param_size);
    CameraStartParam* start_param;
    start_param = const_cast<CameraStartParam*>(&param);
    memcpy(blob.data(), reinterpret_cast<void*>(start_param), param_size);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_START_CAMERA), data, &reply);
    blob.release();
    return reply.readInt32();
  }

  status_t StopCamera(const uint32_t camera_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(camera_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_STOP_CAMERA), data, &reply);
    return reply.readInt32();
  }

  status_t CreateSession(uint32_t *session_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_CREATE_SESSION), data, &reply);
    uint32_t id;
    reply.readUint32(&id);
    *session_id = id;
    assert(id != 0);
    return reply.readInt32();
  }

  status_t DeleteSession(const uint32_t session_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    assert(session_id != 0);
    data.writeUint32(session_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_DELETE_SESSION), data, &reply);
    return reply.readInt32();
  }

  status_t StartSession(const uint32_t session_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    assert(session_id != 0);
    data.writeUint32(session_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
        RECORDER_START_SESSION), data, &reply);
    return reply.readInt32();
  }

  status_t StopSession(const uint32_t session_id, bool do_flush) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    assert(session_id != 0);
    data.writeUint32(session_id);
    data.writeInt32(do_flush);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
        RECORDER_STOP_SESSION), data, &reply);
    return reply.readInt32();
  }

  status_t PauseSession(const uint32_t session_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    assert(session_id != 0);
    data.writeUint32(session_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_PAUSE_SESSION), data, &reply);
    return reply.readInt32();
  }

  status_t ResumeSession(const uint32_t session_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    assert(session_id != 0);
    data.writeUint32(session_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_RESUME_SESSION), data, &reply);
    return reply.readInt32();
  }

  status_t CreateAudioTrack(const uint32_t session_id, const uint32_t track_id,
                            const AudioTrackCreateParam& param) {
    QMMF_DEBUG("%s:%s Enter", TAG, __func__);
    QMMF_VERBOSE("%s:%s INPARAM: session_id[%u]", TAG, __func__, session_id);
    QMMF_VERBOSE("%s:%s INPARAM: track_id[%u]", TAG, __func__, track_id);
    QMMF_VERBOSE("%s:%s INPARAM: param[%s]", TAG, __func__,
                 param.ToString().c_str());
    Parcel data, reply;

    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(session_id);
    data.writeUint32(track_id);
    AudioTrackCreateParamInternal(param).ToParcel(&data);

    remote()->transact(
        uint32_t(QMMF_RECORDER_SERVICE_CMDS::RECORDER_CREATE_AUDIOTRACK),
        data, &reply);

    return reply.readInt32();
  }

  status_t CreateVideoTrack(const uint32_t session_id,
                            const uint32_t track_id,
                            const VideoTrackCreateParam& params) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(session_id);
    data.writeUint32(track_id);
    uint32_t param_size = sizeof params;
    data.writeUint32(param_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(param_size, false, &blob);
    memset(blob.data(), 0x0, param_size);
    VideoTrackCreateParam* track_params =
        const_cast<VideoTrackCreateParam*>(&params);
    memcpy(blob.data(), reinterpret_cast<void*>(track_params), param_size);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
        RECORDER_CREATE_VIDEOTRACK), data, &reply);
    blob.release();
    return reply.readInt32();
  }

  status_t DeleteAudioTrack(const uint32_t session_id,
                            const uint32_t track_id) {
    QMMF_DEBUG("%s:%s Enter", TAG, __func__);
    QMMF_VERBOSE("%s:%s INPARAM: session_id[%u]", TAG, __func__, session_id);
    QMMF_VERBOSE("%s:%s INPARAM: track_id[%u]", TAG, __func__, track_id);
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(session_id);
    data.writeUint32(track_id);

    remote()->transact(
        uint32_t(QMMF_RECORDER_SERVICE_CMDS::RECORDER_DELETE_AUDIOTRACK),
        data, &reply);

    return reply.readInt32();
  }

  status_t DeleteVideoTrack(const uint32_t session_id,
                            const uint32_t track_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(session_id);
    data.writeUint32(track_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_DELETE_VIDEOTRACK), data, &reply);
    return reply.readInt32();
  }

  status_t ReturnTrackBuffer(const uint32_t session_id,
                             const uint32_t track_id,
                             std::vector<BnBuffer> &buffers) {

    QMMF_DEBUG("%s:%s Enter", TAG, __func__);
    QMMF_VERBOSE("%s:%s INPARAM: session_id[%u]", TAG, __func__, session_id);
    QMMF_VERBOSE("%s:%s INPARAM: track_id[%u]", TAG, __func__, track_id);
    for (const BnBuffer& buffer : buffers) {
      QMMF_VERBOSE("%s:%s INPARAM: buffers[%s]", TAG, __func__,
          buffer.ToString().c_str());
    }

    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(session_id);
    data.writeUint32(track_id);
    if (track_id < 100) {
      uint32_t size = buffers.size();
      assert(size > 0);
      data.writeUint32(size);
      // TODO: combine all BnBuffers together in single blob
      for (uint32_t i = 0; i < size; i++) {
        uint32_t param_size = sizeof (BnBuffer);
        data.writeUint32(param_size);
        android::Parcel::WritableBlob blob;
        data.writeBlob(param_size, false, &blob);
        memset(blob.data(), 0x0, param_size);
        buffers[i].ion_fd = buffers[i].buffer_id;
        memcpy(blob.data(), reinterpret_cast<void*>(&buffers[i]), param_size);
      }
    } else {
      data.writeInt32(static_cast<int32_t>(buffers.size()));
      for (const BnBuffer& buffer : buffers) {
        buffer.ToParcel(&data, false);
      }
    }

    remote()->transact(
        uint32_t(QMMF_RECORDER_SERVICE_CMDS::RECORDER_RETURN_TRACKBUFFER),
        data, &reply, IBinder::FLAG_ONEWAY);

    return NO_ERROR;
  }

  status_t SetAudioTrackParam(const uint32_t session_id,
                              const uint32_t track_id,
                              CodecParamType type,
                              void *param,
                              size_t param_size) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(session_id);
    data.writeUint32(track_id);
    data.writeUint32(static_cast<uint32_t>(type));
    data.writeUint32(param_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(param_size, false, &blob);
    memcpy(blob.data(), reinterpret_cast<void*>(param), param_size);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_SET_AUDIOTRACK_PARAMS), data, &reply);
    blob.release();
    return reply.readInt32();
  }

  status_t SetVideoTrackParam(const uint32_t session_id,
                              const uint32_t track_id,
                              CodecParamType type,
                              void *param,
                              size_t param_size) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(session_id);
    data.writeUint32(track_id);
    data.writeUint32(static_cast<uint32_t>(type));
    data.writeUint32(param_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(param_size, false, &blob);
    memcpy(blob.data(), reinterpret_cast<void*>(param), param_size);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_SET_VIDEOTRACK_PARAMS), data, &reply);
    blob.release();
    return reply.readInt32();
  }

  status_t CaptureImage(const uint32_t camera_id, const ImageParam &param,
                        const uint32_t num_images,
                        const std::vector<CameraMetadata> &meta) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(camera_id);
    uint32_t param_size = sizeof param;
    data.writeUint32(param_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(param_size, false, &blob);
    ImageParam *image_param;
    image_param = const_cast<ImageParam*>(&param);
    memcpy(blob.data(), reinterpret_cast<void*>(image_param), param_size);
    data.writeUint32(num_images);
    data.writeUint32(meta.size());
    for (uint8_t i = 0; i < meta.size(); ++i) {
      meta[i].writeToParcel(&data);
    }
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
        RECORDER_CAPTURE_IMAGE), data, &reply);
    blob.release();
    return reply.readInt32();
  }

  status_t ConfigImageCapture(const uint32_t camera_id,
                              const ImageCaptureConfig &config) {
    return 0;
  }

  status_t CancelCaptureImage(const uint32_t camera_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(camera_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                       RECORDER_CANCEL_IMAGECAPTURE), data, &reply);
    return reply.readInt32();
  }

  status_t ReturnImageCaptureBuffer(const uint32_t camera_id,
                                    const int32_t buffer_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(camera_id);
    data.writeUint32(buffer_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                       RECORDER_RETURN_IMAGECAPTURE_BUFFER), data, &reply);
    return reply.readInt32();
  }

  status_t SetCameraParam(const uint32_t camera_id,
                          const CameraMetadata &meta) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(camera_id);
    meta.writeToParcel(&data);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                       RECORDER_SET_CAMERA_PARAMS), data, &reply);
    return reply.readInt32();
  }

  status_t GetCameraParam(const uint32_t camera_id,
                          CameraMetadata &meta) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(camera_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                                RECORDER_GET_CAMERA_PARAMS), data, &reply);
    auto ret = reply.readInt32();
    if (NO_ERROR == ret) {
      ret = meta.readFromParcel(&reply);
    }
    return ret;
  }

  status_t GetDefaultCaptureParam(const uint32_t camera_id,
                                  CameraMetadata &meta) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(camera_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                                RECORDER_GET_DEFAULT_CAPTURE_PARAMS), data,
                                &reply);
    auto ret = reply.readInt32();
    if (NO_ERROR == ret) {
      ret = meta.readFromParcel(&reply);
    }
    return ret;
  }

  status_t CreateOverlayObject(const uint32_t track_id, OverlayParam *params,
                               uint32_t *overlay_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(track_id);
    uint32_t param_size = sizeof(OverlayParam);
    data.writeUint32(param_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(param_size, false, &blob);
    memset(blob.data(), 0x0, param_size);
    memcpy(blob.data(), reinterpret_cast<void*>(params), param_size);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_CREATE_OVERLAYOBJECT), data, &reply);
    blob.release();
    *overlay_id = reply.readUint32();
    return reply.readInt32();;
  }

  status_t DeleteOverlayObject(const uint32_t track_id,
                               const uint32_t overlay_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(track_id);
    data.writeUint32(overlay_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_DELETE_OVERLAYOBJECT), data, &reply);
    return reply.readInt32();
  }

  status_t GetOverlayObjectParams(const uint32_t track_id,
                                  const uint32_t overlay_id,
                                  OverlayParam &param) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(track_id);
    data.writeUint32(overlay_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                          RECORDER_GET_OVERLAYOBJECT_PARAMS), data, &reply);
    auto ret = reply.readInt32();
    if (NO_ERROR == ret) {
        uint32_t param_size;
        reply.readUint32(&param_size);
        android::Parcel::ReadableBlob blob;
        reply.readBlob(param_size, &blob);
        memcpy(&param, blob.data(), param_size);
        blob.release();
    }
    return ret;
  }

  status_t UpdateOverlayObjectParams(const uint32_t track_id,
                                     const uint32_t overlay_id,
                                     OverlayParam *params) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(track_id);
    data.writeUint32(overlay_id);
    uint32_t param_size = sizeof(OverlayParam);
    data.writeUint32(param_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(param_size, false, &blob);
    memset(blob.data(), 0x0, param_size);
    memcpy(blob.data(), reinterpret_cast<void*>(params), param_size);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                       RECORDER_UPDATE_OVERLAYOBJECT_PARAMS), data, &reply);
    blob.release();
    return reply.readInt32();
  }

  status_t SetOverlayObject(const uint32_t track_id,
                            const uint32_t overlay_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(track_id);
    data.writeUint32(overlay_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                       RECORDER_SET_OVERLAYOBJECT), data, &reply);
    return reply.readInt32();
  }

  status_t RemoveOverlayObject(const uint32_t track_id,
                               const uint32_t overlay_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(track_id);
    data.writeUint32(overlay_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                       RECORDER_REMOVE_OVERLAYOBJECT), data, &reply);
    return reply.readInt32();
  }

  status_t CreateMultiCamera(const std::vector<uint32_t> camera_ids,
                             uint32_t *virtual_camera_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    uint32_t vector_size = camera_ids.size();
    data.writeUint32(vector_size);
    for (uint8_t i = 0; i < vector_size; ++i) {
      data.writeUint32(camera_ids[i]);
    }
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                                RECORDER_CREATE_MULTICAMERA), data, &reply);
    *virtual_camera_id = reply.readUint32();
    return reply.readInt32();;
  }

  status_t ConfigureMultiCamera(const uint32_t virtual_camera_id,
                                const uint32_t type,
                                const void *param,
                                const uint32_t param_size) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(virtual_camera_id);
    data.writeUint32(type);
    data.writeUint32(param_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(param_size, false, &blob);
    memcpy(blob.data(), param, param_size);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                                RECORDER_CONFIGURE_MULTICAMERA), data, &reply);
    blob.release();
    return reply.readInt32();
  }

};

IMPLEMENT_META_INTERFACE(RecorderService, QMMF_RECORDER_SERVICE_NAME);

ServiceCallbackHandler::ServiceCallbackHandler(RecorderClient* client)
    : client_(client) {
    QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
    QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

ServiceCallbackHandler::~ServiceCallbackHandler() {
    QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
    QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

void ServiceCallbackHandler::NotifyRecorderEvent(EventType event_type,
                                                 void *event_data,
                                                 size_t event_data_size) {
    QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
    QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

void ServiceCallbackHandler::NotifySessionEvent(EventType event_type,
                                                void *event_data,
                                                size_t event_data_size) {
    QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
    QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

void ServiceCallbackHandler::NotifySnapshotData(uint32_t camera_id,
                                                uint32_t image_sequence_count,
                                                BnBuffer& buffer,
                                                MetaData& meta_data) {
  assert(client_ != NULL);
  client_->NotifySnapshotData(camera_id, image_sequence_count, buffer,
                              meta_data);
}


void ServiceCallbackHandler::NotifyVideoTrackData(uint32_t track_id,
                                                  std::vector<BnBuffer>&
                                                  bn_buffers,
                                                  std::vector<MetaData>&
                                                  meta_buffers) {

  QMMF_VERBOSE("%s:%s Enter ", TAG, __func__);
  assert(client_ != NULL);
  client_->NotifyVideoTrackData(track_id, bn_buffers, meta_buffers);
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

void ServiceCallbackHandler::NotifyVideoTrackEvent(uint32_t track_id,
                                                   EventType event_type,
                                                   void *event_data,
                                                   size_t event_data_size) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

void ServiceCallbackHandler::NotifyAudioTrackData(uint32_t track_id,
                                                  const std::vector<BnBuffer>&
                                                  bn_buffers,
                                                  const std::vector<MetaData>&
                                                  meta_buffers) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  QMMF_VERBOSE("%s:%s INPARAM: track_id[%u]", TAG, __func__, track_id);
  for (const BnBuffer& bn_buffer : bn_buffers)
    QMMF_VERBOSE("%s:%s INPARAM: bn_buffer[%s]", TAG, __func__,
                 bn_buffer.ToString().c_str());
  assert(client_ != NULL);

  client_->NotifyAudioTrackData(track_id, bn_buffers, meta_buffers);

  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

void ServiceCallbackHandler::NotifyAudioTrackEvent(uint32_t track_id,
                                                   EventType event_type,
                                                   void *event_data,
                                                   size_t event_data_size) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  QMMF_VERBOSE("%s:%s INPARAM: track_id[%u]", TAG, __func__, track_id);
  QMMF_VERBOSE("%s:%s INPARAM: event_type[%d]", TAG, __func__,
               static_cast<underlying_type<EventType>::type>(event_type));
  assert(client_ != NULL);

  client_->NotifyAudioTrackEvent(track_id, event_type, event_data,
                                 event_data_size);

  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

void ServiceCallbackHandler::NotifyCameraResult(uint32_t camera_id,
                                                const CameraMetadata &result) {
  assert(client_ != NULL);
  client_->NotifyCameraResult(camera_id, result);
}

class BpRecorderServiceCallback: public BpInterface<IRecorderServiceCallback> {
 public:
  BpRecorderServiceCallback(const sp<IBinder>& impl)
     : BpInterface<IRecorderServiceCallback>(impl) {}

  ~BpRecorderServiceCallback() {
    if (!track_buf_map_.isEmpty()) {
      track_buf_map_.clear();
    }
    //TODO: Expose DeleteTrack Api from Binder proxy and call it from service.
  }

  void NotifyRecorderEvent(EventType event_type, void *event_data,
                           size_t event_data_size) {

  }

  void NotifySessionEvent(EventType event_type, void *event_data,
                          size_t event_data_size) {

  }

  void NotifySnapshotData(uint32_t camera_id, uint32_t image_sequence_count,
                          BnBuffer& buffer, MetaData& meta_data) {

    Parcel data, reply;
    data.writeInterfaceToken(IRecorderServiceCallback::
        getInterfaceDescriptor());
    data.writeUint32(camera_id);
    data.writeUint32(image_sequence_count);
    data.writeFileDescriptor(buffer.ion_fd);
    uint32_t size = sizeof buffer;
    data.writeUint32(size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(size, false, &blob);
    memset(blob.data(), 0x0, size);
    memcpy(blob.data(), reinterpret_cast<void*>(&buffer), size);
    // Pack meta
    size = sizeof meta_data;
    data.writeUint32(size);
    android::Parcel::WritableBlob meta_blob;
    data.writeBlob(size, false, &meta_blob);
    memset(meta_blob.data(), 0x0, size);
    memcpy(meta_blob.data(), reinterpret_cast<void*>(&meta_data), size);

    remote()->transact(uint32_t(RECORDER_SERVICE_CB_CMDS::
        RECORDER_NOTIFY_SNAPSHOT_DATA), data, &reply, IBinder::FLAG_ONEWAY);

    blob.release();
    meta_blob.release();
  }

  void NotifyVideoTrackData(uint32_t track_id, std::vector<BnBuffer>& buffers,
                            std::vector<MetaData>& meta_buffers) {

    QMMF_VERBOSE("%s:Bp%s: Enter", TAG, __func__);

    Parcel data, reply;
    data.writeInterfaceToken(IRecorderServiceCallback::
        getInterfaceDescriptor());

    data.writeUint32(track_id);
    data.writeUint32(buffers.size());
    for(uint32_t i = 0; i < buffers.size(); i++) {

      bool is_mapped = false;
      int32_t idx = -1;
      if (!track_buf_map_.isEmpty()) {
        idx = track_buf_map_.indexOfKey(track_id);
        if (idx >= 0) {
          buffer_map buf_map;
          buf_map  = track_buf_map_.valueFor(track_id);
          int32_t id = buf_map.indexOfKey(buffers[i].ion_fd);
          if (id >= 0) {
            // This ION fd has already been sent to client, no binder packing is
            // required, only index would be sufficient for client to get mapped
            // buffer from his own map.
            is_mapped = buf_map.valueFor(buffers[i].ion_fd);
            QMMF_VERBOSE("%s:Bp%s: buffers[%d].ion_fd=%d is_mapped:%d", TAG,
                __func__, i, buffers[i].ion_fd, is_mapped);

          }
        }
      }
      // If buffer has not been sent to client then pack the file descriptor
      // and provide hint about incoming fd.
      data.writeInt32(!is_mapped);
      if (!is_mapped) {
        // Pack file descriptor.
        data.writeFileDescriptor(buffers[i].ion_fd);
        buffer_map map_to_update;
        if (idx >= 0) {
          map_to_update = track_buf_map_.valueFor(track_id);
        }
        map_to_update.add(buffers[i].ion_fd, true);
        track_buf_map_.replaceValueFor(track_id, map_to_update);
        QMMF_VERBOSE("%s:Bp%s: track_id=%d", TAG, __func__, track_id);
        QMMF_VERBOSE("%s:Bp%s: buffers[%d].ion_fd=%d mapping:%d", TAG, __func__,
            i, buffers[i].ion_fd, true);
      }
      uint32_t size = sizeof (BnBuffer);
      data.writeUint32(size);
      android::Parcel::WritableBlob blob;
      data.writeBlob(size, false, &blob);
      memset(blob.data(), 0x0, size);
      memcpy(blob.data(), reinterpret_cast<void*>(&buffers[i]), size);
    }
    // Pack meta
    data.writeUint32(meta_buffers.size());
    for(uint32_t i = 0; i < meta_buffers.size(); ++i) {
      uint32_t size = sizeof (MetaData);
      data.writeUint32(size);
      android::Parcel::WritableBlob meta_blob;
      data.writeBlob(size, false, &meta_blob);
      memset(meta_blob.data(), 0x0, size);
      memcpy(meta_blob.data(), reinterpret_cast<void*>(&meta_buffers[i]), size);
    }

    remote()->transact(uint32_t(RECORDER_SERVICE_CB_CMDS::
        RECORDER_NOTIFY_VIDEO_TRACK_DATA), data, &reply);

    QMMF_VERBOSE("%s:%s: Exit - Sent Message One Way!!", TAG, __func__);
  }

  void NotifyVideoTrackEvent(uint32_t track_id, EventType event_type,
                             void *event_data, size_t event_data_size) {

  }

  void NotifyAudioTrackData(uint32_t track_id,
                            const std::vector<BnBuffer>& buffers,
                            const std::vector<MetaData>& meta_buffers) {
    QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
    QMMF_VERBOSE("%s:%s INPARAM: track_id[%u]", TAG, __func__, track_id);
    for (const BnBuffer& buffer : buffers)
      QMMF_VERBOSE("%s:%s INPARAM: buffer[%s]", TAG, __func__,
                   buffer.ToString().c_str());
    Parcel data, reply;

    data.writeInterfaceToken(
        IRecorderServiceCallback::getInterfaceDescriptor());
    data.writeUint32(track_id);
    data.writeInt32(static_cast<int32_t>(buffers.size()));
    for (const BnBuffer& buffer : buffers)
      buffer.ToParcel(&data, true);

    remote()->transact(
        uint32_t(RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_AUDIO_TRACK_DATA),
        data, &reply, IBinder::FLAG_ONEWAY);

    QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  }

  void NotifyAudioTrackEvent(uint32_t track_id, EventType event_type,
                             void *event_data, size_t event_data_size) {
    QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
    QMMF_VERBOSE("%s:%s INPARAM: track_id[%u]", TAG, __func__, track_id);
    QMMF_VERBOSE("%s:%s INPARAM: event_type[%d]", TAG, __func__,
                 static_cast<underlying_type<EventType>::type>(event_type));
    Parcel data, reply;

    data.writeInterfaceToken(
        IRecorderServiceCallback::getInterfaceDescriptor());
    data.writeUint32(track_id);
    data.writeInt32(static_cast<underlying_type<EventType>::type>(event_type));

    remote()->transact(
        uint32_t(RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_AUDIO_TRACK_EVENT),
        data, &reply, IBinder::FLAG_ONEWAY);

    QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
  }

  void NotifyCameraResult(uint32_t camera_id, const CameraMetadata &result) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderServiceCallback::getInterfaceDescriptor());
    data.writeUint32(camera_id);
    result.writeToParcel(&data);
    remote()->transact(uint32_t(RECORDER_SERVICE_CB_CMDS::
                                RECORDER_NOTIFY_CAMERA_RESULT), data, &reply,
                                IBinder::FLAG_ONEWAY);
  }

  void NotifyDeleteVideoTrack(uint32_t track_id) {
    QMMF_VERBOSE("%s:Bp%s: Enter", TAG, __func__);
    if (track_buf_map_.isEmpty()) {
      return;
    }
    if (track_buf_map_.indexOfKey(track_id) >= 0) {
      track_buf_map_.removeItem(track_id);
    }
    QMMF_VERBOSE("%s:Bp%s: Exit", TAG, __func__);
  }

 private:
  // vector <ion_fd, bool>
  typedef DefaultKeyedVector <uint32_t, bool> buffer_map;
  // vector <track_id , buffer_map>
  DefaultKeyedVector<uint32_t,  buffer_map > track_buf_map_;
};

IMPLEMENT_META_INTERFACE(RecorderServiceCallback,
                            "recorder.service.IRecorderServiceCallback");

status_t BnRecorderServiceCallback::onTransact(uint32_t code,
                                               const Parcel& data,
                                               Parcel* reply,
                                               uint32_t flags) {
  QMMF_DEBUG("%s:%s: Enter:(BnRecorderServiceCallback::onTransact)", TAG,
      __func__);
  CHECK_INTERFACE(IRecorderServiceCallback, data, reply);

  switch(code) {
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_EVENT: {
    //TODO:
        return NO_ERROR;
    }
    break;
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_SESSION_EVENT: {
      //TODO:
      return NO_ERROR;
    }
    break;
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_SNAPSHOT_DATA: {
      uint32_t camera_id, count, size;
      data.readUint32(&camera_id);
      data.readUint32(&count);
      uint32_t ion_fd = dup(data.readFileDescriptor());
      data.readUint32(&size);
      android::Parcel::ReadableBlob blob;
      data.readBlob(size, &blob);
      void* buf = const_cast<void*>(blob.data());
      BnBuffer bn_buffer;
      memset(&bn_buffer, 0x0, sizeof bn_buffer);
      memcpy(&bn_buffer, buf, size);
      bn_buffer.ion_fd = ion_fd;
      uint32_t meta_size;
      MetaData meta_data;
      memset(&meta_data, 0x0, sizeof meta_data);
      android::Parcel::ReadableBlob meta_blob;
      data.readUint32(&meta_size);
      if (meta_size > 0) {
        data.readBlob(meta_size, &meta_blob);
        void* meta = const_cast<void*>(meta_blob.data());
        memcpy(&meta_data, meta, meta_size);
      }
      NotifySnapshotData(camera_id, count, bn_buffer, meta_data);
      blob.release();
      if (meta_size > 0) {
        meta_blob.release();
      }
      return NO_ERROR;
    }
    break;
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_VIDEO_TRACK_DATA: {

      uint32_t track_id, vector_size;
      std::vector<BnBuffer> buffers;
      data.readUint32(&track_id);
      data.readUint32(&vector_size);
      QMMF_VERBOSE("%s:Bn%s: vector_size=%d", TAG, __func__, vector_size);
      uint32_t size = 0;
      for (uint32_t i = 0; i < vector_size; i++)  {
        int32_t is_fd = 0;
        int32_t ion_fd = -1;
        data.readInt32(&is_fd);
        if (is_fd == 1) {
          ion_fd = dup(data.readFileDescriptor());
        }
        data.readUint32(&size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(size, &blob);
        void* buffer = const_cast<void*>(blob.data());
        BnBuffer track_buffer;
        memcpy(&track_buffer, buffer, size);
        track_buffer.ion_fd = ion_fd;
        buffers.push_back(track_buffer);
        blob.release();
      }
      uint32_t meta_vector_size = 0;
      std::vector<MetaData> meta_buffers;
      data.readUint32(&meta_vector_size);
      QMMF_VERBOSE("%s:Bn%s: meta_vector_size=%d", TAG, __func__,
          meta_vector_size);
      android::Parcel::ReadableBlob meta_blob;
      for (uint32_t i = 0; i < meta_vector_size; i++)  {
        data.readUint32(&size);
        data.readBlob(size, &meta_blob);
        void* buffer = const_cast<void*>(meta_blob.data());
        MetaData meta_data;
        memcpy(&meta_data, buffer, size);
        meta_buffers.push_back(meta_data);
        meta_blob.release();
      }
      NotifyVideoTrackData(track_id, buffers, meta_buffers);
      return NO_ERROR;
    }
    break;
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_VIDEO_TRACK_EVENT: {
      //TODO:
      return NO_ERROR;
    }
    break;
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_AUDIO_TRACK_DATA: {
      uint32_t track_id = data.readUint32();
      size_t num_buffers = static_cast<size_t>(data.readInt32());
      std::vector<BnBuffer> buffers;
      for (size_t index = 0; index < num_buffers; ++index) {
        BnBuffer buffer;
        buffer.FromParcel(data, true);
        buffers.push_back(buffer);
      }
      QMMF_DEBUG("%s:%s-NotifyAudioTrackData() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s:%s-NotifyAudioTrackData() INPARAM: track_id[%u]",
                   TAG, __func__, track_id);
      for (const BnBuffer& buffer : buffers)
        QMMF_VERBOSE("%s:%s-NotifyAudioTrackData() INPARAM: buffer[%s]",
                   TAG, __func__, buffer.ToString().c_str());
      //TODO: Current implementation of audio is not using meta_data, add
      // support to pack meta data at proxy side, and unpack at Bn side.
      std::vector<MetaData> meta_buffers;
      NotifyAudioTrackData(track_id, buffers, meta_buffers);
      return NO_ERROR;
    }
    break;
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_AUDIO_TRACK_EVENT: {
      uint32_t track_id = data.readUint32();
      EventType event_type = static_cast<EventType>(data.readInt32());

      QMMF_DEBUG("%s:%s-NotifyAudioTrackEvent() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s:%s-NotifyAudioTrackEvent() INPARAM: track_id[%u]",
                   TAG, __func__, track_id);
      QMMF_VERBOSE("%s:%s-NotifyAudioTrackEvent() INPARAM: event_type[%d]",
                   TAG, __func__,
                   static_cast<underlying_type<EventType>::type>(event_type));
      NotifyAudioTrackEvent(track_id, event_type, nullptr, 0);

      return NO_ERROR;
    }
    break;
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_CAMERA_RESULT: {
      camera_metadata *meta = NULL;
      uint32_t camera_id = data.readUint32();
      auto ret = CameraMetadata::readFromParcel(data, &meta);
      if ((NO_ERROR == ret) && (NULL != meta)) {
        CameraMetadata result(meta);
        NotifyCameraResult(camera_id, result);
      } else {
        QMMF_ERROR("%s:%s Failed to read camera result from parcel: %d\n",
                     TAG, __func__, ret);
        if (NULL != meta) {
          free_camera_metadata(meta);
          meta = NULL;
        }
      }

      return ret;
    }
    break;
    default: {
      QMMF_ERROR("%s:%s Method not supported ", TAG, __func__);
    }
    break;
  }
  return NO_ERROR;
}

}; //namespace qmmf

}; //namespace recorder
