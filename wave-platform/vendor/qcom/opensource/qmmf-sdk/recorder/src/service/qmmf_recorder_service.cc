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

#define TAG "RecorderService"

#include "recorder/src/client/qmmf_recorder_params_internal.h"
#include "recorder/src/service/qmmf_recorder_service.h"

namespace qmmf {

namespace recorder {

RecorderService::RecorderService()
    : recorder_(nullptr), connected_(false) {

  QMMF_INFO("%s:%s: RecorderService Instantiated! ", TAG, __func__);
}

RecorderService::~RecorderService() {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  QMMF_INFO("%s:%s: Exit ", TAG, __func__);
}

status_t RecorderService::onTransact(uint32_t code, const Parcel& data,
                                     Parcel* reply, uint32_t flag) {

  QMMF_DEBUG("%s:%s: Enter:(BnRecorderService::onTransact)", TAG, __func__);
  CHECK_INTERFACE(IRecorderService, data, reply);
  int32_t ret = 0;

  switch (code) {
    case RECORDER_CONNECT: {
      sp<IRecorderServiceCallback> client_cb_handle = interface_cast
          <IRecorderServiceCallback>(data.readStrongBinder());
      ret = Connect(client_cb_handle);
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;
      case RECORDER_DISCONNECT: {
        ret = Disconnect();
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_START_CAMERA: {
        uint32_t camera_id, enable_flag;
        bool enable_result_cb;
        data.readUint32(&camera_id);
        data.readUint32(&enable_flag);
        enable_result_cb = (1 == enable_flag) ? true : false;
        uint32_t blob_size;
        data.readUint32(&blob_size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(blob_size, &blob);
        void* params = const_cast<void*>(blob.data());
        CameraStartParam camera_start_params;
        memset(&camera_start_params, 0x0, sizeof camera_start_params);
        memcpy(&camera_start_params, params, blob_size);
        ret = StartCamera(camera_id, camera_start_params, enable_result_cb);
        blob.release();
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_STOP_CAMERA: {
        uint32_t camera_id;
        data.readUint32(&camera_id);
        ret = StopCamera(camera_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_CREATE_SESSION: {
        uint32_t session_id;
        ret = CreateSession(&session_id);
        reply->writeUint32(session_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_DELETE_SESSION: {
        uint32_t session_id;
        data.readUint32(&session_id);
        ret = DeleteSession(session_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_START_SESSION: {
        uint32_t session_id;
        data.readUint32(&session_id);
        ret = StartSession(session_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_STOP_SESSION: {
        uint32_t session_id;
        int32_t flush;
        data.readUint32(&session_id);
        data.readInt32(&flush);
        ret = StopSession(session_id, flush);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_PAUSE_SESSION: {
        uint32_t session_id;
        data.readUint32(&session_id);
        ret = PauseSession(session_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_RESUME_SESSION: {
        uint32_t session_id;
        data.readUint32(&session_id);
        ret = ResumeSession(session_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_CREATE_AUDIOTRACK: {
        uint32_t session_id = data.readUint32();
        uint32_t track_id = data.readUint32();
        AudioTrackCreateParamInternal params;
        params.FromParcel(data);
        ret = CreateAudioTrack(session_id, track_id, params);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_CREATE_VIDEOTRACK: {
        uint32_t session_id, track_id;
        uint32_t blob_size;
        data.readUint32(&session_id);
        data.readUint32(&track_id);
        data.readUint32(&blob_size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(blob_size, &blob);
        void* params = const_cast<void*>(blob.data());
        VideoTrackCreateParam video_track_param;
        memset(&video_track_param, 0x0, sizeof video_track_param);
        memcpy(&video_track_param, params, blob_size);
        ret = CreateVideoTrack(session_id, track_id, video_track_param);
        blob.release();
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_DELETE_AUDIOTRACK: {
        uint32_t session_id = data.readUint32();
        uint32_t track_id = data.readUint32();
        ret = DeleteAudioTrack(session_id, track_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_DELETE_VIDEOTRACK: {
        uint32_t session_id, track_id;
        data.readUint32(&session_id);
        data.readUint32(&track_id);
        ret = DeleteVideoTrack(session_id, track_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_RETURN_TRACKBUFFER: {
        uint32_t session_id = data.readUint32();
        uint32_t track_id = data.readUint32();

        std::vector<BnBuffer> buffers;
        if (track_id < 100) {
          uint32_t vector_size;
          data.readUint32(&vector_size);
          for (uint32_t i = 0; i < vector_size; i++)  {
            uint32_t size;
            data.readUint32(&size);
            android::Parcel::ReadableBlob blob;
            data.readBlob(size, &blob);
            void* buffer = const_cast<void*>(blob.data());
            BnBuffer track_buffer;
            assert(size == sizeof(track_buffer));
            memset(&track_buffer, 0x0, sizeof track_buffer);
            memcpy(&track_buffer, buffer, size);
            buffers.push_back(track_buffer);
            blob.release();
          }
        } else {
          size_t num_buffers = data.readInt32();
          for (size_t index = 0; index < num_buffers; ++index) {
            BnBuffer buffer;
            buffer.FromParcel(data, false);
            buffers.push_back(buffer);
          }
        }
        ret = ReturnTrackBuffer(session_id, track_id, buffers);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_SET_AUDIOTRACK_PARAMS: {
        uint32_t session_id, track_id;
        uint32_t param_type, blob_size;
        data.readUint32(&session_id);
        data.readUint32(&track_id);
        data.readUint32(&param_type);
        data.readUint32(&blob_size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(blob_size, &blob);
        void* param = const_cast<void*>(blob.data());
        ret = SetAudioTrackParam(session_id, track_id,
                               static_cast<CodecParamType>(param_type),
                               param, blob_size);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_SET_VIDEOTRACK_PARAMS: {
        uint32_t session_id, track_id;
        uint32_t param_type, blob_size;
        data.readUint32(&session_id);
        data.readUint32(&track_id);
        data.readUint32(&param_type);
        data.readUint32(&blob_size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(blob_size, &blob);
        void* param = const_cast<void*>(blob.data());
        ret = SetVideoTrackParam(session_id, track_id,
                               static_cast<CodecParamType>(param_type),
                               param, blob_size);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_CAPTURE_IMAGE: {
        uint32_t camera_id, blob_size;
        data.readUint32(&camera_id);
        data.readUint32(&blob_size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(blob_size, &blob);
        void* params = const_cast<void*>(blob.data());
        ImageParam image_params;
        memset(&image_params, 0x0, sizeof image_params);
        memcpy(&image_params, params, blob_size);
        uint32_t num_images;
        data.readUint32(&num_images);
        uint32_t meta_size;
        data.readUint32(&meta_size);
        std::vector<CameraMetadata> meta_array;
        for (uint32_t i = 0; i < meta_size; ++i) {
          CameraMetadata meta;
          camera_metadata_t *m = nullptr;
          ret = meta.readFromParcel(data, &m);
          if ((NO_ERROR != ret) || (nullptr == m)) {
            QMMF_ERROR("%s: Metadata parcel read failed: %d meta(%p)",
                __func__, ret, m);
            reply->writeInt32(ret);
            return ret;
          }
          meta.clear();
          meta.append(m);
          meta_array.push_back(meta);
        }
        ret = CaptureImage(camera_id, image_params, num_images, meta_array);
        blob.release();
        reply->writeInt32(ret);
        return ret;
      }
      break;
      case RECORDER_CONFIG_IMAGECAPTURE: {
        //NOT IMPLEMENTED.
      }
      break;
      case RECORDER_CANCEL_IMAGECAPTURE: {
        uint32_t camera_id;
        data.readUint32(&camera_id);
        ret = CancelCaptureImage(camera_id);
        reply->writeInt32(ret);
        return ret;
      }
      break;
      case  RECORDER_RETURN_IMAGECAPTURE_BUFFER: {
        uint32_t camera_id, buffer_id;
        data.readUint32(&camera_id);
        data.readUint32(&buffer_id);
        ret = ReturnImageCaptureBuffer(camera_id, buffer_id);
        return ret;
      }
      break;
      case RECORDER_SET_CAMERA_PARAMS: {
        uint32_t camera_id;
        CameraMetadata meta;
        camera_metadata_t *m = nullptr;
        data.readUint32(&camera_id);
        ret = meta.readFromParcel(data, &m);
        if ((NO_ERROR != ret) || (nullptr == m)) {
          QMMF_ERROR("%s: Metadata parcel read failed: %d meta: %p\n",
              __func__, ret, m);
          reply->writeInt32(ret);
          return ret;
        }
        meta.clear();
        meta.append(m);
        ret = SetCameraParam(camera_id, meta);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_GET_CAMERA_PARAMS: {
        uint32_t camera_id;
        data.readUint32(&camera_id);
        CameraMetadata meta;
        ret = GetCameraParam(camera_id, meta);
        reply->writeInt32(ret);
        if (NO_ERROR == ret) {
          ret = meta.writeToParcel(reply);
          if (NO_ERROR != ret) {
            QMMF_ERROR("%s: Metadata parcel write failed: %d\n",
                       __func__, ret);
          }
        }
        return ret;
      }
      break;
      case RECORDER_GET_DEFAULT_CAPTURE_PARAMS: {
        uint32_t camera_id;
        data.readUint32(&camera_id);
        CameraMetadata meta;
        ret = GetDefaultCaptureParam(camera_id, meta);
        reply->writeInt32(ret);
        if (NO_ERROR == ret) {
          ret = meta.writeToParcel(reply);
          if (NO_ERROR != ret) {
            QMMF_ERROR("%s: Metadata parcel write failed: %d\n",
                       __func__, ret);
          }
        }
        return ret;
      }
      break;
      case RECORDER_CREATE_OVERLAYOBJECT: {
        uint32_t blob_size, track_id;
        data.readUint32(&track_id);
        data.readUint32(&blob_size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(blob_size, &blob);
        void* params = const_cast<void*>(blob.data());
        uint32_t overlay_id;
        ret = CreateOverlayObject(track_id, static_cast<OverlayParam*>(params),
                                  &overlay_id);
        blob.release();
        reply->writeUint32(overlay_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_DELETE_OVERLAYOBJECT: {
        uint32_t overlay_id, track_id;
        data.readUint32(&track_id);
        data.readUint32(&overlay_id);
        ret = DeleteOverlayObject(track_id, overlay_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_GET_OVERLAYOBJECT_PARAMS: {
        uint32_t overlay_id, track_id;
        data.readUint32(&track_id);
        data.readUint32(&overlay_id);
        OverlayParam overlay_param;
        memset(&overlay_param, 0x0, sizeof overlay_param);
        ret = GetOverlayObjectParams(track_id, overlay_id, overlay_param);
        reply->writeInt32(ret);
        if (NO_ERROR == ret) {
          uint32_t param_size = sizeof overlay_param;
          reply->writeUint32(param_size);
          android::Parcel::WritableBlob blob;
          reply->writeBlob(param_size, false, &blob);
          memset(blob.data(), 0x0, param_size);
          memcpy(blob.data(), reinterpret_cast<void*>(&overlay_param),
              sizeof overlay_param);
        }
        return NO_ERROR;
      }
      break;
      case RECORDER_UPDATE_OVERLAYOBJECT_PARAMS: {
        uint32_t blob_size, overlay_id, track_id;
        data.readUint32(&track_id);
        data.readUint32(&overlay_id);
        data.readUint32(&blob_size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(blob_size, &blob);
        void* params = const_cast<void*>(blob.data());
        ret = UpdateOverlayObjectParams(track_id, overlay_id,
                                        static_cast<OverlayParam*>(params));
        blob.release();
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_SET_OVERLAYOBJECT: {
        uint32_t track_id, overlay_id;
        data.readUint32(&track_id);
        data.readUint32(&overlay_id);
        ret = SetOverlayObject(track_id, overlay_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_REMOVE_OVERLAYOBJECT: {
        uint32_t track_id, overlay_id;
        data.readUint32(&track_id);
        data.readUint32(&overlay_id);
        ret = RemoveOverlayObject(track_id, overlay_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_CREATE_MULTICAMERA: {
        uint32_t vector_size = data.readUint32();
        std::vector<uint32_t> camera_ids;
        for (uint32_t i = 0; i < vector_size; ++i) {
          camera_ids.push_back(data.readUint32());
        }
        uint32_t virtual_camera_id;
        ret = CreateMultiCamera(camera_ids, &virtual_camera_id);
        QMMF_INFO("%s:%s: virtual_camera_id=%d", TAG, __func__,
            virtual_camera_id);
        reply->writeUint32(virtual_camera_id);
        reply->writeInt32(ret);
      }
      break;
      case RECORDER_CONFIGURE_MULTICAMERA: {
        uint32_t virtual_camera_id = data.readUint32();
        uint32_t config_type = data.readUint32();
        uint32_t param_size = data.readUint32();
        android::Parcel::ReadableBlob blob;
        data.readBlob(param_size, &blob);
        void* param = const_cast<void*>(blob.data());
        ret = ConfigureMultiCamera(virtual_camera_id, config_type, param,
                                   param_size);
        blob.release();
        reply->writeUint32(ret);
        return NO_ERROR;
      }
      break;
      default: {
        QMMF_ERROR("RecorderService:%s:Method is not supported !",__func__);
        reply->writeInt32(-1);
      }
      break;
  }
  return NO_ERROR;
}

status_t RecorderService::Connect(const sp<IRecorderServiceCallback>&
                                  service_cb) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);

  recorder_ = RecorderImpl::CreateRecorder();
  if (!recorder_) {
      QMMF_ERROR("%s:%s: Can't create Recorder Instance!!", TAG, __func__);
      return NO_MEMORY;
  }
  remote_callback_ = new RemoteCallBack(service_cb);

  auto ret = recorder_->Connect(remote_callback_);
  assert(ret == NO_ERROR);

  death_notifier_ = new DeathNotifier(this);
  if (nullptr == death_notifier_.get()) {
      QMMF_ERROR("%s:%s: Unable to allocate death notifier!", TAG, __func__);
      return NO_MEMORY;
  }

  IInterface::asBinder(remote_callback_->getRemoteClient())
      ->linkToDeath(death_notifier_);

  connected_ = true;
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  return ret;
}

status_t RecorderService::Disconnect() {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);

  int32_t ret = NO_ERROR;
  if (!connected_)
      return NO_INIT;

  IInterface::asBinder(remote_callback_->getRemoteClient())
      ->unlinkToDeath(death_notifier_);

  if (death_notifier_.get() != nullptr) {
    death_notifier_.clear();
    death_notifier_ = nullptr;
  }

  if (recorder_ != nullptr) {
    ret = recorder_->Disconnect();
    delete recorder_;
    recorder_ = nullptr;
  }

  if (remote_callback_.get() != nullptr) {
    remote_callback_.clear();
    remote_callback_ = nullptr;
  }
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  return ret;
}

status_t RecorderService::StartCamera(const uint32_t camera_id,
                                      const CameraStartParam &params,
                                      bool enable_result_cb) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);

  if(!connected_) {
    QMMF_WARN("%s:%s: Connect Should be called, before calling StartCamera!!",
               TAG, __func__);
    return NO_INIT;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->StartCamera(camera_id, params, enable_result_cb);
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: Can't start Camera!!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  return NO_ERROR;
}

status_t RecorderService::StopCamera(const uint32_t camera_id) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);

  if(!connected_) {
    return NO_INIT;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->StopCamera(camera_id);
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: Can't Stop Camera!!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  return NO_ERROR;
}

status_t RecorderService::CreateSession(uint32_t *session_id) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(recorder_ != nullptr);
  uint32_t id;
  auto ret = recorder_->CreateSession(&id);
  assert(ret == NO_ERROR);
  *session_id = id;

  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::DeleteSession(const uint32_t session_id) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(recorder_ != nullptr);
  auto ret = recorder_->DeleteSession(session_id);
  assert(ret == NO_ERROR);
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::StartSession(const uint32_t session_id) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;
  QMMF_INFO("%s:%s: Session_id(%d) to be Start", TAG, __func__, session_id);

  assert(recorder_ != nullptr);
  auto ret = recorder_->StartSession(session_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: StartSession failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::StopSession(const uint32_t session_id,
                                      bool do_flush) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;
  QMMF_INFO("%s:%s: Session_id(%d) to be Stop with flash=%d", TAG, __func__,
                                      session_id, do_flush);

  assert(recorder_ != nullptr);
  auto ret = recorder_->StopSession(session_id, do_flush);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: StopSession failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::PauseSession(const uint32_t session_id) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;
  QMMF_INFO("%s:%s: Session_id(%d) to be Pause", TAG, __func__, session_id);

  assert(recorder_ != nullptr);
  auto ret = recorder_->PauseSession(session_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: PauseSession failed!", TAG, __func__);
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::ResumeSession(const uint32_t session_id) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;
  QMMF_INFO("%s:%s: Session_id(%d) to be Resume", TAG, __func__, session_id);

  assert(recorder_ != nullptr);
  auto ret = recorder_->ResumeSession(session_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: ResumeSession failed!", TAG, __func__);
  }
   QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::CreateAudioTrack(const uint32_t session_id,
                                           const uint32_t track_id,
                                           const AudioTrackCreateParam& param) {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  QMMF_VERBOSE("%s:%s INPARAM: session_id[%u]", TAG, __func__, session_id);
  QMMF_VERBOSE("%s:%s INPARAM: track_id[%u]", TAG, __func__, track_id);
  QMMF_VERBOSE("%s:%s INPARAM: param[%s]", TAG, __func__,
               param.ToString().c_str());
  assert(recorder_ != nullptr);

  auto ret = recorder_->CreateAudioTrack(session_id, track_id, param);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: CreateAudioTrack failed: %d", TAG, __func__, ret);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return NO_ERROR;
}

status_t RecorderService::CreateVideoTrack(const uint32_t session_id,
                                           const uint32_t track_id,
                                           const VideoTrackCreateParam& param) {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);

  assert(recorder_ != nullptr);
  auto ret = recorder_->CreateVideoTrack(session_id, track_id, param);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: CreateVideoTrack failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::DeleteAudioTrack(const uint32_t session_id,
                                           const uint32_t track_id) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  QMMF_VERBOSE("%s:%s INPARAM: session_id[%u]", TAG, __func__, session_id);
  QMMF_VERBOSE("%s:%s INPARAM: track_id[%u]", TAG, __func__, track_id);
  assert(recorder_ != nullptr);

  auto ret = recorder_->DeleteAudioTrack(session_id, track_id);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: DeleteAudioTrack failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return NO_ERROR;
}

status_t RecorderService::DeleteVideoTrack(const uint32_t session_id,
                                           const uint32_t track_id) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  assert(recorder_ != nullptr);
  auto ret = recorder_->DeleteVideoTrack(session_id, track_id);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: DeleteVideoTrack failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::ReturnTrackBuffer(const uint32_t session_id,
                                            const uint32_t track_id,
                                            std::vector<BnBuffer> &buffers) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  QMMF_VERBOSE("%s:%s: session_id(%u):track_id(%u)", TAG, __func__,
      session_id, track_id);

  assert(recorder_ != nullptr);
  auto ret = recorder_->ReturnTrackBuffer(session_id, track_id, buffers);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: ReturnTrackBuffer failed!", TAG, __func__);
    return BAD_VALUE;
  }

  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::SetAudioTrackParam(const uint32_t session_id,
                                             const uint32_t track_id,
                                             CodecParamType type,
                                             void *param,
                                             size_t param_size) {
  // NOT IMPLEMENTED YET.
  return NO_ERROR;
}

status_t RecorderService::SetVideoTrackParam(const uint32_t session_id,
                                             const uint32_t track_id,
                                             CodecParamType type,
                                             void *param,
                                             size_t param_size) {


  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  assert(recorder_ != nullptr);
  auto ret = recorder_->SetVideoTrackParam(session_id, track_id, type, param,
                                           param_size);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CaptureImage failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::CaptureImage(const uint32_t camera_id,
                                       const ImageParam &param,
                                       const uint32_t num_images,
                                       const std::vector<CameraMetadata>
                                          &meta) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  assert(recorder_ != nullptr);
  auto ret = recorder_->CaptureImage(camera_id, param, num_images, meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CaptureImage failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::ConfigImageCapture(const uint32_t camera_id,
                                             const ImageCaptureConfig &config) {
  // NOT IMPLEMENTED YET.
  return NO_ERROR;
}

status_t RecorderService::CancelCaptureImage(const uint32_t camera_id) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  assert(recorder_ != NULL);
  auto ret = recorder_->CancelCaptureImage(camera_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CancelCaptureImage failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return NO_ERROR;
}


status_t RecorderService::ReturnImageCaptureBuffer(const uint32_t camera_id,
                                                   const int32_t buffer_id) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  assert(recorder_ != nullptr);
  auto ret = recorder_->ReturnImageCaptureBuffer(camera_id, buffer_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: ReturnImageCaptureBuffer failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::SetCameraParam(const uint32_t camera_id,
                                         const CameraMetadata &meta) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  assert(recorder_ != nullptr);
  auto ret = recorder_->SetCameraParam(camera_id, meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: SetCameraParam failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::GetCameraParam(const uint32_t camera_id,
                                         CameraMetadata &meta) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  assert(recorder_ != nullptr);
  auto ret = recorder_->GetCameraParam(camera_id, meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: GetCameraParam failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::GetDefaultCaptureParam(const uint32_t camera_id,
                                                 CameraMetadata &meta) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  assert(recorder_ != nullptr);
  auto ret = recorder_->GetDefaultCaptureParam(camera_id, meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: GetDefaultCaptureParam failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::CreateOverlayObject(const uint32_t track_id,
                                              OverlayParam *param,
                                              uint32_t *overlay_id) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  assert(recorder_ != nullptr);
  auto ret = recorder_->CreateOverlayObject(track_id, param, overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CreateOverlayObject failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::DeleteOverlayObject(const uint32_t track_id,
                                              const uint32_t overlay_id) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  assert(recorder_ != nullptr);
  auto ret = recorder_->DeleteOverlayObject(track_id, overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: DeleteOverlayObject failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::GetOverlayObjectParams(const uint32_t track_id,
                                                 const uint32_t overlay_id,
                                                 OverlayParam &param) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  assert(recorder_ != nullptr);
  auto ret = recorder_->GetOverlayObjectParams(track_id, overlay_id, param);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: GetOverlayObjectParams failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::UpdateOverlayObjectParams(const uint32_t track_id,
                                                    const uint32_t overlay_id,
                                                    OverlayParam *param) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  assert(recorder_ != nullptr);
  auto ret = recorder_->UpdateOverlayObjectParams(track_id, overlay_id, param);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: UpdateOverlayObjectParams failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::SetOverlayObject(const uint32_t track_id,
                                           const uint32_t overlay_id) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  assert(recorder_ != nullptr);
  auto ret = recorder_->SetOverlayObject(track_id, overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: SetOverlayObject failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::RemoveOverlayObject(const uint32_t track_id,
                                              const uint32_t overlay_id) {

  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  assert(recorder_ != nullptr);
  auto ret = recorder_->RemoveOverlayObject(track_id, overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: RemoveOverlayObject failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::CreateMultiCamera(const std::vector<uint32_t>
                                            camera_ids,
                                            uint32_t *virtual_camera_id) {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  assert(recorder_ != nullptr);
  auto ret = recorder_->CreateMultiCamera(camera_ids, virtual_camera_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CreateMultiCamera failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t RecorderService::ConfigureMultiCamera(const uint32_t virtual_camera_id,
                                               const uint32_t type,
                                               const void *param,
                                               const uint32_t param_size) {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  assert(recorder_ != nullptr);
  auto ret = recorder_->ConfigureMultiCamera(virtual_camera_id, type, param,
                                             param_size);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: ConfigureMultiCamera failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

}; //namespace recorder

}; //namespace qmmf
