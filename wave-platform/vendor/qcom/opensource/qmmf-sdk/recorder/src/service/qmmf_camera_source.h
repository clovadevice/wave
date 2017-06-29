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

#pragma once

#include <memory>

#include <camera/CameraMetadata.h>
#include <utils/KeyedVector.h>
#include <utils/Condition.h>

#include "recorder/src/service/qmmf_recorder_common.h"
#include "recorder/src/service/qmmf_camera_interface.h"
#include "recorder/src/service/qmmf_camera_context.h"
#include "common/cameraadaptor/qmmf_camera3_device_client.h"
#include "common/codecadaptor/src/qmmf_avcodec.h"
#include "qmmf-sdk/qmmf_overlay.h"
#include "qmmf-sdk/qmmf_display.h"
#include "qmmf-sdk/qmmf_display_params.h"

using ::qmmf::display::DisplayEventType;
using ::qmmf::display::DisplayType;
using ::qmmf::display::Display;
using ::qmmf::display::DisplayCb;
using ::qmmf::display::SurfaceBuffer;
using ::qmmf::display::SurfaceParam;
using ::qmmf::display::SurfaceConfig;
using ::qmmf::display::SurfaceBlending;
using ::qmmf::display::SurfaceFormat;

namespace qmmf {

using namespace cameraadaptor;
using namespace android;
using namespace overlay;
using namespace avcodec;

namespace recorder {

#define FPS_CHANGE_THRESHOLD  (0.5)

class TrackSource;

class CameraSource {
 public:

  static CameraSource* CreateCameraSource();

  ~CameraSource();

  status_t StartCamera(const uint32_t camera_id, const CameraStartParam &param,
                       const ResultCb &cb = nullptr);

  status_t StopCamera(const uint32_t camera_id);

  status_t CreateMultiCamera(const std::vector<uint32_t> camera_ids,
                             uint32_t *virtual_camera_id);

  status_t ConfigureMultiCamera(const uint32_t virtual_camera_id,
                                const uint32_t type,
                                const void *param,
                                const uint32_t param_size);

  status_t CaptureImage(const uint32_t camera_id, const ImageParam &param,
                        const uint32_t num_images,
                        const std::vector<CameraMetadata> &meta,
                        const SnapshotCb& cb);

  status_t CancelCaptureImage(const uint32_t camera_id);

  status_t ReturnImageCaptureBuffer(const uint32_t camera_id,
                           const int32_t buffer_id);

  status_t CreateTrackSource(const uint32_t track_id,
                             const VideoTrackParams& param);

  status_t DeleteTrackSource(const uint32_t track_id);

  status_t StartTrackSource(const uint32_t track_id);

  status_t StopTrackSource(const uint32_t track_id);

  status_t PauseTrackSource(const uint32_t track_id);

  status_t ResumeTrackSource(const uint32_t track_id);

  status_t ReturnTrackBuffer(const uint32_t track_id,
                             std::vector<BnBuffer> &buffers);

  status_t SetCameraParam(const uint32_t camera_id, const CameraMetadata &meta);

  status_t GetCameraParam(const uint32_t camera_id, CameraMetadata &meta);

  status_t GetDefaultCaptureParam(const uint32_t camera_id,
                                  CameraMetadata &meta);

  status_t UpdateTrackFrameRate(const uint32_t track_id,
                                const uint32_t frame_rate);

  status_t CreateOverlayObject(const uint32_t track_id,
                               OverlayParam *param,
                               uint32_t *overlay_id);

  status_t DeleteOverlayObject(const uint32_t track_id,
                               const uint32_t overlay_id);

  status_t GetOverlayObjectParams(const uint32_t track_id,
                                  const uint32_t overlay_id,
                                  OverlayParam &param);

  status_t UpdateOverlayObjectParams(const uint32_t track_id,
                                     const uint32_t overlay_id,
                                     OverlayParam *param);

  status_t SetOverlayObject(const uint32_t track_id,
                            const uint32_t overlay_id);

  status_t RemoveOverlayObject(const uint32_t track_id,
                               const uint32_t overlay_id);

  const ::std::shared_ptr<TrackSource>& GetTrackSource(uint32_t track_id);

 private:

  bool IsTrackIdValid(const uint32_t track_id);
  void SnapshotCallback(uint32_t count, StreamBuffer& buffer);
  uint32_t GetJpegSize(uint8_t *blobBuffer, uint32_t width);

  // Map of camera id and CameraContext.
  DefaultKeyedVector<uint32_t, sp<CameraInterface>> camera_map_;

  // Map of track it and TrackSources.
  DefaultKeyedVector<uint32_t, ::std::shared_ptr<TrackSource>> track_sources_;

  SnapshotCb client_snapshot_cb_;

  // Not allowed
  CameraSource();
  CameraSource(const CameraSource&);
  CameraSource& operator=(const CameraSource&);
  static CameraSource* instance_;
};

// This class is behaves as producer and consumer both, at one end it takes
// YUV buffers from camera stream and another end it provides buffers to
// Encoder, and manages buffer circulation, skip etc.
class TrackSource : public ICodecSource {
 public:
  TrackSource(const VideoTrackParams& params,
              const sp<CameraInterface>& camera_intf);

  ~TrackSource();

  status_t Init();

  status_t DeInit();

  status_t StartTrack();

  status_t StopTrack();

  // Methods of IInputCodecSource
  // This method to provide input buffer to Encoder.
  status_t GetBuffer(BufferDescriptor& buffer, void* client_data) override;

  // This method is used by Encoder to provide buffer back after encoding.
  status_t ReturnBuffer(BufferDescriptor& buffer, void* client_data) override;

  // This method is used by Encoder to notify stop.
  status_t NotifyPortEvent(PortEventType event_type,
                           void* event_data) override;

  // Global track specific params can be query from TrackSource during its life
  // cycle.
  VideoTrackParams& getParams() { return track_params_; }

  // This method to handle incoming buffers from producer, producer can be
  // anyone, Camera context's port or rescaler.
  void OnFrameAvailable(StreamBuffer& buffer);

  status_t ReturnTrackBuffer(std::vector<BnBuffer>& buffers);

  bool IsStop();

  void ClearInputQueue();

  // Overlay Apis. TrackSource has instance of Overlay to deal with static
  // and dynamic types of overlay.
  status_t CreateOverlayObject(OverlayParam *param, uint32_t *overlay_id);

  status_t DeleteOverlayObject(const uint32_t overlay_id);

  status_t GetOverlayObjectParams(const uint32_t overlay_id,
                                  OverlayParam &param);

  status_t UpdateOverlayObjectParams(const uint32_t overlay_id,
                                     OverlayParam *param);

  status_t SetOverlayObject(const uint32_t overlay_id);

  status_t RemoveOverlayObject(const uint32_t overlay_id);

  void UpdateFrameRate(const uint32_t frame_rate);

  void DisplayCallbackHandler(display::DisplayEventType event_type,
      void *event_data, size_t event_data_size);

  void DisplayVSyncHandler(int64_t time_stamp);

 private:

  // Method to provide consumer interface, it would be used by producer to
  // post buffers.
  sp<IBufferConsumer>& GetConsumerIntf() { return buffer_consumer_impl_; }

  status_t CreateDisplayPreview(display::DisplayType display_type,
      const VideoTrackParams& track_param);

  status_t DeleteDisplayPreview(display::DisplayType display_type);

  void PushFrameToQueue(StreamBuffer& buffer);

  uint32_t TrackId() { return track_params_.track_id; }

  bool IsFrameSkip();

#ifdef ENABLE_FRAME_DUMP
  status_t DumpYUV(StreamBuffer& buffer);
#endif
  status_t PushFrameToDisplay(StreamBuffer& buffer);

  void ReturnBufferToProducer(StreamBuffer& buffer);

  VideoTrackParams    track_params_;
  sp<IBufferConsumer> buffer_consumer_impl_;
  Condition           wait_for_frame_;
  Mutex               lock_;
  bool                is_stop_;
  Mutex               stop_lock_;
  bool                eos_acked_;
  Mutex               eos_lock_;

  // will be used till we make stop api as async.
  Condition           wait_for_idle_;
  Mutex               idle_lock_;

  // Maps of Unique buffer Id and Buffer.
  DefaultKeyedVector<uint32_t, StreamBuffer> buffer_list_;

  Mutex buffer_list_lock_;

  // Input buffer list, to feed buffers to encoder.
  TSQueue<StreamBuffer> frames_received_;

  // List of buffers held by encoder.
  TSQueue<StreamBuffer> frames_being_encoded_;

  sp<CameraInterface>   camera_interface_;

  Overlay  overlay_;
  bool     enable_overlay_;

  float   input_frame_rate_;
  double  input_frame_interval_;
  double  output_frame_interval_;
  double  remaining_frame_skip_time_;
  Mutex   frame_skip_lock_;

  Display*   display_;
  uint32_t   surface_id_;
  SurfaceParam surface_param_;
  SurfaceBuffer surface_buffer_;
  bool display_started_;

#ifdef DEBUG_TRACK_FPS
  struct timeval prevtv_;
  uint32_t count_;
#endif
};

}; //namespace recorder

}; //namespace qmmf
