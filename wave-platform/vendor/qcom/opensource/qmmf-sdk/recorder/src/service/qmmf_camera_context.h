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

#include <utils/KeyedVector.h>
#include <utils/Log.h>
#include <libgralloc/gralloc_priv.h>
#include <condition_variable>

#include "qmmf-sdk/qmmf_recorder_params.h"
#include "common/cameraadaptor/qmmf_camera3_device_client.h"
#include "recorder/src/service/qmmf_camera_interface.h"

namespace qmmf {

using namespace cameraadaptor;

#define VIDEO_STREAM_BUFFER_COUNT   11
#define PREVIEW_STREAM_BUFFER_COUNT 10
#define EXTRA_DCVS_BUFFERS          2

//FIXME: This is temporary change until necessary vendor mode changes are merged
// in HAL3.
#define QCAMERA3_VENDOR_SENSOR_MODE 1

namespace recorder {

class CameraPort;
class CameraReprocess;
class IBufferConsumer;
class IBufferProducer;

// This class deals with Camera3DeviceClient, and exposes simple Apis to create
// Different types of streams (preview, video, and snashot). this class has a
// Concept of ports, maintains vector of ports, each port is mapped one-to-one
// to camera device stream.
class CameraContext : public CameraInterface {
 public:
  CameraContext();

  ~CameraContext();

  status_t OpenCamera(const uint32_t camera_id, const CameraStartParam &param,
                      const ResultCb &cb = nullptr) override;

  status_t CloseCamera(const uint32_t camera_id) override;

  status_t CaptureImage(const ImageParam &param, const uint32_t num_images,
                        const std::vector<CameraMetadata> &meta,
                        const StreamSnapshotCb& cb) override;

  status_t CancelCaptureImage() override;

  status_t CreateStream(const CameraStreamParam& param) override;

  status_t DeleteStream(const uint32_t track_id) override;

  status_t StartStream(const uint32_t track_id,
                       sp<IBufferConsumer>& consumer) override;

  status_t StopStream(const uint32_t track_id) override;

  status_t SetCameraParam(const CameraMetadata &meta) override;

  status_t GetCameraParam(CameraMetadata &meta) override;

  status_t GetDefaultCaptureParam(CameraMetadata &meta) override;

  status_t ReturnImageCaptureBuffer(const uint32_t camera_id,
                                    const int32_t buffer_id) override;

  CameraStartParam& GetCameraStartParam() override;

  Vector<int32_t>& GetSupportedFps() override;

 private:

  struct HFRMode_t {
    uint32_t width;
    uint32_t height;
    uint32_t batch_size;
    uint32_t framerate;
  };

  struct SyncFrame {
    int64_t         last_frame_id;
    Vector<int32_t> stream_ids;
  };

  friend class CameraPort;
  friend class CameraReprocess;
  friend class ZslPort;

  void InitSupportedFPS(const CameraMetadata &static_meta);

  bool IsInputSupported(const CameraMetadata &static_meta);

  status_t CreateZSLStream(const CameraStartParam &param);

  status_t CreateSnapshotStream(const ImageParam &param);

  status_t CreateDeviceStream(CameraStreamParameters& params,
                              uint32_t frame_rate, int32_t* stream_id);

  status_t DeleteDeviceStream(int32_t stream_id, bool cache);

  status_t CreateCaptureRequest(Camera3Request& request,
                                camera3_request_template_t template_type);

  status_t CreateDeviceInputStream(CameraInputStreamParameters& params,
                                   int32_t* stream_id);

  int32_t SubmitRequest(Camera3Request request,
                        bool is_streaming,
                        int64_t *lastFrameNumber);

  status_t UpdateRequest(bool is_streaming);

  status_t CancelRequest();

  status_t ReturnStreamBuffer(int32_t stream_id, StreamBuffer buffer);

  status_t ValidateResolution(const ImageFormat format, const uint32_t width,
                              const uint32_t height);

  void InitHFRModes(CameraMetadata &static_meta);

  status_t CaptureZSLImage(const ImageParam &param);

  //Camera client callbacks.
  void SnapshotCaptureCallback(int32_t stream_id, StreamBuffer buffer);

  void ReprocessCaptureCallback(int32_t stream_id, StreamBuffer buffer);

  void CameraErrorCb(CameraErrorCode errorCode, const CaptureResultExtras &);

  void CameraIdleCb();

  void CameraShutterCb(const CaptureResultExtras &, int64_t time_stamp);

  void CameraPreparedCb(int32_t);

  void CameraResultCb(const CaptureResult &result);

  int32_t ImageToHalFormat(ImageFormat image);

  std::function<void(int32_t, StreamBuffer)> GetStreamCb(const ImageParam &param);

  bool IsReprocessNeed(const ImageParam &param);

  CameraPort* GetPort(const uint32_t track_id);

  void DeletePort(const uint32_t track_id);

  sp<Camera3DeviceClient>  camera_device_;
  CameraClientCallbacks    camera_callbacks_;
  uint32_t                 camera_id_;
  Mutex                    device_access_lock_;
  CameraStartParam         camera_start_params_;

  // Global Capture request.
  int32_t                  streaming_request_id_;
  int32_t                  previous_streaming_request_id_;

  //Non zsl capture request.
  Camera3Request           snapshot_request_;
  int32_t                  snapshot_request_id_;
  ImageParam               snapshot_param_;
  StreamSnapshotCb         client_snapshot_cb_;
  uint32_t                 sequence_cnt_;
  uint32_t                 burst_cnt_;
  bool                     reprocess_enable_;
  std::mutex               capture_count_lock_;
  std::condition_variable  capture_count_signal_;
  bool                     cancel_capture_ = false;

  ResultCb                 result_cb_;
  Vector<int32_t>          supported_fps_;
  sp<CameraPort>           zsl_port_;

  // Map of <consumer id and CameraPort>
  Vector<sp<CameraPort> > active_ports_;

  // Maps of buffer Id and Buffer.
  DefaultKeyedVector<uint32_t, StreamBuffer> snapshot_buffer_list_;

  // User define value for sensor mode
  int32_t sensor_vendor_mode_;

  static uint32_t          kConstrainedModeThreshold;
  static uint32_t          kHFRBatchModeThreshold;
  bool                     hfr_supported_;
  Vector<HFRMode_t>        hfr_batch_modes_list_;
  Vector<Camera3Request>   streaming_active_requests_;

  DefaultKeyedVector<uint32_t, int32_t> snapshot_buffer_stream_list_;
  sp<CameraReprocess>      camera_reprocess_;
  SyncFrame                sync_frame_;
  Condition                sync_frame_cond_;
  Mutex                    sync_frame_lock_;
  static const nsecs_t     kSyncFrameWaitDuration;
};

enum class CameraPortType {
  kVideo,
  kPreview,
  kZSL,
};

enum class PortState {
  PORT_CREATED,
  PORT_READYTOSTART,
  PORT_STARTED,
  PORT_READYTOSTOP,
  PORT_STOPPED,
};

struct ZSLEntry {
  StreamBuffer    buffer;
  CameraMetadata  result;
  int64_t         timestamp;
};

// CameraPort is one to one mapped to Camera device stream. It takes buffers
// from camera stream and passes to its consumers, for optimization reason
// single port can serve multiple consumers if their characterstics are exactly
// same.
class CameraPort : public RefBase {
 public:
  CameraPort(const CameraStreamParam& param, size_t batch_size,
             CameraPortType port_type, CameraContext *context);

  virtual ~CameraPort();

  virtual status_t Init();

  status_t DeInit();

  status_t Start(const uint32_t consumer_id,
                 const sp<IBufferConsumer>& consumer);

  status_t Stop(const uint32_t consumer_id);

  // Apis to Add/Remove consumer at run time.
  status_t AddConsumer(const uint32_t consumer_id,
                       const sp<IBufferConsumer>& consumer);

  status_t RemoveConsumer(const uint32_t consumer_id);

  void NotifyBufferReturned(const StreamBuffer& buffer);

  int32_t GetNumConsumers();

  bool IsReadyToStart();

  PortState& getPortState();

  uint32_t GetPortFramerate() { return params_.frame_rate; }

  size_t GetPortBatchSize() { return batch_size_; }

  int32_t GetCameraStreamId() { return camera_stream_id_; }

  uint32_t GetConsumerId() { return consumer_id_; }

  CameraPortType GetPortType() { return port_type_; }

 protected:
  CameraPortType         port_type_;
  CameraContext*         context_;
  int32_t                camera_stream_id_;
  PortState              port_state_;

 private:

  bool IsConsumerIdValid(const uint32_t id);

  void StreamCallback(int32_t stream_id, StreamBuffer Buffer);

  sp<IBufferProducer>    buffer_producer_impl_;
  CameraStreamParam      params_;
  CameraStreamParameters cam_stream_params_;
  Mutex                  consumer_lock_;
  bool                   ready_to_start_;
  size_t                 batch_size_;
  uint32_t               consumer_id_;

  // map of <consumer id, IBufferConsumer>
  DefaultKeyedVector<uint32_t , sp<IBufferConsumer> > consumer_map_;
};

class ZslPort : public CameraPort {

 public:
  ZslPort(const CameraStreamParam& param, size_t batch_size,
          CameraPortType port_type, CameraContext *context);

  ~ZslPort();

  status_t Init() override;

  status_t PauseAndFlushZSLQueue();

  void ResumeZSL();

  bool IsRunning();

  status_t PickZSLBuffer();

  ZSLEntry& GetInputBuffer() { return zsl_input_buffer_; }

  void HandleZSLCaptureResult(const CaptureResult &result);

  int32_t GetInputStreamId() { return input_stream_id_; }

 private:

  status_t SetUpZSL();

  void ZSLCaptureCallback(int32_t stream_id, StreamBuffer buffer);

  void GetZSLInputBuffer(StreamBuffer &buffer);

  void ReturnZSLInputBuffer(StreamBuffer &buffer);

  int32_t         input_stream_id_ = -1;
  Mutex           zsl_queue_lock_;
  List<ZSLEntry>  zsl_queue_;
  ZSLEntry        zsl_input_buffer_ = {};
  bool            zsl_running_ = false;
  uint32_t        zsl_queue_depth_ = 0;
};

struct ReprocParam {
  uint32_t width;
  uint32_t height;
  int32_t format;
};

typedef std::function
    <void(int32_t stream_id, StreamBuffer buffer)>  ReprocessCb;

class CameraReprocess : public RefBase {
 public:
  CameraReprocess(CameraContext *context);

  ~CameraReprocess();

  status_t Create(const int32_t stream_id,
                  const ReprocParam& input,
                  const ReprocParam& output,
                  const uint32_t frame_rate,
                  const uint32_t num_images,
                  const CameraMetadata& static_meta,
                  const ReprocessCb& cb);
  status_t Delete();

  void AddBuff(int32_t stream_id, StreamBuffer buffer);

  void AddResult(const CaptureResult &result);

 private:

  struct BurstData {
    StreamBuffer   buffer;
    CameraMetadata result;
    int64_t        timestamp;
  };

  status_t Start();

  void ReturnAllInputBuffers();

  void ReturnInputBuffer(StreamBuffer &buffer);

  void GetInputBuffer(StreamBuffer &buffer);

  void StreamCallback(int32_t stream_id, StreamBuffer Buffer);

  void ReprocessCallback(int32_t stream_id, StreamBuffer buffer);

  status_t ValidateInput(const CameraMetadata& meta,
                         const ReprocParam& input,
                         const ReprocParam& output);

  CameraContext*         context_;

  List<StreamBuffer>     input_buffer_;
  List<StreamBuffer>     input_buffer_done_;

  int32_t                input_stream_id_;
  Camera3Request         reprocess_request_;

  Mutex                  reprocess_lock_;
  bool                   reprocess_flag_;
  bool                   ready_to_start_;

  uint32_t               num_images_;
  ReprocessCb            capture_client_cb_;

  List<BurstData>        burst_queue_;
  List<BurstData>        input_burst_queue_;
  Mutex                  burst_queue_lock_;
  uint32_t               burst_cnt_;
};

}; //namespace recorder

}; //namespace qmmf
