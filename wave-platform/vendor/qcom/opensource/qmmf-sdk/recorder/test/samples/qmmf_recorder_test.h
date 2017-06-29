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

#pragma once

#include <map>

#include "recorder/test/samples/qmmf_recorder_test_wav.h"
#include "recorder/test/samples/qmmf_recorder_test_aac.h"
#include "recorder/test/samples/qmmf_recorder_test_amr.h"

#include <qmmf-sdk/qmmf_recorder.h>
#include <qmmf-sdk/qmmf_recorder_params.h>
#include <qmmf-sdk/qmmf_codec.h>
#include <camera/CameraMetadata.h>
#include <qmmf-sdk/qmmf_display.h>
#include <qmmf-sdk/qmmf_display_params.h>

// Enable this define to dump YUV data from YUV track
#define DUMP_YUV_FRAMES

// Enable this define to dump encoded bit stream data.
#define DUMP_BITSTREAM

// Sleep for specified seconds to allow settling after parameter change
#define PARAMETER_SETTLE_INTERVAL(x) sleep(x)

#define FEATURE_NOT_AVAILABLE  "Not available"

using namespace qmmf;
using namespace recorder;
using namespace android;

using ::qmmf::display::DisplayEventType;
using ::qmmf::display::DisplayType;
using ::qmmf::display::Display;
using ::qmmf::display::DisplayCb;
using ::qmmf::display::SurfaceBuffer;
using ::qmmf::display::SurfaceParam;
using ::qmmf::display::SurfaceConfig;
using ::qmmf::display::SurfaceBlending;
using ::qmmf::display::SurfaceFormat;

#define AEC_SETTLE_INTERVAL 2

enum class AfMode {
  kNone,
  kOff,
  kAuto,
  kMacro,
  kContinousVideo,
  kContinuousPicture,
  kEdof
};

enum class SnapshotType {
   kNone,
   kJpeg,
   kRawYuv,
   kRawRdi
};

struct SnapshotInfo {
   SnapshotType   type;
   uint32_t       width;
   uint32_t       height;
   uint32_t       count;
};

enum class TrackType {
  kNone,
  kAudioPCM,
  kAudioAAC,
  kAudioAMR,
  kAudioG711,
  kVideoYUV,
  kVideoRDI,
  kVideoAVC,
  kVideoHEVC,
  kVideoPreview
};

struct TrackInfo {
  uint32_t  width;
  uint32_t  height;
  uint32_t  fps;
  TrackType track_type;
  uint32_t  bitrate;
  uint32_t  session_id;
  uint32_t  track_id;
  uint32_t  camera_id;
  uint32_t  low_power_mode;
  DeviceId  device_id;
};

class TestTrack;
class CmdMenu;

class TestInitParams {
public:
    int32_t                camera_id;
    uint8_t                camera_fps;
    SnapshotInfo           snapshot_info;
    AfMode                 af_mode;
    uint32_t               recordTime;
    uint32_t               numStream;
    bool                   tnr;
    bool                   vhdr;
    bool                   binning_correct;

    TestInitParams() :
            camera_id(-1),
            camera_fps(0),
            snapshot_info {
              SnapshotType::kNone,
              0,
              0,
              0
            },
            af_mode(AfMode::kOff),
            recordTime(0),
            numStream(0),
            tnr(0),
            vhdr(0),
            binning_correct(false) {};
};


class RecorderTest {
 public:
  RecorderTest();

  ~RecorderTest();

  status_t Connect();

  status_t Disconnect();

  status_t StartCamera();

  status_t StopCamera();

  status_t TakeSnapshot();

  status_t TakeSnapshotWithConfig(const SnapshotInfo& snapshot_info);

  status_t Session4KAnd1080pYUVTracks();

  status_t Session4KEncTrack(const TrackType& type);

  status_t Session1080pEncTrack(const TrackType& type);

  status_t Session1080pEnc1080YUV(const TrackType& type);

  status_t Session4KHEVCAnd1080pYUVTracks(const TrackType& type);

  status_t Session4KYUVAnd1080pEncTracks(const TrackType& type);

  status_t SessionTwo1080pEncTracks(const TrackType& type);

  status_t Session720pLPMTrack(const TrackType& type);

  status_t Session1080pEnc1080pLPMTracks(const TrackType& track_type);

  status_t Session1080pYUVTrackWithDisplay();

  status_t Session1080pYUVTrackWithPreview();

  status_t CreateAudioPCMTrack();

  status_t CreateAudio2PCMTrack();

  status_t CreateAudioSCOTrack();

  status_t CreateAudioPCMSCOTrack();

  status_t CreateAudioA2DPTrack();

  status_t CreateAudioPCMA2DPTrack();

  status_t CreateAudioAACTrack();

  status_t CreateAudio2AACTrack();

  status_t CreateAudioPCMAACTrack();

  status_t CreateAudioAMRTrack();

  status_t CreateAudio2AMRTrack();

  status_t CreateAudioPCMAMRTrack();

  status_t CreateAudioG711Track();

  status_t CreateAudio2G711Track();

  status_t CreateAudioPCMG711Track();

  status_t SessionRDITrack();

  status_t StartSession();

  status_t StopSession();

  status_t SetParams();

  status_t PauseSession();

  status_t ResumeSession();

  status_t DeleteSession();

  status_t EnableOverlay();

  status_t DisableOverlay();

  void printInitParameterAndTtrackInfo(const TestInitParams&
                           initParams,const TrackInfo& track_info);

  int32_t ToggleNR();
  int32_t ToggleVHDR();
  int32_t ToggleIR();
  status_t ToggleAFMode(const AfMode& af_mode);
  int32_t ToggleBinningCorrectionMode();
  int32_t ChooseCamera();
  int32_t SetAntibandingMode();
  std::string GetCurrentNRMode();
  std::string GetCurrentVHDRMode();
  std::string GetCurrentIRMode();
  std::string GetCurrentBinningCorrectionMode();
  status_t GetCurrentAFMode(int32_t& mode);

  // Auto Mode
  int32_t RunAutoMode();
  // Config file related.
  int32_t RunFromConfig(int32_t argc, char *argv[]);

  int32_t ParseConfig(char *fileName, TestInitParams* initParams,
                      std::vector<TrackInfo>* infos);

  void SnapshotCb(uint32_t camera_id, uint32_t image_sequence_count,
                  BufferDescriptor buffer, MetaData meta_data);

  void RecorderCallbackHandler(EventType event_type, void *event_data,
                               size_t event_data_size);

  void SessionCallbackHandler(EventType event_type,
                              void *event_data, size_t event_data_size);

  status_t DumpFrameToFile(BufferDescriptor& buffer,
                           CameraBufferMetaData& meta_data, String8& file_name);

  Recorder& GetRecorder() { return recorder_; }

 private:
  Recorder recorder_;

  friend class CmdMenu;

  typedef std::map <uint8_t, std::string> nr_modes_map;
  typedef std::map <uint8_t, std::string>::iterator nr_modes_iter;
  typedef std::map <int32_t, std::string> vhdr_modes_map;
  typedef std::map <int32_t, std::string>::iterator vhdr_modes_iter;
  typedef std::map <int32_t, std::string> ir_modes_map;
  typedef std::map <int32_t, std::string>::iterator ir_modes_iter;
  typedef std::map <int32_t, std::string> bc_modes_map;
  typedef std::map <int32_t, std::string>::iterator bc_modes_iter;
  void InitSupportedNRModes();
  void InitSupportedVHDRModes();
  void InitSupportedIRModes();
  void InitSupportedBinningCorrectionModes();

  int32_t SetBinningCorrectionMode(const bool& mode);

  // <session_id, vector<TestTrack*> >
  std::map <uint32_t , std::vector<TestTrack*> > sessions_;
  typedef std::map <uint32_t, std::vector<TestTrack*> >::iterator session_iter_;

  uint32_t camera_id_;
  bool session_enabled_;
  CameraMetadata static_info_;
  nr_modes_map supported_nr_modes_;
  vhdr_modes_map supported_hdr_modes_;
  ir_modes_map supported_ir_modes_;
  bc_modes_map supported_bc_modes_;
  bool use_display;

};

// Track can be types of Audio or Video, this class is responsible for creating
// tracks, setting required parameters, registering the data/event callback,
// dumping the data for verification purpose etc.
class TestTrack {

 public:
  TestTrack(RecorderTest* recorder_test);

  ~TestTrack();

  TrackType& GetTrackType() { return track_info_.track_type; }

  uint32_t GetTrackId() { return track_info_.track_id; }

  status_t SetUp(TrackInfo& track_info);

  // Set up file to dump track data.
  status_t Prepare();

  // Clean up file.
  status_t CleanUp();

  status_t EnableOverlay();

  status_t DisableOverlay();

  void DisplayCallbackHandler(DisplayEventType event_type, void *event_data,
      size_t event_data_size);

  void DisplayVSyncHandler(int64_t time_stamp);

  status_t StartDisplay(DisplayType display_type);

  status_t StopDisplay(DisplayType display_type);

 private:

  void TrackEventCB(uint32_t track_id, EventType event_type, void *event_data,
                    size_t event_data_size);

  void TrackDataCB(uint32_t track_id, std::vector<BufferDescriptor> buffers,
                   std::vector<MetaData> meta_buffers);

  status_t DumpBitStream(std::vector<BufferDescriptor>& buffers);

  status_t PushFrameToDisplay(BufferDescriptor& buffer,
    CameraBufferMetaData& meta_data);

  int32_t file_fd_;

  TrackInfo track_info_;

  // One track can have multiple overlay objects.
  std::vector<uint32_t> overlay_ids_;

  RecorderTest* recorder_test_;

  RecorderTestWav wav_output_;
  RecorderTestAac aac_output_;
  RecorderTestAmr amr_output_;

  uint32_t num_yuv_frames_;

  Display*   display_;
  uint32_t   surface_id_;
  SurfaceParam surface_param_;
  SurfaceBuffer surface_buffer_;
  bool display_started_;
};

class CmdMenu
{
public:
    enum CommandType {
        CONNECT_CMD                             = '1',
        DISCONNECT_CMD                          = '2',
        START_CAMERA_CMD                        = '3',
        STOP_CAMERA_CMD                         = '4',
        CREATE_YUV_SESSION_CMD                  = '5',
        CREATE_4KENC_AVC_SESSION_CMD            = '6',
        CREATE_4KENC_HEVC_SESSION_CMD           = '7',
        CREATE_1080pENC_AVC_SESSION_CMD         = '8',
        CREATE_1080pENC_HEVC_SESSION_CMD        = '9',
        CREATE_4KYUV_1080pENC_SESSION_CMD       = 'V',
        CREATE_TWO_1080pENC_SESSION_CMD         = 'M',
        CREATE_1080pENC_AVC_1080YUV_SESSION_CMD = 'E',
        CREATE_4KHEVC_AVC_1080YUV_SESSION_CMD   = 'F',
        CREATE_720pLPM_SESSION_CMD              = 'G',
        CREATE_1080pENC_AVC_1080LPM_SESSION_CMD = 'J',
        CREATE_PCM_AUD_SESSION_CMD              = 'a',
        CREATE_2PCM_AUD_SESSION_CMD             = 'b',
        CREATE_SCO_AUD_SESSION_CMD              = 'c',
        CREATE_PCM_SCO_AUD_SESSION_CMD          = 'd',
        CREATE_A2DP_AUD_SESSION_CMD             = 'e',
        CREATE_PCM_A2DP_AUD_SESSION_CMD         = 'f',
        CREATE_AAC_AUD_SESSION_CMD              = 'g',
        CREATE_2AAC_AUD_SESSION_CMD             = 'h',
        CREATE_PCM_AAC_AUD_SESSION_CMD          = 'i',
        CREATE_AMR_AUD_SESSION_CMD              = 'j',
        CREATE_2AMR_AUD_SESSION_CMD             = 'k',
        CREATE_PCM_AMR_AUD_SESSION_CMD          = 'l',
        CREATE_G7ll_AUD_SESSION_CMD             = 'm',
        CREATE_2G7ll_AUD_SESSION_CMD            = 'n',
        CREATE_PCM_G7ll_AUD_SESSION_CMD         = 'o',
        CREATE_RDI_SESSION_CMD                  = 'r',
        CREATE_YUV_SESSION_DISPLAY_CMD          = 'Z',
        CREATE_YUV_SESSION_PREVIEW_CMD          = 'Y',
        START_SESSION_CMD                       = 'A',
        STOP_SESSION_CMD                        = 'B',
        TAKE_SNAPSHOT_CMD                       = 'S',
        SET_PARAM_CMD                           = 'T',
        PAUSE_SESSION_CMD                       = 'P',
        RESUME_SESSION_CMD                      = 'R',
        ENABLE_OVERLAY_CMD                      = 'O',
        DISABLE_OVERLAY_CMD                     = 'L',
        DELETE_SESSION_CMD                      = 'D',
        NOISE_REDUCTION_CMD                     = 'N',
        VIDEO_HDR_CMD                           = 'H',
        IR_MODE_CMD                             = 'I',
        EXIT_CMD                                = 'X',
        CHOOSE_CAMERA_CMD                       = 'C',
        SET_ANTIBANDING_MODE_CMD                = 'W',
        BINNING_CORRECTION_CMD                  = '#',
        NEXT_CMD                                = '\n',
        INVALID_CMD                             = '0'
    };

    struct Command {
        Command( CommandType cmd)
        : cmd(cmd) {}
        Command()
        : cmd(INVALID_CMD) {}
        CommandType cmd;
    };

    CmdMenu(RecorderTest &ctx) :  ctx_(ctx) {};

    ~CmdMenu() {};

    Command GetCommand(bool& is_print_menu);

    void PrintMenu();

    RecorderTest &ctx_;
};
