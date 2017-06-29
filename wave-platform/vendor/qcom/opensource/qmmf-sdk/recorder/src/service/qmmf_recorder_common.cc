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

#define TAG "RecorderCommon"

#include "recorder/src/service/qmmf_recorder_common.h"

namespace qmmf {

namespace recorder {

extern "C" void DebugCameraStartParams (const char* _func_,
                                        CameraStartParam* params)
{
  QMMF_INFO("%s: zsl_mode = %d", _func_, params->zsl_mode);
  QMMF_INFO("%s: zsl_queue_depth = %d", _func_, params->zsl_queue_depth);
  QMMF_INFO("%s: zsl_width = %d", _func_, params->zsl_width);
  QMMF_INFO("%s: zsl_height = %d", _func_, params->zsl_height);
  QMMF_INFO("%s: frame_rate = %d", _func_, params->frame_rate);
  QMMF_INFO("%s: flags = %d", _func_, params->flags);
}

extern "C" void DebugVideoTrackCreateParam (const char* _func_,
                                            VideoTrackCreateParam* params)
{
  QMMF_INFO("%s: num_cameras = %d", _func_, params->num_cameras);
  for(uint8_t i = 0; i < params->num_cameras; i++) {
    QMMF_INFO("%s: camera_id[%d]", _func_, params->camera_ids[i]);
  }
  QMMF_INFO("%s: width = %d", _func_, params->width);
  QMMF_INFO("%s: height = %d", _func_, params->height);
  QMMF_INFO("%s: frame_rate = %d", _func_, params->frame_rate);
  QMMF_INFO("%s: format_type = %d", _func_, params->format_type);
  QMMF_INFO("%s: out_device = %d", _func_, params->out_device);
}

extern "C" void DebugVideoTrackParams (const char* _func_,
                                       VideoTrackParams* params)
{
  QMMF_INFO("%s: track_id = %d", _func_, params->track_id);
  QMMF_INFO("%s: num_cameras = %d", _func_, params->camera_ids.size());
  for(uint32_t i; i < params->camera_ids.size(); i++) {
    QMMF_INFO("%s: camera_id[%d]", _func_, params->camera_ids[i]);
  }
  QMMF_INFO("%s: width = %d", _func_, params->width);
  QMMF_INFO("%s: height = %d", _func_, params->height);
  QMMF_INFO("%s: frame_rate = %d", _func_, params->frame_rate);
  QMMF_INFO("%s: format_type = %d", _func_, params->format_type);
  QMMF_INFO("%s: camera_stream_type = %d", _func_, params->camera_stream_type);
}

}; //namespace recorder.

}; //namespace qmmf.