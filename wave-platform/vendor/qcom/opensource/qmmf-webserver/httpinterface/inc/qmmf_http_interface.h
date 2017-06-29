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
#ifndef QMMF_HTTP_INTERFACE_H_
#define QMMF_HTTP_INTERFACE_H_

#include <sys/types.h>

typedef struct qmmf_camera_start_param_t {
    uint32_t zsl_mode;
    uint32_t zsl_queue_depth;
    uint32_t zsl_width;
    uint32_t zsl_height;
    uint32_t frame_rate;
    uint32_t flags;
} qmmf_camera_start_param;

typedef enum qmmf_video_track_codec_type_t {
  CODEC_HEVC = 0,
  CODEC_AVC,
  CODEC_YUV,
  CODEC_RDI,
  CODEC_RAWIDEAL,
  CODEC_MAX,
} qmmf_video_track_codec_type;

typedef enum qmmf_audio_track_codec_type_t {
  CODEC_PCM = 0,
  CODEC_AAC,
  CODEC_AMR,
  CODEC_AMRWB,
  CODEC_MAX_AUDIO,
} qmmf_video_audio_codec_type;

typedef enum qmmf_video_track_output_t {
  TRACK_OUTPUT_RTSP = 0,
  TRACK_OUTPUT_VAM,
  TRACK_OUTPUT_MP4,
  TRACK_OUTPUT_3GP,
  TRACK_OUTPUT_MPEGTS,
  TRACK_OUTPUT_RTMP,
  TRACK_OUTPUT_MAX,
} qmmf_video_track_output;

typedef enum qmmf_audio_track_output_t {
  AUDIO_TRACK_OUTPUT_MP4 = 0,
  AUDIO_TRACK_OUTPUT_3GP,
  AUDIO_TRACK_OUTPUT_MPEGTS,
  AUDIO_TRACK_OUTPUT_MAX,
} qmmf_audio_track_output;

typedef struct qmmf_video_track_param_t {
  uint32_t camera_id;
  uint32_t width;
  uint32_t height;
  uint32_t framerate;
  uint32_t codec;
  uint32_t bitrate;
  uint32_t output;
  uint32_t low_power_mode;
  uint32_t track_id;
  uint32_t session_id;
} qmmf_video_track_param;

typedef struct qmmf_video_track_status_t {
  uint32_t camera_id;
  uint32_t width;
  uint32_t height;
  uint32_t framerate;
  uint32_t codec;
  uint32_t bitrate;
  uint32_t output;
  uint32_t low_power_mode;
  uint32_t track_id;
  uint32_t session_id;
  const char *rtsp_url;
} qmmf_video_track_status;

typedef struct qmmf_camera_status_t {
  uint32_t camera_id;
  qmmf_camera_start_param param;
  char *supported_nr_modes;
  char *supported_hdr_modes;
  char *supported_ir_modes;
} qmmf_camera_status;

typedef struct qmmf_audio_track_param_t {
  uint32_t sample_rate;
  uint32_t num_channels;
  uint32_t bit_depth;
  uint32_t codec;
  uint32_t bitrate;
  uint32_t output;
  uint32_t track_id;
  uint32_t session_id;
} qmmf_audio_track_param;

typedef struct qmmf_status_t {
  uint32_t num_tracks;
  qmmf_video_track_status *tracks;
  uint32_t num_cameras;
  qmmf_camera_status *cameras;
  uint32_t num_audio_tracks;
  qmmf_audio_track_param *audio_tracks;
} qmmf_status;

typedef struct qmmf_image_param_t {
  uint32_t width;
  uint32_t height;
  uint32_t quality;
  uint32_t camera_id;
} qmmf_image_param;

typedef struct qmmf_image_result_t {
  uint8_t *snapshot_buffer;
  size_t snapshot_size;
  int64_t timestamp;
} qmmf_image_result;

typedef struct qmmf_vam_enrollment_info_t {
    char *id;
    char *display_name;
    char *img_id;
    uint8_t *data;
    uint32_t object_type;
    uint32_t event_type;
    uint32_t image_format;
    uint32_t image_width;
    uint32_t image_height;
} qmmf_vam_enrollment_info;

typedef struct qmmf_camera_parameters_t {
  uint32_t camera_id;
  char *nr_mode;
  uint8_t nr_mode_set;
  char *hdr_mode;
  uint8_t hdr_mode_set;
  char *ir_mode;
  uint8_t ir_mode_set;
} qmmf_camera_parameters;

typedef enum qmmf_overlay_type_t {
    DATE_TIME,
    USERTEXT,
    STATICIMAGE,
    BOUNDINGBOX,
    PRIVACYMASK,
} qmmf_overlay_type;

typedef enum qmmf_overlay_position_t {
    TOPLEFT,
    TOPRIGHT,
    CENTER,
    BOTTOMLEFT,
    BOTTOMRIGHT,
    NONE,
} qmmf_overlay_position;

typedef enum qmmf_overlay_time_t {
    HHMMSS_24HR,
    HHMMSS_AMPM,
    HHMM_24HR,
    HHMM_AMPM,
} qmmf_overlay_time;

typedef enum qmmf_overlay_date_t {
    YYYYMMDD,
    MMDDYYYY,
} qmmf_overlay_date;

typedef struct qmmf_overlay_param_t {
    enum qmmf_overlay_type_t ov_type;
    enum qmmf_overlay_position_t position;
    uint32_t color;
    enum qmmf_overlay_time_t time;
    enum qmmf_overlay_date_t date;
    uint32_t start_x;
    uint32_t start_y;
    uint32_t width;
    uint32_t height;
    char *box_name;
    char *user_text;
    char *image_location;
} qmmf_overlay_param;

typedef struct qmmf_module_t {
  int32_t (*connect) (struct qmmf_module_t *module);
  int32_t (*disconnect) (struct qmmf_module_t *module);
  int32_t (*start_camera) (struct qmmf_module_t *module, uint32_t camera_id,
                           qmmf_camera_start_param start_parm);
  int32_t (*stop_camera) (struct qmmf_module_t *module, uint32_t camera_id);
  int32_t (*create_session) (struct qmmf_module_t *module,
                             uint32_t *session_id);
  int32_t (*delete_session) (struct qmmf_module_t *module,
                             uint32_t session_id);
  int32_t (*create_video_track) (struct qmmf_module_t *module,
                                 qmmf_video_track_param track_parm);
  int32_t (*delete_video_track) (struct qmmf_module_t *module,
                                 uint32_t session_id, uint32_t track_id);
  int32_t (*create_audio_track) (struct qmmf_module_t *module,
                                 qmmf_audio_track_param track_parm);
  int32_t (*delete_audio_track) (struct qmmf_module_t *module,
                                 uint32_t session_id, uint32_t track_id);
  int32_t (*start_session) (struct qmmf_module_t *module, uint32_t session_id);
  int32_t (*stop_session) (struct qmmf_module_t *module, uint32_t session_id,
                           uint32_t flush);
  qmmf_image_result (*capture_image) (struct qmmf_module_t *module,
                                      qmmf_image_param image_param);
  struct qmmf_status_t * (*get_status) (struct qmmf_module_t *module);
  int32_t (*vam_config) (struct qmmf_module_t *module,
                         const char *json_config);
  int32_t (*vam_remove_config) (struct qmmf_module_t *module,
                                const char *json_config);
  int32_t (*vam_enroll_data) (struct qmmf_module_t *module,
                              qmmf_vam_enrollment_info enroll_info);
  int32_t (*set_camera_params) (struct qmmf_module_t *module,
                                qmmf_camera_parameters params);
  int32_t (*create_overlay) (struct qmmf_module_t *module, uint32_t track_id,
                             uint32_t *overlay_id,
                             struct qmmf_overlay_param_t *params);
  int32_t (*delete_overlay) (struct qmmf_module_t *module, uint32_t track_id,
                             uint32_t overlay_id);
  int32_t (*set_overlay) (struct qmmf_module_t *module, uint32_t track_id,
                          uint32_t overlay_id);
  int32_t (*remove_overlay) (struct qmmf_module_t *module, uint32_t track_id,
                              uint32_t overlay_id);
  int32_t (*update_overlay) (struct qmmf_module_t *module, uint32_t track_id,
                             uint32_t overlay_id,
                             struct qmmf_overlay_param_t *params);
  int32_t (*get_overlay) (struct qmmf_module_t *module, uint32_t track_id,
                          uint32_t overlay_id,
                          struct qmmf_overlay_param_t *params);
  void *priv;
} qmmf_module;

typedef struct qmmf_http_interface_t {
  int32_t (*open) (struct qmmf_module_t *module);
  int32_t (*close) (struct qmmf_module_t *module);
} qmmf_http_interface;

#endif /* QMMF_HTTP_INTERFACE_H_ */
