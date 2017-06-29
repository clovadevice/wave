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
package main

/*
#cgo CFLAGS: -Ihttpinterface/inc
#cgo LDFLAGS: -ldl

#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>
#include <sys/types.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>

#include "qmmf_http_interface.h"

#define LIB_PATH "libhttp_interface.so"
#define MODULE_NAME "QMMF_MODULE"

typedef struct qmmf_handle_t {
    void *lib_handle;
    struct qmmf_module_t qmmf_mod;
    struct qmmf_http_interface_t *qmmf_intf;
} qmmf_handle;

qmmf_handle *qmmf_open() {
    qmmf_handle *ret = NULL;
    void *handle;
    struct qmmf_http_interface_t *qmmf_intf;
    int32_t status;

    handle = dlopen(LIB_PATH, RTLD_NOW);
    if (NULL == handle) {
        printf("%s: Unable to open web interface library: %s!\n",
            __func__, dlerror());
        errno = -1;
        goto EXIT;
    }

    qmmf_intf = (struct qmmf_http_interface_t *) dlsym(handle, MODULE_NAME);
    if (NULL == qmmf_intf) {
        printf("%s: Cannot find web interface: %s!\n",
            __func__, dlerror());
        errno = -1;
        goto EXIT;
    }

    ret = (qmmf_handle *) malloc(sizeof(qmmf_handle));
    if (NULL == ret) {
        printf("%s: No resources for QMMF handle!", __func__);
        errno = -ENOMEM;
        goto EXIT;
    }

    status = qmmf_intf->open(&ret->qmmf_mod);
    if (0 != status) {
        printf("%s: Module open failed: %d", __func__, status);
        errno = status;
        goto EXIT;
    }

    ret->qmmf_intf = qmmf_intf;
    ret->lib_handle = handle;

    return ret;

EXIT:

    if (handle) {
        dlclose(handle);
    }

    if (ret) {
        free(ret);
    }

    return NULL;
}

int32_t qmmf_close(qmmf_handle *handle) {
    int32_t ret;
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    if (NULL == handle->qmmf_intf) {
        printf("%s: Invalid interface!\n", __func__);
        return -EINVAL;
    }

    ret = handle->qmmf_intf->close(&handle->qmmf_mod);
    dlclose(handle->lib_handle);
    free(handle);

    return ret;
}

int32_t qmmf_connect(qmmf_handle *handle) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.connect(&handle->qmmf_mod);
}

int32_t qmmf_disconnect(qmmf_handle *handle) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.disconnect(&handle->qmmf_mod);
}

int32_t qmmf_start_camera(qmmf_handle *handle, uint32_t camera_id,
                          struct qmmf_camera_start_param_t start_parm) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.start_camera(&handle->qmmf_mod, camera_id,
                                         start_parm);
}

int32_t qmmf_stop_camera(qmmf_handle *handle, uint32_t camera_id) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.stop_camera(&handle->qmmf_mod, camera_id);
}

int32_t qmmf_create_session(qmmf_handle *handle, uint32_t *session_id) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.create_session(&handle->qmmf_mod, session_id);
}

int32_t qmmf_delete_session(qmmf_handle *handle, uint32_t session_id) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.delete_session(&handle->qmmf_mod, session_id);
}

int32_t qmmf_create_video_track(qmmf_handle *handle,
                                struct qmmf_video_track_param_t params) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.create_video_track(&handle->qmmf_mod, params);
}

int32_t qmmf_delete_video_track(qmmf_handle *handle, uint32_t session_id,
                                uint32_t track_id) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.delete_video_track(&handle->qmmf_mod, session_id,
                                               track_id);
}

int32_t qmmf_create_audio_track(qmmf_handle *handle,
        struct qmmf_audio_track_param_t params) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.create_audio_track(&handle->qmmf_mod, params);
}

int32_t qmmf_delete_audio_track(qmmf_handle *handle, uint32_t session_id,
                                uint32_t track_id) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.delete_audio_track(&handle->qmmf_mod, session_id,
                                               track_id);
}

int32_t qmmf_stop_session(qmmf_handle *handle, uint32_t session_id,
                          uint32_t flush) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.stop_session(&handle->qmmf_mod, session_id, flush);
}

int32_t qmmf_start_session(qmmf_handle *handle, uint32_t session_id) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.start_session(&handle->qmmf_mod, session_id);
}

struct qmmf_image_result_t qmmf_capture_image(qmmf_handle *handle,
                           struct qmmf_image_param_t image_param) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        struct qmmf_image_result_t ret;
        memset(&ret, 0, sizeof(ret));
        return ret;
    }

    return handle->qmmf_mod.capture_image(&handle->qmmf_mod, image_param);
}

struct qmmf_status_t *qmmf_get_status(qmmf_handle *handle) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return NULL;
    }

    return handle->qmmf_mod.get_status(&handle->qmmf_mod);
}

int32_t qmmf_vam_config(qmmf_handle *handle,
                        const char *json_config) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.vam_config(&handle->qmmf_mod, json_config);
}

int32_t qmmf_vam_remove_config(qmmf_handle *handle,
                               const char *json_config) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.vam_remove_config(&handle->qmmf_mod, json_config);
}

int32_t qmmf_vam_enroll(qmmf_handle *handle,
                        struct qmmf_vam_enrollment_info_t enroll_info) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.vam_enroll_data(&handle->qmmf_mod, enroll_info);
}

int32_t qmmf_set_camera_params(qmmf_handle *handle,
                               struct qmmf_camera_parameters_t params) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.set_camera_params(&handle->qmmf_mod, params);
}

int32_t qmmf_create_overlay(qmmf_handle *handle, uint32_t track_id,
                             uint32_t *overlay_id,
                             struct qmmf_overlay_param_t *params) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.create_overlay(&handle->qmmf_mod, track_id,
                                           overlay_id, params);
}

int32_t qmmf_delete_overlay(qmmf_handle *handle, uint32_t track_id,
                            uint32_t overlay_id) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.delete_overlay(&handle->qmmf_mod, track_id,
                                           overlay_id);
}

int32_t qmmf_set_overlay(qmmf_handle *handle, uint32_t track_id,
                         uint32_t overlay_id) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.set_overlay(&handle->qmmf_mod, track_id,
                                        overlay_id);
}

int32_t qmmf_remove_overlay(qmmf_handle *handle, uint32_t track_id,
                            uint32_t overlay_id) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.remove_overlay(&handle->qmmf_mod, track_id,
                                           overlay_id);
}

int32_t qmmf_get_overlay(qmmf_handle *handle, uint32_t track_id,
                         uint32_t overlay_id,
                         struct qmmf_overlay_param_t *params) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.get_overlay(&handle->qmmf_mod, track_id,
                                        overlay_id, params);
}

int32_t qmmf_update_overlay(qmmf_handle *handle, uint32_t track_id,
                            uint32_t overlay_id,
                            struct qmmf_overlay_param_t *params) {
    if (NULL == handle) {
        printf("%s: Invalid handle!\n", __func__);
        return -EINVAL;
    }

    return handle->qmmf_mod.update_overlay(&handle->qmmf_mod, track_id,
                                           overlay_id, params);
}
*/
import "C"
import "log"
import "fmt"
import "unsafe"
import "strconv"
import "net/http"
import "encoding/json"
import "encoding/base64"
import "github.com/gorilla/mux"

type HttpHeaderContent struct {
    ContentType string
    ContentValue string
}

var HTTPHeader = HttpHeaderContent {"Content-Type", "application/json"}

var StatusUnsupported = "Not supported yet"
var StatusSuccess = "Success"
var StatusUnknown = "Unknown"

var ErrorTrue = "true"
var ErrorNone = "none"

type Result struct {
    Status string
    Error string
}

type SnapshotResult struct {
    Timestamp C.int64_t
    Data string
}

type SessionId struct {
    SessionId C.uint32_t
}

type InputParameters struct {
    SessionId string
    CameraId string
    ZSLWidth string
    ZSLHeight string
    ZSLMode string
    ZSLQueueDepth string
    Flags string
    Framerate string
    Bitrate string
    LowPowerMode string
    TrackId string
    TrackWidth string
    TrackHeight string
    TrackCodec string
    TrackOutput string
    ImageWidth string
    ImageHeight string
    ImageQuality string
    Flush string
    SampleRate string
    NumChannels string
    BitDepth string
    VAMConfig string
    VAMEnrollObjectType string
    VAMEnrollEventType string
    VAMEnrollFormat string
    VAMEnrollWidth string
    VAMEnrollHeight string
    VAMEnrollId string
    VAMEnrollName string
    VAMEnrollImageId string
    VAMEnrollData string
    NRMode string
    HDRMode string
    IRMode string
    OverlayId string
    OverlayType string
    OverlayPosition string
    OverlayColor string
    OverlayUserText string
    OverlayWidth string
    OverlayHeight string
    OverlayStartX string
    OverlayStartY string
    OverlayImageLocation string
    OverlayBoxName string
    OverlayDate string
    Overlaytime string
}

var input_params = InputParameters {
    "session_id",
    "camera_id",
    "zsl_width",
    "zsl_height",
    "zsl_mode",
    "zsl_queue_depth",
    "flags",
    "framerate",
    "bitrate",
    "low_power_mode",
    "track_id",
    "track_width",
    "track_height",
    "track_codec",
    "track_output",
    "image_width",
    "image_height",
    "image_quality",
    "flush",
    "sample_rate",
    "num_channels",
    "bit_depth",
    "vam_config",
    "vam_enroll_object_type",
    "vam_enroll_event_type",
    "vam_enroll_format",
    "vam_enroll_width",
    "vam_enroll_height",
    "vam_enroll_id",
    "vam_enroll_name",
    "vam_enroll_image_id",
    "vam_enroll_data",
    "nr_mode",
    "hdr_mode",
    "ir_mode",
    "ov_id",
    "ov_type",
    "ov_position",
    "ov_color",
    "ov_user_text",
    "ov_width",
    "ov_height",
    "ov_start_x",
    "ov_start_y",
    "ov_image_location",
    "ov_box_name",
    "ov_date",
    "ov_time",
}

var qmmf_handle = unsafe.Pointer(C.qmmf_open())

func PostConnectHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var err = C.qmmf_connect(qmmf_handle)
    if (0 == err) {
        res = Result{StatusSuccess, ErrorNone}
    } else {
        var status = fmt.Sprintf("Connect failed: %d", err)
        res = Result{status, ErrorTrue}
    }
    j, _ := json.Marshal(res)

    w.Write(j)
}

func PostDisconnectHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var err = C.qmmf_disconnect(qmmf_handle)
    if (0 == err) {
        res = Result{StatusSuccess, ErrorNone}
    } else {
        var status = fmt.Sprintf("Disconnect failed: %d", err)
        res = Result{status, ErrorTrue}
    }
    j, _ := json.Marshal(res)

    w.Write(j)
}

func PostCreatesessionHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var session_id = C.uint32_t(0)
    var err = C.qmmf_create_session(qmmf_handle, &session_id)
    if (0 == err) {
        var session_result = SessionId{session_id}
        j, _ := json.Marshal(session_result)
        w.Write(j)
    } else {
        var status = fmt.Sprintf("Session create failed: %d", err)
        res = Result{status, ErrorTrue}
        j, _ := json.Marshal(res)
        w.Write(j)
    }
}

func PostDeletesessionHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var err = 0
    var session_id = C.uint32_t(0)
    session_id, err, res = queryUintParam(r, input_params.SessionId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var rc = C.qmmf_delete_session(qmmf_handle, session_id)
    if (0 == rc) {
        res = Result{StatusSuccess, ErrorNone}
    } else {
        var status = fmt.Sprintf("Session delete failed: %d", rc)
        res = Result{status, ErrorTrue}
    }

    j, _ := json.Marshal(res)
    w.Write(j)
}

func queryUintParam(r *http.Request, param string) (ret C.uint32_t,
    err int, res Result) {
    var value = r.PostFormValue(param)
    err = 0
    if 0 < len(value) {
        var uintValue, status = strconv.ParseUint(value, 10, 32)
        if nil != status {
            res = Result {"Error converting string to integer", ErrorTrue}
            err = -1
        } else {
            ret = C.uint32_t(uintValue)
        }
    } else {
        res = Result {"Parameter missing", ErrorTrue}
        err = -1
    }

    return
}

func PostStartcameraHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var err = 0
    var params = C.struct_qmmf_camera_start_param_t {0, 0, 0, 0, 0, 0}
    var camera_id = C.uint32_t(0)

    camera_id, err, res = queryUintParam(r, input_params.CameraId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.zsl_width, err, res = queryUintParam(r, input_params.ZSLWidth)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.zsl_height, err, res = queryUintParam(r, input_params.ZSLHeight)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.frame_rate, err, res = queryUintParam(r, input_params.Framerate)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.flags, err, res = queryUintParam(r, input_params.Flags)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.zsl_queue_depth, err, res = queryUintParam(r, input_params.ZSLQueueDepth)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.zsl_mode, err, res = queryUintParam(r, input_params.ZSLMode)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var stat = C.qmmf_start_camera(qmmf_handle, camera_id, params)
    if (0 == stat) {
        res = Result{StatusSuccess, ErrorNone}
    } else {
        var status = fmt.Sprintf("camera start failed: %d", stat)
        res = Result{status, ErrorTrue}
    }
    j, _ := json.Marshal(res)

    w.Write(j)
}

func PostStopcameraHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var err = 0
    var camera_id = C.uint32_t(0)

    camera_id, err, res = queryUintParam(r, input_params.CameraId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var stat = C.qmmf_stop_camera(qmmf_handle, camera_id)
    if (0 == stat) {
        res = Result{StatusSuccess, ErrorNone}
    } else {
        var status = fmt.Sprintf("camera stop failed: %d", stat)
        res = Result{status, ErrorTrue}
    }
    j, _ := json.Marshal(res)

    w.Write(j)
}

func PostCaptureImageHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var err = 0
    var params = C.struct_qmmf_image_param_t{0, 0, 0, 0}

    params.camera_id, err, res = queryUintParam(r, input_params.CameraId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.width, err, res = queryUintParam(r, input_params.ImageWidth)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.height, err, res = queryUintParam(r, input_params.ImageHeight)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.quality, err, res = queryUintParam(r, input_params.ImageQuality)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var stat = C.qmmf_capture_image(qmmf_handle, params)
    if nil != stat.snapshot_buffer {
        var buffer = (*[1<<30]byte)(unsafe.Pointer(stat.snapshot_buffer))[0:stat.snapshot_size]
        var snapshot = SnapshotResult {Timestamp: stat.timestamp,
                                       Data: base64.StdEncoding.EncodeToString(buffer)}
        j,_ := json.Marshal(snapshot)
        w.Write(j)
    } else {
        var status = fmt.Sprintf("image capture failed: %d", stat)
        res = Result{status, ErrorTrue}
        j, _ := json.Marshal(res)
        w.Write(j)
    }

    if nil != stat.snapshot_buffer {
        C.free(unsafe.Pointer(stat.snapshot_buffer))
    }
}

func PostCreateAudiotrackHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var err = 0
    var params = C.struct_qmmf_audio_track_param_t {sample_rate: 0,
                                                    num_channels: 0,
                                                    bit_depth: 0,
                                                    bitrate: 0,
                                                    codec:C.CODEC_MAX_AUDIO,
                                                    output:C.AUDIO_TRACK_OUTPUT_MAX,
                                                    track_id: 0, session_id: 0}

    params.codec, err, res = queryUintParam(r, input_params.TrackCodec)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }
    if C.CODEC_MAX_AUDIO <= params.codec {
        var status = fmt.Sprintf("Invalid codec value: %d max: %d",
                                 params.codec, C.CODEC_MAX_AUDIO-1)
        res = Result{status, ErrorTrue}
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    } else if (C.CODEC_PCM != params.codec) {
        params.bitrate, err, res = queryUintParam(r, input_params.Bitrate)
        if 0 != err {
            j, _ := json.Marshal(res)
            w.Write(j)
            return
        }
    }

    params.output, err, res = queryUintParam(r, input_params.TrackOutput)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }
    if C.AUDIO_TRACK_OUTPUT_MAX <= params.output {
        var status = fmt.Sprintf("Invalid output value: %d max: %d",
                                 params.output, C.AUDIO_TRACK_OUTPUT_MAX-1)
        res = Result{status, ErrorTrue}
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.session_id, err, res = queryUintParam(r, input_params.SessionId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.track_id, err, res = queryUintParam(r, input_params.TrackId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.sample_rate, err, res = queryUintParam(r, input_params.SampleRate)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.num_channels, err, res = queryUintParam(r, input_params.NumChannels)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.bit_depth, err, res = queryUintParam(r, input_params.BitDepth)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var stat = C.qmmf_create_audio_track(qmmf_handle, params)
    if (0 == stat) {
        res = Result{StatusSuccess, ErrorNone}
    } else {
        var status = fmt.Sprintf("audio track create failed: %d", stat)
        res = Result{status, ErrorTrue}
    }
    j, _ := json.Marshal(res)

    w.Write(j)
}

func PostDeleteAudiotrackHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var err = 0
    var session_id = C.uint32_t(0)
    session_id, err, res = queryUintParam(r, input_params.SessionId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var track_id = C.uint32_t(0)
    track_id, err, res = queryUintParam(r, input_params.TrackId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var rc = C.qmmf_delete_audio_track(qmmf_handle, session_id, track_id)
    if (0 == rc) {
        res = Result{StatusSuccess, ErrorNone}
    } else {
        var status = fmt.Sprintf("audio track delete failed: %d", rc)
        res = Result{status, ErrorTrue}
    }

    j, _ := json.Marshal(res)
    w.Write(j)
}

func PostCreateVideotrackHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var err = 0
    var params = C.struct_qmmf_video_track_param_t {camera_id: 0, width:0,
                                                    height: 0, framerate: 0,
                                                    codec:C.CODEC_MAX,
                                                    output:C.TRACK_OUTPUT_RTSP,
                                                    low_power_mode: 0,
                                                    track_id: 0, session_id: 0}

    params.camera_id, err, res = queryUintParam(r, input_params.CameraId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.width, err, res = queryUintParam(r, input_params.TrackWidth)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.height, err, res = queryUintParam(r, input_params.TrackHeight)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.codec, err, res = queryUintParam(r, input_params.TrackCodec)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }
    if C.CODEC_MAX <= params.codec {
        var status = fmt.Sprintf("Invalid codec value: %d max: %d",
                                 params.codec, C.CODEC_MAX-1)
        res = Result{status, ErrorTrue}
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    } else if (C.CODEC_HEVC == params.codec) || (C.CODEC_AVC == params.codec) {
        params.bitrate, err, res = queryUintParam(r, input_params.Bitrate)
        if 0 != err {
            j, _ := json.Marshal(res)
            w.Write(j)
            return
        }
    }

    params.output, err, res = queryUintParam(r, input_params.TrackOutput)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }
    if C.TRACK_OUTPUT_MAX <= params.output {
        var status = fmt.Sprintf("Invalid output value: %d max: %d",
                                 params.output, C.TRACK_OUTPUT_MAX)
        res = Result{status, ErrorTrue}
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.session_id, err, res = queryUintParam(r, input_params.SessionId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.track_id, err, res = queryUintParam(r, input_params.TrackId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.framerate, err, res = queryUintParam(r, input_params.Framerate)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.low_power_mode, err, res = queryUintParam(r, input_params.LowPowerMode)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var stat = C.qmmf_create_video_track(qmmf_handle, params)
    if (0 == stat) {
        res = Result{StatusSuccess, ErrorNone}
    } else {
        var status = fmt.Sprintf("video track create failed: %d", stat)
        res = Result{status, ErrorTrue}
    }
    j, _ := json.Marshal(res)

    w.Write(j)
}

func PostDeleteVideotrackHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var err = 0
    var session_id = C.uint32_t(0)
    session_id, err, res = queryUintParam(r, input_params.SessionId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var track_id = C.uint32_t(0)
    track_id, err, res = queryUintParam(r, input_params.TrackId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var rc = C.qmmf_delete_video_track(qmmf_handle, session_id, track_id)
    if (0 == rc) {
        res = Result{StatusSuccess, ErrorNone}
    } else {
        var status = fmt.Sprintf("video track delete failed: %d", rc)
        res = Result{status, ErrorTrue}
    }

    j, _ := json.Marshal(res)
    w.Write(j)
}

func PostStartsessionHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var err = 0
    var session_id = C.uint32_t(0)
    session_id, err, res = queryUintParam(r, input_params.SessionId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var rc = C.qmmf_start_session(qmmf_handle, session_id)
    if (0 == rc) {
        res = Result{StatusSuccess, ErrorNone}
    } else {
        var status = fmt.Sprintf("session start failed: %d", rc)
        res = Result{status, ErrorTrue}
    }

    j, _ := json.Marshal(res)
    w.Write(j)
}

func PostStopsessionHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var err = 0
    var session_id = C.uint32_t(0)
    session_id, err, res = queryUintParam(r, input_params.SessionId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var flush = C.uint32_t(0)
    flush, err, res = queryUintParam(r, input_params.Flush)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var rc = C.qmmf_stop_session(qmmf_handle, session_id, flush)
    if (0 == rc) {
        res = Result{StatusSuccess, ErrorNone}
    } else {
        var status = fmt.Sprintf("session stop failed: %d", rc)
        res = Result{status, ErrorTrue}
    }

    j, _ := json.Marshal(res)
    w.Write(j)
}

func PostVamConfigHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var value = r.PostFormValue(input_params.VAMConfig)
    if 0 >= len(value) {
        res = Result {"JSON config parameter missing", ErrorTrue}
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var json_config = C.CString(value)
    var rc = C.qmmf_vam_config(qmmf_handle, json_config)
    if (0 == rc) {
        res = Result{StatusSuccess, ErrorNone}
    } else {
        var status = fmt.Sprintf("VAM configuration failed: %d", rc)
        res = Result{status, ErrorTrue}
    }
    C.free(unsafe.Pointer(json_config))

    j, _ := json.Marshal(res)
    w.Write(j)
}

func PostVamRemoveConfigHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var value = r.PostFormValue(input_params.VAMConfig)
    if 0 >= len(value) {
        res = Result {"JSON config parameter missing", ErrorTrue}
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var json_config = C.CString(value)
    var rc = C.qmmf_vam_remove_config(qmmf_handle, json_config)
    if (0 == rc) {
        res = Result{StatusSuccess, ErrorNone}
    } else {
        var status = fmt.Sprintf("VAM remove configuration failed: %d", rc)
        res = Result{status, ErrorTrue}
    }
    C.free(unsafe.Pointer(json_config))

    j, _ := json.Marshal(res)
    w.Write(j)
}

func PostVamEnrollHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var err = 0
    var params = C.struct_qmmf_vam_enrollment_info_t{nil, nil, nil, nil, 0,
                                                     0, 0, 0, 0}

    params.object_type, err, res = queryUintParam(r, input_params.VAMEnrollObjectType)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.event_type, err, res = queryUintParam(r, input_params.VAMEnrollEventType)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.image_format, err, res = queryUintParam(r, input_params.VAMEnrollFormat)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.image_width, err, res = queryUintParam(r, input_params.VAMEnrollWidth)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.image_height, err, res = queryUintParam(r, input_params.VAMEnrollHeight)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    data, stat := base64.StdEncoding.DecodeString(r.PostFormValue(input_params.VAMEnrollData))
    if nil != stat {
        fmt.Println("status:", stat)
        res = Result {"Failed to decode base64 data", ErrorTrue}
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }
    if 0 == len(data) {
        res = Result {"Empty enrollment data!", ErrorTrue}
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.data = (*C.uint8_t)(unsafe.Pointer(&data[0]))

    params.id = C.CString(r.PostFormValue(input_params.VAMEnrollId))
    params.display_name = C.CString(r.PostFormValue(input_params.VAMEnrollName))
    params.img_id = C.CString(r.PostFormValue(input_params.VAMEnrollImageId))

    var rc = C.qmmf_vam_enroll(qmmf_handle, params)
    if (0 == rc) {
        res = Result{StatusSuccess, ErrorNone}
    } else {
        var status = fmt.Sprintf("VAM enrollment failed: %d", rc)
        res = Result{status, ErrorTrue}
    }
    C.free(unsafe.Pointer(params.id))
    C.free(unsafe.Pointer(params.display_name))
    C.free(unsafe.Pointer(params.img_id))

    j, _ := json.Marshal(res)
    w.Write(j)
}

func queryStringParameter (r *http.Request, query string) (res *C.char,
                                                           present C.uint8_t) {
    present = 0
    res = nil
    var value = r.PostFormValue(query)
    if 0 < len(value) {
        res = C.CString(value)
        present = 1;
    }

    return
}

func PostSetCameraParamHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var err = 0
    var params = C.struct_qmmf_camera_parameters_t{camera_id:0,
                                                   nr_mode: nil,
                                                   nr_mode_set: 0,
                                                   hdr_mode: nil,
                                                   hdr_mode_set: 0,
                                                   ir_mode: nil,
                                                   ir_mode_set: 0}

    params.camera_id, err, res = queryUintParam(r, input_params.CameraId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params.nr_mode, params.nr_mode_set = queryStringParameter(r, input_params.NRMode)
    params.hdr_mode, params.hdr_mode_set = queryStringParameter(r, input_params.HDRMode)
    params.ir_mode, params.ir_mode_set = queryStringParameter(r, input_params.IRMode)
    var rc = C.qmmf_set_camera_params(qmmf_handle, params);
    if (0 == rc) {
        res = Result{StatusSuccess, ErrorNone}
    } else {
        var status = fmt.Sprintf("Camera configuration failed: %d", rc)
        res = Result{status, ErrorTrue}
    }

    j, _ := json.Marshal(res)

    w.Write(j)

    if nil != params.nr_mode {
        C.free(unsafe.Pointer(params.nr_mode))
    }
    if nil != params.hdr_mode {
        C.free(unsafe.Pointer(params.hdr_mode))
    }
    if nil != params.ir_mode {
        C.free(unsafe.Pointer(params.ir_mode))
    }
}

func ParseOverlayParams(r *http.Request) (ret C.struct_qmmf_overlay_param_t,
                                          err int, res Result) {
    res = Result{StatusSuccess, ErrorNone}
    err = 0

    var v = r.PostFormValue(input_params.OverlayType)
    if 0 < len(v) {
        ret.ov_type, err = GetOverlayTypeString(v)
        if 0 != err {
            res = Result {"Error parsing overlay type", ErrorTrue}
            return ret, err, res
        }
    } else {
        err = -1
        res =  Result {"Overlay type is missing!", ErrorTrue}
        return ret, err, res
    }

    switch  ret.ov_type {
    case C.DATE_TIME:
        v = r.PostFormValue(input_params.OverlayPosition)
        if 0 < len(v) {
            ret.position, err = GetOverlayPositionString(v)
            if 0 != err {
                res = Result {"Error parsing overlay position", ErrorTrue}
                return ret, err, res
            }
        } else {
            err = -1
            res =  Result {"Overlay position is missing!", ErrorTrue}
            return ret, err, res
        }

        ret.color, err, res = queryUintParam(r, input_params.OverlayColor)
        if 0 != err {
            return ret, err, res
        }

        v = r.PostFormValue(input_params.OverlayDate)
        if 0 < len(v) {
            ret.date, err = GetOverlayDateString(v)
            if 0 != err {
                res = Result {"Error parsing overlay date", ErrorTrue}
                return ret, err, res
            }
        } else {
            err = -1
            res =  Result {"Overlay date parameter is missing!", ErrorTrue}
            return ret, err, res
        }

        v = r.PostFormValue(input_params.Overlaytime)
        if 0 < len(v) {
            ret.time, err = GetOverlayTimeString(v)
            if 0 != err {
                res = Result {"Error parsing overlay time", ErrorTrue}
                return ret, err, res
            }
        } else {
            err = -1
            res =  Result {"Overlay time parameter is missing!", ErrorTrue}
            return ret, err, res
        }
    case C.USERTEXT:
        v = r.PostFormValue(input_params.OverlayPosition)
        if 0 < len(v) {
            ret.position, err = GetOverlayPositionString(v)
            if 0 != err {
                res = Result {"Error parsing overlay position", ErrorTrue}
                return ret, err, res
            }
        } else {
            err = -1
            res =  Result {"Overlay position is missing!", ErrorTrue}
            return ret, err, res
        }

        ret.color, err, res = queryUintParam(r, input_params.OverlayColor)
        if 0 != err {
            return ret, err, res
        }

        v = r.PostFormValue(input_params.OverlayUserText)
        if 0 < len(v) {
            ret.user_text = C.CString(v)
        } else {
            err = -1
            res =  Result {"Overlay user text is missing!", ErrorTrue}
            return ret, err, res
        }
    case C.STATICIMAGE:
        v = r.PostFormValue(input_params.OverlayPosition)
        if 0 < len(v) {
            ret.position, err = GetOverlayPositionString(v)
            if 0 != err {
                res = Result {"Error parsing overlay position", ErrorTrue}
                return ret, err, res
            }
        } else {
            err = -1
            res =  Result {"Overlay position is missing!", ErrorTrue}
            return ret, err, res
        }

        v = r.PostFormValue(input_params.OverlayImageLocation)
        if 0 < len(v) {
            ret.image_location = C.CString(v)
        } else {
            err = -1
            res =  Result {"Overlay image location is missing!", ErrorTrue}
            return ret, err, res
        }

        ret.width, err, res = queryUintParam(r, input_params.OverlayWidth)
        if 0 != err {
            return ret, err, res
        }

        ret.height, err, res = queryUintParam(r, input_params.OverlayHeight)
        if 0 != err {
            return ret, err, res
        }
    case C.BOUNDINGBOX:
        ret.color, err, res = queryUintParam(r, input_params.OverlayColor)
        if 0 != err {
            return ret, err, res
        }

        v = r.PostFormValue(input_params.OverlayBoxName)
        if 0 < len(v) {
            ret.box_name = C.CString(v)
        } else {
            err = -1
            res =  Result {"Overlay bounding box name is missing!", ErrorTrue}
            return ret, err, res
        }

        ret.start_x, err, res = queryUintParam(r, input_params.OverlayStartX)
        if 0 != err {
            return ret, err, res
        }

        ret.start_y, err, res = queryUintParam(r, input_params.OverlayStartY)
        if 0 != err {
            return ret, err, res
        }

        ret.width, err, res = queryUintParam(r, input_params.OverlayWidth)
        if 0 != err {
            return ret, err, res
        }

        ret.height, err, res = queryUintParam(r, input_params.OverlayHeight)
        if 0 != err {
            return ret, err, res
        }
    case C.PRIVACYMASK:
        ret.color, err, res = queryUintParam(r, input_params.OverlayColor)
        if 0 != err {
            return ret, err, res
        }

        ret.start_x, err, res = queryUintParam(r, input_params.OverlayStartX)
        if 0 != err {
            return ret, err, res
        }

        ret.start_y, err, res = queryUintParam(r, input_params.OverlayStartY)
        if 0 != err {
            return ret, err, res
        }

        ret.width, err, res = queryUintParam(r, input_params.OverlayWidth)
        if 0 != err {
            return ret, err, res
        }

        ret.height, err, res = queryUintParam(r, input_params.OverlayHeight)
        if 0 != err {
            return ret, err, res
        }
    default:
        err = -1
        res = Result {"Unsupported overlay type!", ErrorTrue}
        return ret, err, res
    }

    return ret, err, res
}

func GetStringOverlayTime(ovtime C.enum_qmmf_overlay_time_t) string {
    switch ovtime {
    case C.HHMMSS_24HR:
        return "hhmmss_24hr"
    case C.HHMMSS_AMPM:
        return "hhmmss_ampm"
    case C.HHMM_24HR:
        return "hhmm_24hr"
    case C.HHMM_AMPM:
        return "hhmm_ampm"
    }

    return "invalid"
}

func GetOverlayDateString(ovdate string) (ret C.enum_qmmf_overlay_date_t,
    err int) {
    err = 0
    switch ovdate {
    case "yyyymmdd":
        return C.YYYYMMDD, err
    case "mmddyyyy":
        return C.MMDDYYYY, err
    default:
        err = -1
    }

    return C.YYYYMMDD, err
}

func GetStringOverlayDate(ovdate C.enum_qmmf_overlay_date_t) string {
    switch ovdate {
    case C.YYYYMMDD:
        return "yyyymmdd"
    case C.MMDDYYYY:
        return "mmddyyyy"
    }

    return "invalid"
}

func GetOverlayTimeString(ovtime string) (ret C.enum_qmmf_overlay_time_t,
                                          err int) {
    err = 0
    switch ovtime {
    case "hhmmss_24hr":
        return C.HHMMSS_24HR, err
    case "hhmmss_ampm":
        return C.HHMMSS_AMPM, err
    case "hhmm_24hr":
        return C.HHMM_24HR, err
    case "hhmm_ampm":
        return C.HHMM_AMPM, err
    default:
        err = -1
    }

    return C.HHMMSS_24HR, err
}

func GetStringOverlayPosition(ovpos C.enum_qmmf_overlay_position_t) string {
    switch ovpos {
    case C.TOPLEFT:
        return "topleft"
    case C.TOPRIGHT:
        return "topright"
    case C.CENTER:
        return "center"
    case C.BOTTOMLEFT:
        return "bottomleft"
    case C.BOTTOMRIGHT:
        return "bottomright"
    case C.NONE:
        return "none"
    }

    return "invalid"
}

func GetOverlayPositionString(ovposition string) (ret C.enum_qmmf_overlay_position_t,
                                                  err int) {
    err = 0
    switch ovposition {
    case "topleft":
        return C.TOPLEFT, err
    case "topright":
        return C.TOPRIGHT, err
    case "center":
        return C.CENTER, err
    case "bottomleft":
        return C.BOTTOMLEFT, err
    case "bottomright":
        return C.BOTTOMRIGHT, err
    case "none":
        return C.NONE, err
    default:
        err = -1
    }

    return C.NONE, err
}

func GetStringOverlayType(ovtype C.enum_qmmf_overlay_type_t) string {
    switch ovtype {
    case C.DATE_TIME:
        return "datetime"
    case C.USERTEXT:
        return "usertext"
    case C.STATICIMAGE:
        return "staticimage"
    case C.BOUNDINGBOX:
        return "boundingbox"
    case C.PRIVACYMASK:
        return "privacymask"
    }

    return "invalid"
}

func GetOverlayTypeString(ovtype string) (ret C.enum_qmmf_overlay_type_t,
                                          err int) {
    err = 0
    switch ovtype {
    case "datetime":
        return C.DATE_TIME, err
    case "usertext":
        return C.USERTEXT, err
    case "staticimage":
        return C.STATICIMAGE, err
    case "boundingbox":
        return C.BOUNDINGBOX, err
    case "privacymask":
        return C.PRIVACYMASK, err
    default:
        err = -1
    }
    return C.DATE_TIME, err
}

func releaseOverlayParams(params *C.struct_qmmf_overlay_param_t) {
    if nil != params.box_name {
        C.free(unsafe.Pointer(params.box_name))
    }

    if nil != params.user_text {
        C.free(unsafe.Pointer(params.user_text))
    }

    if nil != params.image_location {
        C.free(unsafe.Pointer(params.image_location))
    }
}

func PostCreateOverlayHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var err int = 0
    var overlayId = C.uint32_t(0)
    var params = C.struct_qmmf_overlay_param_t {ov_type: C.DATE_TIME,
                                                position: C.NONE,
                                                color:0,
                                                time: C.HHMMSS_24HR,
                                                date: C.YYYYMMDD,
                                                start_x:0,
                                                start_y:0,
                                                width:0,
                                                height:0,
                                                box_name:nil,
                                                user_text:nil,
                                                image_location:nil}

    var track_id = C.uint32_t(0)
    track_id, err, res = queryUintParam(r, input_params.TrackId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params, err, res = ParseOverlayParams(r)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var rc = C.qmmf_create_overlay(qmmf_handle, track_id, &overlayId, &params);
    if (0 != rc) {
        var status = fmt.Sprintf("Error during overlay create : %d", rc)
        res = Result{status, ErrorTrue}
    } else {
        var status = fmt.Sprintf("%d", overlayId)
        res = Result{status, ErrorNone}
    }

    releaseOverlayParams(&params)
    j, _ := json.Marshal(res)
    w.Write(j)
}

func PostDeleteOverlayHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var err int = 0
    var overlayId = C.uint32_t(0)

    var track_id = C.uint32_t(0)
    track_id, err, res = queryUintParam(r, input_params.TrackId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    overlayId, err, res = queryUintParam(r, input_params.OverlayId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var rc = C.qmmf_delete_overlay(qmmf_handle, track_id, overlayId);
    if (0 == rc) {
        res = Result{StatusSuccess, ErrorNone}
    } else {
        var status = fmt.Sprintf("Error during overlay delete : %d", rc)
        res = Result{status, ErrorTrue}
    }

    j, _ := json.Marshal(res)

    w.Write(j)
}

func PostSetOverlayHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var err int = 0

    var track_id = C.uint32_t(0)
    track_id, err, res = queryUintParam(r, input_params.TrackId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var overlayId = C.uint32_t(0)
    overlayId, err, res = queryUintParam(r, input_params.OverlayId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var rc = C.qmmf_set_overlay(qmmf_handle, track_id, overlayId);
    if (0 == rc) {
        res = Result{StatusSuccess, ErrorNone}
    } else {
        var status = fmt.Sprintf("Error during overlay set : %d", rc)
        res = Result{status, ErrorTrue}
    }

    j, _ := json.Marshal(res)

    w.Write(j)
}

func PostRemoveOverlayHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var err int = 0

    var track_id = C.uint32_t(0)
    track_id, err, res = queryUintParam(r, input_params.TrackId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var overlayId = C.uint32_t(0)
    overlayId, err, res = queryUintParam(r, input_params.OverlayId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var rc = C.qmmf_remove_overlay(qmmf_handle, track_id, overlayId);
    if (0 == rc) {
        res = Result{StatusSuccess, ErrorNone}
    } else {
        var status = fmt.Sprintf("Error during overlay remove : %d", rc)
        res = Result{status, ErrorTrue}
    }

    j, _ := json.Marshal(res)

    w.Write(j)
}

func PostGetOverlayHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var err int = 0
    var params = C.struct_qmmf_overlay_param_t {ov_type: C.DATE_TIME,
                                                position: C.NONE,
                                                color:0,
                                                time: C.HHMMSS_24HR,
                                                date: C.YYYYMMDD,
                                                start_x:0,
                                                start_y:0,
                                                width:0,
                                                height:0,
                                                box_name:nil,
                                                user_text:nil,
                                                image_location:nil}

    var track_id = C.uint32_t(0)
    track_id, err, res = queryUintParam(r, input_params.TrackId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var overlayId = C.uint32_t(0)
    overlayId, err, res = queryUintParam(r, input_params.OverlayId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var rc = C.qmmf_get_overlay(qmmf_handle, track_id, overlayId, &params);
    if (0 != rc) {
        var status = fmt.Sprintf("Error during overlay get : %d", rc)
        res = Result{status, ErrorTrue}
        j, _ := json.Marshal(res)
        w.Write(j)
    } else {
        var ov_params = map[string]interface{} {
            input_params.OverlayType   : GetStringOverlayType(params.ov_type),
        }

        switch params.ov_type {
        case C.DATE_TIME:
            ov_params[input_params.OverlayDate] = GetStringOverlayDate(params.date)
            ov_params[input_params.Overlaytime] = GetStringOverlayTime(params.time)
            ov_params[input_params.OverlayColor] = params.color
            ov_params[input_params.OverlayPosition] =
                GetStringOverlayPosition(params.position)
        case C.USERTEXT:
            ov_params[input_params.OverlayUserText] = C.GoString(params.user_text)
            ov_params[input_params.OverlayColor] = params.color
            ov_params[input_params.OverlayPosition] =
                GetStringOverlayPosition(params.position)
        case C.STATICIMAGE:
            ov_params[input_params.OverlayWidth] = params.width
            ov_params[input_params.OverlayHeight] = params.height
            ov_params[input_params.OverlayImageLocation] =
                C.GoString(params.image_location)
            ov_params[input_params.OverlayPosition] =
                GetStringOverlayPosition(params.position)
        case C.BOUNDINGBOX:
            ov_params[input_params.OverlayStartX] = params.start_x
            ov_params[input_params.OverlayStartY] = params.start_y
            ov_params[input_params.OverlayWidth] = params.width
            ov_params[input_params.OverlayHeight] = params.height
            ov_params[input_params.OverlayBoxName] = C.GoString(params.box_name)
            ov_params[input_params.OverlayColor] = params.color
        case C.PRIVACYMASK:
            ov_params[input_params.OverlayColor] = params.color
            ov_params[input_params.OverlayStartX] = params.start_x
            ov_params[input_params.OverlayStartY] = params.start_y
            ov_params[input_params.OverlayWidth] = params.width
            ov_params[input_params.OverlayHeight] = params.height
        }

        j, _ := json.Marshal(ov_params)
        w.Write(j)
    }

    releaseOverlayParams(&params)
}

func PostUpdateOverlayHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnknown, ErrorTrue}
    var err int = 0
    var params = C.struct_qmmf_overlay_param_t {ov_type: C.DATE_TIME,
                                                position: C.NONE,
                                                color:0,
                                                time: C.HHMMSS_24HR,
                                                date: C.YYYYMMDD,
                                                start_x:0,
                                                start_y:0,
                                                width:0,
                                                height:0,
                                                box_name:nil,
                                                user_text:nil,
                                                image_location:nil}

    var track_id = C.uint32_t(0)
    track_id, err, res = queryUintParam(r, input_params.TrackId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var overlayId = C.uint32_t(0)
    overlayId, err, res = queryUintParam(r, input_params.OverlayId)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    params, err, res = ParseOverlayParams(r)
    if 0 != err {
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    var rc = C.qmmf_update_overlay(qmmf_handle, track_id, overlayId, &params);
    if (0 != rc) {
        var status = fmt.Sprintf("Error during overlay update : %d", rc)
        res = Result{status, ErrorTrue}
    } else {
        var status = fmt.Sprintf("%d", overlayId)
        res = Result{status, ErrorNone}
    }

    releaseOverlayParams(&params)
    j, _ := json.Marshal(res)
    w.Write(j)
}

func GetUnsupportedHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var res = Result{StatusUnsupported, ErrorTrue}
    j, _ := json.Marshal(res)
    w.Write(j)
}

type ServerStatus struct{
    Tracks []map[string]interface{}
    AudioTracks []map[string]interface{}
    Cameras []map[string]interface{}
}

func (s C.struct_qmmf_status_t) MarshalJSON() ([]byte, error) {
    var tracks []map[string]interface{} = nil
    var track_count = int(s.num_tracks)
    if 0 < track_count {
        var track_slice = (*[1 << 10]C.struct_qmmf_video_track_status_t) (unsafe.Pointer(s.tracks))[:track_count:track_count]
        for i := 0; i < track_count; i++ {
            var st = map[string]interface{} {
                "camera_id"      : track_slice[i].camera_id,
                "track_id"       : track_slice[i].track_id,
                "session_id"     : track_slice[i].session_id,
                "rtsp_url"       : C.GoString(track_slice[i].rtsp_url),
                "width"          : track_slice[i].width,
                "height"         : track_slice[i].height,
                "framerate"      : track_slice[i].framerate,
                "bitrate"        : track_slice[i].bitrate,
                "codec"          : track_slice[i].codec,
                "output"         : track_slice[i].output,
                "low_power_mode" : track_slice[i].low_power_mode,
            }
            tracks = append(tracks, st)
        }
    }

    var audio_tracks []map[string]interface{} = nil
    track_count = int(s.num_audio_tracks)
    if 0 < track_count {
        var track_slice = (*[1 << 10]C.struct_qmmf_audio_track_param_t) (unsafe.Pointer(s.audio_tracks))[:track_count:track_count]
        for i := 0; i < track_count; i++ {
            var st = map[string]interface{} {
                "sample_rate"    : track_slice[i].sample_rate,
                "track_id"       : track_slice[i].track_id,
                "session_id"     : track_slice[i].session_id,
                "num_channels"   : track_slice[i].num_channels,
                "bit_depth"      : track_slice[i].bit_depth,
                "bitrate"        : track_slice[i].bitrate,
                "codec"          : track_slice[i].codec,
                "output"         : track_slice[i].output,
            }
            audio_tracks = append(audio_tracks, st)
        }
    }

    var cameras []map[string]interface{} = nil
    var camera_count = int(s.num_cameras)
    if 0 < camera_count {
        var camera_slice = (*[1 << 10]C.struct_qmmf_camera_status_t) (unsafe.Pointer(s.cameras))[:camera_count:camera_count]
        for i := 0; i < camera_count; i++ {
            var c = map[string]interface{} {
                "camera_id"     : camera_slice[i].camera_id,
                "zsl_mode"      : camera_slice[i].param.zsl_mode,
                "zsl_q_depth"   : camera_slice[i].param.zsl_queue_depth,
                "zsl_width"     : camera_slice[i].param.zsl_width,
                "zsl_height"    : camera_slice[i].param.zsl_height,
                "framerate"     : camera_slice[i].param.frame_rate,
                "flags"         : camera_slice[i].param.flags,
                "nr_modes"      : C.GoString(camera_slice[i].supported_nr_modes),
                "hdr_modes"     : C.GoString(camera_slice[i].supported_hdr_modes),
                "ir_modes"     : C.GoString(camera_slice[i].supported_ir_modes),
            }
            cameras = append(cameras, c)
        }
    }

    var status = ServerStatus {
        tracks,
        audio_tracks,
        cameras,
    }

    return json.Marshal(status)
}

func RootHandler(w http.ResponseWriter, r *http.Request) {
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)
    var status = C.qmmf_get_status(qmmf_handle)
    if nil == status {
        var err = fmt.Sprintf("Error during status query!")
        var res = Result{err, ErrorTrue}
        j, _ := json.Marshal(res)
        w.Write(j)
        return
    }

    j, _ := json.Marshal(status)
    w.Write(j)

    if 0 < status.num_tracks {
        C.free(unsafe.Pointer(status.tracks))
    }
    if 0 < status.num_cameras {
        C.free(unsafe.Pointer(status.cameras))
    }
    if 0 < status.num_audio_tracks {
        C.free(unsafe.Pointer(status.audio_tracks))
    }
    C.free(unsafe.Pointer(status))
}

func main() {
    fmt.Println("Starting webserver!")
    r := mux.NewRouter()

    r.HandleFunc("/", RootHandler)
    r.HandleFunc("/connect", PostConnectHandler).Methods("POST")
    r.HandleFunc("/connect", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/disconnect", PostDisconnectHandler).Methods("POST")
    r.HandleFunc("/disconnect", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/startcamera", PostStartcameraHandler).Methods("POST")
    r.HandleFunc("/startcamera", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/stopcamera", PostStopcameraHandler).Methods("POST")
    r.HandleFunc("/stopcamera", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/createsession", PostCreatesessionHandler).Methods("POST")
    r.HandleFunc("/createsession", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/deletesession", PostDeletesessionHandler).Methods("POST")
    r.HandleFunc("/deletesession", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/createvideotrack", PostCreateVideotrackHandler).Methods("POST")
    r.HandleFunc("/createvideotrack", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/deletevideotrack", PostDeleteVideotrackHandler).Methods("POST")
    r.HandleFunc("/deletevideotrack", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/startsession", PostStartsessionHandler).Methods("POST")
    r.HandleFunc("/startsession", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/stopsession", PostStopsessionHandler).Methods("POST")
    r.HandleFunc("/stopsession", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/captureimage", PostCaptureImageHandler).Methods("POST")
    r.HandleFunc("/captureimage", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/createaudiotrack", PostCreateAudiotrackHandler).Methods("POST")
    r.HandleFunc("/createaudiotrack", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/deleteaudiotrack", PostDeleteAudiotrackHandler).Methods("POST")
    r.HandleFunc("/deleteaudiotrack", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/vamconfig", PostVamConfigHandler).Methods("POST")
    r.HandleFunc("/vamconfig", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/vamremoveconfig", PostVamRemoveConfigHandler).Methods("POST")
    r.HandleFunc("/vamremoveconfig", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/vamenroll", PostVamEnrollHandler).Methods("POST")
    r.HandleFunc("/vamenroll", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/setcameraparam", PostSetCameraParamHandler).Methods("POST")
    r.HandleFunc("/setcameraparam", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/createoverlay", PostCreateOverlayHandler).Methods("POST")
    r.HandleFunc("/createoverlay", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/deleteoverlay", PostDeleteOverlayHandler).Methods("POST")
    r.HandleFunc("/deleteoverlay", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/setoverlay", PostSetOverlayHandler).Methods("POST")
    r.HandleFunc("/setoverlay", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/removeoverlay", PostRemoveOverlayHandler).Methods("POST")
    r.HandleFunc("/removeoverlay", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/getoverlay", PostGetOverlayHandler).Methods("POST")
    r.HandleFunc("/getoverlay", GetUnsupportedHandler).Methods("GET")
    r.HandleFunc("/updateoverlay", PostUpdateOverlayHandler).Methods("POST")
    r.HandleFunc("/updateoverlay", GetUnsupportedHandler).Methods("GET")

    err := http.ListenAndServe(":4000", r)
    if err != nil {
        log.Fatal(err)
    }
    C.qmmf_close(qmmf_handle)
}
