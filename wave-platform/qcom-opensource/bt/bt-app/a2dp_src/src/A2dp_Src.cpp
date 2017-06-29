 /*
  * Copyright (c) 2016, The Linux Foundation. All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are
  * met:
  *  * Redistributions of source code must retain the above copyright
  *    notice, this list of conditions and the following disclaimer.
  *  * Redistributions in binary form must reproduce the above
  *    copyright notice, this list of conditions and the following
  *    disclaimer in the documentation and/or other materials provided
  *    with the distribution.
  *  * Neither the name of The Linux Foundation nor the names of its
  *    contributors may be used to endorse or promote products derived
  *    from this software without specific prior written permission.
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

#include <iostream>
#include <string.h>
#include <hardware/bluetooth.h>
#if (defined(BT_AUDIO_HAL_INTEGRATION))
#include <hardware/hardware.h>
#include <hardware/audio.h>
#endif
#include <hardware/bt_av.h>
#include <hardware/bt_rc.h>
#include <list>
#include <map>
#include <dlfcn.h>

#include "A2dp_Src.hpp"
#include "Gap.hpp"
#include "hardware/bt_av_vendor.h"
#include "hardware/bt_rc_vendor.h"

#define LOGTAG_A2DP "A2DP_SRC "
#define LOGTAG_AVRCP "AVRCP_TG "

using namespace std;
using std::list;
using std::string;

A2dp_Source *pA2dpSource = NULL;
static pthread_t playback_thread = NULL;
bool media_playing = false;

#if (defined(BT_AUDIO_HAL_INTEGRATION))
audio_hw_device_t *a2dp_device = NULL;
struct audio_stream_out *output_stream = NULL;
static pthread_mutex_t a2dp_hal_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

#define AUDIO_STREAM_OUTPUT_BUFFER_SZ      (20*512)

#ifdef __cplusplus
extern "C" {
#endif

void BtA2dpSourceMsgHandler(void *msg) {
    BtEvent* pEvent = NULL;
    if(!msg) {
        ALOGE("Msg is NULL, bail out!!");
        return;
    }

    pEvent = ( BtEvent *) msg;
    switch(pEvent->event_id) {
        case PROFILE_API_START:
            ALOGD(LOGTAG_A2DP "enable a2dp source");
            if (pA2dpSource) {
                pA2dpSource->HandleEnableSource();
            }
            break;
        case PROFILE_API_STOP:
            ALOGD(LOGTAG_A2DP "disable a2dp source");
            if (pA2dpSource) {
                pA2dpSource->HandleDisableSource();
            }
            break;
        case AVRCP_TARGET_CONNECTED_CB:
        case AVRCP_TARGET_DISCONNECTED_CB:
        case A2DP_SOURCE_AUDIO_CMD_REQ:
            if (pA2dpSource) {
                pA2dpSource->HandleAvrcpEvents(( BtEvent *) msg);
            }
            break;
        default:
            if(pA2dpSource) {
               pA2dpSource->ProcessEvent(( BtEvent *) msg);
            }
            break;
    }
    delete pEvent;
}

#ifdef __cplusplus
}
#endif

static void BtA2dpLoadA2dpHal() {
#if (defined(BT_AUDIO_HAL_INTEGRATION))
    const hw_module_t *module;
    ALOGD(LOGTAG_A2DP "Load A2dp HAL");
    if (hw_get_module_by_class(AUDIO_HARDWARE_MODULE_ID,
                               AUDIO_HARDWARE_MODULE_ID_A2DP,
                               &module)) {
        ALOGE(LOGTAG_A2DP "A2dp Hal module not found");
        return;
    }
    pthread_mutex_lock(&a2dp_hal_mutex);
    if (audio_hw_device_open(module, &a2dp_device)) {
        a2dp_device = NULL;
        ALOGE(LOGTAG_A2DP "A2dp Hal device can not be opened");
        pthread_mutex_unlock(&a2dp_hal_mutex);
        return;
    }
    pthread_mutex_unlock(&a2dp_hal_mutex);
#endif
    ALOGD(LOGTAG_A2DP "A2dp HAL successfully loaded");
}

static void BtA2dpStopStreaming()
{
    ALOGD(LOGTAG_A2DP "Stop A2dp Streaming");
#if (defined(BT_AUDIO_HAL_INTEGRATION))
    pthread_mutex_lock(&a2dp_hal_mutex);
    if(!output_stream)
    {
        pthread_mutex_unlock(&a2dp_hal_mutex);
        return;
    }
    output_stream->common.standby(&output_stream->common);
    pthread_mutex_unlock(&a2dp_hal_mutex);
#endif
    ALOGD(LOGTAG_A2DP "A2dp stream successfully stopped");
}

static void BtA2dpCloseOutputStream()
{
    ALOGD(LOGTAG_A2DP "Close A2dp Output Stream");
    media_playing = false;
    if (playback_thread != NULL)
        pthread_join(playback_thread, NULL);
#if (defined(BT_AUDIO_HAL_INTEGRATION))
    pthread_mutex_lock(&a2dp_hal_mutex);
    if(!a2dp_device)
    {
        pthread_mutex_unlock(&a2dp_hal_mutex);
        return;
    }
    if(!output_stream)
    {
        pthread_mutex_unlock(&a2dp_hal_mutex);
        return;
    }
    a2dp_device->close_output_stream(a2dp_device, output_stream);
    output_stream = NULL;
    pthread_mutex_unlock(&a2dp_hal_mutex);
#endif
    ALOGD(LOGTAG_A2DP "A2dp Output Stream successfully closed");
}

static void BtA2dpUnloadA2dpHal() {
    ALOGD(LOGTAG_A2DP "Unload A2dp HAL");
    BtA2dpCloseOutputStream();
#if (defined(BT_AUDIO_HAL_INTEGRATION))
    pthread_mutex_lock(&a2dp_hal_mutex);
    if(!a2dp_device)
    {
        pthread_mutex_unlock(&a2dp_hal_mutex);
        return;
    }
    if (audio_hw_device_close(a2dp_device) < 0) {
        ALOGE(LOGTAG_A2DP "A2dp HAL could not be closed gracefully");
        pthread_mutex_unlock(&a2dp_hal_mutex);
        return;
    }
    a2dp_device = NULL;
    pthread_mutex_unlock(&a2dp_hal_mutex);
#endif
    ALOGD(LOGTAG_A2DP "A2dp HAL successfully Unloaded");
}

static void BtA2dpOpenOutputStream()
{
    int ret = -1;
    ALOGD(LOGTAG_A2DP "Open A2dp Output Stream");
#if (defined(BT_AUDIO_HAL_INTEGRATION))
    pthread_mutex_lock(&a2dp_hal_mutex);
    if (!a2dp_device) {
        ALOGE(LOGTAG_A2DP "Invalid A2dp HAL device. Bail out!");
        pthread_mutex_unlock(&a2dp_hal_mutex);
        return;
    }
    ret = a2dp_device->open_output_stream(a2dp_device, 0, AUDIO_DEVICE_OUT_ALL_A2DP,
            AUDIO_OUTPUT_FLAG_NONE, NULL, &output_stream, NULL);
    if (ret < 0) {
        output_stream = NULL;
        ALOGE(LOGTAG_A2DP "open output stream returned %d\n", ret);
    }
    pthread_mutex_unlock(&a2dp_hal_mutex);
#endif
    ALOGD(LOGTAG_A2DP "A2dp Output Stream successfully opened");
}

static void BtA2dpSuspendStreaming()
{
    ALOGD(LOGTAG_A2DP "Suspend A2dp Stream");
#if (defined(BT_AUDIO_HAL_INTEGRATION))
    pthread_mutex_lock(&a2dp_hal_mutex);
    if(!output_stream)
    {
        pthread_mutex_unlock(&a2dp_hal_mutex);
        return;
    }
    output_stream->common.set_parameters(&output_stream->common, "A2dpSuspended=true");
    pthread_mutex_unlock(&a2dp_hal_mutex);
#endif
    ALOGD(LOGTAG_A2DP "A2dp Stream suspended successfully");
}

static void BtA2dpResumeStreaming()
{
    ALOGD(LOGTAG_A2DP "Resume A2dp Stream");
#if (defined(BT_AUDIO_HAL_INTEGRATION))
    pthread_mutex_lock(&a2dp_hal_mutex);
    if(!output_stream)
    {
        pthread_mutex_unlock(&a2dp_hal_mutex);
        return;
    }
    output_stream->common.set_parameters(&output_stream->common, "A2dpSuspended=false");
    pthread_mutex_unlock(&a2dp_hal_mutex);
#endif
    ALOGD(LOGTAG_A2DP "A2dp Stream resumed successfully");
}

static void *thread_func(void *in_param)
{
    size_t len = 0;
    ssize_t write_len = 0;
    FILE *in_file = (FILE *)in_param;
    size_t out_buffer_size = 0;
    short buffer[AUDIO_STREAM_OUTPUT_BUFFER_SZ];

    ALOGD(LOGTAG_A2DP "Streaming thread started");
#if (defined(BT_AUDIO_HAL_INTEGRATION))
    pthread_mutex_lock(&a2dp_hal_mutex);
    if(!output_stream)
    {
        pthread_mutex_unlock(&a2dp_hal_mutex);
        return NULL;
    }
    out_buffer_size = output_stream->common.get_buffer_size(&output_stream->common);
    pthread_mutex_unlock(&a2dp_hal_mutex);
    if (out_buffer_size <= 0 || out_buffer_size > AUDIO_STREAM_OUTPUT_BUFFER_SZ) {
        ALOGE(LOGTAG_A2DP "Wrong buffer size. Bail out %u!!", out_buffer_size);
        if (in_file) fclose(in_file);
        return NULL;
    }
#endif
    do {
        len = fread(buffer, out_buffer_size, 1, in_file);
        if (len == 0) {
            ALOGD(LOGTAG_A2DP "Read %d bytes from file", len);
            fseek(in_file, 0, SEEK_SET);
            continue;
        }
        ALOGD(LOGTAG_A2DP "Read %d bytes from file", len);
#if (defined(BT_AUDIO_HAL_INTEGRATION))
        pthread_mutex_lock(&a2dp_hal_mutex);
        if (!output_stream) {
            pthread_mutex_unlock(&a2dp_hal_mutex);
            break;
        }
        write_len = output_stream->write(output_stream, buffer, out_buffer_size);
        pthread_mutex_unlock(&a2dp_hal_mutex);
#endif
        ALOGD(LOGTAG_A2DP "Wrote %d bytes to A2dp Hal", write_len);
    } while (media_playing);
    media_playing = false;
    if (in_file) fclose(in_file);
    ALOGD(LOGTAG_A2DP "Streaming thread about to finish");
    return NULL;
}

static void BtA2dpStartStreaming()
{
    FILE *in_file = NULL;

    ALOGD(LOGTAG_A2DP "Start A2dp Stream");
    in_file = fopen("/data/misc/bluetooth/pcmtest.wav", "r");
    if (!in_file) {
        ALOGE(LOGTAG_A2DP "Cannot open input file. Bail out!!");
        return;
    }
    ALOGD(LOGTAG_A2DP "Successfully opened input file for playback");
    media_playing = true;
    if (pthread_create(&playback_thread, NULL, thread_func, in_file) != 0) {
        ALOGD(LOGTAG_A2DP "Cannot create playback thread!\n");
        if (in_file) fclose(in_file);
        return;
    }
    return;
}

static void bta2dp_connection_state_callback(btav_connection_state_t state, bt_bdaddr_t* bd_addr) {
    ALOGD(LOGTAG_A2DP " Connection State CB");
    BtEvent *pEvent = new BtEvent;
    memcpy(&pEvent->a2dpSourceEvent.bd_addr, bd_addr, sizeof(bt_bdaddr_t));
    switch( state ) {
        case BTAV_CONNECTION_STATE_DISCONNECTED:
            pEvent->a2dpSourceEvent.event_id = A2DP_SOURCE_DISCONNECTED_CB;
        break;
        case BTAV_CONNECTION_STATE_CONNECTING:
            pEvent->a2dpSourceEvent.event_id = A2DP_SOURCE_CONNECTING_CB;
        break;
        case BTAV_CONNECTION_STATE_CONNECTED:
            pEvent->a2dpSourceEvent.event_id = A2DP_SOURCE_CONNECTED_CB;
        break;
        case BTAV_CONNECTION_STATE_DISCONNECTING:
            pEvent->a2dpSourceEvent.event_id = A2DP_SOURCE_DISCONNECTING_CB;
        break;
    }
    PostMessage(THREAD_ID_A2DP_SOURCE, pEvent);
}

static void bta2dp_audio_state_callback(btav_audio_state_t state, bt_bdaddr_t* bd_addr) {
    ALOGD(LOGTAG_A2DP " Audio State CB");
    BtEvent *pEvent = new BtEvent;
    memcpy(&pEvent->a2dpSourceEvent.bd_addr, bd_addr, sizeof(bt_bdaddr_t));
    switch( state ) {
        case BTAV_AUDIO_STATE_REMOTE_SUSPEND:
            pEvent->a2dpSourceEvent.event_id = A2DP_SOURCE_AUDIO_SUSPENDED;
        break;
        case BTAV_AUDIO_STATE_STOPPED:
            pEvent->a2dpSourceEvent.event_id = A2DP_SOURCE_AUDIO_STOPPED;
        break;
        case BTAV_AUDIO_STATE_STARTED:
            pEvent->a2dpSourceEvent.event_id = A2DP_SOURCE_AUDIO_STARTED;
        break;
    }
    PostMessage(THREAD_ID_A2DP_SOURCE, pEvent);
}

static void bta2dp_connection_priority_vendor_callback(bt_bdaddr_t* bd_addr) {
    BtEvent *pEvent = new BtEvent;
    memcpy(&pEvent->a2dpSourceEvent.bd_addr, bd_addr, sizeof(bt_bdaddr_t));
    pEvent->a2dpSourceEvent.event_id = A2DP_SOURCE_CONNECTION_PRIORITY_REQ;
    PostMessage(THREAD_ID_A2DP_SOURCE, pEvent);
}

static void bta2dp_multicast_state_vendor_callback(int state) {
    ALOGD(LOGTAG_A2DP " Multicast State CB");
}

static btav_callbacks_t sBluetoothA2dpSourceCallbacks = {
    sizeof(sBluetoothA2dpSourceCallbacks),
    bta2dp_connection_state_callback,
    bta2dp_audio_state_callback,
    NULL,
};

static btav_vendor_callbacks_t sBluetoothA2dpSourceVendorCallbacks = {
    sizeof(sBluetoothA2dpSourceVendorCallbacks),
    bta2dp_connection_priority_vendor_callback,
    bta2dp_multicast_state_vendor_callback,
    NULL,
};

static void btavrc_target_passthrough_cmd_vendor_callback(int id, int key_state, bt_bdaddr_t* bd_addr) {
    ALOGD(LOGTAG_AVRCP " btavrcp_target_passthrough_cmd_callback id = %d key_state = %d", id, key_state);
    if (key_state == KEY_PRESSED) {
        BtEvent *event = new BtEvent;
        event->avrcpTargetEvent.event_id = A2DP_SOURCE_AUDIO_CMD_REQ;
        /*As there is no player impl available at this point hence STOP/PAUSE has got same functionality*/
        if(id == CMD_ID_PAUSE)
            id = CMD_ID_STOP;
        event->avrcpTargetEvent.key_id = id;
        PostMessage (THREAD_ID_A2DP_SOURCE, event);
    }
}

static void btavrc_target_connection_state_vendor_callback(bool state, bt_bdaddr_t* bd_addr) {
    ALOGD(LOGTAG_AVRCP " btavrcp_target_connection_state_callback state = %d", state);
    BtEvent *pEvent = new BtEvent;
    memcpy(&pEvent->avrcpTargetEvent.bd_addr, bd_addr, sizeof(bt_bdaddr_t));
    if (state == true)
        pEvent->avrcpTargetEvent.event_id = AVRCP_TARGET_CONNECTED_CB;
    else
        pEvent->avrcpTargetEvent.event_id = AVRCP_TARGET_DISCONNECTED_CB;
    PostMessage(THREAD_ID_A2DP_SOURCE, pEvent);
}

static void btavrcp_target_rcfeatures_callback( bt_bdaddr_t* bd_addr, btrc_remote_features_t features) {
    ALOGD(LOGTAG_AVRCP " btavrcp_target_rcfeatures_callback features = %d", features);
}

static btrc_callbacks_t sBluetoothAvrcpTargetCallbacks = {
   sizeof(sBluetoothAvrcpTargetCallbacks),
   btavrcp_target_rcfeatures_callback,
   NULL,
   NULL,
   NULL,
   NULL,
   NULL,
   NULL,
   NULL,
   NULL,
   NULL,
   NULL,
   NULL,
};

static btrc_vendor_callbacks_t sBluetoothAvrcpTargetVendorCallbacks = {
   sizeof(sBluetoothAvrcpTargetVendorCallbacks),
   NULL,
   NULL,
   NULL,
   NULL,
   NULL,
   NULL,
   NULL,
   NULL,
   NULL,
   NULL,
   btavrc_target_passthrough_cmd_vendor_callback,
   NULL,
   NULL,
   NULL,
   NULL,
   NULL,
   NULL,
   btavrc_target_connection_state_vendor_callback,
   NULL,
};

void A2dp_Source::HandleAvrcpEvents(BtEvent* pEvent) {
    ALOGD(LOGTAG_AVRCP " HandleAvrcpEvents event = %s",
            dump_message(pEvent->avrcpTargetEvent.event_id));
    switch(pEvent->avrcpTargetEvent.event_id) {
    case AVRCP_TARGET_CONNECTED_CB:
        mAvrcpConnected = true;
        memcpy(&mConnectedAvrcpDevice, &pEvent->avrcpTargetEvent.bd_addr,
                sizeof(bt_bdaddr_t));
        break;
    case AVRCP_TARGET_DISCONNECTED_CB:
        mAvrcpConnected = false;
        memset(&mConnectedAvrcpDevice, 0, sizeof(bt_bdaddr_t));
        break;
    case A2DP_SOURCE_AUDIO_CMD_REQ:
        uint8_t key_id = pEvent->avrcpTargetEvent.key_id;
        if (!mAvrcpConnected || (memcmp(&mConnectedAvrcpDevice, &mConnectedDevice,
                                                          sizeof(bt_bdaddr_t)) != 0)) {
            ALOGD(LOGTAG_AVRCP " No Active connection. Bail out!! ");
            break;
        }
        switch(key_id) {
        case CMD_ID_PLAY:
            if (media_playing)
                BtA2dpResumeStreaming();
            else
                BtA2dpStartStreaming();
            break;
        case CMD_ID_PAUSE:
            /*Pause key id is mapped to A2dp suspend*/
            BtA2dpSuspendStreaming();
            break;
        case CMD_ID_STOP:
            /*Pause and Stop passthrough commands are handled here*/
            media_playing = false;
            BtA2dpStopStreaming();
            break;
        default:
            ALOGD(LOGTAG_AVRCP " Command not supported ");
        }
        break;
    }
}

void A2dp_Source::HandleEnableSource(void) {
    BtEvent *pEvent = new BtEvent;
    if (bluetooth_interface != NULL)
    {
        sBtA2dpSourceInterface = (btav_interface_t *)bluetooth_interface->
                get_profile_interface(BT_PROFILE_ADVANCED_AUDIO_ID);
        sBtA2dpSourceVendorInterface = (btav_vendor_interface_t *)bluetooth_interface->
                get_profile_interface(BT_PROFILE_ADVANCED_AUDIO_VENDOR_ID);
        if (sBtA2dpSourceInterface == NULL)
        {
             pEvent->profile_start_event.event_id = PROFILE_EVENT_START_DONE;
             pEvent->profile_start_event.profile_id = PROFILE_ID_A2DP_SOURCE;
             pEvent->profile_start_event.status = false;
             PostMessage(THREAD_ID_GAP, pEvent);
             return;
        }
#ifdef USE_LIBHW_AOSP
        sBtA2dpSourceInterface->init(&sBluetoothA2dpSourceCallbacks);
#else
        sBtA2dpSourceInterface->init(&sBluetoothA2dpSourceCallbacks, 1, 0);
#endif
        sBtA2dpSourceVendorInterface->init_vendor(&sBluetoothA2dpSourceVendorCallbacks, 1, 0);
        pEvent->profile_start_event.event_id = PROFILE_EVENT_START_DONE;
        pEvent->profile_start_event.profile_id = PROFILE_ID_A2DP_SOURCE;
        pEvent->profile_start_event.status = true;
        // AVRCP TG Initialization
        sBtAvrcpTargetInterface = (btrc_interface_t *)bluetooth_interface->
                get_profile_interface(BT_PROFILE_AV_RC_ID);
        if (sBtAvrcpTargetInterface != NULL) {
#ifdef USE_LIBHW_AOSP
            sBtAvrcpTargetInterface->init(&sBluetoothAvrcpTargetCallbacks);
#else
            sBtAvrcpTargetInterface->init(&sBluetoothAvrcpTargetCallbacks, 1);
#endif
        }
        // AVRCP TG vendor Initialization
        sBtAvrcpTargetVendorInterface = (btrc_vendor_interface_t *)bluetooth_interface->
                get_profile_interface(BT_PROFILE_AV_RC_VENDOR_ID);
        if (sBtAvrcpTargetVendorInterface != NULL) {
            sBtAvrcpTargetVendorInterface->init_vendor(&sBluetoothAvrcpTargetVendorCallbacks, 1);
        }
        change_state(STATE_A2DP_SOURCE_DISCONNECTED);
        PostMessage(THREAD_ID_GAP, pEvent);
        BtA2dpLoadA2dpHal();
        media_playing = false;
    }
}

void A2dp_Source::HandleDisableSource(void) {
   change_state(STATE_A2DP_SOURCE_NOT_STARTED);
   BtA2dpUnloadA2dpHal();
   if(sBtA2dpSourceInterface != NULL) {
       sBtA2dpSourceInterface->cleanup();
       sBtA2dpSourceInterface = NULL;
   }
   if (sBtAvrcpTargetInterface != NULL) {
       sBtAvrcpTargetInterface->cleanup();
       sBtAvrcpTargetInterface = NULL;
   }
   BtEvent *pEvent = new BtEvent;
   pEvent->profile_stop_event.event_id = PROFILE_EVENT_STOP_DONE;
   pEvent->profile_stop_event.profile_id = PROFILE_ID_A2DP_SOURCE;
   pEvent->profile_stop_event.status = true;
   PostMessage(THREAD_ID_GAP, pEvent);
   media_playing = false;
}

void A2dp_Source::ProcessEvent(BtEvent* pEvent) {
    switch(mSourceState) {
        case STATE_A2DP_SOURCE_DISCONNECTED:
            state_disconnected_handler(pEvent);
            break;
        case STATE_A2DP_SOURCE_PENDING:
            state_pending_handler(pEvent);
            break;
        case STATE_A2DP_SOURCE_CONNECTED:
            state_connected_handler(pEvent);
            break;
        case STATE_A2DP_SOURCE_NOT_STARTED:
            cout << "Ignore!! Make sure BT is turned on!!" << endl;
            ALOGE(LOGTAG_A2DP " STATE UNINITIALIZED, return");
            break;
    }
}

char* A2dp_Source::dump_message(BluetoothEventId event_id) {
    switch(event_id) {
    case A2DP_SOURCE_API_CONNECT_REQ:
        return"API_CONNECT_REQ";
    case A2DP_SOURCE_API_DISCONNECT_REQ:
        return "API_DISCONNECT_REQ";
    case A2DP_SOURCE_DISCONNECTED_CB:
        return "DISCONNECTED_CB";
    case A2DP_SOURCE_CONNECTING_CB:
        return "CONNECING_CB";
    case A2DP_SOURCE_CONNECTED_CB:
        return "CONNECTED_CB";
    case A2DP_SOURCE_DISCONNECTING_CB:
        return "DISCONNECTING_CB";
    case A2DP_SOURCE_AUDIO_SUSPENDED:
        return "AUDIO_SUSPENDED_CB";
    case A2DP_SOURCE_AUDIO_STOPPED:
        return "AUDIO_STOPPED_CB";
    case A2DP_SOURCE_AUDIO_STARTED:
        return "AUDIO_STARTED_CB";
    case AVRCP_TARGET_CONNECTED_CB:
        return "AVRCP_TARGET_CONNECTED_CB";
    case AVRCP_TARGET_DISCONNECTED_CB:
        return "AVRCP_TARGET_DISCONNECTED_CB";
    case A2DP_SOURCE_AUDIO_CMD_REQ:
        return "AUDIO_CMD_REQ";
    case A2DP_SOURCE_CONNECTION_PRIORITY_REQ:
        return "CONNECTION_PRIORITY_REQ";
    }
    return "UNKNOWN";
}

void A2dp_Source::state_disconnected_handler(BtEvent* pEvent) {
    char str[18];
    ALOGD(LOGTAG_A2DP "state_disconnected_handler Processing event %s", dump_message(pEvent->event_id));
    switch(pEvent->event_id) {
        case A2DP_SOURCE_API_CONNECT_REQ:
            memcpy(&mConnectingDevice, &pEvent->a2dpSourceEvent.bd_addr, sizeof(bt_bdaddr_t));
            if (sBtA2dpSourceInterface != NULL) {
                sBtA2dpSourceInterface->connect(&pEvent->a2dpSourceEvent.bd_addr);
            }
            bdaddr_to_string(&mConnectingDevice, str, 18);
            cout << "A2DP Source Connecting to " << str << endl;
            change_state(STATE_A2DP_SOURCE_PENDING);
            break;
        case A2DP_SOURCE_API_DISCONNECT_REQ:
            cout << "A2DP Source Disconnect can not be processed" << endl;
            break;
        case A2DP_SOURCE_CONNECTING_CB:
            memcpy(&mConnectingDevice, &pEvent->a2dpSourceEvent.bd_addr, sizeof(bt_bdaddr_t));
            bdaddr_to_string(&mConnectingDevice, str, 18);
            cout << "A2DP Source Connecting to " << str << endl;
            change_state(STATE_A2DP_SOURCE_PENDING);
            break;
        case A2DP_SOURCE_CONNECTED_CB:
            memset(&mConnectingDevice, 0, sizeof(bt_bdaddr_t));
            memcpy(&mConnectedDevice, &pEvent->a2dpSourceEvent.bd_addr, sizeof(bt_bdaddr_t));
            bdaddr_to_string(&mConnectedDevice, str, 18);
            cout << "A2DP Source Connected to " << str << endl;
            change_state(STATE_A2DP_SOURCE_CONNECTED);
            BtA2dpOpenOutputStream();
            break;
        case A2DP_SOURCE_CONNECTION_PRIORITY_REQ:
            if (sBtA2dpSourceVendorInterface != NULL) {
                sBtA2dpSourceVendorInterface->allow_connection_vendor(1, &pEvent->a2dpSourceEvent.bd_addr);
            }
            break;
        default:
            cout << "Event not processed in disconnected state "<< pEvent->event_id << endl;
            ALOGE(LOGTAG_A2DP " event not handled %d ", pEvent->event_id);
            break;
    }
}
void A2dp_Source::state_pending_handler(BtEvent* pEvent) {
    char str[18];
    ALOGD(LOGTAG_A2DP "state_pending_handler Processing event %s", dump_message(pEvent->event_id));
    switch(pEvent->event_id) {
        case A2DP_SOURCE_CONNECTED_CB:
            memcpy(&mConnectedDevice, &pEvent->a2dpSourceEvent.bd_addr, sizeof(bt_bdaddr_t));
            memset(&mConnectingDevice, 0, sizeof(bt_bdaddr_t));
            bdaddr_to_string(&mConnectedDevice, str, 18);
            cout << "A2DP Source Connected to " << str << endl;
            change_state(STATE_A2DP_SOURCE_CONNECTED);
            BtA2dpOpenOutputStream();
            break;
        case A2DP_SOURCE_DISCONNECTED_CB:
            cout << "A2DP Source DisConnected "<< endl;
            media_playing = false;
            BtA2dpCloseOutputStream();
            memset(&mConnectedDevice, 0, sizeof(bt_bdaddr_t));
            memset(&mConnectingDevice, 0, sizeof(bt_bdaddr_t));
            change_state(STATE_A2DP_SOURCE_DISCONNECTED);
            break;
        case A2DP_SOURCE_API_CONNECT_REQ:
            bdaddr_to_string(&mConnectingDevice, str, 18);
            cout << "A2DP Source already Connecting to " << str << endl;
            break;
        case A2DP_SOURCE_API_DISCONNECT_REQ:
            cout << "A2DP Source Disconnect can not be processed" << endl;
            break;
        case A2DP_SOURCE_CONNECTION_PRIORITY_REQ:
            if (sBtA2dpSourceVendorInterface != NULL) {
                sBtA2dpSourceVendorInterface->allow_connection_vendor(1, &pEvent->a2dpSourceEvent.bd_addr);
            }
            break;
        default:
            cout << "Event not processed in pending state "<< pEvent->event_id << endl;
            ALOGE(LOGTAG_A2DP " event not handled %d ", pEvent->event_id);
            break;
    }
}

void A2dp_Source::state_connected_handler(BtEvent* pEvent) {
    char str[18];
    BtEvent *pControlRequest, *pReleaseControlReq;
    ALOGD(LOGTAG_A2DP "state_connected_handler Processing event %s", dump_message(pEvent->event_id));
    switch(pEvent->event_id) {
        case A2DP_SOURCE_API_CONNECT_REQ:
            bdaddr_to_string(&mConnectedDevice, str, 18);
            cout << "A2DP Source Already Connected to " << str << endl;
            break;
        case A2DP_SOURCE_API_DISCONNECT_REQ:
            if (memcmp(&mConnectedDevice, &pEvent->a2dpSourceEvent.bd_addr, sizeof(bt_bdaddr_t)))
            {
                bdaddr_to_string(&pEvent->a2dpSourceEvent.bd_addr, str, 18);
                cout << "Device not connected: " << str << endl;
                break;
            }
            bdaddr_to_string(&mConnectedDevice, str, 18);
            cout << "A2DP Source DisConnecting: " << str << endl;
            media_playing = false;
            if (sBtA2dpSourceInterface != NULL) {
                sBtA2dpSourceInterface->disconnect(&pEvent->a2dpSourceEvent.bd_addr);
            }
            change_state(STATE_A2DP_SOURCE_PENDING);
            break;
        case A2DP_SOURCE_CONNECTION_PRIORITY_REQ:
            if (sBtA2dpSourceVendorInterface != NULL) {
                sBtA2dpSourceVendorInterface->allow_connection_vendor(1, &pEvent->a2dpSourceEvent.bd_addr);
            }
            break;
        case A2DP_SOURCE_DISCONNECTED_CB:
            media_playing = false;
            BtA2dpCloseOutputStream();
            memset(&mConnectedDevice, 0, sizeof(bt_bdaddr_t));
            memset(&mConnectingDevice, 0, sizeof(bt_bdaddr_t));
            cout << "A2DP Source DisConnected " << endl;
            change_state(STATE_A2DP_SOURCE_DISCONNECTED);
            break;
        case A2DP_SOURCE_DISCONNECTING_CB:
            cout << "A2DP Source DisConnecting " << endl;
            change_state(STATE_A2DP_SOURCE_PENDING);
            break;
        case A2DP_SOURCE_AUDIO_STARTED:
        case A2DP_SOURCE_AUDIO_SUSPENDED:
        case A2DP_SOURCE_AUDIO_STOPPED:
            cout << "A2DP Source Audio state changes to: " << pEvent->event_id << endl;
            break;
        default:
            cout << "Event not processed in connected state "<< pEvent->event_id << endl;
            ALOGE(LOGTAG_A2DP " event not handled %d ", pEvent->event_id);
            break;
    }
}

void A2dp_Source::change_state(A2dpSourceState mState) {
   ALOGD(LOGTAG_A2DP " current State = %d, new state = %d", mSourceState, mState);
   pthread_mutex_lock(&lock);
   mSourceState = mState;
   pthread_mutex_unlock(&lock);
   ALOGD(LOGTAG_A2DP " state changed to %d ", mState);
}

A2dp_Source :: A2dp_Source(const bt_interface_t *bt_interface, config_t *config) {
    this->bluetooth_interface = bt_interface;
    this->config = config;
    sBtA2dpSourceInterface = NULL;
    sBtAvrcpTargetInterface = NULL;
    mSourceState = STATE_A2DP_SOURCE_NOT_STARTED;
    mAvrcpConnected = false;
    memset(&mConnectedDevice, 0, sizeof(bt_bdaddr_t));
    memset(&mConnectingDevice, 0, sizeof(bt_bdaddr_t));
    memset(&mConnectedAvrcpDevice, 0, sizeof(bt_bdaddr_t));
    pthread_mutex_init(&this->lock, NULL);
}

A2dp_Source :: ~A2dp_Source() {
    mAvrcpConnected = false;
    pthread_mutex_destroy(&lock);
}
