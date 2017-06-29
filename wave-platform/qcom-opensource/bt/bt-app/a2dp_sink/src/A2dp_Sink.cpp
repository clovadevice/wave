 /*
  * Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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

#include <list>
#include <map>
#include <iostream>
#include <string.h>
#include <hardware/bluetooth.h>
#include <hardware/hardware.h>
#include <hardware/bt_av.h>
#include "A2dp_Sink_Streaming.hpp"
#include "A2dp_Sink.hpp"
#include "Avrcp.hpp"
#include "Gap.hpp"
#include "hardware/bt_av_vendor.h"

#define LOGTAG "A2DP_SINK"

using namespace std;
using std::list;
using std::string;

A2dp_Sink *pA2dpSink = NULL;
A2dp_Sink_Streaming *pA2dpSinkStream;
extern Avrcp *pAvrcp;

static const bt_bdaddr_t bd_addr_null= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#ifdef __cplusplus
extern "C" {
#endif

void BtA2dpSinkMsgHandler(void *msg) {
    BtEvent* pEvent = NULL;
    BtEvent* pCleanupEvent = NULL;
    BtEvent *pCleanupSinkStreaming = NULL;
    if(!msg) {
        printf("Msg is NULL, return.\n");
        return;
    }

    pEvent = ( BtEvent *) msg;
    switch(pEvent->event_id) {
        case PROFILE_API_START:
            ALOGD(LOGTAG " enable a2dp sink");
            if (pA2dpSink) {
                pA2dpSink->HandleEnableSink();
            }
            break;
        case PROFILE_API_STOP:
            ALOGD(LOGTAG " disable a2dp sink");
            if (pA2dpSink) {
                pA2dpSink->HandleDisableSink();
            }
            break;
        case A2DP_SINK_CLEANUP_REQ:
            ALOGD(LOGTAG " cleanup a2dp sink");
            pCleanupSinkStreaming = new BtEvent;
            pCleanupSinkStreaming->a2dpSinkStreamingEvent.event_id =
                    A2DP_SINK_STREAMING_CLEANUP_REQ;
            if (pA2dpSinkStream) {
                thread_post(pA2dpSinkStream->threadInfo.thread_id,
                pA2dpSinkStream->threadInfo.thread_handler, (void*)pCleanupSinkStreaming);
            }
            pCleanupEvent = new BtEvent;
            pCleanupEvent->event_id = A2DP_SINK_CLEANUP_DONE;
            PostMessage(THREAD_ID_GAP, pCleanupEvent);
            break;
        case A2DP_SINK_STREAMING_DISABLE_DONE:
            ALOGD(LOGTAG " a2dp sink streaming disable done");
            if (pA2dpSink) {
                pA2dpSink->HandleSinkStreamingDisableDone();
            }
            break;
        default:
            if(pA2dpSink) {
               pA2dpSink->EventManager(( BtEvent *) msg, pEvent->a2dpSinkEvent.bd_addr);
            }
            break;
    }
    delete pEvent;
}

#ifdef __cplusplus
}
#endif

list<A2dp_Device>::iterator FindDeviceByAddr(list<A2dp_Device>& pA2dpDev, bt_bdaddr_t dev) {
    list<A2dp_Device>::iterator p = pA2dpDev.begin();
    while(p != pA2dpDev.end()) {
        if (memcmp(&dev, &p->mDevice, sizeof(bt_bdaddr_t)) == 0) {
            break;
        }
        p++;
    }
    return p;
}

static void bta2dp_connection_state_callback(btav_connection_state_t state, bt_bdaddr_t* bd_addr) {
    ALOGD(LOGTAG " Connection State CB state = %d", state);
    BtEvent *pEvent = new BtEvent;
    memcpy(&pEvent->a2dpSinkEvent.bd_addr, bd_addr, sizeof(bt_bdaddr_t));
    switch( state ) {
        case BTAV_CONNECTION_STATE_DISCONNECTED:
            pEvent->a2dpSinkEvent.event_id = A2DP_SINK_DISCONNECTED_CB;
        break;
        case BTAV_CONNECTION_STATE_CONNECTING:
            pEvent->a2dpSinkEvent.event_id = A2DP_SINK_CONNECTING_CB;
        break;
        case BTAV_CONNECTION_STATE_CONNECTED:
            pEvent->a2dpSinkEvent.event_id = A2DP_SINK_CONNECTED_CB;
        break;
        case BTAV_CONNECTION_STATE_DISCONNECTING:
            pEvent->a2dpSinkEvent.event_id = A2DP_SINK_DISCONNECTING_CB;
        break;
    }
    PostMessage(THREAD_ID_A2DP_SINK, pEvent);
}

static void bta2dp_audio_state_callback(btav_audio_state_t state, bt_bdaddr_t* bd_addr) {
    ALOGD(LOGTAG " Audio State CB state = %d", state);
    BtEvent *pEvent = new BtEvent;
    memcpy(&pEvent->a2dpSinkEvent.bd_addr, bd_addr, sizeof(bt_bdaddr_t));
    switch( state ) {
        case BTAV_AUDIO_STATE_REMOTE_SUSPEND:
            pEvent->a2dpSinkEvent.event_id = A2DP_SINK_AUDIO_SUSPENDED;
        break;
        case BTAV_AUDIO_STATE_STOPPED:
            pEvent->a2dpSinkEvent.event_id = A2DP_SINK_AUDIO_STOPPED;
        break;
        case BTAV_AUDIO_STATE_STARTED:
            pEvent->a2dpSinkEvent.event_id = A2DP_SINK_AUDIO_STARTED;
        break;
    }
    PostMessage(THREAD_ID_A2DP_SINK, pEvent);
}

static void bta2dp_audio_config_callback(bt_bdaddr_t *bd_addr, uint32_t sample_rate,
        uint8_t channel_count) {
    ALOGD(LOGTAG " Audio Config CB sample_rate %d, channel_count %d", sample_rate, channel_count);
    list<A2dp_Device>::iterator iter = FindDeviceByAddr(pA2dpSink->pA2dpDeviceList, *bd_addr);
    if(iter != pA2dpSink->pA2dpDeviceList.end())
    {
        ALOGD(LOGTAG " Audio Config CB: found matching device");
        iter->av_config.sample_rate = sample_rate;
        iter->av_config.channel_count = channel_count;
    }
    else
    {
        ALOGE(LOGTAG " ERROR: Audio Config CB: No matching device");
    }
}
static void bta2dp_audio_focus_request_vendor_callback(bt_bdaddr_t *bd_addr) {
    ALOGD(LOGTAG " bta2dp_audio_focus_request_vendor_callback ");
    BtEvent *pEvent = new BtEvent;
    pEvent->a2dpSinkEvent.event_id = A2DP_SINK_FOCUS_REQUEST_CB;
    memcpy(&pEvent->a2dpSinkEvent.bd_addr, bd_addr, sizeof(bt_bdaddr_t));
    PostMessage(THREAD_ID_A2DP_SINK, pEvent);
}

static void bta2dp_audio_codec_config_vendor_callback(bt_bdaddr_t *bd_addr, uint16_t codec_type,
        btav_codec_config_t codec_config) {
    ALOGD(LOGTAG " bta2dp_audio_codec_config_vendor_callback ");

    BtEvent *pEvent = new BtEvent;
    pEvent->a2dpSinkEvent.event_id = A2DP_SINK_CODEC_CONFIG;
    memcpy(&pEvent->a2dpSinkEvent.bd_addr, bd_addr, sizeof(bt_bdaddr_t));
    pEvent->a2dpSinkEvent.buf_size = sizeof(btav_codec_config_t);
    pEvent->a2dpSinkEvent.buf_ptr = (uint8_t*)osi_malloc(pEvent->a2dpSinkEvent.buf_size);
    memcpy(pEvent->a2dpSinkEvent.buf_ptr, &codec_config, pEvent->a2dpSinkEvent.buf_size);
    pEvent->a2dpSinkEvent.arg1 = codec_type;
    PostMessage(THREAD_ID_A2DP_SINK, pEvent);
}

static btav_callbacks_t sBluetoothA2dpSinkCallbacks = {
    sizeof(sBluetoothA2dpSinkCallbacks),
    bta2dp_connection_state_callback,
    bta2dp_audio_state_callback,
    bta2dp_audio_config_callback,
};

static btav_sink_vendor_callbacks_t sBluetoothA2dpSinkVendorCallbacks = {
    sizeof(sBluetoothA2dpSinkVendorCallbacks),
    bta2dp_audio_focus_request_vendor_callback,
    bta2dp_audio_codec_config_vendor_callback,
};

void A2dp_Sink::HandleEnableSink(void) {
    ALOGD(LOGTAG " HandleEnableSink ");

    BtEvent *pEvent = new BtEvent;
    max_a2dp_conn = config_get_int (config,
            CONFIG_DEFAULT_SECTION, "BtMaxA2dpConn", 1);

    if (bluetooth_interface != NULL)
    {
        sBtA2dpSinkInterface = (btav_interface_t *)bluetooth_interface->
                get_profile_interface(BT_PROFILE_ADVANCED_AUDIO_SINK_ID);
        sBtA2dpSinkVendorInterface = (btav_sink_vendor_interface_t *)bluetooth_interface->
                get_profile_interface(BT_PROFILE_ADVANCED_AUDIO_SINK_VENDOR_ID);

        if (sBtA2dpSinkInterface == NULL)
        {
             pEvent->profile_start_event.event_id = PROFILE_EVENT_START_DONE;
             pEvent->profile_start_event.profile_id = PROFILE_ID_A2DP_SINK;
             pEvent->profile_start_event.status = false;
             PostMessage(THREAD_ID_GAP, pEvent);
             return;
        }
        pA2dpSink->mSinkState = SINK_STATE_STARTED;
        pA2dpSinkStream->fetch_rtp_info = config_get_bool (config,
                         CONFIG_DEFAULT_SECTION, "BtFetchRTPForSink", false);
        ALOGD(LOGTAG " Fetch RTP Info %d", pA2dpSinkStream->fetch_rtp_info);
#ifdef USE_LIBHW_AOSP
        sBtA2dpSinkInterface->init(&sBluetoothA2dpSinkCallbacks);
#else
        sBtA2dpSinkInterface->init(&sBluetoothA2dpSinkCallbacks, max_a2dp_conn, 0);
#endif
        if (pA2dpSinkStream->fetch_rtp_info) {
            sBtA2dpSinkVendorInterface->init_vendor(&sBluetoothA2dpSinkVendorCallbacks,
                    max_a2dp_conn, 0,
                    A2DP_SINK_ENABLE_SBC_DECODING|A2DP_SINK_RETREIVE_RTP_HEADER);
        } else {
            sBtA2dpSinkVendorInterface->init_vendor(&sBluetoothA2dpSinkVendorCallbacks,
                    max_a2dp_conn, 0,
                    A2DP_SINK_ENABLE_SBC_DECODING);
        }
        pEvent->profile_start_event.event_id = PROFILE_EVENT_START_DONE;
        pEvent->profile_start_event.profile_id = PROFILE_ID_A2DP_SINK;
        pEvent->profile_start_event.status = true;
        pA2dpSinkStream->GetLibInterface(sBtA2dpSinkVendorInterface);

        PostMessage(THREAD_ID_GAP, pEvent);
    }
    BtEvent *pEnableSinkStreaming = new BtEvent;
    pEnableSinkStreaming->a2dpSinkStreamingEvent.event_id = A2DP_SINK_STREAMING_API_START;
    if (pA2dpSinkStream) {
        thread_post(pA2dpSinkStream->threadInfo.thread_id,
        pA2dpSinkStream->threadInfo.thread_handler, (void*)pEnableSinkStreaming);
    }
}

void A2dp_Sink::HandleDisableSink(void) {
   ALOGD(LOGTAG " HandleDisableSink ");
   pA2dpSink->mSinkState = SINK_STATE_NOT_STARTED;

   BtEvent *pDisableSinkStreaming = new BtEvent;
   pDisableSinkStreaming->a2dpSinkStreamingEvent.event_id = A2DP_SINK_STREAMING_API_STOP;
   if (pA2dpSinkStream) {
       thread_post(pA2dpSinkStream->threadInfo.thread_id,
       pA2dpSinkStream->threadInfo.thread_handler, (void*)pDisableSinkStreaming);
   }
}

void A2dp_Sink::HandleSinkStreamingDisableDone(void) {
    ALOGD(LOGTAG " HandleSinkStreamingDisableDone ");

   if(sBtA2dpSinkInterface != NULL) {
       sBtA2dpSinkInterface->cleanup();
       sBtA2dpSinkInterface = NULL;
   }
   if(sBtA2dpSinkVendorInterface != NULL) {
       sBtA2dpSinkVendorInterface->cleanup_vendor();
       sBtA2dpSinkVendorInterface = NULL;
   }

   BtEvent *pEvent = new BtEvent;
   pEvent->profile_stop_event.event_id = PROFILE_EVENT_STOP_DONE;
   pEvent->profile_stop_event.profile_id = PROFILE_ID_A2DP_SINK;
   pEvent->profile_stop_event.status = true;
   PostMessage(THREAD_ID_GAP, pEvent);
}

void A2dp_Sink::ProcessEvent(BtEvent* pEvent, list<A2dp_Device>::iterator iter) {
    switch(iter->mSinkDeviceState) {
        case DEVICE_STATE_DISCONNECTED:
            state_disconnected_handler(pEvent, iter);
            break;
        case DEVICE_STATE_PENDING:
            state_pending_handler(pEvent, iter);
            break;
        case DEVICE_STATE_CONNECTED:
            state_connected_handler(pEvent, iter);
            break;
    }
}

void A2dp_Sink::ConnectionManager(BtEvent* pEvent, bt_bdaddr_t dev) {
    ALOGD(LOGTAG " ConnectionManager ");
    A2dp_Device *newNode = NULL;
    list<A2dp_Device>::iterator iter;

    switch(pEvent->event_id) {
        case A2DP_SINK_API_CONNECT_REQ:
            if (pA2dpDeviceList.size() == max_a2dp_conn) {
                ALOGE(LOGTAG " already max devices connected");
                cout << "Already " << max_a2dp_conn << " device connected "<<endl;
                return;
            }
            if (pA2dpDeviceList.size() < max_a2dp_conn) {
                ALOGD(LOGTAG " pA2dpDeviceList.size() < max_a2dp_conn ");
                pA2dpDeviceList.push_back(A2dp_Device(config, dev));
                iter = pA2dpDeviceList.end();
                --iter;
            }
            break;
        case A2DP_SINK_CONNECTING_CB:
        case A2DP_SINK_CONNECTED_CB:
            iter = FindDeviceByAddr(pA2dpDeviceList, dev);
            if (iter != pA2dpDeviceList.end())
            {
                ALOGD(LOGTAG " found a match, donot alloc new");
            }
            else if (pA2dpDeviceList.size() < max_a2dp_conn)
            {
                ALOGD(LOGTAG " reached end of list without a match, alloc new");
                pA2dpDeviceList.push_back(A2dp_Device(config, dev));
                iter = pA2dpDeviceList.end();
                --iter;
            }
            else
            {
                ALOGE(LOGTAG " already max devices connected");
                cout << "Already " << max_a2dp_conn << " device connected "<<endl;
                return;
            }
            break;
        case A2DP_SINK_API_DISCONNECT_REQ:
        case A2DP_SINK_DISCONNECTING_CB:
        case A2DP_SINK_DISCONNECTED_CB:
            if (pA2dpDeviceList.size() == 0) {
                ALOGE(LOGTAG " no device to disconnect");
                cout << "No device connected "<<endl;
                return;
            }
            else
            {
                iter = FindDeviceByAddr(pA2dpDeviceList, dev);
                if (iter == pA2dpDeviceList.end())
                {
                    ALOGE(LOGTAG " reached end of list without a match, cannot disconnect");
                    return;
                }
                else
                {
                    ALOGE(LOGTAG " found a match, disconnect this device iter = %x", iter);
                }
            }
            break;
    }
    ProcessEvent(pEvent, iter);
}

void A2dp_Sink::EventManager(BtEvent* pEvent, bt_bdaddr_t dev) {
    ALOGD(LOGTAG " EventManager ");

    if (pA2dpSink->mSinkState == SINK_STATE_NOT_STARTED)
    {
       ALOGE(LOGTAG " SINK STATE UNINITIALIZED, return");
       return;
    }

    if(isConnectionEvent(pEvent->event_id))
    {
        ConnectionManager(pEvent, dev);
    }
    else
    {
        list<A2dp_Device>::iterator iter = FindDeviceByAddr(pA2dpDeviceList, dev);
        if (iter != pA2dpDeviceList.end())
        {
            ProcessEvent(pEvent, iter);
        }
        else
        {
            ALOGE(LOGTAG " no matching device ignore process event");
        }
    }
}

bool A2dp_Sink::isConnectionEvent(BluetoothEventId event_id) {
    bool ret = false;
    if (event_id >= A2DP_SINK_API_CONNECT_REQ && event_id <= A2DP_SINK_DISCONNECTING_CB)
        ret = true;
    ALOGD(LOGTAG " isConnectionEvent: %d", ret);
    return ret;
}
char* A2dp_Sink::dump_message(BluetoothEventId event_id) {
    switch(event_id) {
    case A2DP_SINK_API_CONNECT_REQ:
        return"API_CONNECT_REQ";
    case A2DP_SINK_API_DISCONNECT_REQ:
        return "API_DISCONNECT_REQ";
    case A2DP_SINK_DISCONNECTED_CB:
        return "DISCONNECTED_CB";
    case A2DP_SINK_CONNECTING_CB:
        return "CONNECING_CB";
    case A2DP_SINK_CONNECTED_CB:
        return "CONNECTED_CB";
    case A2DP_SINK_DISCONNECTING_CB:
        return "DISCONNECTING_CB";
    case A2DP_SINK_FOCUS_REQUEST_CB:
        return "FOCUS_REQUEST_CB";
    case A2DP_SINK_AUDIO_SUSPENDED:
        return "AUDIO_SUSPENDED_CB";
    case A2DP_SINK_AUDIO_STOPPED:
        return "AUDIO_STOPPED_CB";
    case A2DP_SINK_AUDIO_STARTED:
        return "AUDIO_STARTED_CB";
    case A2DP_SINK_CODEC_CONFIG:
        return "A2DP_SINK_CODEC_CONFIG";
    }
    return "UNKNOWN";
}

void A2dp_Sink::state_disconnected_handler(BtEvent* pEvent, list<A2dp_Device>::iterator iter) {
    char str[18];
    BtEvent *pOpenInputStream = NULL;
    ALOGD(LOGTAG "state_disconnected_handler Processing event %s", dump_message(pEvent->event_id));
    switch(pEvent->event_id) {
        case A2DP_SINK_API_CONNECT_REQ:
            memcpy(&iter->mConnectingDevice, &iter->mDevice, sizeof(bt_bdaddr_t));
            if (sBtA2dpSinkInterface != NULL) {
                sBtA2dpSinkInterface->connect(&iter->mDevice);
            }
            change_state(iter, DEVICE_STATE_PENDING);
            break;
        case A2DP_SINK_CONNECTING_CB:
            memcpy(&iter->mConnectingDevice, &iter->mDevice, sizeof(bt_bdaddr_t));
            bdaddr_to_string(&iter->mConnectingDevice, str, 18);
            cout << "A2DP Sink Connecting to " << str << endl;
            change_state(iter, DEVICE_STATE_PENDING);
            break;
        case A2DP_SINK_CONNECTED_CB:
            memset(&iter->mConnectingDevice, 0, sizeof(bt_bdaddr_t));
            memcpy(&iter->mConnectedDevice, &iter->mDevice, sizeof(bt_bdaddr_t));
            bdaddr_to_string(&iter->mConnectedDevice, str, 18);
            cout << "A2DP Sink Connected to " << str << endl;
            change_state(iter, DEVICE_STATE_CONNECTED);
            pOpenInputStream = new BtEvent;
            pOpenInputStream->a2dpSinkStreamingEvent.event_id =
                    A2DP_SINK_STREAMING_OPEN_INPUT_STREAM;
            if (pA2dpSinkStream) {
                thread_post(pA2dpSinkStream->threadInfo.thread_id,
                pA2dpSinkStream->threadInfo.thread_handler, (void*)pOpenInputStream);
            }
            break;
        default:
            ALOGD(LOGTAG " event not handled %d ", pEvent->event_id);
            break;
    }
}
void A2dp_Sink::state_pending_handler(BtEvent* pEvent, list<A2dp_Device>::iterator iter) {
    char str[18];
    BtEvent *pOpenInputStream = NULL;
    ALOGD(LOGTAG " state_pending_handler Processing event %s", dump_message(pEvent->event_id));
    switch(pEvent->event_id) {
        case A2DP_SINK_CONNECTING_CB:
            ALOGD(LOGTAG " dummy event A2DP_SINK_CONNECTING_CB");
            break;
        case A2DP_SINK_CONNECTED_CB:
            memcpy(&iter->mConnectedDevice, &iter->mDevice, sizeof(bt_bdaddr_t));
            memset(&iter->mConnectingDevice, 0, sizeof(bt_bdaddr_t));
            bdaddr_to_string(&iter->mConnectedDevice, str, 18);
            cout << "A2DP Sink Connected to " << str << endl;
            change_state(iter, DEVICE_STATE_CONNECTED);
            pOpenInputStream = new BtEvent;
            pOpenInputStream->a2dpSinkStreamingEvent.event_id =
                    A2DP_SINK_STREAMING_OPEN_INPUT_STREAM;
            if (pA2dpSinkStream) {
                thread_post(pA2dpSinkStream->threadInfo.thread_id,
                pA2dpSinkStream->threadInfo.thread_handler, (void*)pOpenInputStream);
            }
            break;
        case A2DP_SINK_DISCONNECTED_CB:
            cout << "A2DP Sink DisConnected "<< endl;
            memset(&iter->mConnectedDevice, 0, sizeof(bt_bdaddr_t));
            memset(&iter->mConnectingDevice, 0, sizeof(bt_bdaddr_t));

            change_state(iter, DEVICE_STATE_DISCONNECTED);
            break;
        case A2DP_SINK_API_CONNECT_REQ:
            bdaddr_to_string(&iter->mConnectingDevice, str, 18);
            cout << "A2DP Sink Connecting to " << str << endl;
            break;
        case A2DP_SINK_DISCONNECTING_CB:
            ALOGD(LOGTAG " dummy event A2DP_SINK_DISCONNECTING_CB");
            break;
        case A2DP_SINK_CODEC_CONFIG:
            iter->dev_codec_type = pEvent->a2dpSinkEvent.arg1;
            if (pEvent->a2dpSinkEvent.buf_ptr == NULL) {
                break;
            }
            memcpy(&iter->dev_codec_config, pEvent->a2dpSinkEvent.buf_ptr,
                    pEvent->a2dpSinkEvent.buf_size);
            osi_free(pEvent->a2dpSinkEvent.buf_ptr);
            break;
        default:
            ALOGD(LOGTAG " event not handled %d ", pEvent->event_id);
            break;
    }
}

void A2dp_Sink::state_connected_handler(BtEvent* pEvent, list<A2dp_Device>::iterator iter) {
    char str[18];
    uint32_t pcm_data_read = 0;
    BtEvent *pAMReleaseControl = NULL, *pCloseAudioStream = NULL, *pAMRequestControl = NULL;
    ALOGD(LOGTAG " state_connected_handler Processing event %s", dump_message(pEvent->event_id));
    switch(pEvent->event_id) {
        case A2DP_SINK_API_CONNECT_REQ:
            bdaddr_to_string(&iter->mConnectedDevice, str, 18);
            cout << "A2DP Sink Connected to " << str << endl;
            break;
        case A2DP_SINK_API_DISCONNECT_REQ:
            if (!memcmp(&pA2dpSinkStream->mStreamingDevice, &iter->mDevice, sizeof(bt_bdaddr_t)))
            {
                pCloseAudioStream = new BtEvent;
                pCloseAudioStream->a2dpSinkStreamingEvent.event_id =
                        A2DP_SINK_STREAMING_CLOSE_AUDIO_STREAM;
                if (pA2dpSinkStream) {
                    thread_post(pA2dpSinkStream->threadInfo.thread_id,
                    pA2dpSinkStream->threadInfo.thread_handler, (void*)pCloseAudioStream);
                }
            }
            bdaddr_to_string(&iter->mConnectedDevice, str, 18);
            cout << "A2DP Sink DisConnecting from " << str << endl;
            memset(&iter->mConnectedDevice, 0, sizeof(bt_bdaddr_t));
            memset(&iter->mConnectingDevice, 0, sizeof(bt_bdaddr_t));
            if (sBtA2dpSinkInterface != NULL) {
                sBtA2dpSinkInterface->disconnect(&iter->mDevice);
            }
            change_state(iter, DEVICE_STATE_PENDING);
            break;
        case A2DP_SINK_DISCONNECTED_CB:
            if (!memcmp(&pA2dpSinkStream->mStreamingDevice, &iter->mDevice, sizeof(bt_bdaddr_t)))
            {
                pCloseAudioStream = new BtEvent;
                pCloseAudioStream->a2dpSinkStreamingEvent.event_id =
                        A2DP_SINK_STREAMING_CLOSE_AUDIO_STREAM;
                if (pA2dpSinkStream) {
                    thread_post(pA2dpSinkStream->threadInfo.thread_id,
                    pA2dpSinkStream->threadInfo.thread_handler, (void*)pCloseAudioStream);
                }
            }
            memset(&iter->mConnectedDevice, 0, sizeof(bt_bdaddr_t));
            memset(&iter->mConnectingDevice, 0, sizeof(bt_bdaddr_t));
            cout << "A2DP Sink DisConnected " << endl;
            change_state(iter, DEVICE_STATE_DISCONNECTED);
            break;
        case A2DP_SINK_DISCONNECTING_CB:
            if (!memcmp(&pA2dpSinkStream->mStreamingDevice, &iter->mDevice, sizeof(bt_bdaddr_t)))
            {
                pCloseAudioStream = new BtEvent;
                pCloseAudioStream->a2dpSinkStreamingEvent.event_id =
                        A2DP_SINK_STREAMING_CLOSE_AUDIO_STREAM;
                if (pA2dpSinkStream) {
                    thread_post(pA2dpSinkStream->threadInfo.thread_id,
                    pA2dpSinkStream->threadInfo.thread_handler, (void*)pCloseAudioStream);
                }
            }
            cout << "A2DP Sink DisConnecting " << endl;
            change_state(iter, DEVICE_STATE_PENDING);
            break;
        case A2DP_SINK_CODEC_CONFIG:
             iter->dev_codec_type = pEvent->a2dpSinkEvent.arg1;
             if (pEvent->a2dpSinkEvent.buf_ptr == NULL) {
                 break;
             }
             memcpy(&iter->dev_codec_config, pEvent->a2dpSinkEvent.buf_ptr,
                     pEvent->a2dpSinkEvent.buf_size);
             osi_free(pEvent->a2dpSinkEvent.buf_ptr);
             if (iter->dev_codec_type == A2DP_SINK_AUDIO_CODEC_SBC) {
                 iter->av_config.sample_rate = pA2dpSinkStream->
                     get_a2dp_sbc_sampling_rate(iter->dev_codec_config.sbc_config.samp_freq);
                 iter->av_config.channel_count = pA2dpSinkStream->
                     get_a2dp_sbc_channel_mode(iter->dev_codec_config.sbc_config.ch_mode);
             }
             break;
        case A2DP_SINK_AUDIO_STARTED:
        case A2DP_SINK_FOCUS_REQUEST_CB:
            bdaddr_to_string(&pA2dpSinkStream->mStreamingDevice, str, 18);
            ALOGD(LOGTAG " current streaming device %s", str);
            
            if (memcmp(&pA2dpSinkStream->mStreamingDevice, &bd_addr_null, sizeof(bt_bdaddr_t)) &&
                    memcmp(&pA2dpSinkStream->mStreamingDevice, &iter->mDevice, sizeof(bt_bdaddr_t)))
            {
                ALOGD(LOGTAG " another dev started streaming, pause previous one");
                if (pAvrcp != NULL)
                    pAvrcp->SendPassThruCommandNative(CMD_ID_PAUSE,
                            &pA2dpSinkStream->mStreamingDevice, 1);
                if (pA2dpSinkStream && !pA2dpSinkStream->use_bt_a2dp_hal)
                {
                    memset(&pA2dpSinkStream->mStreamingDevice, 0, sizeof(bt_bdaddr_t));
                    pAMReleaseControl = new BtEvent;
                    pAMReleaseControl->a2dpSinkStreamingEvent.event_id =
                            A2DP_SINK_STREAMING_AM_RELEASE_CONTROL;
                    if (pA2dpSinkStream) {
                        thread_post(pA2dpSinkStream->threadInfo.thread_id,
                        pA2dpSinkStream->threadInfo.thread_handler, (void*)pAMReleaseControl);
                    }
                }
                else
                {
                    pCloseAudioStream = new BtEvent;
                    pCloseAudioStream->a2dpSinkStreamingEvent.event_id =
                            A2DP_SINK_STREAMING_CLOSE_AUDIO_STREAM;
                    if (pA2dpSinkStream) {
                        thread_post(pA2dpSinkStream->threadInfo.thread_id,
                        pA2dpSinkStream->threadInfo.thread_handler, (void*)pCloseAudioStream);
                    }
                }
            }
            ALOGE(LOGTAG " updating avconfig parameters for this device");
            if (iter->dev_codec_type == A2DP_SINK_AUDIO_CODEC_SBC) {
                pA2dpSinkStream->sample_rate = iter->av_config.sample_rate;
                pA2dpSinkStream->channel_count = iter->av_config.channel_count;
            }
            pA2dpSinkStream->codec_type = iter->dev_codec_type;
            memcpy(&pA2dpSinkStream->codec_config, &iter->dev_codec_config,
                    sizeof(btav_codec_config_t));

            memcpy(&pA2dpSinkStream->mStreamingDevice, &pEvent->a2dpSinkEvent.bd_addr,
                    sizeof(bt_bdaddr_t));
            bdaddr_to_string(&pA2dpSinkStream->mStreamingDevice, str, 18);
            ALOGD(LOGTAG " A2DP_SINK_AUDIO_STARTED - set current streaming device as %s", str);

            sBtA2dpSinkVendorInterface->
                    update_streaming_device_vendor(&pA2dpSinkStream->mStreamingDevice);

            pAMRequestControl = new BtEvent;
            pAMRequestControl->a2dpSinkStreamingEvent.event_id =
                    A2DP_SINK_STREAMING_AM_REQUEST_CONTROL;
            if (pA2dpSinkStream) {
                thread_post(pA2dpSinkStream->threadInfo.thread_id,
                pA2dpSinkStream->threadInfo.thread_handler, (void*)pAMRequestControl);
            }
            break;
        case A2DP_SINK_AUDIO_SUSPENDED:
        case A2DP_SINK_AUDIO_STOPPED:
            if(memcmp(&pA2dpSinkStream->mStreamingDevice, &pEvent->a2dpSinkEvent.bd_addr,
                    sizeof(bt_bdaddr_t)))
            {
                ALOGD(LOGTAG " A2DP_SINK_AUDIO_SUSPENDED/STOPPED for non streaming device, ignore");
                break;
            }
            memset(&pA2dpSinkStream->mStreamingDevice, 0, sizeof(bt_bdaddr_t));

            pAMReleaseControl = new BtEvent;
            pAMReleaseControl->a2dpSinkStreamingEvent.event_id =
                    A2DP_SINK_STREAMING_AM_RELEASE_CONTROL;
            if (pA2dpSinkStream) {
                thread_post(pA2dpSinkStream->threadInfo.thread_id,
                pA2dpSinkStream->threadInfo.thread_handler, (void*)pAMReleaseControl);
            }
            break;
        default:
            ALOGD(LOGTAG " event not handled %d ", pEvent->event_id);
            break;
    }
}

void A2dp_Sink::change_state(list<A2dp_Device>::iterator iter, A2dpSinkDeviceState mState) {
   BtEvent *pA2dpSinkDisconnected = NULL;
   ALOGD(LOGTAG " current State = %d, new state = %d", iter->mSinkDeviceState, mState);
   pthread_mutex_lock(&lock);
   iter->mSinkDeviceState = mState;
   if (iter->mSinkDeviceState == DEVICE_STATE_DISCONNECTED)
   {
       if (!memcmp(&pA2dpSinkStream->mStreamingDevice, &iter->mDevice, sizeof(bt_bdaddr_t)))
       {
           memset(&pA2dpSinkStream->mStreamingDevice, 0, sizeof(bt_bdaddr_t));

           pA2dpSinkDisconnected = new BtEvent;
           pA2dpSinkDisconnected->a2dpSinkStreamingEvent.event_id =
                   A2DP_SINK_STREAMING_DISCONNECTED;
           if (pA2dpSinkStream) {
               thread_post(pA2dpSinkStream->threadInfo.thread_id,
               pA2dpSinkStream->threadInfo.thread_handler, (void*)pA2dpSinkDisconnected);
           }
       }
       pA2dpDeviceList.erase(iter);
       ALOGD(LOGTAG " iter %x deleted from list", iter);
   }
   ALOGD(LOGTAG " iter %x state changes to %d ", iter, mState);
   pthread_mutex_unlock(&lock);
}

A2dp_Sink :: A2dp_Sink(const bt_interface_t *bt_interface, config_t *config) {

    this->bluetooth_interface = bt_interface;
    this->config = config;
    sBtA2dpSinkInterface = NULL;
    mSinkState = SINK_STATE_NOT_STARTED;
    pthread_mutex_init(&this->lock, NULL);
    pA2dpSinkStream = new A2dp_Sink_Streaming(config);
    pA2dpSinkStream->threadInfo.thread_id = thread_new (pA2dpSinkStream->threadInfo.thread_name);
    max_a2dp_conn = 0;
}

A2dp_Sink :: ~A2dp_Sink() {
    pthread_mutex_destroy(&lock);
}

A2dp_Device :: A2dp_Device(config_t *config, bt_bdaddr_t dev) {
    this->config = config;
    memset(&mDevice, 0, sizeof(bt_bdaddr_t));
    memcpy(&mDevice, &dev, sizeof(bt_bdaddr_t));
    memset(&mConnectedDevice, 0, sizeof(bt_bdaddr_t));
    memset(&mConnectingDevice, 0, sizeof(bt_bdaddr_t));
    mSinkDeviceState = DEVICE_STATE_DISCONNECTED;
    memset(&av_config, 0, sizeof(A2dpSinkConfig_t));
    pthread_mutex_init(&this->lock, NULL);
    mAvrcpConnected = false;
}

A2dp_Device :: ~A2dp_Device() {
    pthread_mutex_destroy(&lock);
}

