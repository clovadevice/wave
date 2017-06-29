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
#include <hardware/bt_rc.h>

#include "Avrcp.hpp"
#include "A2dp_Sink_Streaming.hpp"
#include "Gap.hpp"
#include "hardware/bt_rc_vendor.h"
#include "A2dp_Sink.hpp"

#define LOGTAG "AVRCP"
#define LOGTAG_CTRL "AVRCP_CTRL"

using namespace std;
using std::list;
using std::string;

Avrcp *pAvrcp = NULL;
extern A2dp_Sink_Streaming *pA2dpSinkStream;
extern A2dp_Sink *pA2dpSink;

static const bt_bdaddr_t bd_addr_null= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#ifdef __cplusplus
extern "C" {
#endif

void BtAvrcpMsgHandler(void *msg) {
    BtEvent* pEvent = NULL;
    BtEvent* pCleanupEvent = NULL;
    if(!msg) {
        printf("Msg is NULL, return.\n");
        return;
    }

    pEvent = ( BtEvent *) msg;
    switch(pEvent->event_id) {
        case PROFILE_API_START:
            ALOGD(LOGTAG " enable avrcp");
            if (pAvrcp) {
                pAvrcp->HandleEnableAvrcp();
            }
            break;
        case PROFILE_API_STOP:
            ALOGD(LOGTAG " disable avrcp");
            if (pAvrcp) {
                pAvrcp->HandleDisableAvrcp();
            }
            break;
        case AVRCP_CLEANUP_REQ:
            ALOGD(LOGTAG " cleanup a2dp avrcp");
            pCleanupEvent = new BtEvent;
            pCleanupEvent->event_id = AVRCP_CLEANUP_DONE;
            PostMessage(THREAD_ID_GAP, pCleanupEvent);
            break;
        case AVRCP_CTRL_CONNECTED_CB:
        case AVRCP_CTRL_DISCONNECTED_CB:
        case AVRCP_CTRL_PASS_THRU_CMD_REQ:
            ALOGD( LOGTAG_CTRL " handle avrcp events ");

            if (pAvrcp) {
                pAvrcp->HandleAvrcpEvents(( BtEvent *) msg);
            }
            break;
        default:
            break;
    }
    delete pEvent;
}

#ifdef __cplusplus
}
#endif


static void btavrcpctrl_passthru_rsp_callback(int id, int key_state) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_passthru_rsp_callback id = %d key_state = %d", id, key_state);
}

static void btavrcpctrl_connection_state_callback(bool state, bt_bdaddr_t* bd_addr) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_connection_state_callback state = %d", state);
    BtEvent *pEvent = new BtEvent;
    memcpy(&pEvent->avrcpCtrlEvent.bd_addr, bd_addr, sizeof(bt_bdaddr_t));
    if (state == true)
        pEvent->avrcpCtrlEvent.event_id = AVRCP_CTRL_CONNECTED_CB;
    else
        pEvent->avrcpCtrlEvent.event_id = AVRCP_CTRL_DISCONNECTED_CB;
    PostMessage(THREAD_ID_AVRCP, pEvent);
}

static void btavrcpctrl_rcfeatures_vendor_callback( bt_bdaddr_t* bd_addr, int features) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_rcfeatures_vendor_callback features = %d", features);
}

static void btavrcpctrl_getcap_rsp_vendor_callback( bt_bdaddr_t *bd_addr, int cap_id,
                uint32_t* supported_values, int num_supported, uint8_t rsp_type) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_getcap_rsp_vendor_callback");
}

static void btavrcpctrl_listplayerappsettingattrib_rsp_vendor_callback( bt_bdaddr_t *bd_addr,
                          uint8_t* supported_attribs, int num_attrib, uint8_t rsp_type) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_listplayerappsettingattrib_rsp_vendor_callback");
}

static void btavrcpctrl_listplayerappsettingvalue_rsp_vendor_callback( bt_bdaddr_t *bd_addr,
                       uint8_t* supported_val, uint8_t num_supported, uint8_t rsp_type) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_listplayerappsettingvalue_rsp_vendor_callback");
}

static void btavrcpctrl_currentplayerappsetting_rsp_vendor_callback( bt_bdaddr_t *bd_addr,
        uint8_t* supported_ids, uint8_t* supported_val, uint8_t num_attrib, uint8_t rsp_type) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_currentplayerappsetting_rsp_vendor_callback");
}

static void btavrcpctrl_setplayerappsetting_rsp_vendor_callback( bt_bdaddr_t *bd_addr,uint8_t rsp_type) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_setplayerappsetting_rsp_vendor_callback");
}

static void btavrcpctrl_notification_rsp_vendor_callback( bt_bdaddr_t *bd_addr, uint8_t rsp_type,
        int rsp_len, uint8_t* notification_rsp) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_notification_rsp_vendor_callback");
}

static void btavrcpctrl_getelementattrib_rsp_vendor_callback(bt_bdaddr_t *bd_addr, uint8_t num_attributes,
       int rsp_len, uint8_t* attrib_rsp, uint8_t rsp_type) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_getelementattrib_rsp_vendor_callback");
}

static void btavrcpctrl_getplaystatus_rsp_vendor_callback(bt_bdaddr_t *bd_addr, int param_len,
        uint8_t* play_status_rsp, uint8_t rsp_type) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_getplaystatus_rsp_vendor_callback");
}

static void btavrcpctrl_setabsvol_cmd_vendor_callback(bt_bdaddr_t *bd_addr, uint8_t abs_vol) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_setabsvol_cmd_vendor_callback");
}

static void btavrcpctrl_registernotification_absvol_vendor_callback(bt_bdaddr_t *bd_addr) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_registernotification_absvol_vendor_callback");
}

static btrc_ctrl_callbacks_t sBluetoothAvrcpCtrlCallbacks = {
   sizeof(sBluetoothAvrcpCtrlCallbacks),
   btavrcpctrl_passthru_rsp_callback,
   btavrcpctrl_connection_state_callback,
};

static btrc_ctrl_vendor_callbacks_t sBluetoothAvrcpCtrlVendorCallbacks = {
   sizeof(sBluetoothAvrcpCtrlVendorCallbacks),
   btavrcpctrl_rcfeatures_vendor_callback,
   btavrcpctrl_getcap_rsp_vendor_callback,
   btavrcpctrl_listplayerappsettingattrib_rsp_vendor_callback,
   btavrcpctrl_listplayerappsettingvalue_rsp_vendor_callback,
   btavrcpctrl_currentplayerappsetting_rsp_vendor_callback,
   btavrcpctrl_setplayerappsetting_rsp_vendor_callback,
   btavrcpctrl_notification_rsp_vendor_callback,
   btavrcpctrl_getelementattrib_rsp_vendor_callback,
   btavrcpctrl_getplaystatus_rsp_vendor_callback,
   btavrcpctrl_setabsvol_cmd_vendor_callback,
   btavrcpctrl_registernotification_absvol_vendor_callback,
};

void Avrcp::SendPassThruCommandNative(uint8_t key_id, bt_bdaddr_t* addr, uint8_t direct) {

    if (memcmp(&pA2dpSinkStream->mStreamingDevice, &bd_addr_null, sizeof(bt_bdaddr_t)) &&
            memcmp(&pA2dpSinkStream->mStreamingDevice, addr, sizeof(bt_bdaddr_t)) &&
            (key_id == CMD_ID_PLAY))
        direct = 1;

    if (!direct && pA2dpSinkStream && pA2dpSinkStream->use_bt_a2dp_hal &&
            ((key_id == CMD_ID_PAUSE) || (key_id == CMD_ID_PLAY))) {
        ALOGD(LOGTAG_CTRL " SendPassThruCommandNative: using bt_a2dp_hal ");
        if (CMD_ID_PAUSE == key_id)
        {
            ALOGD( LOGTAG_CTRL " sending stopped event ");

            if (!memcmp(&pA2dpSinkStream->mStreamingDevice, addr, sizeof(bt_bdaddr_t)))
            {
                pA2dpSinkStream->StopDataFetchTimer();
                pA2dpSinkStream->SuspendInputStream();
            }

        }
        if (CMD_ID_PLAY == key_id)
        {
            ALOGD( LOGTAG_CTRL " sending started event ");
            BtEvent *pEvent = new BtEvent;
            pEvent->a2dpSinkEvent.event_id = A2DP_SINK_AUDIO_STARTED;
            memcpy(&pEvent->a2dpSinkEvent.bd_addr, addr, sizeof(bt_bdaddr_t));
            PostMessage(THREAD_ID_A2DP_SINK, pEvent);
        }
    }
    else if (sBtAvrcpCtrlInterface != NULL) {
        ALOGD(LOGTAG_CTRL " SendPassThruCommandNative send_pass_through_cmd");

        sBtAvrcpCtrlInterface->send_pass_through_cmd(addr, key_id, 0);
        sBtAvrcpCtrlInterface->send_pass_through_cmd(addr, key_id, 1);
    }
}

list<A2dp_Device>::iterator FindAvDeviceByAddr(list<A2dp_Device>& pA2dpDev, bt_bdaddr_t dev) {
    list<A2dp_Device>::iterator p = pA2dpDev.begin();
    while(p != pA2dpDev.end()) {
        if (memcmp(&dev, &p->mDevice, sizeof(bt_bdaddr_t)) == 0) {
            break;
        }
        p++;
    }
    return p;
}

void Avrcp::HandleAvrcpEvents(BtEvent* pEvent) {
    list<A2dp_Device>::iterator iter;
    ALOGD(LOGTAG_CTRL " HandleAvrcpEvents event = %s",
            dump_message(pEvent->avrcpCtrlEvent.event_id));
    switch(pEvent->avrcpCtrlEvent.event_id) {
    case AVRCP_CTRL_CONNECTED_CB:
        iter = FindAvDeviceByAddr(pA2dpSink->pA2dpDeviceList, pEvent->avrcpCtrlEvent.bd_addr);
        if (iter != pA2dpSink->pA2dpDeviceList.end())
        {
            ALOGD(LOGTAG_CTRL " Rc connection for AV connected dev, mark avrcp connected");
            iter->mAvrcpConnected = true;
        }
        else
        {
            ALOGE(LOGTAG_CTRL " Rc connection from device without AV connection");
        }
        break;
    case AVRCP_CTRL_DISCONNECTED_CB:
        iter = FindAvDeviceByAddr(pA2dpSink->pA2dpDeviceList, pEvent->avrcpCtrlEvent.bd_addr);
        if (iter != pA2dpSink->pA2dpDeviceList.end())
        {
            ALOGD(LOGTAG_CTRL " Rc disconnection for AV connected dev, mark avrcp disconnected");
            iter->mAvrcpConnected = false;
        }
        else
        {
            ALOGE(LOGTAG_CTRL " Rc disconnection from device without AV connection");
        }
        break;
    case AVRCP_CTRL_PASS_THRU_CMD_REQ:
        iter = FindAvDeviceByAddr(pA2dpSink->pA2dpDeviceList, pEvent->avrcpCtrlEvent.bd_addr);
        if (iter != pA2dpSink->pA2dpDeviceList.end() && (iter->mAvrcpConnected == true))
        {
            ALOGD(LOGTAG_CTRL " passthrough cmd for AV & RC connected device, send to stack");
            SendPassThruCommandNative(pEvent->avrcpCtrlEvent.key_id,
            &pEvent->avrcpCtrlEvent.bd_addr, 0);
        }
        else
        {
            ALOGD(LOGTAG_CTRL " Avrcp not connected or AV not connected");
        }
        break;
    }
}

void Avrcp::HandleEnableAvrcp(void) {
    BtEvent *pEvent = new BtEvent;
    ALOGD(LOGTAG_CTRL " HandleEnableAvrcp ");

    max_avrcp_conn = config_get_int (config,
            CONFIG_DEFAULT_SECTION, "BtMaxA2dpConn", 1);

    if (bluetooth_interface != NULL)
    {
        // AVRCP CT Initialization
        sBtAvrcpCtrlInterface = (btrc_ctrl_interface_t *)bluetooth_interface->
                get_profile_interface(BT_PROFILE_AV_RC_CTRL_ID);

        // AVRCP CT Vendor Initialization
        sBtAvrcpCtrlVendorInterface = (btrc_ctrl_vendor_interface_t *)bluetooth_interface->
                get_profile_interface(BT_PROFILE_AV_RC_CTRL_VENDOR_ID);

        if (sBtAvrcpCtrlInterface == NULL || sBtAvrcpCtrlVendorInterface == NULL)
        {
             pEvent->profile_start_event.event_id = PROFILE_EVENT_START_DONE;
             pEvent->profile_start_event.profile_id = PROFILE_ID_AVRCP;
             pEvent->profile_start_event.status = false;
             PostMessage(THREAD_ID_GAP, pEvent);
             return;
        }

        if (sBtAvrcpCtrlInterface != NULL) {
            sBtAvrcpCtrlInterface->init(&sBluetoothAvrcpCtrlCallbacks);
        }

        if (sBtAvrcpCtrlVendorInterface != NULL) {
            sBtAvrcpCtrlVendorInterface->
                    init_vendor(&sBluetoothAvrcpCtrlVendorCallbacks, max_avrcp_conn);
        }

        pEvent->profile_start_event.event_id = PROFILE_EVENT_START_DONE;
        pEvent->profile_start_event.profile_id = PROFILE_ID_AVRCP;
        pEvent->profile_start_event.status = true;

        PostMessage(THREAD_ID_GAP, pEvent);
    }
}

void Avrcp::HandleDisableAvrcp(void) {
    ALOGD(LOGTAG_CTRL " HandleDisableAvrcp ");

   if (sBtAvrcpCtrlInterface != NULL) {
       sBtAvrcpCtrlInterface->cleanup();
       sBtAvrcpCtrlInterface = NULL;
   }
   if (sBtAvrcpCtrlVendorInterface != NULL) {
       sBtAvrcpCtrlVendorInterface->cleanup_vendor();
       sBtAvrcpCtrlVendorInterface = NULL;
   }
   BtEvent *pEvent = new BtEvent;
   pEvent->profile_stop_event.event_id = PROFILE_EVENT_STOP_DONE;
       pEvent->profile_stop_event.profile_id = PROFILE_ID_AVRCP;
       pEvent->profile_stop_event.status = true;
       PostMessage(THREAD_ID_GAP, pEvent);
}

char* Avrcp::dump_message(BluetoothEventId event_id) {
    switch(event_id) {
    case AVRCP_CTRL_CONNECTED_CB:
        return "AVRCP_CTRL_CONNECTED_CB";
    case AVRCP_CTRL_DISCONNECTED_CB:
        return "AVRCP_CTRL_DISCONNECTED_CB";
    case AVRCP_CTRL_PASS_THRU_CMD_REQ:
        return "PASS_THRU_CMD_REQ";
    }
    return "UNKNOWN";
}

Avrcp :: Avrcp(const bt_interface_t *bt_interface, config_t *config) {
    this->bluetooth_interface = bt_interface;
    this->config = config;
    sBtAvrcpCtrlInterface = NULL;
    max_avrcp_conn = 0;
    memset(&mConnectedAvrcpDevice, 0, sizeof(bt_bdaddr_t));
    pthread_mutex_init(&this->lock, NULL);
}

Avrcp :: ~Avrcp() {
    pthread_mutex_destroy(&lock);
}
