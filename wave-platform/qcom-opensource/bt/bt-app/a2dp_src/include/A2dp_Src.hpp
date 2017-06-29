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

#ifndef A2DP_SOURCE_APP_H
#define A2DP_SOURCE_APP_H

#include <map>
#include <string>
#include <hardware/bluetooth.h>
#include <hardware/bt_av.h>
#include <hardware/bt_rc.h>
#include <pthread.h>
#include "hardware/bt_av_vendor.h"
#include "hardware/bt_rc_vendor.h"

#include "osi/include/log.h"
#include "osi/include/thread.h"
#include "osi/include/config.h"
#include "ipc.h"
#include "utils.h"


typedef enum {
    STATE_A2DP_SOURCE_NOT_STARTED = 0,
    STATE_A2DP_SOURCE_DISCONNECTED,
    STATE_A2DP_SOURCE_PENDING,
    STATE_A2DP_SOURCE_CONNECTED,
}A2dpSourceState;


class A2dp_Source {

  private:
    config_t *config;
    const bt_interface_t * bluetooth_interface;
    const btav_interface_t *sBtA2dpSourceInterface;
    const btrc_interface_t *sBtAvrcpTargetInterface;
    A2dpSourceState mSourceState;
    bool mAvrcpConnected;
    const btav_vendor_interface_t *sBtA2dpSourceVendorInterface;
    const btrc_vendor_interface_t *sBtAvrcpTargetVendorInterface;

  public:
    A2dp_Source(const bt_interface_t *bt_interface, config_t *config);
    ~A2dp_Source();
    void ProcessEvent(BtEvent* pEvent);
    void state_disconnected_handler(BtEvent* pEvent);
    void state_pending_handler(BtEvent* pEvent);
    void state_connected_handler(BtEvent* pEvent);
    void change_state(A2dpSourceState mState);
    char* dump_message(BluetoothEventId event_id);
    pthread_mutex_t lock;
    bt_bdaddr_t mConnectingDevice;
    bt_bdaddr_t mConnectedDevice;
    bt_bdaddr_t mConnectedAvrcpDevice;
    void HandleAvrcpEvents(BtEvent* pEvent);
    void HandleEnableSource();
    void HandleDisableSource();
};

#endif
