/******************************************************************************
 *  Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *  Not a Contribution.
 *
 *  Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *  Not a Contribution.
 *  Copyright (C) 2009-2012 Broadcom Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/


/*****************************************************************************
 *
 *  Filename:      btif_rc.c
 *
 *  Description:   Bluetooth AVRC implementation
 *
 *****************************************************************************/
#include <errno.h>
#include <hardware/bluetooth.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include "bta_api.h"
#include "bta_avk_api.h"
#include "avrc_defs.h"
#include "gki.h"

#define LOG_TAG "bt_btif_avrc"
#include "btif_common.h"
#include "btif_util.h"
#include "btif_avk.h"
#include "hardware/bt_rc.h"
#include "uinput.h"
#include "bdaddr.h"
#include "hardware/bt_rc_vendor.h"

/*****************************************************************************
**  Constants & Macros
******************************************************************************/

/* Support Two RC Handles simultaneously*/
#define BTIF_AVK_RC_NUM_CB       2
/* Default index*/
#define BTIF_AVK_RC_DEFAULT_INDEX 0
/* cod value for Headsets */
#define COD_AV_HEADSETS        0x0404
/* for AVRC 1.4 need to change this */
#define MAX_RC_NOTIFICATIONS AVRC_EVT_VOLUME_CHANGE
//#define TEST_BROWSE_RESPONSE
#define MAX_FOLDER_RSP_SUPPORT 10

#define IDX_GET_PLAY_STATUS_RSP    0
#define IDX_LIST_APP_ATTR_RSP      1
#define IDX_LIST_APP_VALUE_RSP     2
#define IDX_GET_CURR_APP_VAL_RSP   3
#define IDX_SET_APP_VAL_RSP        4
#define IDX_GET_APP_ATTR_TXT_RSP   5
#define IDX_GET_APP_VAL_TXT_RSP    6
#define IDX_GET_ELEMENT_ATTR_RSP   7
#define IDX_GET_FOLDER_ITEMS_RSP   8
#define IDX_SET_FOLDER_ITEM_RSP    9
#define IDX_SET_ADDRESS_PLAYER_RSP 10
#define IDX_SET_BROWSE_PLAYER_RSP  11
#define IDX_CHANGE_PATH_RSP        12
#define IDX_PLAY_ITEM_RSP          13
#define IDX_GET_ITEM_ATTR_RSP      14
#define IDX_GET_TOTAL_ITEMS_RSP    15
#define MAX_VOLUME 128
#define MAX_LABEL 16
#define MAX_TRANSACTIONS_PER_SESSION 16
#define PLAY_STATUS_PLAYING 1
#define MAX_CMD_QUEUE_LEN 16
#define ERR_PLAYER_NOT_ADDRESED 0x13
#define BTRC_FEAT_AVRC_UI_UPDATE 0x08

#if (defined(AVCT_COVER_ART_INCLUDED) && (AVCT_COVER_ART_INCLUDED == TRUE))
#define MAX_ELEM_ATTR_SIZE 8
#else
#define MAX_ELEM_ATTR_SIZE 7
#endif

#define CHECK_AVK_RC_CONNECTED                                                                  \
    int clients;                                                                           \
    int conn_status = BT_STATUS_NOT_READY;                                                      \
    BTIF_TRACE_DEBUG("## %s ##", __FUNCTION__);                                            \
    for (clients = 0; clients < btif_max_rc_clients; clients++)                            \
    {                                                                                      \
        if ((btif_avk_rc_cb[clients].rc_connected == TRUE))                                    \
            conn_status = BT_STATUS_SUCCESS;                                                    \
    }                                                                                      \
    if(conn_status == BT_STATUS_NOT_READY)                                                      \
    {                                                                                      \
        BTIF_TRACE_WARNING("Function %s() called when RC is not connected", __FUNCTION__); \
        return BT_STATUS_NOT_READY;                                                        \
    }

#define TXN_LABEL_ENQUEUE(handle, label, front, rear, size, item, cmd)   \
{                                                                        \
    if (size == MAX_TRANSACTIONS_PER_SESSION)                            \
    {                                                                    \
        send_reject_response(handle, item, cmd, AVRC_STS_INTERNAL_ERR);  \
        break;                                                           \
    }                                                                    \
    rear = (rear + 1) % MAX_TRANSACTIONS_PER_SESSION;                    \
    label[rear] = item;                                                  \
    size = size + 1;                                                     \
}

#define TXN_LABEL_DEQUEUE(label, front, rear, size)        \
{                                                          \
    if (size == 0)                                         \
        return BT_STATUS_UNHANDLED;                        \
    front = (front + 1) % MAX_TRANSACTIONS_PER_SESSION;    \
    size = size - 1;                                       \
}

#define FILL_PDU_QUEUE(idx, ctype, label, pending, index, cmd)                                 \
{                                                                                              \
    btif_avk_rc_cb[index].rc_pdu_info[idx].ctype = ctype;                                          \
    TXN_LABEL_ENQUEUE(btif_avk_rc_cb[index].rc_handle, btif_avk_rc_cb[index].rc_pdu_info[idx].label,   \
            btif_avk_rc_cb[index].rc_pdu_info[idx].front, btif_avk_rc_cb[index].rc_pdu_info[idx].rear, \
            btif_avk_rc_cb[index].rc_pdu_info[idx].size, label, cmd);                              \
    BTIF_TRACE_DEBUG("%s txn label %d enqueued to txn queue of pdu %s, queue size %d \n",      \
            __FUNCTION__, label, dump_rc_pdu(cmd), btif_avk_rc_cb[index].rc_pdu_info[idx].size);   \
    btif_avk_rc_cb[index].rc_pdu_info[idx].is_rsp_pending = pending;                               \
}

/*****************************************************************************
**  Local type definitions
******************************************************************************/
typedef struct {
    UINT8 bNotify;
    UINT8 label;
} btif_avk_rc_reg_notifications_t;

typedef struct
{
    int front;
    int rear;
    int size;
    UINT8 label[MAX_TRANSACTIONS_PER_SESSION];
    UINT8   ctype;
    BOOLEAN is_rsp_pending;
} btif_avk_rc_cmd_ctxt_t;

/* TODO : Merge btif_avk_rc_reg_notifications_t and btif_avk_rc_cmd_ctxt_t to a single struct */
typedef struct {
    BOOLEAN                     rc_connected;
    UINT8                       rc_handle;
    tBTA_AVK_FEAT                rc_features;
    BD_ADDR                     rc_addr;
    UINT16                      rc_pending_play;
    btif_avk_rc_cmd_ctxt_t          rc_pdu_info[MAX_CMD_QUEUE_LEN];
    btif_avk_rc_reg_notifications_t rc_notif[MAX_RC_NOTIFICATIONS];
    unsigned int                rc_volume;
    uint8_t                     rc_vol_label;
    BOOLEAN                     rc_play_processed;
} btif_avk_rc_cb_t;

typedef struct {
    BOOLEAN in_use;
    UINT8 lbl;
    UINT8 handle;
} rc_transaction_t;

typedef struct
{
    pthread_mutex_t lbllock;
    rc_transaction_t transaction[MAX_TRANSACTIONS_PER_SESSION];
    BOOLEAN lbllock_destroyed;
} rc_device_t;


rc_device_t device;

#define MAX_UINPUT_PATHS 3
static int btif_max_rc_clients = 1;
static const char* uinput_dev_path[] =
                       {"/dev/uinput", "/dev/input/uinput", "/dev/misc/uinput" };
static int uinput_fd = -1;
static BD_ADDR bd_null= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static int  send_event (int fd, uint16_t type, uint16_t code, int32_t value);
static void send_key (int fd, uint16_t key, int pressed);
static int  uinput_driver_check();
static int  uinput_create(char *name);
static int  init_uinput (void);
static void close_uinput (void);
static bt_status_t send_passthrough_cmd(bt_bdaddr_t *bd_addr, uint8_t key_code, uint8_t key_state);
static UINT8 btif_avk_rc_idx_by_bdaddr( BD_ADDR bd_addr);
#if (AVRC_CTLR_INCLUDED == TRUE)
static BOOLEAN conn_status = FALSE;
#endif

static const struct {
    const char *name;
    uint8_t avrcp;
    uint16_t mapped_id;
    uint8_t release_quirk;
} key_map[] = {
    { "PLAY",         AVRC_ID_PLAY,     KEY_PLAYCD,       1 },
    { "STOP",         AVRC_ID_STOP,     KEY_STOPCD,       0 },
    { "PAUSE",        AVRC_ID_PAUSE,    KEY_PAUSECD,      1 },
    { "FORWARD",      AVRC_ID_FORWARD,  KEY_NEXTSONG,     0 },
    { "BACKWARD",     AVRC_ID_BACKWARD, KEY_PREVIOUSSONG, 0 },
    { "REWIND",       AVRC_ID_REWIND,   KEY_REWIND,       0 },
    { "FAST FORWARD", AVRC_ID_FAST_FOR, KEY_FAST_FORWARD, 0 },
    { NULL,           0,                0,                0 }
};

static void send_reject_response (UINT8 rc_handle, UINT8 label,
    UINT8 pdu, UINT8 status);
static UINT8 opcode_from_pdu(UINT8 pdu);
static void send_metamsg_rsp (UINT8 rc_handle, UINT8 label,
    tBTA_AVK_CODE code, tAVRC_RESPONSE *pmetamsg_resp);
static void register_volumechange(UINT8 label, int index);
static void lbl_init();
static void lbl_destroy();
static void init_all_transactions();
static bt_status_t  get_transaction(rc_transaction_t **ptransaction);
static void release_transaction(UINT8 label);
static rc_transaction_t* get_transaction_by_lbl(UINT8 label);
static void handle_rc_metamsg_rsp(tBTA_AVK_META_MSG *pmeta_msg);
static void btif_avk_rc_upstreams_evt(UINT16 event, tAVRC_COMMAND* p_param, UINT8 ctype, UINT8 label,
                                    int index);
static void btif_avk_rc_upstreams_rsp_evt(UINT16 event, tAVRC_RESPONSE *pavrc_resp, UINT8 ctype, UINT8 label,
                                    int index);
static bt_status_t set_addrplayer_rsp_vendor(btrc_status_t status_code, bt_bdaddr_t *bd_addr);
static int btif_avk_rc_get_idx_by_addr(BD_ADDR address);
#if (AVRC_CTLR_INCLUDED == TRUE)
static void handle_avk_rc_metamsg_cmd(tBTA_AVK_META_MSG *pmeta_msg);
static void handle_avk_rc_metamsg_rsp(tBTA_AVK_META_MSG *pmeta_msg);
static void btif_avk_rc_ctrl_upstreams_rsp_cmd(UINT16 event, tAVRC_COMMAND *pavrc_cmd,
                                           UINT8* p_buf, UINT16 buf_len, UINT8 index);
static void btif_avk_rc_ctrl_upstreams_rsp_evt(UINT16 event, tAVRC_RESPONSE *pavrc_resp,
                                           UINT8* p_buf, UINT16 buf_len, UINT8 rsp_type, UINT8 index);
#endif

/*Added for Browsing Message Response */
static void send_browsemsg_rsp (UINT8 rc_handle, UINT8 label,
    tBTA_AVK_CODE code, tAVRC_RESPONSE *pmetamsg_resp);


/*****************************************************************************
**  Static variables
******************************************************************************/
/* Two RC CBs needed to handle two connections*/
static btif_avk_rc_cb_t btif_avk_rc_cb[BTIF_AVK_RC_NUM_CB];
static btrc_ctrl_callbacks_t *btif_avk_rc_ctrl_callbacks = NULL;
static btrc_ctrl_vendor_callbacks_t *btif_avk_rc_ctrl_vendor_callbacks = NULL;

/*****************************************************************************
**  Static functions
******************************************************************************/
static UINT8 btif_avk_rc_get_idx_by_rc_handle(UINT8 rc_handle);

/*****************************************************************************
**  Externs
******************************************************************************/
extern BOOLEAN check_cod(const bt_bdaddr_t *remote_bdaddr, uint32_t cod);
extern void btif_avk_get_latest_playing_device(BD_ADDR address); //get the Playing device address
extern BOOLEAN btif_avk_is_playing();
extern BOOLEAN btif_avk_is_device_connected(BD_ADDR address);
extern void btif_avk_trigger_dual_handoff(BOOLEAN handoff, BD_ADDR address);
extern BOOLEAN btif_avk_get_multicast_state();
extern BOOLEAN btif_avk_is_current_device(BD_ADDR address);
extern UINT16 btif_avk_get_num_connected_devices(void);
extern UINT16 btif_avk_get_num_playing_devices(void);
/*****************************************************************************
**  Functions
******************************************************************************/

#if (AVRC_CTLR_INCLUDED == TRUE)
void btif_avk_rc_handle_rc_ctrl_features(int index)
{
    if ((btif_avk_rc_cb[index].rc_features & BTA_AVK_FEAT_RCTG)||
       ((btif_avk_rc_cb[index].rc_features & BTA_AVK_FEAT_RCCT)&&
        (btif_avk_rc_cb[index].rc_features & BTA_AVK_FEAT_ADV_CTRL)))
    {
        bt_bdaddr_t rc_addr;
        int rc_features = 0;
        bdcpy(rc_addr.address, btif_avk_rc_cb[index].rc_addr);

        if ((btif_avk_rc_cb[index].rc_features & BTA_AVK_FEAT_ADV_CTRL)&&
             (btif_avk_rc_cb[index].rc_features & BTA_AVK_FEAT_RCCT))
        {
            rc_features |= BTRC_FEAT_ABSOLUTE_VOLUME;
        }
        if ((btif_avk_rc_cb[index].rc_features & BTA_AVK_FEAT_METADATA)&&
            (btif_avk_rc_cb[index].rc_features & BTA_AVK_FEAT_VENDOR))
        {
            rc_features |= BTRC_FEAT_METADATA;
        }
        BTIF_TRACE_DEBUG("Update rc features to CTRL %d",rc_features);
        HAL_CBACK(btif_avk_rc_ctrl_vendor_callbacks, getrcfeatures_vendor_cb, &rc_addr, rc_features);
    }
}
#endif

/***************************************************************************
 *  Function       btif_avk_rc_init_txn_label_queue
 *
 *  - Argument:    index of rc control block
 *
 *  - Description: initializes the txn label queues for the rc index
 *
 ***************************************************************************/
static void btif_avk_rc_init_txn_label_queue(int index)
{
    int j;
    for (j = 0; j < MAX_CMD_QUEUE_LEN; j++)
    {
        btif_avk_rc_cb[index].rc_pdu_info[j].front = 0;
        btif_avk_rc_cb[index].rc_pdu_info[j].size = 0;
        btif_avk_rc_cb[index].rc_pdu_info[j].rear = MAX_TRANSACTIONS_PER_SESSION - 1;
    }
}

/***************************************************************************
 *  Function       btif_avk_rc_get_connection_state
 *
 *  - Argument:    none
 *
 *  - Description: Return true if any RC is in connected state
 *
 ***************************************************************************/
static BOOLEAN btif_avk_rc_get_connection_state()
{
    int clients;

    for (clients = 0; clients < btif_max_rc_clients; clients++)
    {
        if (btif_avk_rc_cb[clients].rc_connected == TRUE)
        {
            return TRUE;
        }
    }
    return FALSE;
}

/***************************************************************************
 *  Function       btif_avk_rc_get_valid_idx
 *
 *  - Argument:    none
 *
 *  - Description: Gets the index which is ready for new connection
 *
 ***************************************************************************/
static int btif_avk_rc_get_valid_idx()
{
    int i;
    for (i = 0; i < btif_max_rc_clients; i++)
    {
        if (!(btif_avk_rc_cb[i].rc_connected))
            break;
    }
    return i;
}

/***************************************************************************
 *  Function       btif_avk_rc_get_idx_by_rc_handle
 *
 *  - Argument:    rc handle
 *
 *  - Description: Gets the RC handle index of matching handle
 *
 ***************************************************************************/
static UINT8 btif_avk_rc_get_idx_by_rc_handle(UINT8 rc_handle)
{
    UINT8 i;

    for (i = 0; i < btif_max_rc_clients; i++)
    {
        if (btif_avk_rc_cb[i].rc_handle == rc_handle)
            break;
    }
    return i;
}

/* Get the address of device on which PLAY command came
* This address will be used in AV IF layer to determine
* On which device to START playback. */
/***************************************************************************
 *  Function       btif_avk_rc_get_playing_device
 *
 *  - Argument:    bd_addr
 *
 *  - Description: Copies the BD address of current playing device
 *
 ***************************************************************************/
void btif_avk_rc_get_playing_device(BD_ADDR address)
{
    int i;
    for (i = 0; i < btif_max_rc_clients; i++)
    {
        if (btif_avk_rc_cb[i].rc_play_processed)
        {
            //copy bd address
            bdcpy(address, btif_avk_rc_cb[i].rc_addr);
        }
    }
}

/* Reset the Play trigger, once the AVDTP START is
* sent, called from AV IF layer. */
/***************************************************************************
 *  Function       btif_avk_rc_clear_playing_state
 *
 *  - Argument:    BOOLEAN
 *
 *  - Description: Clears the PLAY processed.rc_play_processed denotes
 *                 play command has been processed for this device.
 *
 ***************************************************************************/
void btif_avk_rc_clear_playing_state(BOOLEAN state)
{
    int i;
    for (i = 0; i < btif_max_rc_clients; i++)
    {
        if (btif_avk_rc_cb[i].rc_play_processed)
        {
            btif_avk_rc_cb[i].rc_play_processed = state;
        }
    }
}

/***************************************************************************
 *  Function       btif_avk_rc_clear_priority
 *
 *  - Argument:    Device address
 *
 *  - Description: Clears the priority information for the device
 *                 This can be used while AV disconnection for the device.
 *                 Setting of rc_play_processed flag could have been avoided
 *                 looking at the stream state, but it might still leave some
 *                 corner case of audio suspending just before the play takes
 *                 effect.
 ***************************************************************************/
void btif_avk_rc_clear_priority(BD_ADDR address)
{
    int index;

    index = btif_avk_rc_get_idx_by_addr(address);
    if(index < btif_max_rc_clients)
    {
        btif_avk_rc_cb[index].rc_play_processed = FALSE;
    }
}

/***************************************************************************
 *  Function       btif_avk_rc_handle_rc_connect
 *
 *  - Argument:    tBTA_AVK_RC_OPEN  RC open data structure
 *
 *  - Description: RC connection event handler
 *
 ***************************************************************************/
static void btif_avk_rc_handle_rc_connect (tBTA_AVK_RC_OPEN *p_rc_open)
{
    bt_status_t result = BT_STATUS_SUCCESS;
    bt_bdaddr_t rc_addr;
    int index;

    BTIF_TRACE_IMP("%s: rc_handle: %d", __FUNCTION__, p_rc_open->rc_handle);
    if(p_rc_open->status == BTA_AVK_SUCCESS)
    {
        //Check if already some RC is connected
        /*Now we can have two RC connections
        * Check should be here about 3rd connection too.
        * Get the free or MAX index. Max index should be rejected. */
        index = btif_avk_rc_get_valid_idx();
        if (index == btif_max_rc_clients)
        {
            /*Reached Max, this connection must be rejected*/
            BTIF_TRACE_ERROR("RC OPEN in MAX connected state");
            BTA_AvkCloseRc(p_rc_open->rc_handle);
            return;
        }
        /*Use the index for this RC connection*/
        BTIF_TRACE_DEBUG("Got RC OPEN on the index= %d", index);
        memcpy(btif_avk_rc_cb[index].rc_addr, p_rc_open->peer_addr, sizeof(BD_ADDR));
        btif_avk_rc_cb[index].rc_features = p_rc_open->peer_features;
        btif_avk_rc_cb[index].rc_vol_label = MAX_LABEL;
        btif_avk_rc_cb[index].rc_volume = MAX_VOLUME;
        btif_avk_rc_cb[index].rc_connected = TRUE;
        btif_avk_rc_cb[index].rc_handle = p_rc_open->rc_handle;
        btif_avk_rc_init_txn_label_queue(index);
        bdcpy(rc_addr.address, btif_avk_rc_cb[index].rc_addr);
#if (AVRC_CTLR_INCLUDED == TRUE)
        if(btif_avk_rc_ctrl_callbacks != NULL) {
            HAL_CBACK(btif_avk_rc_ctrl_callbacks, connection_state_cb, TRUE, &rc_addr);
        }
        /* report connection state if remote device is AVRCP target */
        if ((btif_avk_rc_cb[index].rc_features & BTA_AVK_FEAT_RCTG)||
           ((btif_avk_rc_cb[index].rc_features & BTA_AVK_FEAT_RCCT)&&
            (btif_avk_rc_cb[index].rc_features & BTA_AVK_FEAT_ADV_CTRL)))
        {
            btif_avk_rc_handle_rc_ctrl_features(index);
        }
#endif
        /* on locally initiated connection we will get remote features as part of connect
        Delay this update till connection update reaches Apps*/
    }
    else
    {
        BTIF_TRACE_ERROR("%s Connect failed with error code: %d",
            __FUNCTION__, p_rc_open->status);
    }
}

/***************************************************************************
 *  Function       btif_avk_rc_handle_rc_disconnect
 *
 *  - Argument:    tBTA_AVK_RC_CLOSE     RC close data structure
 *
 *  - Description: RC disconnection event handler
 *
 ***************************************************************************/
static void btif_avk_rc_handle_rc_disconnect (tBTA_AVK_RC_CLOSE *p_rc_close)
{
    bt_bdaddr_t rc_addr;
    tBTA_AVK_FEAT features;
    UINT8 index;
    BOOLEAN is_connected = 0;

    index = btif_avk_rc_get_idx_by_rc_handle(p_rc_close->rc_handle);
    BTIF_TRACE_IMP("%s: rc_handle: %d index %d", __FUNCTION__, p_rc_close->rc_handle, index);
    if (index == btif_max_rc_clients)
    {
        BTIF_TRACE_ERROR("Got disconnect of unknown device");
        return;
    }
    if ((p_rc_close->rc_handle != btif_avk_rc_cb[index].rc_handle)
        && (bdcmp(btif_avk_rc_cb[index].rc_addr, p_rc_close->peer_addr)))
    {
        BTIF_TRACE_ERROR("Got disconnect of unknown device");
        return;
    }
#if (AVRC_CTLR_INCLUDED == TRUE)
    bdcpy(rc_addr.address, btif_avk_rc_cb[index].rc_addr);
    features = btif_avk_rc_cb[index].rc_features;
#endif
    btif_avk_rc_cb[index].rc_handle = BTIF_AVK_RC_HANDLE_NONE;
    btif_avk_rc_cb[index].rc_connected = FALSE;
    bdcpy(rc_addr.address, btif_avk_rc_cb[index].rc_addr);
    memset(btif_avk_rc_cb[index].rc_addr, 0, sizeof(BD_ADDR));
    memset(btif_avk_rc_cb[index].rc_notif, 0, sizeof(btif_avk_rc_cb[index].rc_notif));
    btif_avk_rc_cb[index].rc_features = 0;
    btif_avk_rc_cb[index].rc_vol_label = MAX_LABEL;
    btif_avk_rc_cb[index].rc_volume = MAX_VOLUME;
    btif_avk_rc_cb[index].rc_play_processed = FALSE;
    btif_avk_rc_cb[index].rc_pending_play = FALSE;
    btif_avk_rc_init_txn_label_queue(index);

    //CLose Uinput only when all RCs are disconnected
    is_connected = btif_avk_rc_get_connection_state();
    BTIF_TRACE_DEBUG("RC connected : %d", is_connected);
    if (is_connected != TRUE && device.lbllock_destroyed != TRUE)
    {
        BTIF_TRACE_DEBUG("Clear UINPUT and transactions when zero RC left");
        init_all_transactions();
#ifdef ANDROID
        close_uinput();
#endif
    }
    if (!bdcmp(bd_null, rc_addr.address))
    {
        BTIF_TRACE_DEBUG("Cleanup already done");
        return;
    }
#if (AVRC_CTLR_INCLUDED == TRUE)
    /* report connection state if device is AVRCP target */
    if (btif_avk_rc_ctrl_callbacks != NULL)
    {
        HAL_CBACK(btif_avk_rc_ctrl_callbacks, connection_state_cb, FALSE, &rc_addr);
    }
#endif
}

/***************************************************************************
 *  Function       btif_rc_ctrl_send_pause
 *
 *  - Argument:    Index
 *
 *  - Description: Sends PAUSE key event to remote.
 *
 ***************************************************************************/
void btif_avk_rc_ctrl_send_pause(bt_bdaddr_t *bd_addr)
{
// send pass through command  ( AVRCP_PAUSE )to remote.
    BTIF_TRACE_DEBUG("%s: ", __FUNCTION__);
    send_passthrough_cmd(bd_addr, AVRC_ID_PAUSE ,AVRC_STATE_PRESS);
    send_passthrough_cmd(bd_addr, AVRC_ID_PAUSE ,AVRC_STATE_RELEASE);
}
/***************************************************************************
 *  Function       btif_rc_ctrl_send_play
 *
 *  - Argument:    Index
 *
 *  - Description: Sends PLAY key event to remote.
 *
 ***************************************************************************/
void btif_avk_rc_ctrl_send_play(bt_bdaddr_t *bd_addr)
{
// send pass through command (AVRCP_PLAY) to remote.
    BTIF_TRACE_DEBUG("%s: ", __FUNCTION__);
    send_passthrough_cmd(bd_addr, AVRC_ID_PLAY ,AVRC_STATE_PRESS);
    send_passthrough_cmd(bd_addr, AVRC_ID_PLAY ,AVRC_STATE_RELEASE);
}

/***************************************************************************
 *  Function       btif_avk_rc_handle_rc_passthrough_rsp
 *
 *  - Argument:    tBTA_AVK_REMOTE_RSP passthrough command response
 *
 *  - Description: Remote control passthrough response handler
 *
 ***************************************************************************/
static void btif_avk_rc_handle_rc_passthrough_rsp ( tBTA_AVK_REMOTE_RSP *p_remote_rsp)
{
#if (AVRC_CTLR_INCLUDED == TRUE)
    const char *status;
    int index = btif_avk_rc_get_idx_by_rc_handle(p_remote_rsp->rc_handle);
    BTIF_TRACE_DEBUG("%s: index=%d", __FUNCTION__, index);
    if (index >= btif_max_rc_clients)
    {
        BTIF_TRACE_DEBUG("%s: invalid index", __FUNCTION__);
        return;
    }
    if (btif_avk_rc_cb[index].rc_features & BTA_AVK_FEAT_RCTG)
    {
        int key_state;
        if (p_remote_rsp->key_state == AVRC_STATE_RELEASE)
        {
            status = "released";
            key_state = 1;
        }
        else
        {
            status = "pressed";
            key_state = 0;
        }

        BTIF_TRACE_DEBUG("%s: rc_id=%d status=%s", __FUNCTION__, p_remote_rsp->rc_id, status);

        release_transaction(p_remote_rsp->label);
        if (btif_avk_rc_ctrl_callbacks != NULL) {
            HAL_CBACK(btif_avk_rc_ctrl_callbacks, passthrough_rsp_cb, p_remote_rsp->rc_id, key_state);
        }
    }
    else
    {
        BTIF_TRACE_ERROR("%s DUT does not support AVRCP controller role", __FUNCTION__);
    }
#else
    BTIF_TRACE_ERROR("%s AVRCP controller role is not enabled", __FUNCTION__);
#endif
}

/***************************************************************************
 **
 ** Function       btif_avk_rc_handler
 **
 ** Description    RC event handler
 **
 ***************************************************************************/
void btif_avk_rc_handler(tBTA_AVK_EVT event, tBTA_AVK *p_data)
{
    UINT8 index;

    BTIF_TRACE_IMP("%s event:%s", __FUNCTION__, dump_rc_event(event));

    switch (event)
    {
        case BTA_AVK_RC_OPEN_EVT:
        {
            BTIF_TRACE_DEBUG("Peer_features:%x", p_data->rc_open.peer_features);
            btif_avk_rc_handle_rc_connect( &(p_data->rc_open) );
        }break;

        case BTA_AVK_RC_CLOSE_EVT:
        {
            btif_avk_rc_handle_rc_disconnect( &(p_data->rc_close) );
        }break;

#if (AVRC_CTLR_INCLUDED == TRUE)
        case BTA_AVK_REMOTE_RSP_EVT:
        {
            BTIF_TRACE_DEBUG("RSP: rc_id:0x%x key_state:%d", p_data->remote_rsp.rc_id,
                               p_data->remote_rsp.key_state);
            btif_avk_rc_handle_rc_passthrough_rsp( (&p_data->remote_rsp) );
        }
        break;
#endif
        case BTA_AVK_RC_FEAT_EVT:
        {
            BTIF_TRACE_DEBUG("Peer_features:%x on RC handle: %d", p_data->rc_feat.peer_features,
                            p_data->rc_feat.rc_handle);
            index = btif_avk_rc_get_idx_by_rc_handle(p_data->rc_feat.rc_handle);
            if (index == btif_max_rc_clients)
            {
                BTIF_TRACE_ERROR("%s: Invalid RC index for BTA_AVK_RC_FEAT_EVT", __FUNCTION__);
                return;
            }
            btif_avk_rc_cb[index].rc_features = p_data->rc_feat.peer_features;
#if (AVRC_CTLR_INCLUDED == TRUE)
            if ((btif_avk_rc_cb[index].rc_connected) && (btif_avk_rc_ctrl_callbacks != NULL))
            {
                btif_avk_rc_handle_rc_ctrl_features(index);
            }
#endif
        }
        break;
        case BTA_AVK_META_MSG_EVT:
        {
#if (AVRC_CTLR_INCLUDED == TRUE)
            if(btif_avk_rc_ctrl_callbacks != NULL)
            {
                /* This is case of Sink + CT + TG(for abs vol)) */
                BTIF_TRACE_DEBUG("BTA_AVK_META_MSG_EVT  code:%d label:%d",
                                                p_data->meta_msg.code,
                                                p_data->meta_msg.label);
                BTIF_TRACE_DEBUG("  company_id:0x%x len:%d handle:%d",
                                            p_data->meta_msg.company_id,
                                            p_data->meta_msg.len,
                                            p_data->meta_msg.rc_handle);
                if ((p_data->meta_msg.code >= AVRC_RSP_NOT_IMPL)&&
                    (p_data->meta_msg.code <= AVRC_RSP_INTERIM))
                {
                    /* Its a response */
                    handle_avk_rc_metamsg_rsp(&(p_data->meta_msg));
                }
                else if (p_data->meta_msg.code <= AVRC_CMD_GEN_INQ)
                {
                    /* Its a command  */
                    handle_avk_rc_metamsg_cmd(&(p_data->meta_msg));
                }

            }
#endif
            else
            {
                BTIF_TRACE_ERROR("Neither CTRL, nor TG is up, drop meta commands");
            }
        }
        break;
        default:
            BTIF_TRACE_DEBUG("Unhandled RC event : 0x%x", event);
    }
}

/***************************************************************************
 **
 ** Function       btif_avk_rc_get_connected_peer
 **
 ** Description    Fetches the connected headset's BD_ADDR if any
 **
 ***************************************************************************/
BOOLEAN btif_avk_rc_get_connected_peer(BD_ADDR peer_addr)
{
    /*Find the device for which AV is not connected but RC is.*/
    int i;

    for  (i = 0; i < btif_max_rc_clients; i++)
    {
        if (btif_avk_rc_cb[i].rc_connected == TRUE)
        {
            if (!btif_avk_is_device_connected(btif_avk_rc_cb[i].rc_addr))
            {
                bdcpy(peer_addr, btif_avk_rc_cb[i].rc_addr);
                return TRUE;
            }
        }
    }
    return FALSE;
}

static int btif_avk_rc_get_idx_by_addr(BD_ADDR address)
{
    int i;

    for (i = 0; i < btif_max_rc_clients; i++)
    {
        if (bdcmp(btif_avk_rc_cb[i].rc_addr, address) == 0)
        {
            break;
        }
    }
    return i;
}

/***************************************************************************
 **
 ** Function       btif_avk_rc_get_connected_peer_handle
 **
 ** Description    Fetches the connected headset's handle if any
 **
 ***************************************************************************/
UINT8 btif_avk_rc_get_connected_peer_handle(BD_ADDR peer_addr)
{
    int i;
    for  (i = 0; i < btif_max_rc_clients; i++)
    {
        if ((btif_avk_rc_cb[i].rc_connected == TRUE)
             &&(!bdcmp(peer_addr,btif_avk_rc_cb[i].rc_addr)))
        {
            return btif_avk_rc_cb[i].rc_handle;
        }
    }
    return BTIF_AVK_RC_HANDLE_NONE;

}

/***************************************************************************
 **
 ** Function       btif_avk_rc_check_handle_pending_play
 **
 ** Description    Clears the queued PLAY command. if bSend is TRUE, forwards to app
 **
 ***************************************************************************/

/* clear the queued PLAY command. if bSend is TRUE, forward to app */
void btif_avk_rc_check_handle_pending_play (BD_ADDR peer_addr, BOOLEAN bSendToApp)
{
    int index = btif_avk_rc_get_idx_by_addr(peer_addr);

    if (index == btif_max_rc_clients)
    {
        BTIF_TRACE_ERROR("%s: Invalid RC index", __FUNCTION__);
        return;
    }
    UNUSED(peer_addr);

    BTIF_TRACE_DEBUG("%s: bSendToApp=%d", __FUNCTION__, bSendToApp);
    if (btif_avk_rc_cb[index].rc_pending_play)
    {
        if (bSendToApp)
        {
            tBTA_AVK_REMOTE_CMD remote_cmd;
            APPL_TRACE_DEBUG("%s: Sending queued PLAYED event to app", __FUNCTION__);

            memset (&remote_cmd, 0, sizeof(tBTA_AVK_REMOTE_CMD));
            remote_cmd.rc_handle  = btif_avk_rc_cb[index].rc_handle;
            remote_cmd.rc_id      = AVRC_ID_PLAY;
            remote_cmd.hdr.ctype  = AVRC_CMD_CTRL;
            remote_cmd.hdr.opcode = AVRC_OP_PASS_THRU;

            /* delay sending to app, else there is a timing issue in the framework,
             ** which causes the audio to be on th device's speaker. Delay between
             ** OPEN & RC_PLAYs
            */
            GKI_delay (200);
            /* send to app - both PRESSED & RELEASED */
            remote_cmd.key_state  = AVRC_STATE_PRESS;
            handle_rc_passthrough_cmd( &remote_cmd );

            GKI_delay (100);

            remote_cmd.key_state  = AVRC_STATE_RELEASE;
            handle_rc_passthrough_cmd( &remote_cmd );
        }
        btif_avk_rc_cb[index].rc_pending_play = FALSE;
    }
}

/* Generic reject response */
static void send_reject_response (UINT8 rc_handle, UINT8 label, UINT8 pdu, UINT8 status)
{
    UINT8 ctype = AVRC_RSP_REJ;
    tAVRC_RESPONSE avrc_rsp;
    BT_HDR *p_msg = NULL;
    memset (&avrc_rsp, 0, sizeof(tAVRC_RESPONSE));

    avrc_rsp.rsp.opcode = opcode_from_pdu(pdu);
    avrc_rsp.rsp.pdu    = pdu;
    avrc_rsp.rsp.status = status;

    if (AVRC_STS_NO_ERROR == (status = AVRC_BldResponse(rc_handle, &avrc_rsp, &p_msg)) )
    {
        BTIF_TRACE_DEBUG("%s:Sending error notification to handle:%d. pdu:%s,status:0x%02x",
            __FUNCTION__, rc_handle, dump_rc_pdu(pdu), status);
        BTA_AvkMetaRsp(rc_handle, label, ctype, p_msg);
    }
}

static UINT8 opcode_from_pdu(UINT8 pdu)
{
    UINT8 opcode = 0;

    switch (pdu)
    {
    case AVRC_PDU_NEXT_GROUP:
    case AVRC_PDU_PREV_GROUP: /* pass thru */
        opcode  = AVRC_OP_PASS_THRU;
        break;

    default: /* vendor */
        opcode  = AVRC_OP_VENDOR;
        break;
    }

    return opcode;
}

#if (AVRC_CTLR_INCLUDED == TRUE)
/*******************************************************************************
**
** Function         btif_avk_rc_ctrl_upstreams_rsp_cmd
**
** Description      Executes AVRC UPSTREAMS response events in btif context.
**
** Returns          void
**
*******************************************************************************/
static void btif_avk_rc_ctrl_upstreams_rsp_cmd(UINT16 event, tAVRC_COMMAND *pavrc_cmd, UINT8* p_buf, UINT16 buf_len, UINT8 index)
{
    bt_bdaddr_t rc_addr;

    BTIF_TRACE_IMP("%s pdu: %s handle: 0x%x", __FUNCTION__,
        dump_rc_pdu(pavrc_cmd->pdu), btif_avk_rc_cb[index].rc_handle);
    bdcpy(rc_addr.address, btif_avk_rc_cb[index].rc_addr);
#if (AVRC_CTLR_INCLUDED == TRUE)
    switch (event)
    {
    case AVRC_PDU_SET_ABSOLUTE_VOLUME:
         HAL_CBACK(btif_avk_rc_ctrl_vendor_callbacks,setabsvol_cmd_vendor_cb,
                 &rc_addr, pavrc_cmd->volume.volume);
         break;
    case AVRC_PDU_REGISTER_NOTIFICATION:
         if (pavrc_cmd->reg_notif.event_id == AVRC_EVT_VOLUME_CHANGE)
         {
             HAL_CBACK(btif_avk_rc_ctrl_vendor_callbacks,registernotification_absvol_vendor_cb, &rc_addr);
         }
         break;
    }
#endif
}

/*******************************************************************************
**
** Function         btif_avk_rc_ctrl_upstreams_rsp_evt
**
** Description      Executes AVRC UPSTREAMS response events in btif context.
**
** Returns          void
**
*******************************************************************************/
static void btif_avk_rc_ctrl_upstreams_rsp_evt(UINT16 event, tAVRC_RESPONSE *pavrc_resp,
                                        UINT8* p_buf, UINT16 buf_len, UINT8 rsp_type, UINT8 index)
{
    bt_bdaddr_t rc_addr;

    BTIF_TRACE_IMP("%s pdu: %s handle: 0x%x rsp_type:%x", __FUNCTION__,
        dump_rc_pdu(pavrc_resp->pdu), btif_avk_rc_cb[index].rc_handle, rsp_type);

    bdcpy(rc_addr.address, btif_avk_rc_cb[index].rc_addr);

#if (AVRC_CTLR_INCLUDED == TRUE)
    switch (event)
    {
        case AVRC_PDU_GET_CAPABILITIES:
        {
            int xx = 0;
            UINT32 *p_int_array = NULL;
            if (pavrc_resp->get_caps.count > 0)
            {
                p_int_array = (UINT32*)GKI_getbuf(4*pavrc_resp->get_caps.count);
                if (p_int_array == NULL)
                    return;
            }
            for (xx = 0; xx < pavrc_resp->get_caps.count; xx++)
            {
                if (pavrc_resp->get_caps.capability_id == AVRC_CAP_COMPANY_ID)
                {
                    p_int_array[xx] = pavrc_resp->get_caps.param.company_id[xx];
                }
                else if (pavrc_resp->get_caps.capability_id == AVRC_CAP_EVENTS_SUPPORTED)
                {
                    p_int_array[xx] = pavrc_resp->get_caps.param.event_id[xx];
                }
            }
            HAL_CBACK(btif_avk_rc_ctrl_vendor_callbacks, getcap_rsp_vendor_cb, &rc_addr,
            pavrc_resp->get_caps.capability_id,p_int_array,pavrc_resp->get_caps.count, rsp_type);
            if (p_int_array != NULL)
                GKI_freebuf(p_int_array);
        }
            break;
        case AVRC_PDU_LIST_PLAYER_APP_ATTR:
        {
            int xx = 0;
            UINT8  *p_byte_array = NULL;
            if (pavrc_resp->list_app_attr.num_attr > 0)
            {
                p_byte_array = (UINT8*)GKI_getbuf(pavrc_resp->list_app_attr.num_attr);
                if (p_byte_array == NULL)
                    return;
            }
            for (xx = 0; xx < pavrc_resp->list_app_attr.num_attr; xx++)
            {
                p_byte_array[xx] = pavrc_resp->list_app_attr.attrs[xx];
            }
            HAL_CBACK(btif_avk_rc_ctrl_vendor_callbacks,listplayerappsettingattrib_rsp_vendor_cb, &rc_addr,
                                 p_byte_array,pavrc_resp->list_app_attr.num_attr, rsp_type);
            if (p_byte_array != NULL)
                GKI_freebuf(p_byte_array);
        }
            break;
        case AVRC_PDU_LIST_PLAYER_APP_VALUES:
        {
            int xx = 0;
            UINT8  *p_byte_array = NULL;
            if (pavrc_resp->list_app_values.num_val > 0)
            {
                p_byte_array = (UINT8*)GKI_getbuf(pavrc_resp->list_app_values.num_val);
                if (p_byte_array == NULL)
                    return;
            }
            for (xx = 0; xx < pavrc_resp->list_app_values.num_val; xx++)
            {
                p_byte_array[xx] = pavrc_resp->list_app_values.vals[xx];
            }
            HAL_CBACK(btif_avk_rc_ctrl_vendor_callbacks,listplayerappsettingvalue_rsp_vendor_cb, &rc_addr,
                                    p_byte_array,pavrc_resp->list_app_values.num_val, rsp_type);
            if (p_byte_array != NULL)
                GKI_freebuf(p_byte_array);
        }
            break;
        case AVRC_PDU_GET_CUR_PLAYER_APP_VALUE:
        {
            int xx = 0;
            UINT8  *p_supported_ids;
            UINT8  *p_byte_array;

            if (pavrc_resp->get_cur_app_val.num_val <= 0)
            {
                HAL_CBACK(btif_avk_rc_ctrl_vendor_callbacks,currentplayerappsetting_rsp_vendor_cb, &rc_addr,NULL,
                                        NULL,pavrc_resp->get_cur_app_val.num_val, rsp_type);
                break;
            }
            p_supported_ids = (UINT8*)GKI_getbuf(pavrc_resp->get_cur_app_val.num_val);
            if (p_supported_ids == NULL)
                return;
            p_byte_array = (UINT8*)GKI_getbuf(pavrc_resp->get_cur_app_val.num_val);
            if (p_byte_array == NULL)
            {
                if (p_supported_ids != NULL)
                    GKI_freebuf(p_supported_ids);
                return;
            }
            for (xx = 0; xx < pavrc_resp->get_cur_app_val.num_val; xx++)
            {
                p_supported_ids[xx] = pavrc_resp->get_cur_app_val.p_vals[xx].attr_id;
                p_byte_array[xx] = pavrc_resp->get_cur_app_val.p_vals[xx].attr_val;
            }
            HAL_CBACK(btif_avk_rc_ctrl_vendor_callbacks,currentplayerappsetting_rsp_vendor_cb, &rc_addr,p_supported_ids,
                                    p_byte_array,pavrc_resp->get_cur_app_val.num_val, rsp_type);
            GKI_freebuf(pavrc_resp->get_cur_app_val.p_vals);
            GKI_freebuf(p_byte_array);
            GKI_freebuf(p_supported_ids);
        }
            break;
        case AVRC_PDU_SET_PLAYER_APP_VALUE:
        {
            HAL_CBACK(btif_avk_rc_ctrl_vendor_callbacks,setplayerappsetting_rsp_vendor_cb, &rc_addr, rsp_type);
        }
            break;
        case AVRC_PDU_REGISTER_NOTIFICATION:
        {
            UINT8  *p_byte_array;

            if(buf_len <= 0)
            {
                HAL_CBACK(btif_avk_rc_ctrl_vendor_callbacks,notification_rsp_vendor_cb, &rc_addr,
                                        rsp_type,0,NULL);
                break;
            }
            p_byte_array = (UINT8*)GKI_getbuf(buf_len);
            if (p_byte_array == NULL)
                return;
            memcpy(p_byte_array,p_buf,buf_len);
            HAL_CBACK(btif_avk_rc_ctrl_vendor_callbacks,notification_rsp_vendor_cb, &rc_addr,
                                    rsp_type,buf_len,p_byte_array);
            GKI_freebuf(p_byte_array);
        }
            break;
        case AVRC_PDU_GET_ELEMENT_ATTR:
        {
            UINT8  *p_byte_array;

            if (pavrc_resp->get_elem_attrs.num_attr <= 0)
            {
                HAL_CBACK(btif_avk_rc_ctrl_vendor_callbacks,getelementattrib_rsp_vendor_cb, &rc_addr,
                  pavrc_resp->get_elem_attrs.num_attr,0,NULL, rsp_type);
                break;
            }
            p_byte_array = (UINT8*)GKI_getbuf(buf_len);
            if (p_byte_array == NULL)
                return;
            memcpy(p_byte_array,p_buf,buf_len);
            HAL_CBACK(btif_avk_rc_ctrl_vendor_callbacks,getelementattrib_rsp_vendor_cb, &rc_addr,
              pavrc_resp->get_elem_attrs.num_attr,buf_len,p_byte_array, rsp_type);
            GKI_freebuf(p_byte_array);
        }
            break;
        case AVRC_PDU_GET_PLAY_STATUS:
        {
            UINT8  *p_byte_array;

            if (buf_len <= 0)
            {
                HAL_CBACK(btif_avk_rc_ctrl_vendor_callbacks,getplaystatus_rsp_vendor_cb, &rc_addr,
                                        0,NULL, rsp_type);
                break;
            }
            p_byte_array = (UINT8*)GKI_getbuf(buf_len);
            if (p_byte_array == NULL)
                return;
            memcpy(p_byte_array,p_buf,buf_len);
            HAL_CBACK(btif_avk_rc_ctrl_vendor_callbacks,getplaystatus_rsp_vendor_cb, &rc_addr,
                                    buf_len,p_byte_array, rsp_type);
            GKI_freebuf(p_byte_array);
        }
            break;
        default:
            return;
    }
#endif
}
#endif

/************************************************************************************
**  AVRCP API Functions
************************************************************************************/

/*******************************************************************************
**
** Function         init_ctrl
**
** Description      Initializes the AVRC interface
**
** Returns          bt_status_t
**
*******************************************************************************/
static bt_status_t init_ctrl(btrc_ctrl_callbacks_t* callbacks )
{
    bt_status_t result = BT_STATUS_SUCCESS;

    BTIF_TRACE_EVENT("## %s ##", __FUNCTION__);

    if (btif_avk_rc_ctrl_callbacks)
        return BT_STATUS_DONE;

    /* Controller is used only for Certification purposes.
     * In normal case AVRCP controller will not be used, hence
     * updating this is required.
     */
    btif_avk_rc_ctrl_callbacks = callbacks;
    memset (&btif_avk_rc_cb, 0, sizeof(btif_avk_rc_cb));
    btif_avk_rc_cb[BTIF_AVK_RC_DEFAULT_INDEX].rc_vol_label=MAX_LABEL;
    lbl_init();

    return result;
}

/*******************************************************************************
**
** Function         init_ctrl_vendor
**
** Description      Initializes the AVRC controller vendor interface
**
** Returns          bt_status_t
**
*******************************************************************************/
static bt_status_t init_ctrl_vendor(btrc_ctrl_vendor_callbacks_t* callbacks, int max_connections )
{
    int i;
    bt_status_t result = BT_STATUS_SUCCESS;
    btif_avk_rc_ctrl_vendor_callbacks = callbacks;
    btif_max_rc_clients = max_connections;
    memset (&btif_avk_rc_cb, 0, sizeof(btif_avk_rc_cb));
    for (i = 0; i < btif_max_rc_clients; i++)
    {
       btif_avk_rc_cb[i].rc_vol_label=MAX_LABEL;
    }
    return result;
}
#if (AVRC_CTLR_INCLUDED == TRUE)
/***************************************************************************
**
** Function         handle_avk_rc_metamsg_rsp
**
** Description      Handle RC metamessage response
**
** Returns          void
**
***************************************************************************/
static void handle_avk_rc_metamsg_rsp(tBTA_AVK_META_MSG *pmeta_msg)
{
    UINT8 index;  /*For RC it is zero*/
    tAVRC_RESPONSE    avrc_response = {0};
    UINT8             scratch_buf[4096] = {0};// maximum size that can be used
    UINT16            buf_len;
    tAVRC_STS status = BT_STATUS_UNSUPPORTED;
    index = btif_avk_rc_get_idx_by_rc_handle(pmeta_msg->rc_handle);

    BTIF_TRACE_DEBUG(" %s opcode = %d rsp_code = %d  ",__FUNCTION__,
                        pmeta_msg->p_msg->hdr.opcode,pmeta_msg->code);
    if((AVRC_OP_VENDOR==pmeta_msg->p_msg->hdr.opcode)&&
                (pmeta_msg->code >= AVRC_RSP_NOT_IMPL)&&
                (pmeta_msg->code <= AVRC_RSP_INTERIM))
    {
        status=AVRC_Ctrl_ParsResponse(pmeta_msg->p_msg, &avrc_response, scratch_buf, &buf_len);
        BTIF_TRACE_DEBUG(" pdu = %d rsp_status = %d",avrc_response.pdu,
                                    pmeta_msg->p_msg->vendor.hdr.ctype);

        if ((avrc_response.pdu == AVRC_PDU_REGISTER_NOTIFICATION)&&
            (pmeta_msg->code == AVRC_RSP_INTERIM))
        {
            BTIF_TRACE_DEBUG(" Don't release transaction label ");
        }
        else
        {
            BTIF_TRACE_DEBUG(" Releasing label = %d",pmeta_msg->label);
            release_transaction(pmeta_msg->label);
        }
        btif_avk_rc_ctrl_upstreams_rsp_evt((uint16_t)avrc_response.rsp.pdu, &avrc_response,
                               scratch_buf, buf_len,pmeta_msg->p_msg->vendor.hdr.ctype, index);
    }
    else
    {
        BTIF_TRACE_DEBUG("%s:Invalid Vendor Command  code: %d len: %d. Not processing it.",
        __FUNCTION__, pmeta_msg->code, pmeta_msg->len);
        return;
    }
}

/***************************************************************************
**
** Function         handle_avk_rc_metamsg_cmd
**
** Description      Handle RC metamessage response
**
** Returns          void
**
***************************************************************************/
static void handle_avk_rc_metamsg_cmd(tBTA_AVK_META_MSG *pmeta_msg)
{
    UINT8 index;  /*For RC it is zero*/
    tAVRC_COMMAND    avrc_cmd = {0};
    UINT8             scratch_buf[4096] = {0};
    UINT16            buf_len;
    tAVRC_STS status = BT_STATUS_UNSUPPORTED;
    index = btif_avk_rc_get_idx_by_rc_handle(pmeta_msg->rc_handle);

    BTIF_TRACE_DEBUG(" %s opcode = %d rsp_code = %d  ",__FUNCTION__,
                       pmeta_msg->p_msg->hdr.opcode,pmeta_msg->code);
    if((AVRC_OP_VENDOR==pmeta_msg->p_msg->hdr.opcode)&&
                (pmeta_msg->code <= AVRC_CMD_GEN_INQ))
    {
        buf_len = sizeof(scratch_buf);
        status = AVRC_Ctrl_ParsCommand(pmeta_msg->p_msg, &avrc_cmd, scratch_buf, buf_len);
        BTIF_TRACE_DEBUG("Received vendor command.code,PDU and label: %d, %d,%d",pmeta_msg->code,
                           avrc_cmd.pdu, pmeta_msg->label);

        if (status != AVRC_STS_NO_ERROR)
        {
            /* return error */
            BTIF_TRACE_WARNING("%s: Error in parsing received metamsg command. status: 0x%02x",
                __FUNCTION__, status);
            send_reject_response(pmeta_msg->rc_handle, pmeta_msg->label, avrc_cmd.pdu, status);
        }
        else
        {
            if (avrc_cmd.pdu == AVRC_PDU_REGISTER_NOTIFICATION)
            {
                UINT8 event_id = avrc_cmd.reg_notif.event_id;
                BTIF_TRACE_EVENT("%s:New register notification received.event_id:%s,label:0x%x,code:%x"
                ,__FUNCTION__,dump_rc_notification_event_id(event_id), pmeta_msg->label,pmeta_msg->code);
                btif_avk_rc_cb[index].rc_notif[event_id-1].bNotify = TRUE;
                btif_avk_rc_cb[index].rc_notif[event_id-1].label = pmeta_msg->label;
            }
            else if (avrc_cmd.pdu == AVRC_PDU_SET_ABSOLUTE_VOLUME)
            {
                BTIF_TRACE_EVENT("%s:Abs Volume Cmd Recvd,label:0x%x,code:%x",
                __FUNCTION__, pmeta_msg->label,pmeta_msg->code);
                btif_avk_rc_cb[index].rc_vol_label = pmeta_msg->label;
            }
            btif_avk_rc_ctrl_upstreams_rsp_cmd((uint16_t)avrc_cmd.pdu, &avrc_cmd, scratch_buf, buf_len, index);
        }
    }
    else
    {
        BTIF_TRACE_DEBUG("%s:Invalid Vendor Command  code: %d len: %d. Not processing it.",
        __FUNCTION__, pmeta_msg->code, pmeta_msg->len);
        return;
    }
}
#endif

/***************************************************************************
**
** Function         cleanup_ctrl
**
** Description      Closes the AVRC Controller interface
**
** Returns          void
**
***************************************************************************/
static void cleanup_ctrl(void)
{
    BTIF_TRACE_EVENT("## %s ##", __FUNCTION__);

    if (btif_avk_rc_ctrl_callbacks)
    {
        btif_avk_rc_ctrl_callbacks = NULL;
    }
    memset(&btif_avk_rc_cb, 0, sizeof(btif_avk_rc_cb_t));
    lbl_destroy();
    BTIF_TRACE_EVENT("## %s ## completed", __FUNCTION__);
}

/***************************************************************************
**
** Function         cleanup_ctrl_vendor
**
** Description      Closes the AVRC Controller vendor interface
**
** Returns          void
**
***************************************************************************/
static void cleanup_ctrl_vendor(void)
{
    BTIF_TRACE_EVENT("## %s ##", __FUNCTION__);

    if (btif_avk_rc_ctrl_vendor_callbacks)
    {
        btif_avk_rc_ctrl_vendor_callbacks = NULL;
    }
    BTIF_TRACE_EVENT("## %s ## completed", __FUNCTION__);
}

/***************************************************************************
**
** Function         getcapabilities_cmd
**
** Description      GetCapabilties from Remote(Company_ID, Events_Supported)
**
** Returns          void
**
***************************************************************************/
static bt_status_t getcapabilities_cmd_vendor (uint8_t cap_id)
{
    tAVRC_STS status = BT_STATUS_UNSUPPORTED;
    rc_transaction_t *p_transaction=NULL;
    bt_status_t tran_status;
    tAVRC_COMMAND avrc_cmd = {0};
    BT_HDR *p_msg = NULL;
    int index = BTIF_AVK_RC_DEFAULT_INDEX; //For RC it should be 0

    CHECK_AVK_RC_CONNECTED

#if (AVRC_CTLR_INCLUDED == TRUE)
    BTIF_TRACE_DEBUG("%s: cap_id %d", __FUNCTION__, cap_id);

    tran_status = get_transaction(&p_transaction);
    if(BT_STATUS_SUCCESS != tran_status || NULL==p_transaction)
        return BT_STATUS_FAIL;

    avrc_cmd.get_caps.opcode = AVRC_OP_VENDOR;
    avrc_cmd.get_caps.capability_id = cap_id;
    avrc_cmd.get_caps.pdu = AVRC_PDU_GET_CAPABILITIES;
    avrc_cmd.get_caps.status = AVRC_STS_NO_ERROR;
    status = AVRC_BldCommand(&avrc_cmd, &p_msg);
    if (status == AVRC_STS_NO_ERROR)
    {
        UINT8* data_start = (UINT8*)(p_msg + 1) + p_msg->offset;
        BTIF_TRACE_DEBUG("%s msgreq being sent out with label %d",
                __FUNCTION__,p_transaction->lbl);
        if (p_msg != NULL)
        {
            BTA_AvkVendorCmd(btif_avk_rc_cb[index].rc_handle,p_transaction->lbl,AVRC_CMD_STATUS,
                data_start, p_msg->len);
            status =  BT_STATUS_SUCCESS;
        }
    }
    else
    {
         BTIF_TRACE_ERROR("%s: failed to build command. status: 0x%02x",
                             __FUNCTION__, status);
     }
    if (p_msg != NULL)
        GKI_freebuf(p_msg);
#else
    BTIF_TRACE_DEBUG("%s: feature not enabled", __FUNCTION__);
#endif
    return status;
}

/***************************************************************************
**
** Function         list_player_app_setting_attrib_cmd
**
** Description      Get supported List Player Attributes
**
** Returns          void
**
***************************************************************************/
static bt_status_t list_player_app_setting_attrib_cmd_vendor(void)
{
    tAVRC_STS status = BT_STATUS_UNSUPPORTED;
    rc_transaction_t *p_transaction=NULL;
    tAVRC_COMMAND avrc_cmd = {0};
    BT_HDR *p_msg = NULL;
    int index = BTIF_AVK_RC_DEFAULT_INDEX; //For RC it should be 0

#if (AVRC_CTLR_INCLUDED == TRUE)
    bt_status_t tran_status;
    CHECK_AVK_RC_CONNECTED

    BTIF_TRACE_DEBUG("%s: ", __FUNCTION__);

    tran_status = get_transaction(&p_transaction);

    if(BT_STATUS_SUCCESS != tran_status || NULL==p_transaction)
        return BT_STATUS_FAIL;

    avrc_cmd.list_app_attr.opcode = AVRC_OP_VENDOR;
    avrc_cmd.list_app_attr.pdu = AVRC_PDU_LIST_PLAYER_APP_ATTR;
    avrc_cmd.list_app_attr.status = AVRC_STS_NO_ERROR;
    status = AVRC_BldCommand(&avrc_cmd, &p_msg);
    if (status == AVRC_STS_NO_ERROR)
    {
        UINT8* data_start = (UINT8*)(p_msg + 1) + p_msg->offset;
        BTIF_TRACE_DEBUG("%s msgreq being sent out with label %d",
                __FUNCTION__,p_transaction->lbl);
        if(NULL!=p_msg)
        {
            BTA_AvkVendorCmd(btif_avk_rc_cb[index].rc_handle,p_transaction->lbl,AVRC_CMD_STATUS,
                data_start, p_msg->len);
            status =  BT_STATUS_SUCCESS;
        }
    }
    else
    {
         BTIF_TRACE_ERROR("%s: failed to build command. status: 0x%02x",
                            __FUNCTION__, status);
     }
    if (p_msg != NULL)
        GKI_freebuf(p_msg);
#else
    BTIF_TRACE_DEBUG("%s: feature not enabled", __FUNCTION__);
#endif
    return status;
}

/***************************************************************************
**
** Function         list_player_app_setting_value_cmd
**
** Description      Get values of supported Player Attributes
**
** Returns          void
**
***************************************************************************/
static bt_status_t list_player_app_setting_value_cmd_vendor(uint8_t attrib_id)
{
    tAVRC_STS status = BT_STATUS_UNSUPPORTED;
    rc_transaction_t *p_transaction=NULL;
#if (AVRC_CTLR_INCLUDED == TRUE)
    bt_status_t tran_status;
    tAVRC_COMMAND avrc_cmd = {0};
    BT_HDR *p_msg = NULL;
    int index = BTIF_AVK_RC_DEFAULT_INDEX; //For RC it should be 0

    CHECK_AVK_RC_CONNECTED
    tran_status = get_transaction(&p_transaction);

    BTIF_TRACE_DEBUG("%s: attrib_id %d", __FUNCTION__, attrib_id);

    if(BT_STATUS_SUCCESS != tran_status || NULL==p_transaction)
        return BT_STATUS_FAIL;


    avrc_cmd.list_app_values.attr_id = attrib_id;
    avrc_cmd.list_app_values.opcode = AVRC_OP_VENDOR;
    avrc_cmd.list_app_values.pdu = AVRC_PDU_LIST_PLAYER_APP_VALUES;
    avrc_cmd.list_app_values.status = AVRC_STS_NO_ERROR;
    status = AVRC_BldCommand(&avrc_cmd, &p_msg);
    if (status == AVRC_STS_NO_ERROR)
    {
        UINT8* data_start = (UINT8*)(p_msg + 1) + p_msg->offset;
        BTIF_TRACE_DEBUG("%s msgreq being sent out with label %d",
                __FUNCTION__,p_transaction->lbl);
        if (p_msg != NULL)
        {
            BTA_AvkVendorCmd(btif_avk_rc_cb[index].rc_handle,p_transaction->lbl,AVRC_CMD_STATUS,
                data_start, p_msg->len);
            status =  BT_STATUS_SUCCESS;
        }
    }
    else
    {
         BTIF_TRACE_ERROR("%s: failed to build command. status: 0x%02x",
                            __FUNCTION__, status);
     }
    if (p_msg != NULL)
        GKI_freebuf(p_msg);
#else
    BTIF_TRACE_DEBUG("%s: feature not enabled", __FUNCTION__);
#endif
    return status;
}

/***************************************************************************
**
** Function         get_player_app_setting_cmd
**
** Description      Get current values of Player Attributes
**
** Returns          void
**
***************************************************************************/
static bt_status_t get_player_app_setting_cmd_vendor(uint8_t num_attrib, uint8_t* attrib_ids)
{
    tAVRC_STS status = BT_STATUS_UNSUPPORTED;
    rc_transaction_t *p_transaction=NULL;
    int count  = 0;
#if (AVRC_CTLR_INCLUDED == TRUE)
    tAVRC_COMMAND avrc_cmd = {0};
    BT_HDR *p_msg = NULL;
    int index = BTIF_AVK_RC_DEFAULT_INDEX; //For RC it should be 0
    bt_status_t tran_status;
    CHECK_AVK_RC_CONNECTED

    BTIF_TRACE_DEBUG("%s: num attrib_id %d", __FUNCTION__, num_attrib);

    tran_status = get_transaction(&p_transaction);
    if(BT_STATUS_SUCCESS != tran_status || NULL==p_transaction)
        return BT_STATUS_FAIL;

    avrc_cmd.get_cur_app_val.opcode = AVRC_OP_VENDOR;
    avrc_cmd.get_cur_app_val.status = AVRC_STS_NO_ERROR;
    avrc_cmd.get_cur_app_val.num_attr = num_attrib;
    avrc_cmd.get_cur_app_val.pdu = AVRC_PDU_GET_CUR_PLAYER_APP_VALUE;

    for (count = 0; count < num_attrib; count++)
    {
     avrc_cmd.get_cur_app_val.attrs[count] = attrib_ids[count];
    }
    status = AVRC_BldCommand(&avrc_cmd, &p_msg);
    if (status == AVRC_STS_NO_ERROR)
    {
        UINT8* data_start = (UINT8*)(p_msg + 1) + p_msg->offset;
                BTIF_TRACE_DEBUG("%s msgreq being sent out with label %d",
                __FUNCTION__,p_transaction->lbl);
        if (p_msg != NULL)
        {
            BTA_AvkVendorCmd(btif_avk_rc_cb[index].rc_handle,p_transaction->lbl,AVRC_CMD_STATUS,
                data_start, p_msg->len);
            status =  BT_STATUS_SUCCESS;
        }
    }
    else
    {
         BTIF_TRACE_ERROR("%s: failed to build command. status: 0x%02x",
                            __FUNCTION__, status);
     }
    if (p_msg != NULL)
        GKI_freebuf(p_msg);
#else
    BTIF_TRACE_DEBUG("%s: feature not enabled", __FUNCTION__);
#endif
    return status;
}

/***************************************************************************
**
** Function         set_player_app_setting_cmd
**
** Description      Set current values of Player Attributes
**
** Returns          void
**
***************************************************************************/
static bt_status_t set_player_app_setting_cmd_vendor(uint8_t num_attrib, uint8_t* attrib_ids,
                   uint8_t* attrib_vals)
{
    tAVRC_STS status = BT_STATUS_UNSUPPORTED;
    rc_transaction_t *p_transaction=NULL;
    int count  = 0;
#if (AVRC_CTLR_INCLUDED == TRUE)
    tAVRC_COMMAND avrc_cmd = {0};
    BT_HDR *p_msg = NULL;
    int index = BTIF_AVK_RC_DEFAULT_INDEX; //For RC it should be 0
    bt_status_t tran_status;

    CHECK_AVK_RC_CONNECTED
    BTIF_TRACE_DEBUG("%s: num attrib_id %d", __FUNCTION__, num_attrib);

    tran_status = get_transaction(&p_transaction);
    if(BT_STATUS_SUCCESS != tran_status || NULL==p_transaction)
        return BT_STATUS_FAIL;

    avrc_cmd.set_app_val.opcode = AVRC_OP_VENDOR;
    avrc_cmd.set_app_val.status = AVRC_STS_NO_ERROR;
    avrc_cmd.set_app_val.num_val = num_attrib;
    avrc_cmd.set_app_val.pdu = AVRC_PDU_SET_PLAYER_APP_VALUE;
    avrc_cmd.set_app_val.p_vals =
    (tAVRC_APP_SETTING*)GKI_getbuf(sizeof(tAVRC_APP_SETTING)*num_attrib);
    for (count = 0; count < num_attrib; count++)
    {
        avrc_cmd.set_app_val.p_vals[count].attr_id = attrib_ids[count];
        avrc_cmd.set_app_val.p_vals[count].attr_val = attrib_vals[count];
    }
    status = AVRC_BldCommand(&avrc_cmd, &p_msg);
    if (status == AVRC_STS_NO_ERROR)
    {
        UINT8* data_start = (UINT8*)(p_msg + 1) + p_msg->offset;
        BTIF_TRACE_DEBUG("%s msgreq being sent out with label %d",
                __FUNCTION__,p_transaction->lbl);
        if (p_msg != NULL)
        {
            BTA_AvkVendorCmd(btif_avk_rc_cb[index].rc_handle,p_transaction->lbl,AVRC_CMD_CTRL,
                data_start, p_msg->len);
            status =  BT_STATUS_SUCCESS;
        }
    }
    else
    {
         BTIF_TRACE_ERROR("%s: failed to build command. status: 0x%02x",
                            __FUNCTION__, status);
     }
    if (p_msg != NULL)
        GKI_freebuf(p_msg);
     GKI_freebuf(avrc_cmd.set_app_val.p_vals);
#else
    BTIF_TRACE_DEBUG("%s: feature not enabled", __FUNCTION__);
#endif
    return status;
}

/***************************************************************************
**
** Function         register_notification_cmd
**
** Description      Send Command to register for a Notification ID
**
** Returns          void
**
***************************************************************************/
static bt_status_t register_notification_cmd_vendor(uint8_t event_id, uint32_t event_value)
{
    tAVRC_STS status = BT_STATUS_UNSUPPORTED;
    rc_transaction_t *p_transaction=NULL;
    int count  = 0;
#if (AVRC_CTLR_INCLUDED == TRUE)
    tAVRC_COMMAND avrc_cmd = {0};
    BT_HDR *p_msg = NULL;
    int index = BTIF_AVK_RC_DEFAULT_INDEX; //For RC it should be 0
    bt_status_t tran_status;
    CHECK_AVK_RC_CONNECTED
    tran_status = get_transaction(&p_transaction);

    BTIF_TRACE_DEBUG("%s: event_id %d  event_value", __FUNCTION__, event_id, event_value);

    if(BT_STATUS_SUCCESS != tran_status || NULL==p_transaction)
        return BT_STATUS_FAIL;

    avrc_cmd.reg_notif.opcode = AVRC_OP_VENDOR;
    avrc_cmd.reg_notif.status = AVRC_STS_NO_ERROR;
    avrc_cmd.reg_notif.event_id = event_id;
    avrc_cmd.reg_notif.pdu = AVRC_PDU_REGISTER_NOTIFICATION;
    avrc_cmd.reg_notif.param = event_value;
    status = AVRC_BldCommand(&avrc_cmd, &p_msg);
    if (status == AVRC_STS_NO_ERROR)
    {
        UINT8* data_start = (UINT8*)(p_msg + 1) + p_msg->offset;
        BTIF_TRACE_DEBUG("%s msgreq being sent out with label %d",
                __FUNCTION__,p_transaction->lbl);
        if (p_msg != NULL)
        {
            BTA_AvkVendorCmd(btif_avk_rc_cb[index].rc_handle,p_transaction->lbl,AVRC_CMD_NOTIF,
                data_start, p_msg->len);
            status =  BT_STATUS_SUCCESS;
        }
    }
    else
    {
         BTIF_TRACE_ERROR("%s: failed to build command. status: 0x%02x",
                            __FUNCTION__, status);
     }
    if (p_msg != NULL)
        GKI_freebuf(p_msg);
#else
    BTIF_TRACE_DEBUG("%s: feature not enabled", __FUNCTION__);
#endif
    return status;
}

/***************************************************************************
**
** Function         get_element_attribute_cmd
**
** Description      Get Element Attribute for  attributeIds
**
** Returns          void
**
***************************************************************************/
static bt_status_t get_element_attribute_cmd_vendor (uint8_t num_attribute, uint32_t attribute_id)
{
    tAVRC_STS status = BT_STATUS_UNSUPPORTED;
    rc_transaction_t *p_transaction=NULL;
    int count  = 0;
#if (AVRC_CTLR_INCLUDED == TRUE)
    tAVRC_COMMAND avrc_cmd = {0};
    BT_HDR *p_msg = NULL;
    int index = BTIF_AVK_RC_DEFAULT_INDEX; //For RC it should be 0
    bt_status_t tran_status;
    CHECK_AVK_RC_CONNECTED

    BTIF_TRACE_DEBUG("%s: num_attribute  %d attribute_id %d",
                   __FUNCTION__, num_attribute, attribute_id);

    tran_status = get_transaction(&p_transaction);
    if(BT_STATUS_SUCCESS != tran_status || NULL==p_transaction)
        return BT_STATUS_FAIL;

    avrc_cmd.get_elem_attrs.opcode = AVRC_OP_VENDOR;
    avrc_cmd.get_elem_attrs.status = AVRC_STS_NO_ERROR;
    avrc_cmd.get_elem_attrs.num_attr = num_attribute;
    avrc_cmd.get_elem_attrs.pdu = AVRC_PDU_GET_ELEMENT_ATTR;
    avrc_cmd.get_elem_attrs.attrs[0] = 1;
    avrc_cmd.get_elem_attrs.attrs[1] = 2;
    avrc_cmd.get_elem_attrs.attrs[2] = 3;
    avrc_cmd.get_elem_attrs.attrs[3] = 4;
    status = AVRC_BldCommand(&avrc_cmd, &p_msg);
    if (status == AVRC_STS_NO_ERROR)
    {
        UINT8* data_start = (UINT8*)(p_msg + 1) + p_msg->offset;
        BTIF_TRACE_DEBUG("%s msgreq being sent out with label %d",
                __FUNCTION__,p_transaction->lbl);
        if (p_msg != NULL)
        {
            BTA_AvkVendorCmd(btif_avk_rc_cb[index].rc_handle,p_transaction->lbl,AVRC_CMD_STATUS,
                data_start, p_msg->len);
            status =  BT_STATUS_SUCCESS;
        }
    }
    else
    {
         BTIF_TRACE_ERROR("%s: failed to build command. status: 0x%02x",
                            __FUNCTION__, status);
     }
    if (p_msg != NULL)
        GKI_freebuf(p_msg);
#else
    BTIF_TRACE_DEBUG("%s: feature not enabled", __FUNCTION__);
#endif
    return status;
}

/***************************************************************************
**
** Function         get_play_status_cmd
**
** Description      Get Element Attribute for  attributeIds
**
** Returns          void
**
***************************************************************************/
static bt_status_t get_play_status_cmd_vendor(void)
{
    tAVRC_STS status = BT_STATUS_UNSUPPORTED;
    rc_transaction_t *p_transaction=NULL;
#if (AVRC_CTLR_INCLUDED == TRUE)
    tAVRC_COMMAND avrc_cmd = {0};
    BT_HDR *p_msg = NULL;
    int index = BTIF_AVK_RC_DEFAULT_INDEX; //For RC it should be 0
    bt_status_t tran_status;
    CHECK_AVK_RC_CONNECTED

    BTIF_TRACE_DEBUG("%s: ", __FUNCTION__);
    tran_status = get_transaction(&p_transaction);
    if(BT_STATUS_SUCCESS != tran_status || NULL==p_transaction)
        return BT_STATUS_FAIL;

    avrc_cmd.get_play_status.opcode = AVRC_OP_VENDOR;
    avrc_cmd.get_play_status.pdu = AVRC_PDU_GET_PLAY_STATUS;
    avrc_cmd.get_play_status.status = AVRC_STS_NO_ERROR;
    status = AVRC_BldCommand(&avrc_cmd, &p_msg);
    if (status == AVRC_STS_NO_ERROR)
    {
        UINT8* data_start = (UINT8*)(p_msg + 1) + p_msg->offset;
        BTIF_TRACE_DEBUG("%s msgreq being sent out with label %d",
                __FUNCTION__,p_transaction->lbl);
        if (p_msg != NULL)
        {
            BTA_AvkVendorCmd(btif_avk_rc_cb[index].rc_handle,p_transaction->lbl,AVRC_CMD_STATUS,
                data_start, p_msg->len);
            status =  BT_STATUS_SUCCESS;
        }
    }
    else
    {
         BTIF_TRACE_ERROR("%s: failed to build command. status: 0x%02x",
                            __FUNCTION__, status);
     }
    if (p_msg != NULL)
        GKI_freebuf(p_msg);
#else
    BTIF_TRACE_DEBUG("%s: feature not enabled", __FUNCTION__);
#endif
    return status;
}

/***************************************************************************
**
** Function         send_abs_vol_rsp
**
** Description      Rsp for SetAbsoluteVolume Command
**
** Returns          void
**
***************************************************************************/
static bt_status_t send_abs_vol_rsp_vendor(uint8_t abs_vol)
{
    tAVRC_STS status = BT_STATUS_UNSUPPORTED;
#if (AVRC_CTLR_INCLUDED == TRUE)
    tAVRC_RESPONSE avrc_rsp;
    BT_HDR *p_msg = NULL;
    int index = BTIF_AVK_RC_DEFAULT_INDEX; //For RC it should be 0
    CHECK_AVK_RC_CONNECTED

    BTIF_TRACE_DEBUG("%s: abs_vol %d", __FUNCTION__, abs_vol);

    avrc_rsp.volume.opcode = AVRC_OP_VENDOR;
    avrc_rsp.volume.pdu = AVRC_PDU_SET_ABSOLUTE_VOLUME;
    avrc_rsp.volume.status = AVRC_STS_NO_ERROR;
    avrc_rsp.volume.volume = abs_vol;
    status = AVRC_BldResponse(btif_avk_rc_cb[index].rc_handle, &avrc_rsp, &p_msg);
    if (status == AVRC_STS_NO_ERROR)
    {
        UINT8* data_start = (UINT8*)(p_msg + 1) + p_msg->offset;
        BTIF_TRACE_DEBUG("%s msgreq being sent out with label %d",
                __FUNCTION__,btif_avk_rc_cb[index].rc_vol_label);
        if (p_msg != NULL)
        {
            BTA_AvkVendorRsp(btif_avk_rc_cb[index].rc_handle,btif_avk_rc_cb[index].rc_vol_label,BTA_AVK_RSP_ACCEPT,
                  data_start,p_msg->len,0);
            status =  BT_STATUS_SUCCESS;
        }
    }
    else
    {
         BTIF_TRACE_ERROR("%s: failed to build command. status: 0x%02x",
                            __FUNCTION__, status);
     }
    if (p_msg != NULL)
        GKI_freebuf(p_msg);
#else
    BTIF_TRACE_DEBUG("%s: feature not enabled", __FUNCTION__);
#endif
    return status;
}

/***************************************************************************
**
** Function         send_register_abs_vol_rsp
**
** Description      Rsp for Notification of Absolute Volume
**
** Returns          void
**
***************************************************************************/
static bt_status_t send_register_abs_vol_rsp_vendor(uint8_t rsp_type, uint8_t abs_vol)
{
    tAVRC_STS status = BT_STATUS_UNSUPPORTED;
    uint8_t label = 0;
    tAVRC_RESPONSE avrc_rsp;
    BT_HDR *p_msg = NULL;
    int index = BTIF_AVK_RC_DEFAULT_INDEX; //For RC it should be 0

#if (AVRC_CTLR_INCLUDED == TRUE)
    CHECK_AVK_RC_CONNECTED

    BTIF_TRACE_DEBUG("%s: rsp_type  %d abs_vol %d", __FUNCTION__, rsp_type, abs_vol);



    avrc_rsp.reg_notif.opcode = AVRC_OP_VENDOR;
    avrc_rsp.reg_notif.pdu = AVRC_PDU_REGISTER_NOTIFICATION;
    avrc_rsp.reg_notif.status = AVRC_STS_NO_ERROR;
    avrc_rsp.reg_notif.param.volume = abs_vol;
    avrc_rsp.reg_notif.event_id = AVRC_EVT_VOLUME_CHANGE;
    label = btif_avk_rc_cb[index].rc_notif[AVRC_EVT_VOLUME_CHANGE-1].label;
    if ((rsp_type == AVRC_RSP_CHANGED) && (btif_avk_rc_cb[index].rc_notif[AVRC_EVT_VOLUME_CHANGE-1].bNotify))
    {
        btif_avk_rc_cb[index].rc_notif[AVRC_EVT_VOLUME_CHANGE-1].bNotify = FALSE;
    }
    status = AVRC_BldResponse(btif_avk_rc_cb[index].rc_handle, &avrc_rsp, &p_msg);
    if (status == AVRC_STS_NO_ERROR)
    {
        UINT8 *data_start;
        BTIF_TRACE_DEBUG("%s msgreq being sent out with label %d",
                __FUNCTION__,label);
        if (p_msg != NULL)
        {
            data_start = (UINT8*)(p_msg + 1) + p_msg->offset;
            BTA_AvkVendorRsp(btif_avk_rc_cb[index].rc_handle,label,rsp_type,data_start,p_msg->len,0);
        }
        status =  BT_STATUS_SUCCESS;
    }
    else
    {
         BTIF_TRACE_ERROR("%s: failed to build command. status: 0x%02x",
                            __FUNCTION__, status);
     }
    if (p_msg != NULL)
        GKI_freebuf(p_msg);
#else
    BTIF_TRACE_DEBUG("%s: feature not enabled", __FUNCTION__);
#endif
    return status;
}

/*******************************************************************************
**
** Function         btif_avk_rc_idx_by_bdaddr
**
** Description      Get the rc index corresponding to BD addr
**
** Returns          UNIT8
**
*******************************************************************************/

static UINT8 btif_avk_rc_idx_by_bdaddr(BD_ADDR bd_addr)
{
    int i;
    for (i = 0; i < btif_max_rc_clients; i++)
    {
        if ((bdcmp(bd_addr,
                  btif_avk_rc_cb[i].rc_addr) == 0))
            return i;
    }
    return i;
}

/***************************************************************************
**
** Function         send_passthrough_cmd
**
** Description      Send Pass-Through command
**
** Returns          void
**
***************************************************************************/
static bt_status_t send_passthrough_cmd(bt_bdaddr_t *bd_addr, uint8_t key_code, uint8_t key_state)
{
    tAVRC_STS status = BT_STATUS_UNSUPPORTED;
    /* Controller is used only for Certification purposes.
     * In normal case AVRCP controller will not be used, hence
     * updating this is required.
     */
    int index = btif_avk_rc_idx_by_bdaddr(bd_addr->address);
    BTIF_TRACE_DEBUG("%s: index = %d ", __FUNCTION__, index);
    if (index >= btif_max_rc_clients)
    {
        BTIF_TRACE_DEBUG("%s: invalid index", __FUNCTION__);
        return BT_STATUS_FAIL;
    }

#if (AVRC_CTLR_INCLUDED == TRUE)
    rc_transaction_t *p_transaction=NULL;
    CHECK_AVK_RC_CONNECTED

    BTIF_TRACE_DEBUG("%s: key-code: %d, key-state: %d", __FUNCTION__,
                                                    key_code, key_state);
    if (btif_avk_rc_cb[index].rc_features & BTA_AVK_FEAT_RCTG)
    {
        bt_status_t tran_status = get_transaction(&p_transaction);
        if(BT_STATUS_SUCCESS == tran_status && NULL != p_transaction)
        {
            BTA_AvkRemoteCmd(btif_avk_rc_cb[index].rc_handle, p_transaction->lbl,
                (tBTA_AVK_RC)key_code, (tBTA_AVK_STATE)key_state);
            status =  BT_STATUS_SUCCESS;
            BTIF_TRACE_DEBUG("%s: succesfully sent passthrough command to BTA", __FUNCTION__);
        }
        else
        {
            status =  BT_STATUS_FAIL;
            BTIF_TRACE_DEBUG("%s: error in fetching transaction", __FUNCTION__);
        }
    }
    else
    {
        status =  BT_STATUS_FAIL;
        BTIF_TRACE_DEBUG("%s: feature not supported", __FUNCTION__);
    }
#else
    BTIF_TRACE_DEBUG("%s: feature not enabled", __FUNCTION__);
#endif
    return status;
}

static const btrc_ctrl_interface_t btif_avk_rc_ctrl_interface = {
    sizeof(btif_avk_rc_ctrl_interface),
    init_ctrl,
    send_passthrough_cmd,
    cleanup_ctrl,
};

static const btrc_ctrl_vendor_interface_t btif_avk_rc_ctrl_vendor_interface = {
    sizeof(btif_avk_rc_ctrl_vendor_interface),
    init_ctrl_vendor,
    getcapabilities_cmd_vendor,
    list_player_app_setting_attrib_cmd_vendor,
    list_player_app_setting_value_cmd_vendor,
    get_player_app_setting_cmd_vendor,
    set_player_app_setting_cmd_vendor,
    register_notification_cmd_vendor,
    get_element_attribute_cmd_vendor,
    get_play_status_cmd_vendor,
    send_abs_vol_rsp_vendor,
    send_register_abs_vol_rsp_vendor,
    cleanup_ctrl_vendor,
};

/*******************************************************************************
**
** Function         btif_avk_rc_ctrl_get_interface
**
** Description      Get the AVRCP Controller callback interface
**
** Returns          btav_interface_t
**
*******************************************************************************/
const btrc_ctrl_interface_t *btif_avk_rc_ctrl_get_interface(void)
{
    BTIF_TRACE_EVENT("%s", __FUNCTION__);
    return &btif_avk_rc_ctrl_interface;
}

/*******************************************************************************
**
** Function         btif_avk_rc_ctrl_vendor_get_interface
**
** Description      Get the AVRCP Controller callback vendor interface
**
** Returns          btrc_ctrl_vendor_interface_t
**
*******************************************************************************/
const btrc_ctrl_vendor_interface_t *btif_avk_rc_ctrl_vendor_get_interface(void)
{
    BTIF_TRACE_EVENT("%s", __FUNCTION__);
    return &btif_avk_rc_ctrl_vendor_interface;
}

/*******************************************************************************
**      Function         initialize_transaction
**
**      Description    Initializes fields of the transaction structure
**
**      Returns          void
*******************************************************************************/
static void initialize_transaction(int lbl)
{
    pthread_mutex_lock(&device.lbllock);
    if(lbl < MAX_TRANSACTIONS_PER_SESSION)
    {
       device.transaction[lbl].lbl = lbl;
       device.transaction[lbl].in_use=FALSE;
       device.transaction[lbl].handle=0;
    }
    pthread_mutex_unlock(&device.lbllock);
}

/*******************************************************************************
**      Function         lbl_init
**
**      Description    Initializes label structures and mutexes.
**
**      Returns         void
*******************************************************************************/
static void lbl_init()
{
    memset(&device,0,sizeof(rc_device_t));
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutex_init(&(device.lbllock), &attr);
    pthread_mutexattr_destroy(&attr);
    init_all_transactions();
}

/*******************************************************************************
**
** Function         init_all_transactions
**
** Description    Initializes all transactions
**
** Returns          void
*******************************************************************************/
static void init_all_transactions()
{
    UINT8 txn_indx=0;
    for(txn_indx=0; txn_indx < MAX_TRANSACTIONS_PER_SESSION; txn_indx++)
    {
        initialize_transaction(txn_indx);
    }
}

/*******************************************************************************
**
** Function         get_transaction_by_lbl
**
** Description    Will return a transaction based on the label. If not inuse
**                     will return an error.
**
** Returns          bt_status_t
*******************************************************************************/
static rc_transaction_t *get_transaction_by_lbl(UINT8 lbl)
{
    rc_transaction_t *transaction = NULL;
    pthread_mutex_lock(&device.lbllock);

    /* Determine if this is a valid label */
    if (lbl < MAX_TRANSACTIONS_PER_SESSION)
    {
        if (FALSE==device.transaction[lbl].in_use)
        {
            transaction = NULL;
        }
        else
        {
            transaction = &(device.transaction[lbl]);
            BTIF_TRACE_DEBUG("%s: Got transaction.label: %d",__FUNCTION__,lbl);
        }
    }

    pthread_mutex_unlock(&device.lbllock);
    return transaction;
}

/*******************************************************************************
**
** Function         get_transaction
**
** Description    Obtains the transaction details.
**
** Returns          bt_status_t
*******************************************************************************/

static bt_status_t  get_transaction(rc_transaction_t **ptransaction)
{
    bt_status_t result = BT_STATUS_NOMEM;
    UINT8 i=0;
    pthread_mutex_lock(&device.lbllock);

    // Check for unused transactions
    for (i=0; i<MAX_TRANSACTIONS_PER_SESSION; i++)
    {
        if (FALSE==device.transaction[i].in_use)
        {
            BTIF_TRACE_DEBUG("%s:Got transaction.label: %d",__FUNCTION__,device.transaction[i].lbl);
            device.transaction[i].in_use = TRUE;
            *ptransaction = &(device.transaction[i]);
            result = BT_STATUS_SUCCESS;
            break;
        }
    }

    pthread_mutex_unlock(&device.lbllock);
    return result;
}


/*******************************************************************************
**
** Function         release_transaction
**
** Description    Will release a transaction for reuse
**
** Returns          bt_status_t
*******************************************************************************/
static void release_transaction(UINT8 lbl)
{
    rc_transaction_t *transaction = get_transaction_by_lbl(lbl);

    /* If the transaction is in use... */
    if (transaction != NULL)
    {
        BTIF_TRACE_DEBUG("%s: lbl: %d", __FUNCTION__, lbl);
        initialize_transaction(lbl);
    }
}

/*******************************************************************************
**
** Function         lbl_destroy
**
** Description    Cleanup of the mutex
**
** Returns          void
*******************************************************************************/
static void lbl_destroy()
{
    if (!pthread_mutex_destroy(&(device.lbllock)))
    {
        device.lbllock_destroyed = TRUE;
        BTIF_TRACE_EVENT(" %s: lbllock destroy success ", __FUNCTION__);
    }
}
