/*
 * Copyright (c) 2017, The Linux Foundation. All rights reserved.
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

#include "Gatt.hpp"
#include "GattcTest.hpp"
#include "utils.h"



#define LOGTAG "GATTCTEST "
#define UNUSED


ServiceData gattctestServData;

int testest;

int gattctestserverif, gattctestclientif;

GattcTest *gattctest = NULL;

bt_uuid_t client_uuid;
bt_uuid_t gen_uuid;

#define ADDRESS_STR_LEN 18
#define UUID_STR_LEN 37
#define HEX_VAL_STR_LEN 100

#define CHARID_STR_LEN UUID_STR_LEN + 3 + 11
#define SRVCID_STR_LEN UUID_STR_LEN + 3 + 11 + 1 + 11
#define desc_id_to_string gatt_id_to_string

#define MAX_NOTIFY_PARAMS_STR_LEN (SRVCID_STR_LEN + CHARID_STR_LEN \
+ ADDRESS_STR_LEN + HEX_VAL_STR_LEN + 60)
#define MAX_READ_PARAMS_STR_LEN (SRVCID_STR_LEN + CHARID_STR_LEN \
+ UUID_STR_LEN + HEX_VAL_STR_LEN + 80)




static char *uuid_to_string(const bt_uuid_t *uuid, char *buf)
{
int shift = 0;
int i = 16;
int limit = 0;
int j = 0;

/* for bluetooth uuid only 32 bits */
if (0 == memcmp(&uuid->uu, &(gen_uuid.uu),
sizeof(bt_uuid_t) - 4)) {
limit = 12;
/* make it 16 bits */
if (uuid->uu[15] == 0 && uuid->uu[14] == 0)
i = 14;
}

while (i-- > limit) {
if (i == 11 || i == 9 || i == 7 || i == 5) {
buf[j * 2 + shift] = '-';
shift++;
}

sprintf(buf + j * 2 + shift, "%02x", uuid->uu[i]);
++j;
}

return buf;
}


/* service_id formating function */
char *service_id_to_string(const btgatt_srvc_id_t *srvc_id, char *buf){
char uuid_buf[UUID_STR_LEN];
sprintf(buf, "{%s,%d,%d}", uuid_to_string(&srvc_id->id.uuid, uuid_buf),srvc_id->id.inst_id, srvc_id->is_primary);
return buf;
}


static char *gatt_id_to_string(const btgatt_gatt_id_t *char_id, char *buf)
{
char uuid_buf[UUID_STR_LEN];

sprintf(buf, "{%s,%d}", uuid_to_string(&char_id->uuid, uuid_buf),
char_id->inst_id);
return buf;
}

static char *arr_to_string(const uint8_t *v, int size, char *buf, int out_size)
{
int limit = size;
int i;

if (out_size > 0) {
*buf = '\0';
if (size >= 2 * out_size)
limit = (out_size - 2) / 2;

for (i = 0; i < limit; ++i)
sprintf(buf + 2 * i, "%02x", v[i]);

/* output buffer not enough to hold whole field fill with ...*/
if (limit < size)
sprintf(buf + 2 * i, "...");
}

return buf;
}


static char *raw_data_to_string(const btgatt_unformatted_value_t *v,
char *buf, int size)
{
return arr_to_string(v->value, v->len, buf, size);
}

static char *read_param_to_string(const btgatt_read_params_t *data,
             char *buf)
{
char srvc_id[SRVCID_STR_LEN];
char char_id[CHARID_STR_LEN];
char descr_id[UUID_STR_LEN];
char value[HEX_VAL_STR_LEN];
sprintf(buf, "{srvc_id=%s, char_id=%s, descr_id=%s, val=%s value_type=%d, status=%d}",
service_id_to_string(&data->srvc_id, srvc_id),
gatt_id_to_string(&data->char_id, char_id),
desc_id_to_string(&data->descr_id, descr_id),
raw_data_to_string(&data->value, value, 100),
data->value_type, data->status);
return buf;
}


/*****************************/


class gattctestClientCallback : public BluetoothGattClientCallback
{
   public:
   void btgattc_client_register_app_cb(int status,int client_if,bt_uuid_t *uuid) {

        fprintf(stdout,"gattctest btgattc_client_register_app_cb\n ");

        GattcRegisterAppEvent event;
        event.event_id = RSP_ENABLE_EVENT;
        event.status = status;
        event.clientIf = client_if;
        if(gattctest) {
        fprintf(stdout,"gattctest is not null \n");
        }
        gattctest->SetGATTCTESTClientAppData(&event);



//start scan
       // gattctest->ClientSetAdvData("Remote Start Profile");
       // gattctest->StartAdvertisement();
   }

   void btgattc_scan_result_cb(bt_bdaddr_t* bda, int rssi, uint8_t* adv_data) {
         bdstr_t bd_str;
         bdaddr_to_string(bda, &bd_str[0], sizeof(bd_str));
         fprintf(stdout,"btgattc_scan_result_cb %s \n ", bd_str);
   }

   void btgattc_open_cb(int conn_id, int status, int clientIf, bt_bdaddr_t* bda)
   {
    fprintf(stdout,"btgattc_open_cb  gattctest   status is %d\n ", status);

    GattcOpenEvent event;
    event.event_id = BTGATTC_OPEN_EVENT;
    event.conn_id = conn_id;
    event.clientIf = clientIf;
    event.bda = bda;

    if (gattctest) {
        gattctest->SetGATTCTESTConnectionData(&event);
        //if (status == 0)
        //{
            gattctest->SearchService(conn_id);
        //}
    }

   }

   void btgattc_close_cb(int conn_id, int status, int clientIf, bt_bdaddr_t* bda)
   {
        fprintf(stdout,"btgattc_close_cb  gattctest \n ");
   }

   void btgattc_search_complete_cb(int conn_id, int status)
   {
        fprintf(stdout,"btgattc_search_complete_cb  conn_id %d, status %d \n ", conn_id,status);

   }

   void btgattc_search_result_cb(int conn_id, btgatt_srvc_id_t *srvc_id)
   {

         char srvc_id_buf[(SRVCID_STR_LEN)];

         fprintf(stdout,"%s: conn_id=%d srvc_id=%s\n", __func__, conn_id,service_id_to_string(srvc_id, srvc_id_buf));

         gattctestServData.conn_id = conn_id;
         gattctestServData.srvc_id = srvc_id;
         gattctest->app_gatt->get_characteristic(conn_id,srvc_id,NULL);
   }

   void btgattc_get_characteristic_cb(int conn_id, int status,
                                     btgatt_srvc_id_t *srvc_id, btgatt_gatt_id_t *char_id,
                                     int char_prop)
   {

        char srvc_id_buf[SRVCID_STR_LEN];
        char char_id_buf[CHARID_STR_LEN];

        fprintf(stdout,"%s: conn_id=%d status=%d srvc_id=%s char_id=%s, char_prop=%x\n",
           __func__, conn_id, status,
           service_id_to_string(srvc_id, srvc_id_buf),
           gatt_id_to_string(char_id, char_id_buf), char_prop);


          if(status == 0)
         gattctest->app_gatt->get_characteristic(conn_id,srvc_id,char_id);

         gattctest->app_gatt->get_descriptor(conn_id,srvc_id,char_id,NULL);
     
   }

   void btgattc_get_descriptor_cb(int conn_id, int status,
                                 btgatt_srvc_id_t *srvc_id, btgatt_gatt_id_t *char_id,
                                 btgatt_gatt_id_t *descr_id)
   {
       char buf[UUID_STR_LEN];
       char srvc_id_buf[SRVCID_STR_LEN];
       char char_id_buf[CHARID_STR_LEN];

       fprintf(stdout,"%s: conn_id=%d status=%d srvc_id=%s char_id=%s, descr_id=%s\n",
               __func__, conn_id, status,
              service_id_to_string(srvc_id, srvc_id_buf),
              gatt_id_to_string(char_id, char_id_buf),
              desc_id_to_string(descr_id, buf));
          if(status == 0) {
         gattctest->app_gatt->get_descriptor(conn_id,srvc_id,char_id,descr_id);
         gattctestServData.conn_id = conn_id;
         gattctestServData.srvc_id = srvc_id;
         gattctestServData.char_id = char_id;
         gattctestServData.descr_id = descr_id;
          }

   }

   void btgattc_register_for_notification_cb(int conn_id, int registered,
                                                int status, btgatt_srvc_id_t *srvc_id,
                                                btgatt_gatt_id_t *char_id)
   {
        UNUSED
   }

   void btgattc_notify_cb(int conn_id, btgatt_notify_params_t *p_data)
   {
        UNUSED
   }

   void btgattc_read_characteristic_cb(int conn_id, int status,
                                          btgatt_read_params_t *p_data)
   {
        char buf[MAX_READ_PARAMS_STR_LEN];

        fprintf(stdout,"%s: conn_id=%d status=%d data=%s\n", __func__, conn_id,
                status, read_param_to_string(p_data, buf));
   }

   void btgattc_write_characteristic_cb(int conn_id, int status,
                                           btgatt_write_params_t *p_data)
   {
        fprintf(stdout,"btgattc_write_characteristic_cb status is %d conn_id %d \n ", status,conn_id);
   }

   void btgattc_read_descriptor_cb(int conn_id, int status, btgatt_read_params_t *p_data)
   {

   char buf[MAX_READ_PARAMS_STR_LEN];

   fprintf(stdout,"%s: conn_id=%d status=%d data=%s\n", __func__, conn_id,
           status, read_param_to_string(p_data, buf));

   }

    void btgattc_write_descriptor_cb(int conn_id, int status, btgatt_write_params_t *p_data)
    {
        UNUSED
    }

   void btgattc_execute_write_cb(int conn_id, int status)
   {
        UNUSED
   }

   void btgattc_remote_rssi_cb(int client_if,bt_bdaddr_t* bda, int rssi, int status)
   {
       UNUSED
   }

   void btgattc_advertise_cb(int status, int client_if)
   {
        UNUSED

   }

   void btgattc_configure_mtu_cb(int conn_id, int status, int mtu)
   {
        UNUSED
   }

   void btgattc_get_included_service_cb(int conn_id, int status,
                                       btgatt_srvc_id_t *srvc_id, btgatt_srvc_id_t *incl_srvc_id)
   {
        UNUSED
   }

   void btgattc_scan_filter_cfg_cb(int action, int client_if, int status, int filt_type, int avbl_space)
   {
        UNUSED
   }

   void btgattc_scan_filter_param_cb(int action, int client_if, int status, int avbl_space)
   {
        UNUSED
   }

   void btgattc_scan_filter_status_cb(int action, int client_if, int status)
   {
        UNUSED
   }

   void btgattc_multiadv_enable_cb(int client_if, int status)
   {
        UNUSED
   }

   void btgattc_multiadv_update_cb(int client_if, int status)
   {
        UNUSED
   }

    void btgattc_multiadv_setadv_data_cb(int client_if, int status)
   {
        UNUSED
   }

   void btgattc_multiadv_disable_cb(int client_if, int status)
   {
        UNUSED
   }

   void btgattc_congestion_cb(int conn_id, bool congested)
   {
        UNUSED
   }

   void btgattc_batchscan_cfg_storage_cb(int client_if, int status)
   {
        UNUSED
   }

   void btgattc_batchscan_startstop_cb(int startstop_action, int client_if, int status)
   {
        UNUSED

   }

   void btgattc_batchscan_reports_cb(int client_if, int status, int report_format,
        int num_records, int data_len, uint8_t *p_rep_data)
   {
        UNUSED
   }

   void btgattc_batchscan_threshold_cb(int client_if)
   {
        UNUSED
   }

   void btgattc_track_adv_event_cb(btgatt_track_adv_info_t *p_adv_track_info)
   {
        UNUSED
   }

   void btgattc_scan_parameter_setup_completed_cb(int client_if, btgattc_error_t status)
   {
        UNUSED
   }

};

class gattctestServerCallback :public BluetoothGattServerCallback
{

      public:

      void gattServerRegisterAppCb(int status, int server_if, bt_uuid_t *uuid) {

           fprintf(stdout,"gattServerRegisterAppCb status is %d, serverif is %d \n ",
                   status, server_if);

           if (status == BT_STATUS_SUCCESS)
           {
              GattsRegisterAppEvent rev;
              rev.event_id = RSP_ENABLE_EVENT;
              rev.server_if = server_if;
              rev.uuid = uuid;
              rev.status = status;
              fprintf(stdout," set gattctest data \n");
              gattctest->SetGATTCTESTAppData(&rev);
              gattctest->AddService();
           } else {
              fprintf (stdout,"(%s) Failed to registerApp, %d \n",__FUNCTION__, server_if);
           }
      }

      void btgatts_connection_cb(int conn_id, int server_if, int connected, bt_bdaddr_t *bda)
      {

           fprintf(stdout,"btgatts_connection_cb  gattctest \n ");

      }

      void btgatts_service_added_cb(int status, int server_if,
                                    btgatt_srvc_id_t *srvc_id, int srvc_handle)
      {
           fprintf(stdout,"btgatts_service_added_cb \n");
           if (status == BT_STATUS_SUCCESS) {
              GattsServiceAddedEvent event;
               event.event_id =RSP_ENABLE_EVENT;
               event.server_if = server_if;
               event.srvc_id = srvc_id;
               event.srvc_handle = srvc_handle;
               gattctest->SetGATTCTESTSrvcData(&event);
               gattctest->AddCharacteristics();
           } else {
               fprintf(stdout, "(%s) Failed to Add_Service %d ",__FUNCTION__, server_if);
           }
      }

      void btgatts_included_service_added_cb(int status, int server_if, int srvc_handle,
                                                   int incl_srvc_handle)
      {
            UNUSED;
      }

      void btgatts_characteristic_added_cb(int status, int server_if, bt_uuid_t *char_id,
                                                      int srvc_handle, int char_handle)
      {
           fprintf(stdout,"btgatts_characteristic_added_cb \n");
           if (status == BT_STATUS_SUCCESS) {
               GattsCharacteristicAddedEvent event;
               event.event_id =RSP_ENABLE_EVENT;
               event.server_if = server_if;
               event.char_id = char_id;
               event.srvc_handle = srvc_handle;
               event.char_handle = char_handle;
               gattctest->SetGATTCTESTCharacteristicData(&event);
               gattctest->AddDescriptor();
           } else {
               fprintf(stdout, "(%s) Failed to Add Characteristics %d ",__FUNCTION__, server_if);
           }
      }

      void btgatts_descriptor_added_cb(int status, int server_if, bt_uuid_t *descr_id,
                                                  int srvc_handle, int descr_handle)
      {
           fprintf(stdout,"btgatts_descriptor_added_cb \n");
           if (status == BT_STATUS_SUCCESS) {
               GattsDescriptorAddedEvent event;
               event.event_id =RSP_ENABLE_EVENT;
               event.server_if = server_if;
               event.descr_id= descr_id;
               event.srvc_handle = srvc_handle;
               event.descr_handle= descr_handle;
               gattctest->SetGATTCTESTDescriptorData(&event);
               gattctest->StartService();
            } else {
               fprintf(stdout, "(%s) Failed to add descriptor %d \n",__FUNCTION__, server_if);
            }
      }

      void btgatts_service_started_cb(int status, int server_if, int srvc_handle)
      {
           fprintf(stdout,"btgatts_service_started_cb \n");
          // gattctest->RegisterClient();
      }

      void btgatts_service_stopped_cb(int status, int server_if, int srvc_handle)
      {
           fprintf(stdout,"btgatts_service_stopped_cb \n");

          if (gattctest) {
              if (!status)
                  gattctest->DeleteService();
          }
          fprintf(stdout,  "GATTCTEST Service stopped successfully, deleting the service");
      }

      void btgatts_service_deleted_cb(int status, int server_if, int srvc_handle)
      {
         fprintf(stdout,"btgatts_service_deleted_cb \n");

          if (gattctest) {
              if (!status) {
                  gattctest->CleanUp(server_if);
                  delete gattctest;
                  gattctest = NULL;
              }
          }
          fprintf(stdout,"GATTCTEST Service stopped & Unregistered successfully\n");
      }

      void btgatts_request_read_cb(int conn_id, int trans_id, bt_bdaddr_t *bda, int attr_handle,
                                              int offset, bool is_long)
      {
           UNUSED;
      }

      void btgatts_request_write_cb(int conn_id, int trans_id, bt_bdaddr_t *bda, int attr_handle,
                                              int offset, int length, bool need_rsp, bool is_prep,
                                              uint8_t* value)
      {
           fprintf(stdout,"onCharacteristicWriteRequest \n");
           GattsRequestWriteEvent event;
           event.event_id = RSP_ENABLE_EVENT;
           event.conn_id = conn_id;
           event.trans_id = trans_id;
           event.bda = bda;
           event.attr_handle = attr_handle;
           event.offset = offset;
           event.length = length;
           event.need_rsp = need_rsp;
           event.is_prep = is_prep;
           event.value = value;
           gattctest->SendResponse(&event);
      }

      void btgatts_request_exec_write_cb(int conn_id, int trans_id,
                                                      bt_bdaddr_t *bda, int exec_write)
      {
           UNUSED;
      }

      void btgatts_response_confirmation_cb(int status, int handle)
      {
           UNUSED;
      }

      void btgatts_indication_sent_cb(int conn_id, int status)
      {
           UNUSED;
      }

      void btgatts_congestion_cb(int conn_id, bool congested)
      {
           UNUSED;
      }

      void btgatts_mtu_changed_cb(int conn_id, int mtu)
      {
           UNUSED;
      }
};

gattctestServerCallback gattctestServerCb;
gattctestClientCallback gattctestClientCb;



GattcTest::GattcTest(btgatt_interface_t *gatt_itf, Gatt* gatt)
{

    fprintf(stdout,"gattctest instantiated ");
    gatt_interface = gatt_itf;
    app_gatt = gatt;
}


GattcTest::~GattcTest()
{
    fprintf(stdout, "(%s) GATTCTEST DeInitialized",__FUNCTION__);
   // SetDeviceState(WLAN_INACTIVE);
}

bool GattcTest::CopyUUID(bt_uuid_t *uuid)
{
    CHECK_PARAM(uuid)
    for (int i = 0; i < 16; i++) {
        uuid->uu[i] = 0x30;
    }
    return true;
}

bool GattcTest::CopyClientUUID(bt_uuid_t *uuid)
{
    CHECK_PARAM(uuid)
    uuid->uu[0] = 0xff;
    for (int i = 1; i < 16; i++) {
        uuid->uu[i] = 0x30;
    }
    return true;
}

bool GattcTest::CopyGenUUID(bt_uuid_t *uuid)
{
    CHECK_PARAM(uuid)
     uuid->uu[0] = 0xfb;
     uuid->uu[1] = 0x34;
     uuid->uu[2] = 0x9b;
     uuid->uu[3] = 0x5f;
     uuid->uu[4] = 0x80;
     uuid->uu[5] =0x00;
     uuid->uu[6] = 0x00;
     uuid->uu[7] = 0x80;
     uuid->uu[8] =0x00;
     uuid->uu[9] = 0x10;
     uuid->uu[10] = 0x00;
     uuid->uu[11] = 0x00;
     uuid->uu[12] = 0x00;
     uuid->uu[13] = 0x00;
     uuid->uu[14] = 0x00;
     uuid->uu[15] = 0x00;

    return true;
}



bool GattcTest::CopyParams(bt_uuid_t *uuid_dest, bt_uuid_t *uuid_src)
{
    CHECK_PARAM(uuid_dest)
    CHECK_PARAM(uuid_src)

    for (int i = 0; i < 16; i++) {
        uuid_dest->uu[i] = uuid_src->uu[i];
    }
    return true;
}

bool GattcTest::MatchParams(bt_uuid_t *uuid_dest, bt_uuid_t *uuid_src)
{
    CHECK_PARAM(uuid_dest)
    CHECK_PARAM(uuid_src)

    for (int i = 0; i < 16; i++) {
        if(uuid_dest->uu[i] != uuid_src->uu[i])
            return false;
    }
    fprintf(stdout, "(%s) UUID Matches",__FUNCTION__);
    return true;
}

bool GattcTest::EnableGATTCTEST()
{
    fprintf(stdout, "(%s) Enable GATTCTEST Initiated \n",__FUNCTION__);
    CopyClientUUID(&client_uuid);
    CopyGenUUID(&gen_uuid);
    gattctest->RegisterClient();
}

bool GattcTest::DisableGATTCTEST(int server_if)
{
    fprintf(stdout, "(%s) Disable GATTCTEST Initiated",__FUNCTION__);
}

bool GattcTest::RegisterApp()
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }
    bt_uuid_t server_uuid = GetGATTCTESTAttrData()->server_uuid;
    fprintf(stdout,"reg app addr is %d \n", GetGATTCTESTAttrData()->server_uuid);
    app_gatt->RegisterServerCallback(&gattctestServerCb,&GetGATTCTESTAttrData()->server_uuid);
    return app_gatt->register_server(&server_uuid) == BT_STATUS_SUCCESS;
}

bool GattcTest::RegisterClient()
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }
   // bt_uuid_t client_uuid = GetGATTCTESTAttrData()->client_uuid;
    app_gatt->RegisterClientCallback(&gattctestClientCb,&client_uuid);
    return app_gatt->register_client(&client_uuid) == BT_STATUS_SUCCESS;
}

bool GattcTest::UnregisterClient(int client_if)
{
    if (GetGattInterface() == NULL) {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }
    app_gatt->UnRegisterClientCallback(client_if);
    return app_gatt->unregister_client(client_if) == BT_STATUS_SUCCESS;
}

bool GattcTest::ClientSetAdvData(char *str)
{
    bt_status_t        Ret;
    bool              SetScanRsp        = false;
    bool              IncludeName       = true;
    bool              IncludeTxPower    = false;
    int               min_conn_interval = RSP_MIN_CI;
    int               max_conn_interval = RSP_MAX_CI;

    app_gatt->set_adv_data(GetGATTCTESTClientAppData()->clientIf, SetScanRsp,
                                                IncludeName, IncludeTxPower, min_conn_interval,
                                                max_conn_interval, 0,strlen(str), str,
                                                strlen(str), str, 0,NULL);
}

void GattcTest::CleanUp(int server_if)
{
    UnregisterServer(server_if);
    UnregisterClient(GetGATTCTESTClientAppData()->clientIf);
}

bool GattcTest::UnregisterServer(int server_if)
{
    if (GetGattInterface() == NULL) {
        ALOGE(LOGTAG  "Gatt Interface Not present");
        return false;
    }
    app_gatt->UnRegisterServerCallback(server_if);
    return app_gatt->unregister_server(server_if) == BT_STATUS_SUCCESS;
}

bool GattcTest::StartAdvertisement()
{
    if (GetGattInterface() == NULL) {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }
    fprintf(stdout,  "(%s) Listening on the interface (%d) ",__FUNCTION__,
            GetGATTCTESTAppData()->server_if);
    //SetDeviceState(WLAN_INACTIVE);
    return app_gatt->listen(GetGATTCTESTClientAppData()->clientIf, true);
}

bool GattcTest::StartScan()
{
 if (GetGattInterface() == NULL) {
     ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
     return false;
 }
 ALOGE(LOGTAG  "(%s) start scan",__FUNCTION__);

return app_gatt->scan(true, GetGATTCTESTClientAppData()->clientIf);

}

bool GattcTest::StopScan()
{
 if (GetGattInterface() == NULL) {
     ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
     return false;
 }
 ALOGE(LOGTAG  "(%s) stop scan",__FUNCTION__);

return app_gatt->scan(false, GetGATTCTESTClientAppData()->clientIf);

}

bool GattcTest::Connect(const bt_bdaddr_t *bd_addr)
{
 if (GetGattInterface() == NULL) {
     ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
     return false;
 }
 ALOGE(LOGTAG  "(%s) Connect",__FUNCTION__);
;
return app_gatt->clientConnect(GetGATTCTESTClientAppData()->clientIf,bd_addr,true,GATT_TRANSPORT_LE);

}

bool GattcTest::Disconnect(const bt_bdaddr_t *bd_addr)
{
 if (GetGattInterface() == NULL) {
     ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
     return false;
 }
 ALOGE(LOGTAG  "(%s) Disconnect",__FUNCTION__);

return app_gatt->clientDisconnect(GetGATTCTESTConnectionData()->clientIf,bd_addr,GetGATTCTESTConnectionData()->conn_id);

}

bool GattcTest::SendAlert(int alert_level)
{
 if (GetGattInterface() == NULL) {
     ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
     return false;
 }

 fprintf(stdout,"sending alert now \n");
 char alert[20];
 memset( (void *) alert, '\0', sizeof(alert));

 if (alert_level == LOW_ALERT )
     return gattctest->app_gatt->write_characteristic(gattctestServData.conn_id,gattctestServData.srvc_id,gattctestServData.char_id,1,2,0,"00");
 else if (alert_level == MID_ALERT)
     return gattctest->app_gatt->write_characteristic(gattctestServData.conn_id,gattctestServData.srvc_id,gattctestServData.char_id,1,2,0,"01");
 else if (alert_level == HIGH_ALERT)
      return gattctest->app_gatt->write_characteristic(gattctestServData.conn_id,gattctestServData.srvc_id,gattctestServData.char_id,1,2,0,"02");

}

bool GattcTest::SearchService(int conn_id)
{
 if (GetGattInterface() == NULL) {
     ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
     return false;
 }
 ALOGE(LOGTAG  "(%s) SearchService",__FUNCTION__);

return app_gatt->search_service(conn_id, NULL);

}

bool GattcTest::SendResponse(GattsRequestWriteEvent *event)
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present \n",__FUNCTION__);
        return false;
    }
    CHECK_PARAM(event)
    btgatt_response_t att_resp;
    int response = -1;
    memset(att_resp.attr_value.value,0,BTGATT_MAX_ATTR_LEN);
    memcpy(att_resp.attr_value.value, event->value, event->length);
    att_resp.attr_value.handle = event->attr_handle;
    att_resp.attr_value.offset = event->offset;
    att_resp.attr_value.len = event->length;
    att_resp.attr_value.auth_req = 0;

    if(!strncasecmp((const char *)(event->value), "on", 2)) {
     //   if (GetDeviceState() == WLAN_INACTIVE)
       // {
       //     HandleWlanOn();
       //     SetDeviceState(WLAN_TRANSACTION_PENDING);
       // }
        response = 0;
    } else {
        response = -1;
    }

    fprintf(stdout, "(%s) Sending GATTCTEST response to write (%d) value (%s) State (%d)",__FUNCTION__,
            GetGATTCTESTAppData()->server_if, event->value,GetDeviceState());

    return app_gatt->send_response(event->conn_id, event->trans_id,
                                                         response, &att_resp);
}

bool GattcTest::HandleWlanOn()
{
    BtEvent *event = new BtEvent;
    CHECK_PARAM(event);
    event->event_id = SKT_API_IPC_MSG_WRITE;
    event->bt_ipc_msg_event.ipc_msg.type = BT_IPC_REMOTE_START_WLAN;
    event->bt_ipc_msg_event.ipc_msg.status = INITIATED;
    StopAdvertisement();
    fprintf(stdout, "(%s) Posting wlan start to main thread \n",__FUNCTION__);
    PostMessage (THREAD_ID_MAIN, event);
    return true;
}

bool GattcTest::StopAdvertisement()
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }
    fprintf(stdout, "(%s) Stopping listen on the interface (%d) \n",__FUNCTION__,
            GetGATTCTESTClientAppData()->clientIf);
    return app_gatt->listen(GetGATTCTESTClientAppData()->clientIf, false);
}

bool GattcTest::AddService()
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }
    btgatt_srvc_id_t srvc_id;
    srvc_id.id.inst_id = 0;   // 1 instance
    srvc_id.is_primary = 1;   // Primary addition
    srvc_id.id.uuid = GetGATTCTESTAttrData()->service_uuid;
    return app_gatt->add_service(GetGATTCTESTAppData()->server_if, &srvc_id,4)
                                                        ==BT_STATUS_SUCCESS;
}

bool GattcTest::DisconnectServer()
{
    int server_if = GetGATTCTESTConnectionData()->clientIf;
    bt_bdaddr_t * bda = GetGATTCTESTConnectionData()->bda;
    int conn_id = GetGATTCTESTConnectionData()->conn_id;
    fprintf(stdout,  "(%s) Disconnecting interface (%d), connid (%d) ",__FUNCTION__,
            server_if, conn_id);
    return app_gatt->serverDisconnect(server_if, bda, conn_id) == BT_STATUS_SUCCESS;
}

bool GattcTest::DeleteService()
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }
    bool status = false;
    int srvc_handle = GetGATTCTESTSrvcData()->srvc_handle;
    return app_gatt->delete_service(GetGATTCTESTAppData()->server_if,
                                                            srvc_handle) == BT_STATUS_SUCCESS;
}

bool GattcTest::AddCharacteristics()
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }
    bt_uuid_t char_uuid;
    CopyParams(&char_uuid, &(GetGATTCTESTSrvcData()->srvc_id->id.uuid));
    int srvc_handle = GetGATTCTESTSrvcData()->srvc_handle;
    int server_if = GetGATTCTESTSrvcData()->server_if;
    fprintf(stdout,  "(%s) Adding Characteristics server_if (%d), srvc_handle (%d) \n",
            __FUNCTION__, server_if,srvc_handle);
    return app_gatt->add_characteristic(server_if, srvc_handle, &char_uuid,
                                                            GATT_PROP_WRITE, GATT_PERM_WRITE)
                                                            ==BT_STATUS_SUCCESS;
}

bool GattcTest::AddDescriptor(void)
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }

    bt_uuid_t desc_uuid;
    desc_uuid = GetGATTCTESTAttrData()->descriptor_uuid;
    int srvc_handle = GetGATTCTESTSrvcData()->srvc_handle;
    return app_gatt->add_descriptor(GetGATTCTESTAppData()->server_if,
                                                        srvc_handle, &desc_uuid,
                                                        GATT_PERM_READ) == BT_STATUS_SUCCESS;
}

bool GattcTest::StartService()
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }

    int srvc_handle = GetGATTCTESTSrvcData()->srvc_handle;
    return app_gatt->start_service(GetGATTCTESTAppData()->server_if,
                                                        srvc_handle, GATT_TRANSPORT_LE)
                                                        == BT_STATUS_SUCCESS;
}

bool GattcTest::StopService()
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }

    int srvc_handle = GetGATTCTESTSrvcData()->srvc_handle;
    return app_gatt->stop_service(GetGATTCTESTAppData()->server_if,
                                                        srvc_handle) == BT_STATUS_SUCCESS;
}
