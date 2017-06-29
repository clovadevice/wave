/******************************************************************************
 *
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

/*******************************************************************************
 *
 *  Filename:      btif_sock.h
 *
 *  Description:   Bluetooth socket Interface
 *
 *******************************************************************************/

#ifndef BTIF_SOCK_RFC_H
#define BTIF_SOCK_RFC_H

bt_status_t btsock_rfc_init(int handle);
bt_status_t btsock_rfc_cleanup();
bt_status_t btsock_rfc_listen(const char* name, const uint8_t* uuid, int channel,
                              int* sock_fd, int flags);
bt_status_t btsock_rfc_connect(const bt_bdaddr_t *bd_addr, const uint8_t* uuid,
                               int channel, int* sock_fd, int flags);
bt_status_t btsock_rfc_get_sockopt(int channel, btsock_option_type_t option_name,
                                            void *option_value, int *option_len);
bt_status_t btsock_rfc_set_sockopt(int channel, btsock_option_type_t option_name,
                                            void *option_value, int option_len);
void btsock_rfc_signaled(int fd, int flags, uint32_t user_id);
int send_data_test(char *addr, char *data, int len);
#endif