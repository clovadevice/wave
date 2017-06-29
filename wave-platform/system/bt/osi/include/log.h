/******************************************************************************
 *
 *  Copyright (C) 2014 Google, Inc.
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

#pragma once

#ifdef ANDROID
#include <cutils/log.h>

#define LOG_VERBOSE(...) ALOGV(__VA_ARGS__)
#define LOG_DEBUG(...)   ALOGD(__VA_ARGS__)
#define LOG_INFO(...)   ALOGI(__VA_ARGS__)
#define LOG_WARN(...)   ALOGW(__VA_ARGS__)
#define LOG_ERROR(...)   ALOGE(__VA_ARGS__)
#else
#include <errno.h>
#include <limits.h>
#include <stdio.h>

#ifdef USE_ANDROID_LOGGING
#include <utils/Log.h>
#define LOG_TAG "bt_stack"
#define LOG_VERBOSE(...) ALOGV(__VA_ARGS__)
#define LOG_DEBUG(...)   ALOGD(__VA_ARGS__)
#define LOG_INFO(...)   ALOGI(__VA_ARGS__)
#define LOG_WARN(...)   ALOGW(__VA_ARGS__)
#define LOG_ERROR(...)   ALOGE(__VA_ARGS__)
#else
#include <syslog.h>

#define LOG_TAG "bt_stack : "

#define PRI_INFO " I"
#define PRI_WARN " W"
#define PRI_ERROR " E"
#define PRI_DEBUG " D"
#define PRI_VERB " V"

#define ALOGV(fmt, arg...) syslog (LOG_WARNING, LOG_TAG fmt, ##arg)
#define ALOGD(fmt, arg...) syslog (LOG_NOTICE, LOG_TAG fmt, ##arg)
#define ALOGI(fmt, arg...) syslog (LOG_NOTICE, LOG_TAG fmt, ##arg)
#define ALOGW(fmt, arg...) syslog (LOG_WARNING, LOG_TAG fmt, ##arg)
#define ALOGE(fmt, arg...) syslog (LOG_ERR, LOG_TAG fmt, ##arg)

#define LOG_VERBOSE(fmt, arg...) syslog (LOG_WARNING, LOG_TAG fmt, ##arg)
#define LOG_DEBUG(fmt, arg...) syslog (LOG_NOTICE, LOG_TAG fmt, ##arg)
#define LOG_INFO(fmt, arg...)  syslog (LOG_NOTICE, LOG_TAG fmt, ##arg)
#define LOG_WARN(fmt, arg...)  syslog (LOG_WARNING, LOG_TAG fmt, ##arg)
#define LOG_ERROR(fmt, arg...) syslog (LOG_ERR, LOG_TAG fmt, ##arg)
#endif
#endif

