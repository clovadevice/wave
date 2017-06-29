LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES:=     \
    btsnoop_dump.c

LOCAL_C_INCLUDES := . \
        $(LOCAL_PATH)/../../


LOCAL_MODULE_TAGS := debug optional

LOCAL_MODULE:= btsnoop

LOCAL_SHARED_LIBRARIES += libcutils

LOCAL_STATIC_LIBRARIES += \
    libosi

include $(BUILD_EXECUTABLE)
