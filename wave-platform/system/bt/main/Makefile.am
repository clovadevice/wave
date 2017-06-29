# sources and intermediate files are separated

AM_CFLAGS = -Wall \
        -Wundef \
        -Wstrict-prototypes \
        -Wno-trigraphs \
        -std=c99 \
        -DHAS_NO_BDROID_BUILDCFG

AM_CPPFLAGS= \
        -I../ \
        -I../bta/include \
        -I../bta/sys \
        -I../bta/dm \
        -I../btcore/include \
        -I../osi/include \
        -I../gki/common \
        -I../gki/ulinux \
        -I../include \
        -I../stack/include \
        -I../stack/l2cap \
        -I../stack/a2dp \
        -I../stack/btm \
        -I../stack/avdt \
        -I../hcis \
        -I../hcis/include \
        -I../hcis/patchram \
        -I../udrv/include \
        -I../btif/include \
        -I../btif/co \
        -I../hci/include\
        -I../vnd/include \
        -I../brcm/include \
        -I../embdrv/sbc/encoder/include \
        -I../embdrv/sbc/decoder/include \
        -I../audio_a2dp_hw \
        -I../utils/include \
        -I${WORKSPACE}/vendor/qcom/opensource/bluetooth/system_bt_ext/bta/include \
        -I${WORKSPACE}/vendor/qcom/opensource/bluetooth/system_bt_ext/btif/include \
        -I${WORKSPACE}/external/tinyxml2 \
        -I${WORKSPACE}/external/zlib \
        -I${WORKSPACE}/vendor/qcom/opensource/bluetooth/hal/include \
        -I${WORKSPACE}/vendor/qcom/opensource/bluetooth/vhal/include

if MDM_ROME
AM_CFLAGS += -DBT_SOC_TYPE_ROME
AM_CPPFLAGS += -DBT_SOC_TYPE_ROME
else
if MDM_PRONTO
AM_CFLAGS += -DHCI_USE_MCT
AM_CPPFLAGS += -DHCI_USE_MCT
endif
endif

# HAL layer
c_sources = \
        ../btif/src/bluetooth.c

# platform specific
c_sources+= \
        bte_main.c \
        bte_init.c \
        bte_logmsg.c \
        bte_conf.c \
        stack_config.c

# BTC
if BTC_SUPPORT
AM_CFLAGS += -DBTC_INCLUDED=TRUE
AM_CPPFLAGS += -DBTC_INCLUDED=TRUE -I${WORKSPACE}/vendor/qcom/opensource/bluetooth/system_bt_ext/btc/include
c_sources+= \
        ${WORKSPACE}/vendor/qcom/opensource/bluetooth/system_bt_ext/btc/src/btc_common.c
endif


# SSR_CLEANUP
if SSR_CLEANUP_SUPPORT
AM_CFLAGS += -DSSR_CLEANUP=TRUE
endif

# BTIF
c_sources += \
        ../btif/src/btif_av.c \
        ../btif/src/btif_config.c \
        ../btif/src/btif_core.c \
        ../btif/src/btif_debug.c \
        ../btif/src/btif_debug_btsnoop.c \
        ../btif/src/btif_debug_conn.c \
        ../btif/src/btif_dm.c \
        ../btif/src/btif_gatt.c \
        ../btif/src/btif_gatt_client.c \
        ../btif/src/btif_gatt_multi_adv_util.c \
        ../btif/src/btif_gatt_server.c \
        ../btif/src/btif_gatt_test.c \
        ../btif/src/btif_gatt_util.c \
        ../btif/src/btif_hf.c \
        ../btif/src/btif_hf_client.c \
        ../btif/src/btif_hh.c \
        ../btif/src/btif_hd.c \
        ../btif/src/btif_hl.c \
        ../btif/src/btif_media_task.c \
        ../btif/src/btif_sdp.c \
        ../btif/src/btif_pan.c \
        ../btif/src/btif_profile_queue.c \
        ../btif/src/btif_rc.c \
        ../btif/src/btif_sm.c \
        ../btif/src/btif_sock.c \
        ../btif/src/btif_sock_rfc.c \
        ../btif/src/btif_sock_l2cap.c \
        ../btif/src/btif_sock_sdp.c \
        ../btif/src/btif_sock_sco.c \
        ../btif/src/btif_sock_thread.c \
        ../btif/src/btif_sdp_server.c \
        ../btif/src/btif_sock_util.c \
        ../btif/src/btif_stack_log.c \
        ../btif/src/btif_storage.c \
        ../btif/src/btif_util.c \
        ../btif/src/btif_vendor.c \
        ../btif/src/stack_manager.c \
        ../embdrv/sbc/decoder/srce/alloc.c \
        ../embdrv/sbc/decoder/srce/bitalloc.c \
        ../embdrv/sbc/decoder/srce/bitalloc-sbc.c \
        ../embdrv/sbc/decoder/srce/bitstream-decode.c \
        ../embdrv/sbc/decoder/srce/decoder-oina.c \
        ../embdrv/sbc/decoder/srce/decoder-private.c \
        ../embdrv/sbc/decoder/srce/decoder-sbc.c \
        ../embdrv/sbc/decoder/srce/dequant.c \
        ../embdrv/sbc/decoder/srce/framing.c \
        ../embdrv/sbc/decoder/srce/framing-sbc.c \
        ../embdrv/sbc/decoder/srce/oi_codec_version.c \
        ../embdrv/sbc/decoder/srce/synthesis-sbc.c \
        ../embdrv/sbc/decoder/srce/synthesis-dct8.c \
        ../embdrv/sbc/decoder/srce/synthesis-8-generated.c

#if ($(BOARD_USES_WIPOWER), true)
#c_sources += \
#   ../wipowerif/src/wipower.c
#endif

# callouts
c_sources+= \
        ../btif/co/bta_ag_co.c \
        ../btif/co/bta_dm_co.c \
        ../btif/co/bta_hh_co.c \
        ../btif/co/bta_hl_co.c \
        ../btif/co/bta_pan_co.c \
        ../btif/co/bta_gattc_co.c \
        ../btif/co/bta_gatts_co.c \
        ../btif/co/bta_av_co.c

# sbc encoder
c_sources+= \
        ../embdrv/sbc/encoder/srce/sbc_analysis.c \
        ../embdrv/sbc/encoder/srce/sbc_dct.c \
        ../embdrv/sbc/encoder/srce/sbc_dct_coeffs.c \
        ../embdrv/sbc/encoder/srce/sbc_enc_bit_alloc_mono.c \
        ../embdrv/sbc/encoder/srce/sbc_enc_bit_alloc_ste.c \
        ../embdrv/sbc/encoder/srce/sbc_enc_coeffs.c \
        ../embdrv/sbc/encoder/srce/sbc_encoder.c \
        ../embdrv/sbc/encoder/srce/sbc_packing.c

c_sources+= \
        ../udrv/ulinux/uipc.c

c_sources+= \
        ${WORKSPACE}/vendor/qcom/opensource/bluetooth/system_bt_ext/btif/src/btif_avk_media_task.c \
        ${WORKSPACE}/vendor/qcom/opensource/bluetooth/system_bt_ext/btif/src/btif_avk.c \
        ${WORKSPACE}/vendor/qcom/opensource/bluetooth/system_bt_ext/btif/src/btif_avk_rc.c \
        ${WORKSPACE}/vendor/qcom/opensource/bluetooth/system_bt_ext/btif/src/btif_avk_media_aac.c \
        ${WORKSPACE}/vendor/qcom/opensource/bluetooth/system_bt_ext/btif/src/btif_avk_media_mp3.c \
        ${WORKSPACE}/vendor/qcom/opensource/bluetooth/system_bt_ext/btif/src/btif_avk_media_aptx.c \
        ${WORKSPACE}/vendor/qcom/opensource/bluetooth/system_bt_ext/btif/co/bta_avk_co.c

common_LDFLAGS = -ldl -lrt -lresolv ../hci/libbt-hci.la \
        ../utils/libbtutils.la ../osi/libbtosi.la ../gki/libbtgki.la \
        ../stack/libbtstack.la ../bta/libbtbta.la ../btcore/libbtcore.la \
        ../profile/libbtprofile.la ../device/libbtdevice.la

common_CFLAGS =  $(AM_CFLAGS) -fPIC -DUSE_AUDIO_TRACK -D_GNU_SOURCE $(Z_LIB)

library_includedir = $(pkgincludedir)
lib_LTLIBRARIES = libbluetoothdefault.la
libbluetoothdefault_la_CC = @CC@
libbluetoothdefault_la_SOURCES = $(c_sources)
libbluetoothdefault_la_CPPFLAGS = $(AM_CPPFLAGS)
libbluetoothdefault_la_CFLAGS = $(common_CFLAGS)
libbluetoothdefault_la_LDFLAGS = $(common_LDFLAGS)
