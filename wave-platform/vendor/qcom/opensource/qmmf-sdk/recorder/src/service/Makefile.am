source_files = qmmf_recorder_service.cc
source_files += qmmf_recorder_impl.cc
source_files += qmmf_recorder_ion.cc
source_files += qmmf_remote_cb.cc
source_files += qmmf_camera_source.cc
source_files += qmmf_camera_context.cc
if ENABLE_360
source_files += qmmf_multicamera_manager.cc
endif
source_files += qmmf_encoder_core.cc
source_files += qmmf_audio_source.cc
source_files += qmmf_audio_raw_track_source.cc
source_files += qmmf_audio_encoded_track_source.cc
source_files += qmmf_audio_encoder_core.cc

# Build qmmf recorder service library
# libqmmf_recorder_service.so
lib_LTLIBRARIES = libqmmf_recorder_service.la

libqmmf_recorder_service_la_SOURCES = $(source_files)

libqmmf_recorder_service_la_CPPFLAGS = -I$(top_srcdir)/include
libqmmf_recorder_service_la_CPPFLAGS += -I$(top_srcdir)/
libqmmf_recorder_service_la_CPPFLAGS += -I$(LIBGRALLOC)
libqmmf_recorder_service_la_CPPFLAGS += -I$(MMCORE_CFLAGS)
libqmmf_recorder_service_la_CPPFLAGS += -I$(CAMERAHAL_CFLAGS)
libqmmf_recorder_service_la_LDFLAGS  = -llog -lbinder -lutils -lcutils
libqmmf_recorder_service_la_LDFLAGS  += -lcamera_client -lcamera_metadata
libqmmf_recorder_service_la_LDFLAGS  += -lhardware

libqmmf_recorder_service_la_LIBADD   = ../client/libqmmf_recorder_client.la
libqmmf_recorder_service_la_LIBADD   += ../../../common/cameraadaptor/libcamera_adaptor.la
libqmmf_recorder_service_la_LIBADD   += ../../../common/codecadaptor/libcodec_adaptor.la
libqmmf_recorder_service_la_LIBADD   += ../../../common/audio/src/client/libqmmf_audio_client.la
libqmmf_recorder_service_la_LIBADD   += ../../../common/overlay/liboverlay.la
libqmmf_recorder_service_la_LIBADD   += ../../../display/src/client/libqmmf_display_client.la

