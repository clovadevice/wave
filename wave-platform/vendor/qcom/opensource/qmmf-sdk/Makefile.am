BUILD_COMPONENTS =
BUILD_COMPONENTS += common/cameraadaptor
BUILD_COMPONENTS += common/overlay
BUILD_COMPONENTS += common/audio/src/client
BUILD_COMPONENTS += common/audio/src/service
BUILD_COMPONENTS += common/audio/test/samples
BUILD_COMPONENTS += common/codecadaptor
BUILD_COMPONENTS += common/avqueue
BUILD_COMPONENTS += avcodec
BUILD_COMPONENTS += display/src/client
BUILD_COMPONENTS += display/src/service
BUILD_COMPONENTS += display/test/samples
BUILD_COMPONENTS += display/test/gtest
BUILD_COMPONENTS += recorder/src/client
BUILD_COMPONENTS += recorder/src/service
BUILD_COMPONENTS += recorder/test/samples
BUILD_COMPONENTS += recorder/test/gtest
BUILD_COMPONENTS += recorder/test/timelapse
BUILD_COMPONENTS += player/src/client
BUILD_COMPONENTS += player/src/service
BUILD_COMPONENTS += player/test/demuxer
BUILD_COMPONENTS += player/test/samples
BUILD_COMPONENTS += player/test/gtest
BUILD_COMPONENTS += qmmf-server


SUBDIRS = $(BUILD_COMPONENTS)

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = qmmf-sdk.pc
EXTRA_DIST = $(pkgconfig_DATA)
