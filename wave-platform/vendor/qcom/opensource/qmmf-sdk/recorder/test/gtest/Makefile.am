# Build recorder test application binary
bin_PROGRAMS = recorder_gtest

recorder_gtest_SOURCES  = qmmf_recorder_gtest.cc
recorder_gtest_CPPFLAGS = -I$(top_srcdir)/include
recorder_gtest_CPPFLAGS += -I$(top_srcdir)/
recorder_gtest_LDFLAGS  = -lutils -lcutils -llog -lgtest -lgtest_main
recorder_gtest_LDFLAGS  += -lcamera_client -lcamera_metadata
recorder_gtest_LDADD    = ../../src/client/libqmmf_recorder_client.la
recorder_gtest_LDADD    += ../../../common/avqueue/libav_queue.la
