# Build libvam_interface
lib_LTLIBRARIES = libvam_interface.la

libvam_interface_la_SOURCES  = src/qmmf_vam_config_parser.cc

libvam_interface_la_CPPFLAGS = -I$(top_srcdir)/vaminterface/inc
libvam_interface_la_LDFLAGS  = -lutils -lpthread -ldl -llog -ljson_apis
