# sources and intermediate files are separated

AM_CFLAGS = -Wall \
        -Wundef \
        -Wstrict-prototypes \
        -Wno-trigraphs \
        -std=c99 \
        -DHAS_NO_BDROID_BUILDCFG

AM_CPPFLAGS = \
        -I../ \
        -Iinclude \
        -I../btcore/include \
        -I../gki/common \
        -I../hci/include \
        -I../include \
        -I../osi/include \
        -I../utils/include \
        -I../stack/include

c_sources = \
        src/classic/peer.c \
        src/controller.c \
        src/interop.c

common_LDFLAGS = -lpthread ../hci/libbt-hci.la
common_CFLAGS =  $(AM_CFLAGS) -fPIC

library_includedir = $(pkgincludedir)\
library_include_HEADERS = $(h_sources)
lib_LTLIBRARIES = libbtdevice.la
libbtdevice_la_CC = @CC@
libbtdevice_la_SOURCES = $(c_sources)
libbtdevice_la_CPPFLAGS = $(AM_CPPFLAGS)
libbtdevice_la_CFLAGS = $(common_CFLAGS)
libbtdevice_la_LDFLAGS = $(common_LDFLAGS) -static
