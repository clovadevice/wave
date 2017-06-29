# sources and intermediate files are separated

AM_CFLAGS = -Wall \
        -Wundef \
        -Wstrict-prototypes \
        -Wno-trigraphs \
        -std=c99 \
        -DHAS_NO_BDROID_BUILDCFG

AM_CPPFLAGS= \
        -Iinclude \
        -I../btcore/include \
        -I../osi/include \
        -I../stack/include \
        -I../utils/include \
        -I../gki/common \
        -I../include \
        -I../ \
        -I$(bdroid_C_INCLUDES)

c_sources = \
        src/bt_utils.c

common_LDFLAGS = -lpthread ../osi/libbtosi.la ../gki/libbtgki.la
common_CFLAGS =  $(AM_CFLAGS) -fPIC

library_includedir = $(pkgincludedir)
lib_LTLIBRARIES = libbtutils.la
libbtutils_la_CC = @CC@
libbtutils_la_SOURCES = $(c_sources)
libbtutils_la_CPPFLAGS =$(AM_CPPFLAGS)
libbtutils_la_CFLAGS = $(common_CFLAGS)
libbtutils_la_LDFLAGS = $(common_LDFLAGS) -static
