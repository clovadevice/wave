# sources and intermediate files are separated

AM_CFLAGS = -Wall \
        -Wundef \
        -Wstrict-prototypes \
        -Wno-trigraphs \
        -std=c99

AM_CPPFLAGS= \
        -I../inc/ \
        -I../../include \
        -I../../stack/include \
        -I../../test/suite

# HAL layer
c_sources = \
        adapter.c \
        bdtool.c \
        ../../test/suite/support/callbacks.c \
        ../../test/suite/support/gatt.c \
        ../../test/suite/support/hal.c \
        ../../test/suite/support/pan.c

common_CFLAGS =  $(AM_CFLAGS) -fPIC -D_GNU_SOURCE

library_includedir = $(pkgincludedir)
bin_PROGRAMS = bdt
bdt_CC = @CC@
bdt_SOURCES = $(c_sources)
bdt_CFLAGS = $(common_CFLAGS)
bdt_LDFLAGS = $(Z_LIB)
bdt_LDADD = -ldl -lrt -lresolv ../../btcore/libbtcore.la \
    ../../osi/libbtosi.la $(SYS_LIB)/libbtvendorhal.la
