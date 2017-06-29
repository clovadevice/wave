AM_CPPFLAGS = -I.

bin_PROGRAMS = servicemanager

servicemanager_SOURCES  := service_manager.c binder.c
servicemanager_CPPFLAGS := -I$(top_srcdir)/include

servicemanager_LDFLAGS = -llog -lselinux -lcutils
