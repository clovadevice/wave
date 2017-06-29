# Makefile.am - Automake script for device-krypton

etcdir = ${sysconfdir}
dist_etc_DATA = sysctl.conf

ACLOCAL_AMFLAGS = -I m4

if IS_MDM9607
dirs = wlan_sdio
else
if IS_APQ8096_DRONE
dirs = wlan_apq8096_drone
else
if IS_APQ8009_DRONE
dirs = wlan_apq8009_drone
else
if IS_APQ8017_SNAP
dirs = wlan_apq8017_snap
else
if IS_PRONTO_ENABLED
dirs = wlan_pronto
else
dirs = wlan
endif
endif
endif
endif
endif

SUBDIRS = $(dirs)
