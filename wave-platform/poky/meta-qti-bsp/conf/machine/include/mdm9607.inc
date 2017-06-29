BASEMACHINE ?= "mdm9607"
require conf/machine/include/basemachine.inc

SOC_FAMILY ?= "mdm9x07"
include conf/machine/include/soc-family.inc

# Uncomment the following line to enable the hard floating point abi. Note that
# this breaks some binary libraries and 3D (neither of which ship with
# meta-yocto). For maximum compatibility, leave this disabled.
#DEFAULTTUNE ?= "cortexa8hf-neon"

require conf/machine/include/tune-cortexa8.inc

BBMASK = ".*/*/recipes-kernel/linux-msm-4.4"

MACHINE_DISPLAY_WIDTH_PIXELS ?= "480"
MACHINE_DISPLAY_HEIGHT_PIXELS ?= "640"

SERIAL_CONSOLE ?= "115200 ttyHSL0"

DISTRO ?= "mdm"

MACHINE_FEATURES += "kernel26 keyboard"

PAGE_SIZE ?= '4096'

# Max supported inodes
EXTRA_IMAGECMD ?= "-N 2048"

#UBINIZE Args
UBINIZE_ARGS ?= "-m 4096 -p 256KiB -s 4096"
#MKUBIFS Arg
MKUBIFS_ARGS ?= "-m 4096 -e 253952 -c 2146 -F"
#Volume sizes
SYSTEM_VOLUME_SIZE ?= "63MiB"
CACHE_VOLUME_SIZE ?= "55MiB"
