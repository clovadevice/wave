BASEMACHINE ?= "apq8009"
require conf/machine/include/basemachine.inc

SOC_FAMILY ?= "msm8909"
include conf/machine/include/soc-family.inc

# Uncomment the following line to enable the hard floating point abi. Note that
# this breaks some binary libraries and 3D (neither of which ship with
# meta-yocto). For maximum compatibility, leave this disabled.
#DEFAULTTUNE ?= "cortexa8hf-neon"

include conf/machine/include/tune-cortexa7.inc

BBMASK = ".*/*/recipes-kernel/linux-msm-4.4"

MACHINE_DISPLAY_WIDTH_PIXELS ?= "480"
MACHINE_DISPLAY_HEIGHT_PIXELS ?= "640"

SERIAL_CONSOLE ?= "115200 ttyHSL0"

DISTRO ?="msm"

DISTRO_FEATURES += "persist-image"

MACHINE_FEATURES += "kernel26 keyboard"

PAGE_SIZE ?= '4096'

# Max supported inodes
EXTRA_IMAGECMD ?= "-N 2048"

# Sizes for EXT4 (in bytes)
SYSTEM_SIZE_EXT4 ?= "1258292000"
USERDATA_SIZE_EXT4 ?= "4000000000"
CACHE_SIZE_EXT4 ?= "1048576000"
RECOVERY_SIZE_EXT4 ?= "524288000"

