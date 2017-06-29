#Include the drone settings

# Get base product settings
require apq8009-base-image.inc

IMAGE_INSTALL += "qcacld-hl"
IMAGE_INSTALL += "iw"
IMAGE_INSTALL_remove = "wlan-opensource"
IMAGE_INSTALL += "sigma-dut"
IMAGE_INSTALL += "qcacld-utils"
IMAGE_INSTALL += "data"
IMAGE_INSTALL += "wlan-services"
IMAGE_INSTALL += "libcap"
IMAGE_INSTALL += "attr"
IMAGE_INSTALL += "iputils"
