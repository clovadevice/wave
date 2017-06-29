require apq8017-base-image.inc

IMAGE_INSTALL += "qcacld-hl"
IMAGE_INSTALL_remove = "wcnss"
IMAGE_INSTALL_remove = "wlan-opensource"

IMAGE_INSTALL += "libcap"
IMAGE_INSTALL += "attr"
IMAGE_INSTALL += "iputils"
IMAGE_INSTALL += "iw"
IMAGE_INSTALL += "qcacld-utils"
IMAGE_INSTALL += "data"
IMAGE_INSTALL += "wlan-services"
IMAGE_INSTALL += "sigma-dut"
