# The mkbootimg production process.
require ../include/mdm-recovery-bootimg.inc

DEPENDS += "bzip2 fsconfig-native"

# Image output types
IMAGE_FSTYPES += "ubi"

MKUBIFS_ARGS = "-m 4096 -e 253952 -c 83 -F"

UBI_VOLNAME = "rootfs"

# The OE-Core minimal boot image files.
IMAGE_INSTALL += "packagegroup-core-boot"
IMAGE_INSTALL += "${ROOTFS_PKGMANAGE_BOOTSTRAP}"
IMAGE_INSTALL += "${CORE_IMAGE_EXTRA_INSTALL}"

# The MSM Linux minimal boot image files.
IMAGE_INSTALL += "base-files"
IMAGE_INSTALL += "base-passwd"
IMAGE_INSTALL += "system-core-adbd"
IMAGE_INSTALL += "system-core-logd"
IMAGE_INSTALL += "system-core-usb"
IMAGE_INSTALL += "recovery"
IMAGE_INSTALL += "powerapp"
IMAGE_INSTALL += "powerapp-powerconfig"
IMAGE_INSTALL += "powerapp-reboot"
IMAGE_INSTALL += "powerapp-shutdown"
IMAGE_INSTALL += "start-scripts-find-recovery-partitions"
IMAGE_INSTALL += "start-scripts-firmware-links"
IMAGE_INSTALL += "mtd-utils-ubifs"
IMAGE_INSTALL += "e2fsprogs-e2fsck"
IMAGE_INSTALL += "e2fsprogs-mke2fs"
IMAGE_INSTALL += "libcomerr"
IMAGE_INSTALL += "libss"
IMAGE_INSTALL += "libe2p"
IMAGE_INSTALL += "libext2fs"

# Below is to generate sparse ext4 recovery image (OE by default supports raw ext4 images)

do_makesystem() {
	make_ext4fs -s -l ${SYSTEM_SIZE_EXT4} ${DEPLOY_DIR_IMAGE}/mdm-recovery-image-${MACHINE}.ext4 ${IMAGE_ROOTFS}
}
