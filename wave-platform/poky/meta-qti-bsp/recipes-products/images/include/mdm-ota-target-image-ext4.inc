DEPENDS += "machine-image"

OTA_TARGET_BASENAME_EXT4     = "${MACHINE}-ota-target-image-ext4"
OTA_TARGET_IMAGE_ROOTFS_EXT4 = "${IMAGE_ROOTFS}/../${OTA_TARGET_BASENAME_EXT4}"
OUTPUT_FILE_EXT4             = "${DEPLOY_DIR_IMAGE}/${OTA_TARGET_BASENAME_EXT4}.zip"
BASE_IMAGE_ROOTFS_EXT4       = "$(echo ${IMAGE_ROOTFS} | sed 's#-recovery##')"
#OUTPUT_FILE_EXT4 = "${OTA_TARGET_BASENAME_EXT4}.zip"

python do_rootfs_append(){
    bb.build.exec_func('recovery_ext4', d)
}

recovery_ext4[cleandirs] = "${OTA_TARGET_IMAGE_ROOTFS_EXT4}"
recovery_ext4[dirs]      = "${OTA_TARGET_IMAGE_ROOTFS_EXT4}"

recovery_ext4() {
rm -rf ${OTA_TARGET_IMAGE_ROOTFS_EXT4}
mkdir -p ${OTA_TARGET_IMAGE_ROOTFS_EXT4}

#Create directory structure for targetfiles.zip
mkdir -p ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/BOOTABLE_IMAGES
mkdir -p ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/DATA
mkdir -p ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/META
mkdir -p ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/OTA
mkdir -p ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/RECOVERY
mkdir -p ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/SYSTEM
mkdir -p ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/RADIO

echo "base image rootfs: ${BASE_IMAGE_ROOTFS_EXT4}"

# copy radio directory content into RADIO directory
cp -r ${WORKSPACE}/poky/meta-qti-bsp/recipes-bsp/base-files-recovery/${MACHINE}/radio/. ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/RADIO/.

# copy the boot\recovery images
cp ${DEPLOY_DIR_IMAGE}/${MACHINE}-boot.img ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/BOOTABLE_IMAGES/boot.img

cp ${DEPLOY_DIR_IMAGE}/${MACHINE}-boot.img ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/BOOTABLE_IMAGES/recovery.img


# copy the contents of system rootfs
cp -r ${BASE_IMAGE_ROOTFS_EXT4}/. ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/SYSTEM/.
#copy contents of recovery rootfs
cp -r ${IMAGE_ROOTFS}/. ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/RECOVERY/.
#generate recovery.fstab which is used by the updater-script
#echo #mount point fstype device [device2] >> ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/RECOVERY/recovery.fstab
echo /boot     emmc  /dev/block/bootdevice/by-name/boot >> ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/RECOVERY/recovery.fstab
echo /cache    ext4  cache >> ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/RECOVERY/recovery.fstab
echo /data     ext4  userdata >> ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/RECOVERY/recovery.fstab
echo /recovery emmc  /dev/block/bootdevice/by-name/recovery >> ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/RECOVERY/recovery.fstab
echo /system   ext4  system >> ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/RECOVERY/recovery.fstab

#Copy contents of userdata rootfs
if [ -d ${TMPDIR}/rootfs/${MACHINE}-usrfs/ ]; then
   cp -r ${TMPDIR}/rootfs/${MACHINE}-usrfs/. ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/DATA/.
fi

#Getting content for OTA folder
mkdir -p ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/OTA/bin
cp   ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/RECOVERY/usr/bin/applypatch ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/OTA/bin/.

cp   ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/RECOVERY/usr/bin/updater ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/OTA/bin/.

# copy contents of META folder
#recovery_api_version is from recovery module
echo recovery_api_version=3 >> ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/META/misc_info.txt

#blocksize = BOARD_FLASH_BLOCK_SIZE
echo blocksize=131072 >> ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/META/misc_info.txt

if [ "${MACHINE}" == "apq8053" ]; then
   #boot_size: Size of boot partition from partition.xml
   echo boot_size=0x01800000 >> ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/META/misc_info.txt

   #recovery_size : Size of recovery partition from partition.xml
   echo recovery_size=0x01800000 >> ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/META/misc_info.txt
else
   #boot_size: Size of boot partition from partition.xml
   echo boot_size=0x00F00000 >> ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/META/misc_info.txt

   #recovery_size : Size of recovery partition from partition.xml
   echo recovery_size=0x00F00000 >> ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/META/misc_info.txt
fi

#system_size : Size of system partition from partition.xml
echo system_size=0x00A00000 >> ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/META/misc_info.txt

#userdate_size : Size of data partition from partition.xml
echo userdata_size=0x00A00000 >> ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/META/misc_info.txt

#cache_size : Size of data partition from partition.xml
echo cache_size=0x00A00000 >> ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/META/misc_info.txt

#mkyaffs2_extra_flags : -c $(BOARD_KERNEL_PAGESIZE) -s $(BOARD_KERNEL_SPARESIZE)
echo mkyaffs2_extra_flags=-c 4096 -s 16 >> ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/META/misc_info.txt

#extfs_sparse_flag : definition in build
echo extfs_sparse_flags=-s >> ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/META/misc_info.txt

#default_system_dev_certificate : Dummy location
echo default_system_dev_certificate=build/abcd >> ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/META/misc_info.txt

#delete old files
rm -rf ${DEPLOY_DIR_IMAGE}/${OTA_TARGET_BASENAME_EXT4}
# Pack the files into the deploy dir
cd  ${OTA_TARGET_IMAGE_ROOTFS_EXT4}/SYSTEM
rm -rf var/run
ln -snf ../run var/run
cp -r ${OTA_TARGET_IMAGE_ROOTFS_EXT4} ${DEPLOY_DIR_IMAGE}
}
