inherit deploy

DESCRIPTION = "Little Kernel bootloader"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/\
${LICENSE};md5=0835ade698e0bcf8506ecda2f7b4f302"
HOMEPAGE = "https://www.codeaurora.org/gitweb/quic/la?p=kernel/lk.git"

PROVIDES = "virtual/bootloader"

PV       = "3.0"
PR       = "r1"

PACKAGE_ARCH = "${MACHINE_ARCH}"
FILESPATH =+ "${WORKSPACE}:"

SRC_URI   =  "file://bootable/bootloader/lk"
S         =  "${WORKDIR}/bootable/lk"

MY_TARGET_mdm9650 = "mdm9640"
MY_TARGET_sdxhedgehog = "mdm9640"
MY_TARGET_apq8009  = "msm8909"
MY_TARGET_msm8909  = "msm8909"
MY_TARGET_msm8909w = "msm8909"
MY_TARGET_apq8096  = "msm8996"
MY_TARGET_mdm9607  = "mdm9607"
MY_TARGET_apq8053  = "msm8953"
MY_TARGET_apq8017  = "msm8952"
MY_TARGET         ?= "${BASEMACHINE}"

BOOTLOADER_NAME = "${@base_contains('DISTRO_FEATURES', 'emmc-boot', 'emmc_appsboot', 'appsboot', d)}"

emmc_bootloader = "${@base_contains('DISTRO_FEATURES', 'emmc-boot', '1', '0', d)}"

LIBGCC = "${STAGING_LIBDIR}/${TARGET_SYS}/4.9.3/libgcc.a"

DISPLAY_SCREEN = "${@base_conditional('PRODUCT', 'drone', '0', '1', d)}"

EXTRA_OEMAKE = "${MY_TARGET} TOOLCHAIN_PREFIX='${TARGET_PREFIX}'  LIBGCC='${LIBGCC}' DISPLAY_SCREEN=${DISPLAY_SCREEN}"

EXTRA_OEMAKE_append = " VERIFIED_BOOT=0 DEFAULT_UNLOCK=true EMMC_BOOT=${emmc_bootloader} APPEND_CMDLINE=${emmc_bootloader}"

do_install() {
        install -d ${D}/boot
        install build-${MY_TARGET}/*.mbn ${D}/boot
}


FILES_${PN} = "/boot"
FILES_${PN}-dbg = "/boot/.debug"

do_deploy() {
        install ${S}/build-${MY_TARGET}/*.mbn ${DEPLOYDIR}
}

do_deploy[dirs] = "${S} ${DEPLOYDIR}"
addtask deploy before do_build after do_install

PACKAGE_STRIP = "no"
