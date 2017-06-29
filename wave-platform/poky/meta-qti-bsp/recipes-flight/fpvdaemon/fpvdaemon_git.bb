inherit package

SUMMARY = "FPV daemon"
LICENSE = "BSD"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/\
${LICENSE};md5=3775480a712fc46a69647678acb234cb"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI   = "file://camera/services/fpvdaemon"

SRCREV = "${AUTOREV}"
S = "${WORKDIR}/camera/"

DEPENDS += "media"
DEPENDS += "camera"
DEPENDS += "live555"
DEPENDS += "libcamera"

CPPFLAGS += "-I ${STAGING_KERNEL_BUILDDIR}/usr/include"
CPPFLAGS += "-I ${PKG_CONFIG_SYSROOT_DIR}/usr/include/android_compat"
CPPFLAGS += "-I ${PKG_CONFIG_SYSROOT_DIR}/usr/include/live555"
CPPFLAGS += "-I ${PKG_CONFIG_SYSROOT_DIR}/usr/include/adreno"
CPPFLAGS += "-I ${PKG_CONFIG_SYSROOT_DIR}/usr/include/mm-core/omxcore"
CPPFLAGS += "-I ${PKG_CONFIG_SYSROOT_DIR}/usr/include"

LDFLAGS += "-L ${PKG_CONFIG_SYSROOT_DIR}/usr/lib/hw"

do_compile() {
    # Current support is limited to msm8996 32-bit build
    #
    if [ "${MLPREFIX}" = "lib32-" ];  then
        make -C ${S}/fpvdaemon/
    else
        die "not supported"
    fi
}

PACKAGES = "${PN} ${PN}-dbg"
FILES_${PN} = "${bindir}/*"

do_install () {
   install -D ${S}/fpvdaemon/fpvdaemon ${D}${bindir}/fpvdaemon
   install -D ${S}/fpvdaemon/fpvclient ${D}${bindir}/fpvclient
}
