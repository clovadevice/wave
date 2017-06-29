inherit autotools pkgconfig

DESCRIPTION = "Android Multimedia Framework"
HOMEPAGE = "http://developer.android.com/"
LICENSE = "Apache-2.0"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/\
${LICENSE};md5=89aea4e17d99a7cacdbeed46a0096b10"

PR = "r1"

DEPENDS = "native-frameworks libhardware system-media"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI   = "file://frameworks/av/"

S = "${WORKDIR}/frameworks/av"

FILES_${PN}-dbg    = "${libdir}/.debug/libcamera_client.*"
FILES_${PN}        = "${libdir}/libcamera_client.so.* ${libdir}/pkgconfig/* ${bindir}/mtpserver"
FILES_${PN}-dev    = "${libdir}/libcamera_client.so ${libdir}/libcamera_client.la ${includedir}"


do_configure_append() {
    install -m 0644 ${WORKSPACE}/kernel/msm-3.18/include/uapi/linux/usb/f_mtp.h ${STAGING_INCDIR}/linux/usb/
}
