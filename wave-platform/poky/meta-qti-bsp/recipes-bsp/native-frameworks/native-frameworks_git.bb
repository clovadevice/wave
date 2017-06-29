inherit autotools pkgconfig

DESCRIPTION = "Android IPC utilities"
HOMEPAGE = "http://developer.android.com/"
LICENSE = "Apache-2.0"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/\
${LICENSE};md5=89aea4e17d99a7cacdbeed46a0096b10"

PR = "r1"

DEPENDS = "liblog libcutils libhardware libselinux system-core glib-2.0"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI   = "file://frameworks/native"

S = "${WORKDIR}/native"

EXTRA_OECONF = " --with-core-includes=${WORKSPACE}/system/core/include --with-glib"

CFLAGS += "-I${STAGING_INCDIR}/libselinux"

FILES_${PN}-servicemanager-dbg = "${bindir}/.debug/servicemanager"
FILES_${PN}-servicemanager     = "${bindir}/servicemanager"

FILES_${PN}-libbinder-dbg    = "${libdir}/.debug/libbinder.*"
FILES_${PN}-libbinder        = "${libdir}/libbinder.so.*"
FILES_${PN}-libbinder-dev    = "${libdir}/libbinder.so ${libdir}/libbinder.la ${includedir}"
FILES_${PN}-libbinder-static = "${libdir}/libbinder.a"

FILES_${PN}-libui-dbg    = "${libdir}/.debug/libui.*"
FILES_${PN}-libui        = "${libdir}/libui.so.*"
FILES_${PN}-libbui-dev    = "${libdir}/libui.so ${libdir}/libui.la ${includedir}"

