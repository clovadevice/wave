inherit autotools qcommon

DESCRIPTION = "display Library"
LICENSE = "BSD"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/\
${LICENSE};md5=3775480a712fc46a69647678acb234cb"

PR = "r8"

PACKAGES = "${PN}"

SRC_DIR = "${WORKSPACE}/display/display-hal/"
S = "${WORKDIR}/display/display-hal/"

DEPENDS += "system-core"
DEPENDS += "libhardware"
DEPENDS += "native-frameworks"

EXTRA_OECONF = " --with-core-includes=${WORKSPACE}/system/core/include"
EXTRA_OECONF += " --with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include"

LDFLAGS += "-llog -lhardware -lutils -lcutils"

CPPFLAGS += "-DTARGET_HEADLESS"
CPPFLAGS += "-DVENUS_COLOR_FORMAT"
CPPFLAGS += "-DPAGE_SIZE=4096"
CPPFLAGS += "-I${WORKSPACE}/display/display-hal/libqdutils"
CPPFLAGS += "-I${WORKSPACE}/display/display-hal/libqservice"
CPPFLAGS += "-I${WORKSPACE}/display/display-hal/sdm/include"
CPPFLAGS += "-I${WORKSPACE}/display/display-hal/include"
CPPFLAGS += "-I${WORKSPACE}/display/display-hal/libgralloc"
CPPFLAGS += "-I${WORKSPACE}/system/core/include"

do_install_append () {
    # libhardware expects to find /usr/lib/hw/gralloc.*.so
    install -d ${D}/usr/lib/hw
    ln -s /usr/lib/libgralloc.so ${D}/usr/lib/hw/gralloc.default.so
}

FILES_${PN} = "${libdir}/*.so"
FILES_${PN} += "/usr/lib/hw/gralloc.default.so"
INSANE_SKIP_${PN} = "dev-so"
