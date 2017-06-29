inherit autotools-brokensep pkgconfig

DESCRIPTION = "GPS Loc HAL"
PR = "r5"
LICENSE = "BSD"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/\
${LICENSE};md5=3775480a712fc46a69647678acb234cb"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://hardware/qcom/gps/"
S = "${WORKDIR}/hardware/qcom/gps"
DEPENDS = "glib-2.0 libhardware gps-utils qmi qmi-framework data loc-pla loc-flp-hdr"
EXTRA_OECONF = "--with-libhardware-includes=${STAGING_INCDIR} \
                --with-core-includes=${WORKSPACE}/system/core/include \
                --with-locflp-includes=${STAGING_INCDIR}/loc-flp-hdr \
                --with-glib"


CPPFLAGS += "-I${WORKSPACE}/base/include"

PACKAGES = "${PN}"
INHIBIT_PACKAGE_DEBUG_SPLIT = "1"
FILES_${PN} = "${libdir}/* ${sysconfdir}"
# The loc-hal package contains symlinks that trip up insane
INSANE_SKIP_${PN} = "dev-so"

do_install_append() {
   install -m 0644 -D ${S}/etc/gps.conf ${D}${sysconfdir}/gps.conf
}
