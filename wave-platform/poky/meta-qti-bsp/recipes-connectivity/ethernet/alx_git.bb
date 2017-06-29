inherit autotools-brokensep module update-rc.d qperf

DESCRIPTION = "Qualcomm Atheros Gigabit Ethernet Driver"
LICENSE = "ISC"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/${LICENSE};md5=f3b90e78ea0cffb20bf5cca7947a896d"

DEPENDS = "virtual/kernel"

do_unpack[deptask] = "do_populate_sysroot"
PR = "r4"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://external/compat-wireless/drivers/net/ethernet/atheros/alx/ \
           file://start_alx_le"
S = "${WORKDIR}/external/compat-wireless/drivers/net/ethernet/atheros/alx/"

FILES_${PN}="/etc/init.d/start_alx_le"

do_install() {
    module_do_install
    install -d ${D}${sysconfdir}/init.d
    install -m 0755 ${WORKDIR}/start_alx_le ${D}${sysconfdir}/init.d
}

INITSCRIPT_NAME = "start_alx_le"
INITSCRIPT_PARAMS = "start 91 5 . stop 15 0 1 6 ."

#pkg_postinst_${PN} () {
#[ -n "$D" ] && OPT="-r $D" || OPT="-s"
#update-rc.d $OPT -f start_alx_le remove
#update-rc.d $OPT start_alx_le start 91 5 . stop 15 0 1 6 .
#}
