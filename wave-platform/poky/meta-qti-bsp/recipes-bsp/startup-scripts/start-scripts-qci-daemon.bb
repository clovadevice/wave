DESCRIPTION = "Startup script for QCI daemons"
HOMEPAGE = "http://www.quanta.com.tw/"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/Proprietary;md5=0557f9d92cf58f2ccdd50f62f8ac0b28"
LICENSE = "Proprietary"
RDEPENDS_${PN} = "bash"

SRC_URI +="file://${BASEMACHINE}/start_qci-daemon"
SRC_URI +="file://qci-daemon.service"

S = "${WORKDIR}/${BASEMACHINE}"

PR = "r0"

inherit update-rc.d

INITSCRIPT_NAME = "qci-daemon"
INITSCRIPT_PARAMS = "start 99 5 . stop 1 0 1 6 ."

do_install() {
    install -m 0755 ${S}/start_qci-daemon -D ${D}${sysconfdir}/init.d/${INITSCRIPT_NAME}
    if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
        install -d ${D}/etc/systemd/system/
        install -m 0644 ${WORKDIR}/${INITSCRIPT_NAME}.service -D ${D}/etc/systemd/system/${INITSCRIPT_NAME}.service
        install -d ${D}/etc/systemd/system/multi-user.target.wants/
        # enable the service for multi-user.target
        ln -sf /etc/systemd/${INITSCRIPT_NAME}.service \
            ${D}/etc/systemd/system/multi-user.target.wants/${INITSCRIPT_NAME}.service
    fi
}

FILES_${PN} += "/etc/systemd/system/${INITSCRIPT_NAME}.service"
FILES_${PN} += "/etc/systemd/system/multi-user.target.wants/${INITSCRIPT_NAME}.service"
