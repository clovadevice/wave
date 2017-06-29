DESCRIPTION = "Start up script for detecting partitions"
HOMEPAGE = "http://codeaurora.org"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/BSD;md5=3775480a712fc46a69647678acb234cb"
LICENSE = "BSD"

SRC_URI +="file://${BASEMACHINE}/find_partitions.sh"

S = "${WORKDIR}/${BASEMACHINE}"

PR = "r4"

inherit update-rc.d

INITSCRIPT_NAME = "find_partitions.sh"
INITSCRIPT_PARAMS = "start 36 S ."

do_install() {
    install -m 0755 ${WORKDIR}/${BASEMACHINE}/find_partitions.sh -D ${D}${sysconfdir}/init.d/find_partitions.sh
}
