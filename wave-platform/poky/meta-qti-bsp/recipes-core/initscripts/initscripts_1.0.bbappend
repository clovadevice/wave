PR = "r157"

FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}-${PV}:"

SRC_URI += "file://umountfs"
SRC_URI += "file://bsp_paths.sh"
SRC_URI += "file://set_core_pattern.sh"

do_install_append() {
        update-rc.d -f -r ${D} mountnfs.sh remove
        update-rc.d -f -r ${D} urandom remove
        update-rc.d -f -r ${D} checkroot.sh remove

        install -m 0755 ${WORKDIR}/bsp_paths.sh  ${D}${sysconfdir}/init.d
        update-rc.d -r ${D} bsp_paths.sh start 15 2 3 4 5 .

        install -m 0755 ${WORKDIR}/set_core_pattern.sh  ${D}${sysconfdir}/init.d
        update-rc.d -r ${D} set_core_pattern.sh start 01 S 2 3 4 5 S .
        echo "test ! -x /sbin/restorecon || /sbin/restorecon -F /tmp" >> ${D}${sysconfdir}/init.d/populate-volatile.sh
}
