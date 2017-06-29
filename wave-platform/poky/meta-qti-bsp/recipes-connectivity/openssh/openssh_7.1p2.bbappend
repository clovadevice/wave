BASEPRODUCT = "${@d.getVar('PRODUCT', False)}"
do_install_append () {
    if [ "${BASEPRODUCT}" == "drone" ]; then
        sed -i -e 's:#PermitRootLogin yes:PermitRootLogin yes:' ${WORKDIR}/sshd_config ${D}${sysconfdir}/ssh/sshd_config
        sed -i -e 's:#PasswordAuthentication yes:PasswordAuthentication yes:' ${WORKDIR}/sshd_config ${D}${sysconfdir}/ssh/sshd_config
    fi
}

RDEPENDS_${PN} += "${PN}-sftp"
