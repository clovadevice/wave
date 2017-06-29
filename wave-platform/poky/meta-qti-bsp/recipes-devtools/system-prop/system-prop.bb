inherit autotools-brokensep

PR = "r0"

FILESPATH =+ "${WORKSPACE}/android_compat/device/qcom/:"
SRC_URI   = "file://${SOC_FAMILY}"
SRC_URI  += "file://persist-prop.sh"

DESCRIPTION = "Script to populate system properties"

LICENSE = "BSD"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/\
${LICENSE};md5=3775480a712fc46a69647678acb234cb"

do_compile() {
    # Remove empty lines and lines starting with '#'
    sed -e 's/#.*$//' -e '/^$/d' ${WORKDIR}/${SOC_FAMILY}/system.prop >> ${S}/build.prop
}

inherit update-rc.d

INITSCRIPT_NAME   = "persist-prop.sh"
INITSCRIPT_PARAMS = "start 50 2 3 4 5 ."

do_install() {
    install -d ${D}
    install ${S}/build.prop ${D}/build.prop
    install -m 0755 ${WORKDIR}/persist-prop.sh -D ${D}${sysconfdir}/init.d/persist-prop
}

pkg_postinst_${PN} () {
        update-alternatives --install ${sysconfdir}/init.d/persist-prop.sh persist-prop.sh  persist-prop 50
        [ -n "$D" ] && OPT="-r $D" || OPT="-s"
        # remove all rc.d-links potentially created from alternatives
        update-rc.d $OPT -f persist-prop.sh remove
        update-rc.d $OPT persist-prop.sh multiuser
}

PACKAGES = "${PN}"
FILES_${PN} += "/build.prop"
