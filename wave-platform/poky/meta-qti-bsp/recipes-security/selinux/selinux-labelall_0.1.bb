inherit update-rc.d

SUMMARY = "SELinux init script"
DESCRIPTION = "\
    Script to label all file directories other than /sbin \
"

LICENSE = "BSD"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/\
BSD;md5=3775480a712fc46a69647678acb234cb"

FILESPATH =+ "${THISDIR}/files/:"
SRC_URI = "file://selinux-labelall.sh"
INITSCRIPT_NAME = "selinux-labelall"
INITSCRIPT_PARAMS = "start 06 S ."

do_install_append() {
    install -m 0755 ${WORKDIR}/selinux-labelall.sh -D ${D}${sysconfdir}/init.d/selinux-labelall
}
