FILESEXTRAPATHS_prepend := "${THISDIR}/systemd:"

SRC_URI += "file://1001-systemd-Disable-unused-mount-points.patch"
SRC_URI += "file://mountpartitions.rules"
SRC_URI += "file://systemd-udevd.service"

EXTRA_OECONF += " --disable-efi"

# Place systemd-udevd.service in /etc/systemd/system
do_install_append () {
   install -d ${D}/etc/systemd/system/
   install -m 0644 ${WORKDIR}/systemd-udevd.service \
       -D ${D}/etc/systemd/system/systemd-udevd.service
}
