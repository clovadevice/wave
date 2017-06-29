FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}-${PV}:"

SRC_URI += "file://root-home.patch \
           file://add-hash.patch \
           file://add-diag-user.patch \
           file://add-sdcard-diag-groups.patch \
           file://add-reboot-daemon-group.patch \
"

PR = "r1"
