FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}-${PV}:"
DEPENDS = "base-passwd"
SRC_URI_append += "${@base_contains('DISTRO_FEATURES','ro-rootfs','file://ro/fstab','file://fstab',d)}"

dirs755 += "/media/cf /media/net /media/ram \
            /media/union /media/realroot /media/hdd \
            /media/mmc1"

dirs755_append_apq8009 += "/firmware /persist /cache /userconf /factory"
dirs755_append_apq8053 +="/persist /cache /dsp "
#TODO Enabling systemd we need to add /firmware in dirs_755 list.
do_install_append(){
    install -m 755 -o diag -g diag -d ${D}/media
    install -m 755 -o diag -g diag -d ${D}/mnt/sdcard
    if ${@base_contains('DISTRO_FEATURES','ro-rootfs','true','false',d)}; then
        install -m 0644 ${WORKDIR}/ro/fstab ${D}${sysconfdir}/fstab
    else
        install -m 0644 ${WORKDIR}/fstab ${D}${sysconfdir}/fstab
    fi
    ln -s /mnt/sdcard ${D}/sdcard
    rmdir ${D}/tmp
    ln -s /var/tmp ${D}/tmp
    ln -s /var/run/resolv.conf ${D}/etc/resolv.conf
}
