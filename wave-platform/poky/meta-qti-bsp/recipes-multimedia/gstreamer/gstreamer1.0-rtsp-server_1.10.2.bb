require gstreamer1.0-rtsp-server.inc

SRC_URI[md5sum] = "3ed8878f076e84c59b3ed5bd38ab31ed"
SRC_URI[sha256sum] = "822dd6f754fea2bbf3369a7c388372f49b74668fb57943c1888675e544b07235"

LIC_FILES_CHKSUM = "file://COPYING;md5=6762ed442b3822387a51c92d928ead0d"

do_fetch_prepend () {
    import subprocess
    cmd = "cd ${WORKSPACE}/poky/meta-qti-bsp/recipes-multimedia/gstreamer && rm -rf gstreamer1.0-rtsp-server && mkdir gstreamer1.0-rtsp-server && wget https://source.codeaurora.org/quic/qyocto/oss/poky/plain/meta/recipes-multimedia/gstreamer/gstreamer1.0-rtsp-server/0001-Don-t-hardcode-libtool-name-when-using-introspection.patch?h=yoctoproject.org/master -O gstreamer1.0-rtsp-server/0001-Don-t-hardcode-libtool-name-when-using-introspection.patch"
    subprocess.call(cmd, shell=True)
}
