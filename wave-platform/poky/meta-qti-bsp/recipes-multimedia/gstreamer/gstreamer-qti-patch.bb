deltask do_package_qa

DEFAULT_PREFERENCE = "-1"

LICENSE = "BSD"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/\
${LICENSE};md5=3775480a712fc46a69647678acb234cb"

SRC_URI = ""

do_fetch_prepend () {
    import os
    cmd = "cd ${WORKSPACE}/poky/meta-qti-bsp/recipes-multimedia/gstreamer && wget https://bug776038.bugzilla-attachments.gnome.org/attachment.cgi?id=341862 -O gstreamer1.0-plugins-base/0001-audioringbuffer-do-not-require-4-byte-multiple-for-e.patch && wget https://cgit.freedesktop.org/gstreamer/gst-plugins-base/patch/gst-libs/gst/audio?id=c46607095ea5dab7bf42d7b6d84f090eaaa681f1 -O gstreamer1.0-plugins-base/0001-audioringbuffer-Also-support-raw-AAC.patch && wget https://cgit.freedesktop.org/gstreamer/gst-plugins-base/patch/gst-libs/gst/audio?id=6134dab3bba13674f82c5befae7e459a67d979ad -O gstreamer1.0-plugins-base/Add-Flac.patch && wget https://bug777655.bugzilla-attachments.gnome.org/attachment.cgi?id=344065 -O gstreamer1.0-plugins-base/Add-Vorbis.patch && wget https://bug777655.bugzilla-attachments.gnome.org/attachment.cgi?id=344066 -O gstreamer1.0-plugins-base/0001-audioringbuffer-add-wma-and-alac-to-encoded-audio-fo.patch"

    os.system(cmd)
}
