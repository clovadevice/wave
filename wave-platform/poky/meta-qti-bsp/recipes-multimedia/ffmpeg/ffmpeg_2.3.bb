SUMMARY = "FFmpeg is a complete, cross-platform solution to record, convert and stream audio and video."
HOMEPAGE = "http://ffmpeg.org"
BUGTRACKER = "http://ffmpeg.org/trac/ffmpeg"
LICENSE = "LGPLv2.1"
PRIORITY = "optional"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/LGPL-2.1;md5=1a6d268fd218675ffea8be556788b780"
DEPENDS = "openssl"

# Package Revision (update whenever recipe is changed)
PR = "r0"

SRC_URI = "\
    http://www.ffmpeg.org/releases/${PN}-${PV}.tar.gz \
"
SRC_URI[md5sum] = "d63e952716c27e23927bfd64518d6dee"
SRC_URI[sha256sum] = "f3b437dbdf9f1519fa5e0a923428e77ba3babefbcfeda9aebb7cd72ae8924c1d"

EXTRA_OEMAKE = ""

FILES_${PN} += "/lib/lib*.so.*"
FILES_${PN} += "/lib/pkgconfig/*"
FILES_${PN} += "/usr/bin/*"
FILES_${PN}-dev += "/usr/share/*"
FILES_${PN}-dev += "/lib/lib*.so"

do_configure () {
    ./configure --enable-cross-compile --cross-prefix=${TARGET_PREFIX} \
    --cpu=armv7-a --target-os=linux --sysroot=${STAGING_DIR_TARGET} \
    --arch=${TARGET_ARCH} --disable-mmx --enable-shared --disable-doc \
    --disable-htmlpages --disable-manpages --disable-podpages \
    --disable-txtpages  --enable-demuxer=matroska --disable-altivec \
    --enable-fft --libdir=${base_libdir} --shlibdir=${base_libdir} \
    --prefix=${base_libdir} --incdir=${includedir} --enable-protocol=tcp \
    --enable-protocol=mmst --enable-demuxer=asf --enable-decoder=wmav2 \
    --enable-protocol=http --enable-protocol=hls --enable-demuxer=mpegts \
    --enable-openssl
}

do_install() {
    oe_runmake 'DESTDIR=${D}' install
    # Info dir listing isn't interesting at this point so remove it if it exists.
    if [ -e "${D}${infodir}/dir" ]; then
    rm -f ${D}${infodir}/dir
    fi
}
