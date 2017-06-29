SUMMARY = "A tool to load and stress a computer system"
HOMEPAGE = "http://kernel.ubuntu.com/~cking/stress-ng/"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=b234ee4d69f5fce4486a80fdaf4a4263"

DEPENDS = "zlib libaio"

SRC_URI = "http://kernel.ubuntu.com/~cking/tarballs/${BPN}/${BP}.tar.gz"

SRC_URI[md5sum] = "bf2fef1445fa248b9a55d91f492d2309"
SRC_URI[sha256sum] = "e699f90ec1260eb5815fdaf7b7c410c7aa32f8802adad1eaa7a27260a97d7581"

CFLAGS += "-Wall -Wextra -DVERSION='"$(VERSION)"' -O2"

do_install() {
	oe_runmake DESTDIR=${D} install
}
