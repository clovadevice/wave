inherit autotools gettext
SUMMARY = "XML Parser library "
HOMEPAGE = "https://github.com/zeux/pugixml"
LICENSE = "MIT"
PRIORITY = "optional"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

# Package Revision (update whenever recipe is changed)
PR = "r0"

TARGET_CC_ARCH += "${LDFLAGS}"

SRC_URI = "	https://github.com/zeux/pugixml/archive/v${PV}.zip \
		file://001_Makefile.patch \
"

SRC_URI[md5sum] = "634c5df78cf0ba08c151c06a9f4c733d"
SRC_URI[sha256sum] = "e7fa883c53fd67443bd47829eba943c15bfc7d170f245fa78db13f1db4c7fc70"

S = "${WORKDIR}/${PN}-${PV}"

inherit autotools pkgconfig

do_compile () {
    oe_runmake
    rm -f *.o
}

