inherit androidmk

SUMMARY = "Display HAL"
LICENSE = "Apache-2.0"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/Apache-2.0;md5=89aea4e17d99a7cacdbeed46a0096b10"

SRCREV = "${AUTOREV}"

FILESPATH =+ "${WORKSPACE}/display:"
SRC_URI   = "file://display-hal"
S = "${WORKDIR}/display-hal"

PR = "r1"

DEPENDS += "system-core"
DEPENDS += "libhardware"
DEPENDS += "native-frameworks"

export TARGET_LIBRARY_SUPPRESS_LIST="libbacktrace libbase libunwind libhardware libbinder"

EXTRA_OEMAKE += "TARGET_IS_HEADLESS=true"
EXTRA_OEMAKE += "QTI_LINUX_DISPLAY_HAL=true"
EXTRA_OECONF = " --with-core-includes=${WORKSPACE}/system/core/include --with-glib"

CFLAGS += "-include ${STAGING_KERNEL_BUILDDIR}/include/generated/asm-offsets.h"
CFLAGS += "-Wno-error=unused-parameter"
CFLAGS += "-Wno-error=conversion"
CFLAGS += "-Wno-uninitialized -Wno-error=attributes"
CFLAGS += "-I${STAGING_INCDIR}/system"
CFLAGS += "-I${STAGING_INCDIR}"
CFLAGS += "-DPAGE_SIZE=PAGE_SZ"
CFLAGS += "-DAID_CAMERASERVER=1047"

LDFLAGS += "-llog -lbinder -lutils -lcutils"

do_compile() {
        androidmk_setenv
        oe_runmake -f ${LA_COMPAT_DIR}/build/core/main.mk BUILD_MODULES_IN_PATHS=${S} \
            all_modules SHOW_COMMANDS=true || die "make failed"
}

do_install_append () {
    install -d ${D}${includedir}
    install -m 0644 ${S}/libgralloc/gralloc_priv.h -D ${D}${includedir}/gralloc_priv.h
    install -m 0644 ${S}/libqdutils/qdMetaData.h   -D ${D}${includedir}/libqdutils/qdMetaData.h
}

# Both libhardware and display-hal provides gralloc_priv.h
# However display-hal's header has vendor specific definitions
# and it should be used whenever available.
do_fix_sysroot () {
   if [ -f ${STAGING_INCDIR}/gralloc_priv.h ]; then
      rm ${STAGING_INCDIR}/gralloc_priv.h
   else
      echo "${STAGING_INCDIR}/gralloc_priv.h not found"
   fi
}
addtask fix_sysroot after do_install before do_populate_sysroot
