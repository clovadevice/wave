AC_PREREQ(2.61)
AC_INIT([qmmf-support], 1.0.0)
AM_INIT_AUTOMAKE([foreign])
AC_CONFIG_SRCDIR([Makefile.am])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# Check for programs
LT_INIT
AC_PROG_LIBTOOL
AC_PROG_CXX
AC_PROG_CC
AM_PROG_CC_C_O
AC_PROG_AWK
AC_PROG_CPP
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_MAKE_SET
PKG_PROG_PKG_CONFIG

AS_CASE([$host],
        [arm*], [ARM=yes],
        [ARM=no]
)

ARM=yes

[CPPFLAGS="$CPPFLAGS -std=c++14 -Wall -Wextra -Werror"]

AC_SUBST([CFLAGS])
AC_SUBST([CPPFLAGS])
AC_SUBST([LIBS])

AC_ARG_WITH([camerahal],
      AC_HELP_STRING([with-camerahal=@<:@dir@:>@],
         [Specify the location of CameraHal]),
      [camerahal_libdir=$withval],
      with_camerahal=no)

if test "x$with_camerahal" != "xno"; then
   AC_SUBST([CAMERAHAL_CFLAGS], "${camerahal_libdir}")
fi

AC_CONFIG_FILES([ \
    Makefile \
    rtspinterface/Makefile \
    muxinterface/Makefile \
    vaminterface/Makefile \
    httpinterface/Makefile \
])
AC_OUTPUT
