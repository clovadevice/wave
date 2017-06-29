# Force a few additional dependencies in the mix so that we get the needed
# recipes to build in the right order so we can make the bootimg file and
# our image...
DEPENDS = " \
    virtual/kernel \
    pkgconfig-native \
    gtk-doc-native \
    gettext-native \
    mkbootimg-native \
    dtbtool-native \
"
DEPENDS_append_aarch64 += "lib32-lk"
DEPENDS_append_arm += "lk"

# Image output types
IMAGE_FSTYPES = "tar.gz"
