#common project used in plugin bb.

do_patch_append() {
    import os
    os.system("cd ${S} && git rm common")
    os.system("cd ${S} && touch NEWS && touch AUTHORS && touch ChangeLog && touch README && touch REQUIREMENTS")
    cmd = "cd ${S} && git submodule add -f git://anongit.freedesktop.org/gstreamer/common && git submodule init"
    os.system(cmd)
}
