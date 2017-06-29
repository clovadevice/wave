#
# This class helps to rebuild recipes for perf distributions
#

# Fetch PR value and append _perf while building perf DISTROs.
python __anonymous() {
    if bb.utils.contains('DISTRO_FEATURES', 'qti-perf', True, False, d):
        revision = d.getVar('PR', True)
        revision += "_perf"
        d.setVar('PR', revision)
}
