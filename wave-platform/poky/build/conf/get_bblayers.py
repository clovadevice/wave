# Generate the bblayers.conf file for the current build workspace
# emitted to stdout for simplicity.

import os, sys, fnmatch, re
from operator import itemgetter

# Trawl the OEROOT as passed to us and find all the layer files that meet our
# metadata directory criteria...
def getLayerPaths(target,  fnexpr) :
    retList = []
    for file in os.listdir(target) :
        if (fnmatch.fnmatch(file, fnexpr) and not
            (fnmatch.fnmatch(file, "meta-hob") or
             fnmatch.fnmatch(file, "meta-skeleton") or
             fnmatch.fnmatch(file, "meta-selftest"))):
            # Found what might be a metadata layer...
            layerPath = target + "/" + file
            layerConfPath = layerPath + "/conf/layer.conf"
            if os.path.exists(layerConfPath) :
                # Found a layer.  Find the priority for it...
                confFile = open(layerConfPath, "r")
                if (confFile != None) :
                    for line in confFile :
                        fields = line.split()
                        if (len(fields) > 0 and re.match("BBFILE_PRIORITY",  fields[0])) :
                            # Add the path, priority as a tuple to the list for the layer.
                            retList += [( layerPath,  int(fields[2].strip("\"")) )]
                            break
                    confFile.close()
    # In order to avoid potential namespace conflicts, between recipes on layers
    # we sort the list in descending order of priority.
    return sorted(retList,  key=itemgetter(1), reverse=True)


# Just spool the tuple list's paths out in order to a string...
def generatePathString ( pathList ):
    retString = ""
    for path, priority in pathList:
        retString = retString + path + " "
    return retString.strip()


# Emit our config file...
print "# This configuration file is dynamically generated every time" 
print "# set_bb_env.sh is sourced to set up a workspace.  DO NOT EDIT."
print "#--------------------------------------------------------------"
print "LCONF_VERSION = \"6\""
print
print "export WORKSPACE := \"${@os.path.abspath(os.path.join(os.path.dirname(bb.data.getVar('FILE', d)),'../../..'))}\""
#print "# Make sure WORKSPACE isn't exported"
#print "WORKSPACE[unexport] = \"1\""
print 
print "BBPATH = \"${TOPDIR}\""
print "BBFILES ?= \"\""
print "BBLAYERS = \"" + generatePathString(getLayerPaths(sys.argv[1].strip("\""), sys.argv[2].strip("\""))) + "\""



