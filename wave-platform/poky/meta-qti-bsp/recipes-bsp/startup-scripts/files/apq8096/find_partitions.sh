#!/bin/sh
# Copyright (c) 2016, The Linux Foundation. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials provided
#       with the distribution.
#     * Neither the name of The Linux Foundation nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
# ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
# IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# find_partitions        init.d script to dynamically find partitions
#

#following function is just for debugging not enabled by default
timestamp()
{
   seconds="$(date +%s)"
}

#following function is just for debugging not enabled by default
RESTORECON()
{
    timestamp
    echo "$seconds $2 start" > /dev/console
    context_check="$(matchpathcon -V $2)"
    if test "${context_check#*verified}" == "$context_check"
    then
       # Only do a restorecon if necessary
        /sbin/restorecon $1 $2
    fi
    timestamp
    echo "$seconds $2 end" > /dev/console
}

FindAndMountEXT4 () {
   partition=$1
   dir=$2
   mmc_block_device=/dev/block/bootdevice/by-name/$partition
   mkdir -p $dir
   mount -t ext4 $mmc_block_device $dir -o relatime,data=ordered,noauto_da_alloc,discard
}

#For now we only have firmware with VFAT and which need protection ( selinux)
# It mounted with secontext as firmware_t
FindAndMountVFAT () {
   partition=$1
   dir=$2
   mmc_block_device=/dev/block/bootdevice/by-name/$partition
   mkdir -p $dir
   mount -t vfat $mmc_block_device $dir -o context=system_u:object_r:firmware_t:s0
}


FindAndMountEXT4 userdata /data
FindAndMountVFAT modem   /firmware
FindAndMountEXT4 persist /persist
FindAndMountEXT4 dsp /dsp

/sbin/restorecon -RF /persist
#/sbin/restorecon -RF /data
/sbin/restorecon -RF /persist
/sbin/restorecon -RF /dsp

exit 0
