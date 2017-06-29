#!/bin/sh

/usr/sbin/selinuxenabled 2>/dev/null || exit 0

RESTORECON()
{
    context_check="$(matchpathcon -V $2)"
    if test "${context_check#*verified}" == "$context_check"
    then
        # Only do a restorecon if necessary
        #/sbin/restorecon $1 $2
        echo "WARNING: $2 not labelled" > /dev/console
    fi
}

# Run restorecon on all top level directories that currently exist
#  Top level directories are currently whitelisted here, this may
#  change to exclude manually controlled directories in the future.
RESTORECON -F /
RESTORECON -F /*
RESTORECON -RF /bin
RESTORECON -RF /boot
RESTORECON -RF /cache
RESTORECON -RF /usr
RESTORECON -RF /data
RESTORECON -RF /dev
RESTORECON -RF /etc
RESTORECON -RF /home
RESTORECON -RF /kernel-tests
RESTORECON -RF /lib
RESTORECON -RF /lib64
RESTORECON -RF /linuxrc
RESTORECON -RF /location-mq-s
RESTORECON -RF /media
RESTORECON -RF /mnt
RESTORECON -RF /proc
RESTORECON -RF /run
RESTORECON -RF /sdcard
RESTORECON -RF /share
RESTORECON -RF /sys
RESTORECON -RF /system
RESTORECON -RF /target
RESTORECON -RF /tmp
RESTORECON -RF /var

exit 0

