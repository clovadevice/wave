#!/bin/sh

 for uevent in /sys/devices/78b8000.i2c/i2c-4/4-0068/iio:device?*/uevent; do
      . $uevent
      if [ -e $uevent ]; then
        if [ ! -e /dev/iio:device$MINOR ]; then
            mknod /dev/iio:device$MINOR c 250 $MINOR
        fi
      fi
 done
