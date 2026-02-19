#!/bin/sh
mount -t romfs mtd:flash /mnt
exec switch_root /mnt /sbin/init
