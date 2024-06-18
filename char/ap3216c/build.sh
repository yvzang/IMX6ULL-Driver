#!/bin/bash

driversrc="icm20608"
drivername="icm20608.ko"
appname="icm20608App"
fsdir="/home/yuzang/linux/nfs/rootfs/lib/modules/4.1.15/"

while getopts "abc" arg
do
	case $arg in
		a)
			make clean
			make 
			cp -f $drivername $fsdir$drivername
			;;
		b)
			arm-linux-gnueabihf-gcc $appname.c -march=armv7-a -mfpu=neon -mfloat-abi=hard -o $appname
			cp -f $appname $fsdir$appname
			;;
		c)
			make clean
			rm -f $appname
			;;
		?)
			echo "unkowned argument."
			;;
	esac
done