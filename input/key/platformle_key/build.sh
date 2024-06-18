#!/bin/bash

driversrc="platformled"
drivername="platformled.ko"
appname="platformledApp"
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
			arm-linux-gnueabihf-gcc $appname.c -o $appname
			cp -f $appname $fsdir$appname
			;;
		c)
			make clean
			;;
		?)
			echo "unkowned argument."
			;;
	esac
done