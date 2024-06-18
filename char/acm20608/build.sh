#!/bin/bash

driversrc="ap3216c"
drivername="ap3216c.ko"
appname="ap3216cApp"
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
			rm $appname $drivername
			;;
		?)
			echo "unkowned argument."
			;;
	esac
done