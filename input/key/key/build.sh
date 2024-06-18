#!/bin/bash

drivername="inputkey.ko"
appname="inputkeyApp"
copydir="/home/yuzang/linux/nfs/rootfs/lib/modules/4.1.15/"

while getopts "cab" arg
do
	case $arg in
		c)
			make clean
			rm $appname
			;;
		a)
			make clean
			make 
			cp -f $drivername $copydir$drivername
			;;
		b)
			arm-linux-gnueabihf-gcc $appname.c -o $appname
			cp -f $appname $copydir$appname
			;;
		?)
			echo "unkowned argument."
			;;
	esac
done