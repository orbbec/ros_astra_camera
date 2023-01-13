#!/bin/sh

# Check if user is root/running with sudo
if [ `whoami` != root ]; then
    echo Please run this script with sudo
    exit
fi

ORIG_PATH=`pwd`
cd `dirname $0`
SCRIPT_PATH=`pwd`
cd $ORIG_PATH

if [ "`uname -s`" != "Darwin" ]; then
    # Install UDEV rules for USB device
    cp ${SCRIPT_PATH}/56-orbbec-usb.rules /etc/udev/rules.d/56-orbbec-usb.rules 
    echo "usb rules file install at /etc/udev/rules.d/56-orbbec-usb.rules" 
fi
echo "exit"