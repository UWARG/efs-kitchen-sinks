#!/bin/sh

# Script that returns the ttyUSB path given a vendor and device id
# 0403:6001 - Vectornav
# 0403:6015 - XBee

# shopt -s nullglob
selectedDevice=$(for i in /dev/ttyUSB*; do 
  udevadm info -r -q all "$i" | awk -F= '
     /DEVNAME/{d=$2}
     /ID_VENDOR_ID/{v=$2}
     /ID_MODEL_ID/{m=$2}
     d&&v&&m{print d,v":"m;d=v=m=""}
  ' | grep $1 | cut -d " " -f 1
done)
echo $selectedDevice
