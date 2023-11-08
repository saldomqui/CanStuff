#!/bin/bash -x

while [ ! -e /sys/class/net/slcan0 ]; do
  echo "slcan0 CAN interface has not been created yet"
  /etc/init.d/slcan.sh
  sleep 3
done
echo "slcan0 CAN interface EXISTS"


