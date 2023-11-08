#!/usr/bin/env sh

### BEGIN INIT INFO
# Provides:          scriptname
# Required-Start:    
# Required-Stop:     
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Start daemon at boot time
# Description:       Enable service provided by daemon.
### END INIT INFO

SHELL=/bin/sh
PATH=/usr/local/sbin:/usr/local/bin:/sbin:/bin:/usr/sbin:/usr/bin

sudo modprobe can
sudo modprobe can-raw
sudo modprobe slcan


do=true
while  $do; do 
  echo "slcan0 CAN interface is down. Setting it up"
  sudo slcand -o -s6 -t hw -S 500000 /dev/can_reader slcan0
  sudo ip link set up slcan0
  sleep 3
  
  do=false
  #ip a | grep -Eq ': slcan0:.*state UP' || do=false
  if cat /sys/class/net/slcan0/operstate | grep -q "down"; then
   do=true
  fi

done
echo "slcan0 CAN interface is UP"



  
#./filter_can_ids

