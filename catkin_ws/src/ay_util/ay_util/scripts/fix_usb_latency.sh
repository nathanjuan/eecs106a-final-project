#!/bin/bash
#\file    fix_usb_latency.sh
#\brief   Fix the latency issue of USB.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.01, 2018
#\version 0.2
#\date    May.20, 2022
#         Modified to set arbitrary latency timer.

#Usage(1): $ ./fix_usb_latency.sh [ttyUSB0 [LATENCY_TIMER]]
#Usage(2): $ rosrun ay_util fix_usb_latency.sh [ttyUSB0 [LATENCY_TIMER]]
#Usage(3):
#  In order to let a normal user use this script without a root privelege,
#  $ sudo ln -s `rospack find ay_util`/scripts/fix_usb_latency.sh /sbin/
#  $ sudo visudo
#  (To give a user akihikoy the permission:)
#   akihikoy ALL=PASSWD: ALL, NOPASSWD: /sbin/fix_usb_latency.sh
#  (To give a group dialout the permission:)
#   %dialout ALL=PASSWD: ALL, NOPASSWD: /sbin/fix_usb_latency.sh
#  $ sudo /sbin/fix_usb_latency.sh ttyUSB0      #No password requested.

dev=ttyUSB0
latency_timer=1
if [ $# -ge 1 ];then
  dev=$1
fi
num_re='^[0-9]+$'
if [ $# -ge 2 ] && [[ $2 =~ $num_re ]];then
  latency_timer=$2
fi
config_file=/sys/bus/usb-serial/devices/$dev/latency_timer
if [ -f $config_file ];then
  echo "Config file: $config_file"
  if [ `cat $config_file` -ne $latency_timer ];then
    echo "Setting the latency timer of $dev to $latency_timer..."
    echo "$latency_timer" | sudo tee $config_file
  else
    echo "The latency timer of $dev is already $latency_timer."
  fi
else
  echo "Config file does not exist: $config_file"
fi
