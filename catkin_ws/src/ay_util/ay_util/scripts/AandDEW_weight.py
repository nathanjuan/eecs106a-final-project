#!/usr/bin/python
#\file    AandDEW_weight.py
#\brief   ROS node to read and publish data from the A and D EW weight.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Aug.26, 2019
#\version 0.2
#\date    Feb.03, 2020
#         Modified to publish messages with with time stamps.

import roslib;
roslib.load_manifest('ay_util_msgs')
import rospy
import std_msgs.msg
import ay_util_msgs.msg
import sys
import serial

if __name__=='__main__':
  rospy.init_node('weight')
  dev= sys.argv[1] if len(sys.argv)>1 else '/dev/ttyUSB0'
  baudrate= int(sys.argv[2]) if len(sys.argv)>2 else 2400
  #echo= int(sys.argv[3]) if len(sys.argv)>3 else False

  ser= serial.Serial(dev,baudrate,serial.SEVENBITS,serial.PARITY_EVEN)

  pub_value= rospy.Publisher('~value', ay_util_msgs.msg.Float64Stamped, queue_size=1)
  pub_raw= rospy.Publisher('~raw', ay_util_msgs.msg.StringStamped, queue_size=1)

  header= std_msgs.msg.Header()
  header.frame_id= ''  #Shall we use node name? (rospy.get_name())

  t_start= rospy.Time.now()
  try:
    while not rospy.is_shutdown():
      raw= ser.readline()
      header.stamp= rospy.Time.now()
      if (header.stamp-t_start).to_sec()<10.0:
        print '(displayed only first 10sec) "{raw}" ({l})'.format(raw=repr(raw), l=len(raw))
      if len(raw)!=17:  continue
      value= float(raw[3:12])

      value_msg= ay_util_msgs.msg.Float64Stamped()
      value_msg.header= header
      value_msg.data= value
      raw_msg= ay_util_msgs.msg.StringStamped()
      raw_msg.header= header
      raw_msg.data= raw
      pub_value.publish(value_msg)
      pub_raw.publish(raw_msg)

  finally:
    ser.close()
