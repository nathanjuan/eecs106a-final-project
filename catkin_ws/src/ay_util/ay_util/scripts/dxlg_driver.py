#!/usr/bin/python
#\file    dxlg_driver.py
#\brief   ROS driver of Dynamixel-based grippers.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Feb.28, 2020
import roslib;
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('ay_py')
roslib.load_manifest('ay_util_msgs')
import rospy
import sensor_msgs.msg
import sys
import ay_util_msgs.srv
from ay_py.misc.dxl_util import DxlPortHandler

class TDxlGripperDriver(object):
  def __init__(self, dev='/dev/ttyUSB0', gripper_type='DxlGripper', finger_type=None):
    self.dev= dev
    self.gripper_type= gripper_type
    if self.gripper_type=='DxlGripper':
      mod= __import__('ay_py.misc.dxl_gripper',globals(),None,('TDynamixelGripper',))
      self.gripper= mod.TDynamixelGripper(dev=self.dev)
      self.joint_names= ['joint0']
      self.dxl= {'joint0':self.gripper.dxl}
    elif self.gripper_type=='RHP12RNGripper':
      mod= __import__('ay_py.misc.dxl_rhp12rn',globals(),None,('TRHP12RN',))
      self.gripper= mod.TRHP12RN(dev=self.dev)
      self.joint_names= ['joint0']
      self.dxl= {'joint0':self.gripper.dxl}
    elif self.gripper_type=='EZGripper':
      mod= __import__('ay_py.misc.dxl_ezg',globals(),None,('TEZG',))
      self.gripper= mod.TEZG(dev=self.dev)
      self.joint_names= ['joint0']
      self.dxl= {'joint0':self.gripper.dxl}
    elif self.gripper_type=='DxlpO2Gripper':
      mod= __import__('ay_py.misc.dxl_dxlpo2',globals(),None,('TDxlpO2',))
      self.gripper= mod.TDxlpO2(dev=self.dev, finger_type=finger_type)
      self.joint_names= ['joint0']
      self.dxl= {'joint0':self.gripper.dxl}
    elif self.gripper_type=='DxlpY1Gripper':
      mod= __import__('ay_py.misc.dxl_dxlpy1',globals(),None,('TDxlpY1',))
      self.gripper= mod.TDxlpY1(dev=self.dev)
      self.joint_names= ['joint0']
      self.dxl= {'joint0':self.gripper.dxl}
    elif self.gripper_type=='DxlO3Gripper':
      mod= __import__('ay_py.misc.dxl_dxlo3',globals(),None,('TDxlO3',))
      self.gripper= mod.TDxlO3(dev=self.dev)
      self.joint_names= ['joint0','joint1']
      self.dxl= {'joint0':self.gripper.dxl[0], 'joint1':self.gripper.dxl[1]}
    else:
      raise Exception('Invalid gripper type: {gripper_type}'.format(gripper_type=gripper_type))

    #Set callback to exit when Ctrl+C is pressed.
    DxlPortHandler.ReopenCallback= lambda: not rospy.is_shutdown()

    self.pub_js= rospy.Publisher('~joint_states', sensor_msgs.msg.JointState, queue_size=1)

    self.js= None

    print 'Initializing and activating {gripper_type}({finger_type}) gripper...'.format(gripper_type=self.gripper_type,finger_type=finger_type)
    if not self.gripper.Init():
      raise Exception('Failed to setup {gripper_type}({finger_type}) gripper.'.format(gripper_type=self.gripper_type,finger_type=finger_type))
    self.gripper.StartStateObs(self.JointStatesCallback)
    self.gripper.StartMoveTh()

    self.srv_move= rospy.Service('~move', ay_util_msgs.srv.DxlGMove, self.MoveHandler)
    self.srv_io= rospy.Service('~dxl_io', ay_util_msgs.srv.DxlIO, self.DxlIOHandler)

    while not rospy.is_shutdown():
      rospy.sleep(0.5)
    self.Cleanup()

  def __del__(self):
    self.Cleanup()

  def Cleanup(self):
    print 'Cleanup'
    self.gripper.StopMoveTh()
    self.gripper.StopStateObs()
    self.gripper.Cleanup()

  def JointStatesCallback(self, state):
    if rospy.is_shutdown():
      return False
    if self.js is None:
      self.js= sensor_msgs.msg.JointState()
      self.js.name= self.joint_names
    selector= lambda v: [None] if v is None else [v] if len(self.joint_names)==1 else v
    self.js.header.stamp= rospy.Time(state['stamp'])
    self.js.position= selector(state['position'])
    self.js.velocity= selector(state['velocity'])
    self.js.effort=   selector(state['effort'])
    if None in self.js.position+self.js.velocity+self.js.effort:
      return  #We do not publish the state if a part of it is not observed.
    self.pub_js.publish(self.js)
    return True

  # Handler of move service (ay_util_msgs/DxlGMove).
  def MoveHandler(self, req):
    res= ay_util_msgs.srv.DxlGMoveResponse()
    res.success= True
    if req.command=='Open':  #Open the gripper.
      self.gripper.Open(blocking=req.blocking)
    elif req.command=='Close':  #Close the gripper.
      self.gripper.Close(blocking=req.blocking)
    elif req.command=='Stop':   #Stop the gripper motion.
      self.gripper.Stop()
    elif req.command=='Move':   #Move the gripper with the specified parameters.
      selector= lambda v: None if len(v)==0 else (v[0] if len(self.joint_names)==1 or len(v)==1 else v)
      pos= req.pos[0] if len(self.joint_names)==1 else req.pos
      max_effort= selector(req.max_effort)
      speed= selector(req.speed)
      if max_effort is None and speed is None:
        self.gripper.MoveTh(pos, blocking=req.blocking)
      elif max_effort is None:
        self.gripper.MoveTh(pos, speed=speed, blocking=req.blocking)
      elif speed is None:
        self.gripper.MoveTh(pos, max_effort=max_effort, blocking=req.blocking)
      else:
        self.gripper.MoveTh(pos, max_effort=max_effort, speed=speed, blocking=req.blocking)
    elif req.command=='StartHolding':   #Start the holding controller (not available in all grippers).
      if 'StartHolding' in dir(self.gripper):
        self.gripper.StartHolding()
    elif req.command=='StopHolding':    #Stop the holding controller.
      if 'StopHolding' in dir(self.gripper):
        self.gripper.StopHolding()
    else:
      res.success= False
      res.message= 'Unknown command: {cmd}'.format(cmd=req.command)
      print res.message
    return res

  # Handler of dxl_io service (ay_util_msgs/DxlIO).
  def DxlIOHandler(self, req):
    res= ay_util_msgs.srv.DxlIOResponse()
    joint_names= self.joint_names if len(req.joint_names)==0 else req.joint_names
    if req.command=='Read':  #Read from Dynamixel. input: joint_names, data_s (address name).  return: res_ia.
      with self.gripper.port_locker:
        res.res_ia= [self.dxl[j].Read(req.data_s) for j in joint_names]
    elif req.command=='Write':  #Write to Dynamixel. input: joint_names, data_s (address name), data_ia (values).
      with self.gripper.port_locker:
        for j,value in zip(joint_names,req.data_ia):
          self.dxl[j].Write(req.data_s, value)
    elif req.command=='EnableTorque':  #Enable joint_names (joint_names is [], all joints are enabled).
      with self.gripper.port_locker:
        for j in joint_names:  self.dxl[j].EnableTorque()
    elif req.command=='DisableTorque':  #Disable joint_names (joint_names is [], all joints are disabled).
      with self.gripper.port_locker:
        for j in joint_names:  self.dxl[j].DisableTorque()
    elif req.command=='Reboot':  #Reboot joint_names (joint_names is [], all joints are rebooted).
      with self.gripper.port_locker:
        for j in joint_names:  self.dxl[j].Reboot()

    j= joint_names[-1]
    res.result= self.dxl[j].dxl_result  #dynamixel.getLastTxRxResult
    res.error= self.dxl[j].dxl_err  #dynamixel.getLastRxPacketError
    return res

if __name__=='__main__':
  rospy.init_node('gripper_driver')
  dev= sys.argv[1] if len(sys.argv)>1 else '/dev/ttyUSB0'
  gripper_type= sys.argv[2] if len(sys.argv)>2 else 'DxlGripper'
  finger_type= sys.argv[3] if len(sys.argv)>3 else None
  print 'args=',sys.argv
  robot= TDxlGripperDriver(dev, gripper_type, finger_type)
  #rospy.spin()
