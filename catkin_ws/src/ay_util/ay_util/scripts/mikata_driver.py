#!/usr/bin/python
#\file    mikata_driver.py
#\brief   Mikata ROS node.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Aug.29, 2018
#\version 0.2
#\date    Nov.01, 2018
#         Added support of Crane-X7
import roslib;
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('ay_py')
roslib.load_manifest('ay_util_msgs')
import rospy
import sensor_msgs.msg
import trajectory_msgs.msg
import actionlib
import control_msgs.msg
import sys
import ay_util_msgs.srv
from ay_py.misc.dxl_util import DxlPortHandler

class TMikataDriver(object):
  def __init__(self, dev='/dev/ttyUSB0', robot_type='Mikata'):
    self.dev= dev
    self.robot_type= robot_type
    if self.robot_type=='Mikata':
      mod= __import__('ay_py.misc.dxl_mikata',globals(),None,('TMikata',))
      self.mikata= mod.TMikata(dev=self.dev)
    elif self.robot_type=='CraneX7':
      mod= __import__('ay_py.misc.dxl_cranex7',globals(),None,('TCraneX7',))
      self.mikata= mod.TCraneX7(dev=self.dev)
    elif self.robot_type=='Mikata6':
      mod= __import__('ay_py.misc.dxl_mikata6',globals(),None,('TMikata6',))
      self.mikata= mod.TMikata6(dev=self.dev)
    else:
      raise Exception('Invalid robot type: {robot_type}'.format(robot_type=robot_type))

    #Set callback to exit when Ctrl+C is pressed.
    DxlPortHandler.ReopenCallback= lambda: not rospy.is_shutdown()

    self.pub_js= rospy.Publisher('/joint_states', sensor_msgs.msg.JointState, queue_size=1)

    self.js= None
    self.joint_names= self.mikata.JointNames()

    print 'Initializing and activating {robot_type} arm...'.format(robot_type=self.robot_type)
    if not self.mikata.Setup():
      raise Exception('Failed to setup {robot_type} arm.'.format(robot_type=self.robot_type))
    #self.mikata.EnableTorque()
    self.mikata.StartStateObs(self.JointStatesCallback)

    self.sub_jpc= rospy.Subscriber('/joint_path_command', trajectory_msgs.msg.JointTrajectory, self.PathCmdCallback)
    #self.sub_jsc= rospy.Subscriber('/joint_speed_command', trajectory_msgs.msg.JointTrajectory, self.SpeedCmdCallback, queue_size=1)

    self.ftaction_feedback= control_msgs.msg.FollowJointTrajectoryFeedback()
    self.ftaction_result= control_msgs.msg.FollowJointTrajectoryResult()
    self.ftaction_name= '/follow_joint_trajectory'
    self.ftaction_actsrv= actionlib.SimpleActionServer(self.ftaction_name, control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.FollowTrajActionCallback, auto_start=False)
    self.ftaction_actsrv.start()

    self.srv_io= rospy.Service('~robot_io', ay_util_msgs.srv.DxlIO, self.DxlIOHandler)

  def __del__(self):
    self.Cleanup()

  def Cleanup(self):
    print 'Cleanup'
    self.mikata.StopStateObs()
    self.mikata.Quit()

  def JointStatesCallback(self, state):
    if rospy.is_shutdown():
      return False
    if self.js is None:
      self.js= sensor_msgs.msg.JointState()
      self.js.name= state['name']
      self.js.header.seq= 0
    self.js.header.seq= self.js.header.seq+1
    self.js.header.stamp= rospy.Time.now()
    self.js.position= state['position']
    self.js.velocity= state['velocity']
    self.js.effort= state['effort']
    self.pub_js.publish(self.js)
    return True

  def PathCmdCallback(self, msg):
    def callback(context,t,q,dq):
      if context=='loop_begin':
        if rospy.is_shutdown():  return False
        return True
    q_traj= [p.positions for p in msg.points]
    t_traj= [p.time_from_start.to_sec() for p in msg.points]
    self.mikata.FollowTrajectory(msg.joint_names, q_traj, t_traj, blocking=False, callback=callback)

  # Callback for actionlib.SimpleActionServer.
  def FollowTrajActionCallback(self, goal):
    def callback(context,t,q,dq):
      #print context,t,q,dq,'---',self.ftaction_result.error_code
      if context=='loop_begin':
        if rospy.is_shutdown():  return False
        if self.ftaction_actsrv.is_preempt_requested():
          print '%s: Preempted' % self.ftaction_name
          self.ftaction_actsrv.set_preempted()
          self.ftaction_result.error_code= None
          return False
        self.ftaction_actsrv.publish_feedback(self.ftaction_feedback)
        return True
      elif context=='loop_end':
        pass
      elif context=='final':
        pass
    self.ftaction_result.error_code= self.ftaction_result.SUCCESSFUL
    q_traj= [p.positions for p in goal.trajectory.points]
    t_traj= [p.time_from_start.to_sec() for p in goal.trajectory.points]
    self.mikata.FollowTrajectory(goal.trajectory.joint_names, q_traj, t_traj, blocking=True, callback=callback)
    if self.ftaction_result.error_code==self.ftaction_result.SUCCESSFUL:
      self.ftaction_result.error_code= self.ftaction_result.SUCCESSFUL
      print '%s: Succeeded' % self.ftaction_name
      self.ftaction_actsrv.set_succeeded(self.ftaction_result)

  # Handler of robot_io service (ay_util_msgs/DxlIO).
  def DxlIOHandler(self, req):
    res= ay_util_msgs.srv.DxlIOResponse()
    if req.command=='Read':  #Read from Dynamixel. input: joint_names, data_s (address name).  return: res_ia.
      with self.mikata.port_locker:
        res.res_ia= [self.mikata.dxl[j].Read(req.data_s) for j in req.joint_names]
    elif req.command=='Write':  #Write to Dynamixel. input: joint_names, data_s (address name), data_ia (values).
      with self.mikata.port_locker:
        for j,value in zip(req.joint_names,req.data_ia):
          self.mikata.dxl[j].Write(req.data_s, value)
    elif req.command=='EnableTorque':  #Enable joint_names (joint_names is [], all joints are enabled).
      if len(req.joint_names)==0:  self.mikata.EnableTorque()
      else:  self.mikata.EnableTorque(req.joint_names)
    elif req.command=='DisableTorque':  #Disable joint_names (joint_names is [], all joints are disabled).
      if len(req.joint_names)==0:  self.mikata.DisableTorque()
      else:  self.mikata.DisableTorque(req.joint_names)
    elif req.command=='Reboot':  #Reboot joint_names (joint_names is [], all joints are rebooted).
      if len(req.joint_names)==0:  self.mikata.Reboot()
      else:  self.mikata.Reboot(req.joint_names)
    elif req.command=='MoveTo':  #Move to target position.  input: joint_names, data_fa (joint positions in radian), data_b (blocking).
      #print 'MoveTo',dict(zip(req.joint_names,req.data_fa))
      self.mikata.MoveTo(dict(zip(req.joint_names,req.data_fa)), blocking=req.data_b)
    elif req.command=='SetCurrent':  #Set current.  input: joint_names, data_fa (currents in mA).
      self.mikata.SetCurrent(dict(zip(req.joint_names,req.data_fa)))
    elif req.command=='SetVelocity':  #Set velocity.  input: joint_names, data_fa (velocities in rad/s).
      self.mikata.SetVelocity(dict(zip(req.joint_names,req.data_fa)))
    elif req.command=='SetPWM':  #Set PWM.  input: joint_names, data_fa (PWM values in percentage).
      #print 'SetPWM',dict(zip(req.joint_names,req.data_fa))
      self.mikata.SetPWM(dict(zip(req.joint_names,req.data_fa)))

    j= req.joint_names[-1] if len(req.joint_names)>0 else self.joint_names[-1]
    res.result= self.mikata.dxl[j].dxl_result  #dynamixel.getLastTxRxResult
    res.error= self.mikata.dxl[j].dxl_err  #dynamixel.getLastRxPacketError
    return res

if __name__=='__main__':
  rospy.init_node('mikata_driver')
  dev= sys.argv[1] if len(sys.argv)>1 else '/dev/ttyUSB0'
  robot_type= sys.argv[2] if len(sys.argv)>2 else 'Mikata'
  print 'args=',sys.argv
  robot= TMikataDriver(dev,robot_type)
  rospy.spin()
