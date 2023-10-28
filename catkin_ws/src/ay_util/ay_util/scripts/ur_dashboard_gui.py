#!/usr/bin/python
#\file    ur_dashboard_gui.py
#\brief   Dashboard GUI for monitoring and controlling Universal Robots.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.16, 2021
import roslib
roslib.load_manifest('ay_py')
roslib.load_manifest('ay_trick_msgs')
from ay_py.core import CPrint
from ay_py.tool.py_panel import TSimplePanel, InitPanelApp, RunPanelApp, AskYesNoDialog, QtCore, QtGui
from ay_py.ros.base import SetupServiceProxy
import sys
import subprocess
import threading
import rospy
import std_msgs.msg
import std_srvs.srv
import sensor_msgs.msg
import ay_trick_msgs.msg
import ay_trick_msgs.srv
try:
  roslib.load_manifest('ur_dashboard_msgs')
  import ur_dashboard_msgs.msg
  roslib.load_manifest('ur_msgs')
  import ur_msgs.msg
  import ur_msgs.srv
except Exception as e:
  print e

class TProcessManager(QtCore.QObject):

  #Definition of states:
  UNDEFINED= -10
  FAULT= -4
  EMERGENCY= -3
  PROTECTIVE_STOP= -2
  ROBOT_EMERGENCY_STOP= -1
  NO_CORE_PROGRAM= 0
  POWER_OFF= 1
  BOOTING= 2
  IDLE= 3
  TORQUE_ENABLED= 4
  ROBOT_READY= 5
  WAIT_REQUEST= 6
  PROGRAM_RUNNING= 7

  onstatuschanged= QtCore.pyqtSignal(int)

  def UpdateStatus(self):
    #self.ur_robot_mode= None  #ur_dashboard_msgs.msg.RobotMode.{POWER_OFF,BOOTING,IDLE,RUNNING}
    #self.ur_safety_mode= None  #ur_dashboard_msgs.msg.SafetyMode.{NORMAL,PROTECTIVE_STOP,ROBOT_EMERGENCY_STOP}
    #self.ur_topic_stamp= None
    #self.ur_program_running= None  #{True,False}
    #self.script_node_status= None  #ay_trick_msgs.msg.ROSNodeMode.{BUSY,READY,PROGRAM_RUNNING}
    #self.script_node_status_stamp= None

    #FIXME: In which case, we should set EMERGENCY?
    t_now= rospy.Time.now()
    #print ((t_now-self.ur_topic_stamp).to_sec()) if self.ur_topic_stamp is not None else '--'
    #print ((t_now-self.script_node_status_stamp).to_sec()) if self.script_node_status_stamp is not None else '++'
    self.ur_ros_running= ((t_now-self.ur_topic_stamp).to_sec() < 0.1) if self.ur_topic_stamp is not None else False
    self.script_node_running= ((t_now-self.script_node_status_stamp).to_sec() < 0.2) if self.script_node_status_stamp is not None else False

    status= self.UNDEFINED
    if not self.ur_ros_running:
      status= self.NO_CORE_PROGRAM
    else:
      if self.ur_safety_mode == ur_dashboard_msgs.msg.SafetyMode.FAULT:
        status= self.FAULT
      elif self.ur_safety_mode == ur_dashboard_msgs.msg.SafetyMode.ROBOT_EMERGENCY_STOP:
        status= self.ROBOT_EMERGENCY_STOP
      elif self.ur_safety_mode == ur_dashboard_msgs.msg.SafetyMode.PROTECTIVE_STOP:
        status= self.PROTECTIVE_STOP
      elif self.ur_safety_mode == ur_dashboard_msgs.msg.SafetyMode.NORMAL:
        if self.ur_robot_mode == ur_dashboard_msgs.msg.RobotMode.POWER_OFF:
          status= self.POWER_OFF
        elif self.ur_robot_mode == ur_dashboard_msgs.msg.RobotMode.BOOTING:
          status= self.BOOTING
        elif self.ur_robot_mode == ur_dashboard_msgs.msg.RobotMode.IDLE:
          status= self.IDLE
        elif self.ur_robot_mode == ur_dashboard_msgs.msg.RobotMode.RUNNING:
          if not self.ur_program_running:
            status= self.TORQUE_ENABLED
          else:
            if not self.script_node_running:
              status= self.ROBOT_READY
            elif self.script_node_status == ay_trick_msgs.msg.ROSNodeMode.READY:
              status= self.WAIT_REQUEST
            elif self.script_node_status == ay_trick_msgs.msg.ROSNodeMode.PROGRAM_RUNNING:
              status= self.PROGRAM_RUNNING

    if status in (self.UNDEFINED, self.EMERGENCY, self.ROBOT_EMERGENCY_STOP):
      self.status_color= ['red']
    elif status in (self.WAIT_REQUEST,):
      self.status_color= ['green','yellow']
    elif status in (self.PROGRAM_RUNNING,):
      self.status_color= ['green']
    else:
      self.status_color= ['yellow']

    if self.status != status:
      self.status= status
      self.OnStatusChanged()
    #self.onstatuschanged.emit(self.status)

  def OnStatusChanged(self):
    #if self.panel is None:  return
    #for w_name, (w_type, w_param) in self.panel.widgets_in.iteritems():
      #if 'onstatuschanged' in w_param and w_param['onstatuschanged'] is not None:
        #w_param['onstatuschanged'](self.panel, self.status)
    self.onstatuschanged.emit(self.status)

    self.TurnOffLEDAll()
    for color in self.status_color:  self.SetLEDLight(color, True)

    if self.status in (self.PROGRAM_RUNNING,):
      self.SetStartStopLEDs(True, True)

  def UpdateStatusThread(self):
    rate= rospy.Rate(20)
    while self.thread_status_update_running and not rospy.is_shutdown():
      self.UpdateStatus()
      rate.sleep()

  def StartUpdateStatusThread(self):
    self.thread_status_update_running= True
    self.thread_status_update= threading.Thread(name='status_update', target=self.UpdateStatusThread)
    self.thread_status_update.start()

  def StopUpdateStatusThread(self):
    self.thread_status_update_running= False
    if self.thread_status_update is not None:
      self.thread_status_update.join()

  def __init__(self, node_name='ur_dashboard'):
    QtCore.QObject.__init__(self)
    self.node_name= node_name

    self.procs= {}

    self.status_names= {
      self.UNDEFINED:           'UNDEFINED'           ,
      self.FAULT:               'FAULT'               ,
      self.EMERGENCY:           'EMERGENCY'           ,
      self.PROTECTIVE_STOP:     'PROTECTIVE_STOP'     ,
      self.ROBOT_EMERGENCY_STOP:'ROBOT_EMERGENCY_STOP',
      self.NO_CORE_PROGRAM:     'NO_CORE_PROGRAM'     ,
      self.POWER_OFF:           'POWER_OFF'           ,
      self.BOOTING:             'BOOTING'             ,
      self.IDLE:                'IDLE'                ,
      self.TORQUE_ENABLED:      'TORQUE_ENABLED'      ,
      self.ROBOT_READY:         'ROBOT_READY'         ,
      self.WAIT_REQUEST:        'WAIT_REQUEST'        ,
      self.PROGRAM_RUNNING:     'PROGRAM_RUNNING'     ,
      }
    self.ur_robot_mode_names= {
      ur_dashboard_msgs.msg.RobotMode.POWER_OFF: 'POWER_OFF',
      ur_dashboard_msgs.msg.RobotMode.BOOTING: 'BOOTING',
      ur_dashboard_msgs.msg.RobotMode.IDLE: 'IDLE',
      ur_dashboard_msgs.msg.RobotMode.RUNNING: 'RUNNING' }
    self.ur_safety_mode_names= {
      ur_dashboard_msgs.msg.SafetyMode.NORMAL: 'NORMAL',
      ur_dashboard_msgs.msg.SafetyMode.PROTECTIVE_STOP: 'PROTECTIVE_STOP',
      ur_dashboard_msgs.msg.SafetyMode.ROBOT_EMERGENCY_STOP: 'ROBOT_EMERGENCY_STOP',
      ur_dashboard_msgs.msg.SafetyMode.FAULT: 'FAULT' }
    self.script_node_status_names= {
      ay_trick_msgs.msg.ROSNodeMode.BUSY: 'BUSY',
      ay_trick_msgs.msg.ROSNodeMode.READY: 'READY',
      ay_trick_msgs.msg.ROSNodeMode.PROGRAM_RUNNING: 'PROGRAM_RUNNING' }

    #self.panel= None
    self.status= self.UNDEFINED
    self.status_color= []  #List of 'red','yellow','green'
    self.ur_ros_running= None
    self.script_node_running= None
    self.ur_topic_stamp= None
    self.srvp_ur_dashboard= {}
    self.srvp_ur_set_io= None
    self.ur_robot_mode= None
    self.ur_safety_mode= None
    self.ur_program_running= None
    self.script_node_status= None
    self.script_node_status_stamp= None

    self.thread_status_update= None
    self.thread_status_update_running= False

  #command: list of command and arguments.
  def RunFGProcess(self, command, shell=False):
    p= subprocess.Popen(command, shell=shell)
    p.wait()

  #command: list of command and arguments.
  def RunBGProcess(self, name, command, shell=False):
    self.TerminateBGProcess(name)
    p= subprocess.Popen(command, shell=shell)
    self.procs[name]= p

  def TerminateBGProcess(self, name):
    if name not in self.procs:
      print 'No process named',name
      return
    self.procs[name].terminate()
    self.procs[name].wait()
    #TODO: wait(): It is safer to have timeout.  For ver<3.3, implement like:
    #while p.poll() is None:
      #print 'Process still running...'
      #time.sleep(0.1)
    del self.procs[name]

  def TerminateAllBGProcesses(self):
    for name,p in self.procs.iteritems():
      print 'Terminating',name
      p.terminate()
      p.wait()
      #TODO: wait(): It is safer to have timeout.  For ver<3.3, implement like:
      #while p.poll() is None:
        #print 'Process still running...'
        #time.sleep(0.1)
    self.procs= {}

  #WARNING: This is not safe.  When killing roscore, rosmaster is still alive.
  def KillBGProcess(self, name):
    if name not in self.procs:
      print 'No process named',name
      return
    self.procs[name].kill()
    self.procs[name].wait()
    #TODO: wait(): It is safer to have timeout.  For ver<3.3, implement like:
    #while p.poll() is None:
      #print 'Process still running...'
      #time.sleep(0.1)
    del self.procs[name]

  def IsBGProcessRunning(self, name):
    return self.procs[name].poll() is None

  def InitNode(self):
    rospy.init_node(self.node_name)
    rospy.sleep(0.1)

  def ConnectToURDashboard(self, timeout=6.0):
    services= ['power_on', 'power_off', 'brake_release', 'play', 'stop', 'shutdown', 'unlock_protective_stop', 'restart_safety', 'close_safety_popup']
    #for service in services:
      #self.srvp_ur_dashboard[service]= SetupServiceProxy('/ur_hardware_interface/dashboard/{0}'.format(service), std_srvs.srv.Trigger, persistent=False, time_out=3.0)
    #self.srvp_ur_set_io= SetupServiceProxy('/ur_hardware_interface/set_io', ur_msgs.srv.SetIO, persistent=False, time_out=10.0)
    threads= {}
    for service in services:
      threads[service]= threading.Thread(name=service, target=lambda service=service:(self.srvp_ur_dashboard.__setitem__(service,SetupServiceProxy('/ur_hardware_interface/dashboard/{0}'.format(service), std_srvs.srv.Trigger, persistent=False, time_out=timeout))))
      threads[service].start()
    threads['srvp_ur_set_io']= threading.Thread(name='srvp_ur_set_io', target=lambda:(setattr(self,'srvp_ur_set_io',SetupServiceProxy('/ur_hardware_interface/set_io', ur_msgs.srv.SetIO, persistent=False, time_out=timeout))))
    threads['srvp_ur_set_io'].start()
    for name,th in threads.iteritems():  th.join()

    #Connect to io_states to publish a fake io_states for debug.
    self.pub_io_states= rospy.Publisher('/ur_hardware_interface/io_states', ur_msgs.msg.IOStates, queue_size=10)

    self.sub_joint_states= rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self.JointStatesCallback)
    self.sub_robot_mode= rospy.Subscriber('/ur_hardware_interface/robot_mode', ur_dashboard_msgs.msg.RobotMode, self.RobotModeCallback)
    self.sub_safety_mode= rospy.Subscriber('/ur_hardware_interface/safety_mode', ur_dashboard_msgs.msg.SafetyMode, self.SafetyModeCallback)
    self.sub_program_running= rospy.Subscriber('/ur_hardware_interface/robot_program_running', std_msgs.msg.Bool, self.ProgramRunningCallback)

  def DisconnectUR(self):
    self.TurnOffLEDAll()
    self.srvp_ur_set_io= None  #TODO: srvp_ur_set_io may be used from other thread, so the LED may be turned on before this.
    self.srvp_ur_dashboard= {}

    self.sub_joint_states.unregister()
    self.sub_robot_mode.unregister()
    self.sub_safety_mode.unregister()
    self.sub_program_running.unregister()
    self.sub_joint_states= None
    self.sub_robot_mode= None
    self.sub_safety_mode= None
    self.sub_program_running= None

    self.pub_io_states.unregister()
    self.pub_io_states= None

  def RunURDashboard(self, service):
    if service not in self.srvp_ur_dashboard:
      return False
    res= self.srvp_ur_dashboard[service]()
    return res.success

  #int8 fun, int8 pin, float32 state
  #fun: ur_msgs.srv.SetIORequest.{FUN_SET_DIGITAL_OUT,FUN_SET_FLAG,FUN_SET_ANALOG_OUT,FUN_SET_TOOL_VOLTAGE}
  #state: ur_msgs.srv.SetIORequest.{STATE_OFF,STATE_ON}
  def SetURIO(self, fun, pin, state):
    if self.srvp_ur_set_io is None:  return
    return self.srvp_ur_set_io(ur_msgs.srv.SetIORequest(fun, pin, state)).success

  def TurnOffLEDAll(self):
    self.SetLEDLight('red', False)
    self.SetLEDLight('green', False)
    self.SetLEDLight('yellow', False)
    self.SetBeep(False)
    self.SetStartStopLEDs(False, False)

  #color: 'red','yellow','green'  #FIXME: the pin id is hard-coded, should be put in a config file.
  def SetLEDLight(self, color, is_on):
    pin= {'red':0,'yellow':1,'green':2}[color]
    state= ur_msgs.srv.SetIORequest.STATE_ON if is_on else ur_msgs.srv.SetIORequest.STATE_OFF
    return self.SetURIO(ur_msgs.srv.SetIORequest.FUN_SET_DIGITAL_OUT, pin, state)

  #FIXME: the pin id is hard-coded, should be put in a config file.
  def SetBeep(self, is_on):
    pin= 3
    state= ur_msgs.srv.SetIORequest.STATE_ON if is_on else ur_msgs.srv.SetIORequest.STATE_OFF
    return self.SetURIO(ur_msgs.srv.SetIORequest.FUN_SET_DIGITAL_OUT, pin, state)

  #FIXME: the pin id is hard-coded, should be put in a config file.
  def SetStartStopLEDs(self, is_start_on, is_stop_on):
    start_pin= 4
    start_state= ur_msgs.srv.SetIORequest.STATE_ON if is_start_on else ur_msgs.srv.SetIORequest.STATE_OFF
    self.SetURIO(ur_msgs.srv.SetIORequest.FUN_SET_DIGITAL_OUT, start_pin, start_state)
    stop_pin= 5
    stop_state= ur_msgs.srv.SetIORequest.STATE_ON if is_stop_on else ur_msgs.srv.SetIORequest.STATE_OFF
    self.SetURIO(ur_msgs.srv.SetIORequest.FUN_SET_DIGITAL_OUT, stop_pin, stop_state)

  def JointStatesCallback(self, msg):
    self.ur_topic_stamp= rospy.Time.now()

  def RobotModeCallback(self, msg):
    self.ur_robot_mode= msg.mode

  def SafetyModeCallback(self, msg):
    self.ur_safety_mode= msg.mode
    if self.ur_safety_mode!=ur_dashboard_msgs.msg.SafetyMode.NORMAL and self.ur_program_running:
      self.RunURDashboard('stop')
      self.WaitForProgramRunning(False, timeout=1.0)

  def ProgramRunningCallback(self, msg):
    self.ur_program_running= msg.data

  #mode: ur_dashboard_msgs.msg.RobotMode.{POWER_OFF,BOOTING,IDLE,RUNNING}
  def WaitForRobotMode(self, mode, timeout=20):
    if mode==ur_dashboard_msgs.msg.RobotMode.POWER_OFF and\
      self.ur_robot_mode in (ur_dashboard_msgs.msg.RobotMode.BOOTING,
                             ur_dashboard_msgs.msg.RobotMode.IDLE,
                             ur_dashboard_msgs.msg.RobotMode.RUNNING):  return False
    if mode==ur_dashboard_msgs.msg.RobotMode.IDLE and\
      self.ur_robot_mode in (ur_dashboard_msgs.msg.RobotMode.RUNNING,):  return False
    t_start= rospy.Time.now()
    rate= rospy.Rate(20)
    #print 'WaitForRobotMode',mode,self.ur_robot_mode
    while self.ur_robot_mode != mode:
      rate.sleep()
      if (rospy.Time.now()-t_start).to_sec()>timeout:
        print 'WaitForRobotMode timeout.'
        return False
    #print '  done.',mode,self.ur_robot_mode
    return True

  #mode: ur_dashboard_msgs.msg.SafetyMode.{NORMAL,PROTECTIVE_STOP,ROBOT_EMERGENCY_STOP,FAULT}
  def WaitForSafetyMode(self, mode, timeout=20):
    t_start= rospy.Time.now()
    rate= rospy.Rate(20)
    while self.ur_safety_mode != mode:
      rate.sleep()
      if (rospy.Time.now()-t_start).to_sec()>timeout:
        print 'WaitForSafetyMode timeout.'
        return False
    return True

  #program_running: True or False
  def WaitForProgramRunning(self, program_running, timeout=20):
    t_start= rospy.Time.now()
    rate= rospy.Rate(20)
    while self.ur_program_running != program_running:
      rate.sleep()
      if (rospy.Time.now()-t_start).to_sec()>timeout:
        print 'WaitForProgramRunning timeout.'
        return False
    return True

  #ur_ros_running: True or False
  def WaitForURROSRunning(self, ur_ros_running, timeout=20):
    t_start= rospy.Time.now()
    rate= rospy.Rate(20)
    while self.ur_ros_running != ur_ros_running:
      rate.sleep()
      if (rospy.Time.now()-t_start).to_sec()>timeout:
        print 'WaitForURROSRunning timeout.'
        return False
    return True

  def SendFakeDigitalInDignal(self, signal_idx, signal_trg):
    msg= ur_msgs.msg.IOStates()
    msg.digital_in_states= [ur_msgs.msg.Digital(pin,False) for pin in range(18)]
    msg.digital_out_states= [ur_msgs.msg.Digital(pin,False) for pin in range(18)]
    msg.flag_states= [ur_msgs.msg.Digital(pin,False) for pin in range(2)]
    msg.analog_in_states= [ur_msgs.msg.Analog(pin,0,0) for pin in range(2)]
    msg.analog_out_states= [ur_msgs.msg.Analog(pin,0,0) for pin in range(2)]
    msg.digital_in_states[signal_idx]= ur_msgs.msg.Digital(signal_idx,signal_trg)
    self.pub_io_states.publish(msg)


  def ConnectToScriptNode(self, timeout=20):
    self.connected_to_script_node= False
    try:
      rospy.wait_for_service('/ros_node/wait_finish', timeout=timeout)
      rospy.wait_for_service('/ros_node/get_result_as_yaml', timeout=timeout)
      rospy.wait_for_service('/ros_node/get_attr_as_yaml', timeout=timeout)
      rospy.wait_for_service('/ros_node/set_attr_with_yaml', timeout=timeout)
      self.srvp_wait_finish= rospy.ServiceProxy('/ros_node/wait_finish', std_srvs.srv.Empty, persistent=False)
      self.srvp_get_result_as_yaml= rospy.ServiceProxy('/ros_node/get_result_as_yaml', ay_trick_msgs.srv.GetString, persistent=False)
      self.srvp_get_attr_as_yaml= rospy.ServiceProxy('/ros_node/get_attr_as_yaml', ay_trick_msgs.srv.GetAttrAsString, persistent=False)
      self.srvp_set_attr_with_yaml= rospy.ServiceProxy('/ros_node/set_attr_with_yaml', ay_trick_msgs.srv.SetAttrWithString, persistent=False)
      self.connected_to_script_node= True
    except rospy.ROSException as e:
      CPrint(4,'Error in ConnectToScriptNode:', e)
      self.srvp_wait_finish= None
      self.srvp_get_result_as_yaml= None
      self.srvp_get_attr_as_yaml= None
      self.srvp_set_attr_with_yaml= None
    self.pub_cmd= rospy.Publisher('/ros_node/command', std_msgs.msg.String, queue_size=10)
    self.pub_key= rospy.Publisher('/ros_node/stdin', std_msgs.msg.String, queue_size=10)
    self.sub_stdout= rospy.Subscriber('/ros_node/stdout', std_msgs.msg.String, self.StdOutCallback)
    self.script_node_status= None
    self.sub_script_node_status= rospy.Subscriber('/ros_node/node_status', ay_trick_msgs.msg.ROSNodeMode, self.ScriptNodeStatusCallback)

  def RunFGScript(self, cmd):
    if not hasattr(self,'connected_to_script_node'):
      CPrint(4,'Not connected to the script node.')
      return
    self.srvp_wait_finish()  #Wait for previously executed scripts.
    self.pub_cmd.publish(std_msgs.msg.String(cmd))
    print '###DEBUG/RunFGScript###',cmd
    self.srvp_wait_finish()

  def RunBGScript(self, cmd):
    if not hasattr(self,'connected_to_script_node'):
      CPrint(4,'Not connected to the script node.')
      return
    self.srvp_wait_finish()  #Wait for previously executed scripts.
    self.pub_cmd.publish(std_msgs.msg.String(cmd))
    print '###DEBUG/RunBGScript###',cmd

  def SendString(self, key):
    if not hasattr(self,'connected_to_script_node'):
      CPrint(4,'Not connected to the script node.')
      return
    self.pub_key.publish(std_msgs.msg.String(key))

  def WaitForBGScript(self):
    if not hasattr(self,'connected_to_script_node'):
      CPrint(4,'Not connected to the script node.')
      return
    self.srvp_wait_finish()

  def GetScriptResult(self):
    if not hasattr(self,'connected_to_script_node'):
      CPrint(4,'Not connected to the script node.')
      return
    res= self.srvp_get_result_as_yaml()
    if res.success:
      return yamlload(res.result, Loader=YLoader)
    else:
      return None

  def StdOutCallback(self, msg):
    #sys.stdout.write(msg.data)
    #sys.stdout.flush()
    pass

  def ScriptNodeStatusCallback(self, msg):
    self.script_node_status= msg.mode
    self.script_node_status_stamp= rospy.Time.now()

  #status: ay_trick_msgs.msg.ROSNodeMode.{BUSY,READY,PROGRAM_RUNNING}
  def WaitForScriptNodeStatus(self, status, timeout=10):
    if not hasattr(self,'script_node_status'):
      CPrint(4,'Script node status is not subscribed.')
      return False
    t_start= rospy.Time.now()
    rate= rospy.Rate(20)
    while self.script_node_status != status:
      rate.sleep()
      if (rospy.Time.now()-t_start).to_sec()>timeout:
        print 'WaitForScriptNodeStatus timeout.'
        return False
    return True

  def GetScriptNodeStatus(self):
    return self.script_node_status


if __name__=='__main__':
  #NOTE: Some constants are defined in ctrl_paramX.yaml
  #Parameters:
  config={
    'START_BTN': {
      'SIGNAL_IDX': 4,
      'SIGNAL_ON': True,
      },
    'STOP_BTN': {
      'SIGNAL_IDX': 5,
      'SIGNAL_ON': True,
      },
    }

  pm= TProcessManager()

  def UpdateStatusTextBox(w,obj,status):
    #obj= w.widgets['status_textbox']
    obj.document().setPlainText(
'''Status: {status}
SafetyMode: {safety_mode}
RobotMode: {robot_mode}
URProgram: {program_running}
MainProgram: {script_status}'''.format(
      status=pm.status_names[pm.status],
      safety_mode=pm.ur_safety_mode_names[pm.ur_safety_mode] if pm.ur_ros_running and pm.ur_safety_mode in pm.ur_safety_mode_names else 'UNRECOGNIZED',
      robot_mode=pm.ur_robot_mode_names[pm.ur_robot_mode] if pm.ur_ros_running and pm.ur_robot_mode in pm.ur_robot_mode_names else 'UNRECOGNIZED',
      program_running=pm.ur_ros_running and pm.ur_program_running,
      script_status=pm.script_node_status_names[pm.script_node_status] if pm.script_node_running and pm.script_node_status in pm.script_node_status_names else 'UNRECOGNIZED' ))

  widgets_common= {
    'status_signal_bar1': (
      'primitive_painer',{
        'color': (0,255,0),
        'onstatuschanged':lambda w,obj,status:(
                      obj.setPaintColor({'red':(255,0,0),'green':(0,255,0),'yellow':(255,255,0),None:(128,128,128)}[
                        pm.status_color[0] if len(pm.status_color)>0 else None]), ),
        'margin': (0,0),
        'minimum_size': (None,20),
        'maximum_size': (None,20),
        'size_policy': ('expanding', 'fixed')}),
    'status_signal_bar2': (
      'primitive_painer',{
        'color': (0,255,0),
        'onstatuschanged':lambda w,obj,status:(
                      obj.setPaintColor({'red':(255,0,0),'green':(0,255,0),'yellow':(255,255,0),None:(128,128,128)}[
                        pm.status_color[1] if len(pm.status_color)>1 else \
                          pm.status_color[0] if len(pm.status_color)>0 else None]), ),
        'margin': (0,0),
        'minimum_size': (None,20),
        'maximum_size': (None,20),
        'size_policy': ('expanding', 'fixed')}),
    'status_textbox': (
      'textedit',{
        'read_only': True,
        'font_size_range': (6,24),
        'text': 'Status Text Box',
        'onstatuschanged': UpdateStatusTextBox, }),
    'spacer_cmn1': ('spacer', {
        'w': 400,
        'h': 1,
        'size_policy': ('fixed', 'fixed')      }),
    }

  widgets_init= {
    'btn_ur_power': (
      'buttonchk',{
        'text':('Power on robot','Power off robot'),
        'onstatuschanged':lambda w,obj,status:(
                      obj.setEnabled(status in (pm.POWER_OFF,pm.TORQUE_ENABLED,pm.ROBOT_READY)),
                      obj.setChecked(status in (pm.TORQUE_ENABLED,pm.ROBOT_READY,pm.WAIT_REQUEST,pm.PROGRAM_RUNNING,pm.PROTECTIVE_STOP) ),
                      status==pm.ROBOT_EMERGENCY_STOP and (
                        #stop_cmd('ur_gripper'),
                        pm.RunURDashboard('stop'),
                        pm.WaitForProgramRunning(False),
                        pm.RunURDashboard('power_off'),
                        ),
                      ),
        'onclick':(lambda w,obj:(
                      #run_cmd('ur_gripper'),
                      pm.WaitForRobotMode(ur_dashboard_msgs.msg.RobotMode.POWER_OFF),
                      pm.RunURDashboard('power_on'),
                      pm.WaitForRobotMode(ur_dashboard_msgs.msg.RobotMode.IDLE),
                      pm.RunURDashboard('brake_release'),
                      pm.WaitForRobotMode(ur_dashboard_msgs.msg.RobotMode.RUNNING),
                      rospy.sleep(0.2),
                      pm.RunURDashboard('play'),
                      pm.WaitForProgramRunning(True),
                     ),
                   lambda w,obj:(
                      #stop_cmd('ur_gripper'),
                      pm.RunURDashboard('stop'),
                      pm.WaitForProgramRunning(False),
                      pm.RunURDashboard('power_off'),
                     ) )}),
    'btn_reset_estop': (
      'button',{
        'text': 'Reset E-Stop',
        'onstatuschanged':lambda w,obj,status:(
                      obj.setEnabled(status in (pm.FAULT,)) ),
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj: (
                      pm.RunURDashboard('close_safety_popup'),
                      pm.RunURDashboard('restart_safety'),
                      ) if AskYesNoDialog(w,'1. Please make sure that everything is safe.\n2. Release the E-Stop.\n3. Press Yes.',title='Reset E-Stop') else None }),
    'btn_shutdown_ur': (
      'button',{
        'text': 'Shutdown robot',
        'onstatuschanged':lambda w,obj,status:(
                      obj.setEnabled(status in (pm.ROBOT_EMERGENCY_STOP,pm.POWER_OFF)) ),
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj: pm.RunURDashboard('shutdown'), }),
    }

  widgets_operation= {
    'btn_start': (
      'button',{
        'text': 'START',
        #'onstatuschanged':lambda w,obj,status:(obj.setEnabled(status in (pm.PROGRAM_RUNNING,)) ),
        'onclick': lambda w,obj: pm.SendFakeDigitalInDignal(config['START_BTN']['SIGNAL_IDX'], config['START_BTN']['SIGNAL_ON']), }),
    'btn_stop': (
      'button',{
        'text': 'STOP',
        #'onstatuschanged':lambda w,obj,status:(obj.setEnabled(status in (pm.PROGRAM_RUNNING,)) ),
        'onclick': lambda w,obj: pm.SendFakeDigitalInDignal(config['STOP_BTN']['SIGNAL_IDX'], config['STOP_BTN']['SIGNAL_ON']), }),
    }

  widgets_recovery= {
    'chkbx_safety_to_recover': (
      'checkbox',{
        'text': 'Please check the safety of\nthe robot and environment',
        'onstatuschanged':lambda w,obj,status:(obj.setEnabled(status in (pm.PROTECTIVE_STOP,)) ),
        'onclick': lambda w,obj: w.widgets['btn_revoery'].setEnabled(obj.isChecked()) }),
    'btn_revoery': (
      'button',{
        'text':'Recovery motion',
        'enabled':False,
        'onclick':lambda w,obj:(
                      pm.RunURDashboard('stop'),
                      pm.WaitForProgramRunning(False),
                      pm.RunURDashboard('unlock_protective_stop'),
                      pm.WaitForSafetyMode(ur_dashboard_msgs.msg.SafetyMode.NORMAL),
                      rospy.sleep(0.2),
                      pm.RunURDashboard('play'),
                      pm.WaitForProgramRunning(True),
                      #run_script('moveto_wait_r'),
                      w.widgets['chkbx_safety_to_recover'].setChecked(False),
                      obj.setEnabled(False),
                     ) if w.widgets['chkbx_safety_to_recover'].isChecked() else None }),
    }

  widgets_debug= {
    'btn_ur_power_on': (
      'button',{
        'text': 'PowerOn',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('power_on'), }),
    'btn_ur_brake_release': (
      'button',{
        'text': 'BrakeRelease',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('brake_release'), }),
    'btn_ur_power_off': (
      'button',{
        'text': 'PowerOff',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('power_off'), }),
    'btn_ur_play': (
      'button',{
        'text': 'Play',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('play'), }),
    'btn_ur_stop': (
      'button',{
        'text': 'Stop',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('stop'), }),
    'btn_ur_shutdown': (
      'button',{
        'text': 'Shutdown',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('shutdown'), }),
    'btn_ur_unlock_protective_stop': (
      'button',{
        'text': 'UnlockProtectiveStop',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('unlock_protective_stop'), }),
    'btn_ur_restart_safety': (
      'button',{
        'text': 'RestartSafety',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('restart_safety'), }),
    'btn_ur_close_safety_popup': (
      'button',{
        'text': 'CloseSafetyPopup',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('close_safety_popup'), }),
    'btn_dbg_exit': (
      'button',{
        'text': 'Exit',
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj: w.close(), }),
    }

  layout_main= (
    'boxv',None,(
      'status_signal_bar1',
      ('boxh',None, (
        ('boxv',None, ('status_textbox','btn_start','btn_stop')),
        ('boxv',None, (
          ('boxh',None, ('btn_ur_power','btn_shutdown_ur')),
          ('boxv',None, (
                        ('boxh',None, (
                            'btn_ur_power_on',
                            'btn_ur_brake_release',
                            'btn_ur_power_off',
                            'btn_ur_shutdown',)),
                        ('boxh',None, (
                            'btn_ur_play',
                            'btn_ur_stop',
                            'btn_ur_unlock_protective_stop',)),
                        ('boxh',None, (
                            'btn_ur_restart_safety',
                            'btn_ur_close_safety_popup')),
                        )),
          ('boxh',None, ('chkbx_safety_to_recover','btn_revoery','btn_reset_estop')),
          'btn_dbg_exit',
          'spacer_cmn1'
          )),
        )),
      'status_signal_bar2',
      ))

  app= InitPanelApp()
  win_size= (800,400)
  panel= TSimplePanel('UR Dashboard', size=win_size, font_height_scale=300.0)
  panel.AddWidgets(widgets_common)
  panel.AddWidgets(widgets_init)
  panel.AddWidgets(widgets_operation)
  panel.AddWidgets(widgets_recovery)
  panel.AddWidgets(widgets_debug)
  panel.Construct(layout_main)

  #Since the onstatuschanged signal is emitted from TProcessManager,
  #we connect the onstatuschanged slots of panel to it.
  for w_name, (w_type, w_param) in panel.widgets_in.iteritems():
    if 'onstatuschanged' in w_param and w_param['onstatuschanged'] is not None:
      pm.onstatuschanged.connect(lambda status,w_param=w_param,w_name=w_name: w_param['onstatuschanged'](panel,panel.widgets[w_name],status))

  pm.InitNode()
  pm.StartUpdateStatusThread()
  pm.ConnectToURDashboard()
  if not pm.WaitForURROSRunning(True):  #Should be done after ConnectToURDashboard
    CPrint(4, 'UR ROS driver is not running!')
  else:
    CPrint(2, 'The process manager is ready.')
  panel.close_callback= lambda event: (
      pm.TerminateAllBGProcesses(),
      pm.StopUpdateStatusThread(),
      pm.DisconnectUR(),
      True)[-1]

  RunPanelApp()
