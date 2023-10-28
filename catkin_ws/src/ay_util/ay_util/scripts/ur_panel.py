#!/usr/bin/python
#\file    ur_panel.py
#\brief   UR + FV demo panel.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    May.30, 2022
import roslib
roslib.load_manifest('ay_trick_msgs')
import os,sys
import rospy
import rospkg
import ay_trick_msgs.msg
import ay_trick_msgs.srv
from ur_dashboard_gui import *
from ay_py.core import InsertDict, LoadYAML, SaveYAML

class TProcessManagerJoy(TProcessManager):
  def __init__(self):
    super(TProcessManagerJoy,self).__init__(node_name='ur_panel')

  def StartVirtualJoyStick(self):
    self.pub_joy= rospy.Publisher('/joy', sensor_msgs.msg.Joy, queue_size=10)
    self.joy_state= sensor_msgs.msg.Joy()
    self.joy_state.axes= [0.0]*6
    self.joy_state.buttons= [0]*16

  '''
  joy_state.axes[0]  #Y, -WX(roll), Gripper step
  joy_state.axes[1]  #X, WY(pitch)
  joy_state.axes[3]  #WZ(yaw)
  joy_state.axes[4]  #Z
  joy_state.axes[5]  #RT: Negative multiplier [1(base),-1]
  joy_state.axes[2]  #LT: Positive multiplier [1(base),-1]
  #multiplier= (1.0+joy_state.axes[5])*0.5 + (1.0-joy_state.axes[2])*2.0
  joy_state.buttons[7]  #START button (quit)
  joy_state.buttons[0]  #A (gripper mode)
  joy_state.buttons[1]  #B (fv.tracko)
  joy_state.buttons[2]  #X (fv.trackf2)
  joy_state.buttons[3]  #Y (fv.pickup2b)
  joy_state.buttons[4]  #LB (0:position, 1:orientation)
  joy_state.buttons[5]  #RB (0:inactive, 1:active)
  joy_state.buttons[11]  #d-pad, LEFT (fv.open)
  joy_state.buttons[12]  #d-pad, RIGHT (fv.grasp)
  joy_state.buttons[13]  #d-pad, UP (fv.hold)
  joy_state.buttons[14]  #d-pad, DOWN (fv.openif)
  '''
  def SetJoy(self, kind, value=None, is_active=0):
    if not hasattr(self,'pub_joy'):
      CPrint(4,'Joystick publisher is not ready.')
      return
    joy_state= self.joy_state
    #reset:
    joy_state.axes[:]= [0.0]*6
    joy_state.axes[5]= -0.5  #RT: Negative multiplier [1(base),-1]
    joy_state.axes[2]= 1.0  #LT: Positive multiplier [1(base),-1]
    joy_state.buttons[:]= [0]*16
    if kind=='reset':
      pass
    elif kind=='xy':
      joy_state.axes[0]= -1.5*value[0]  #Y, -WX(roll), Gripper step
      joy_state.axes[1]= value[1]  #X, WY(pitch)
      joy_state.buttons[5]= is_active  #RB (0:inactive, 1:active)
    elif kind=='z':
      joy_state.axes[4]= value[0]  #Z
      joy_state.buttons[5]= 1  #RB (0:inactive, 1:active)
    elif kind=='pitch':
      joy_state.axes[1]= value[0]  #X, WY(pitch)
      joy_state.buttons[4]= 1  #LB (0:position, 1:orientation)
      joy_state.buttons[5]= is_active  #RB (0:inactive, 1:active)
    elif kind=='yaw':
      joy_state.axes[3]= -value[0]  #WZ(yaw)
      joy_state.buttons[5]= is_active  #RB (0:inactive, 1:active)
    elif kind=='grip':
      joy_state.axes[0]= -4.0*value[0]  #Y, -WX(roll), Gripper step
      joy_state.buttons[0]= 1  #A (gripper mode)
      joy_state.buttons[5]= is_active  #RB (0:inactive, 1:active)
    elif kind=='quit':
      joy_state.buttons[7]= 1  #START button (quit)
    elif kind=='trackf_on':
      joy_state.buttons[2]= 1  #X (fv.trackf2)
      joy_state.buttons[5]= is_active  #RB (0:inactive, 1:active)
    elif kind=='trackf_off':
      joy_state.buttons[2]= 1  #X (fv.trackf2)
      joy_state.buttons[5]= 0  #RB (0:inactive, 1:active)
    elif kind=='pickup_on':
      joy_state.buttons[3]= 1  #Y (fv.pickup2b)
      joy_state.buttons[5]= is_active  #RB (0:inactive, 1:active)
    elif kind=='pickup_off':
      joy_state.buttons[3]= 1  #Y (fv.pickup2b)
      joy_state.buttons[5]= 0  #RB (0:inactive, 1:active)
    elif kind=='hold_on':
      joy_state.buttons[13]= 1  #d-pad, UP (fv.hold)
      joy_state.buttons[5]= is_active  #RB (0:inactive, 1:active)
    elif kind=='hold_off':
      joy_state.buttons[13]= 1  #d-pad, UP (fv.hold)
      joy_state.buttons[5]= 0  #RB (0:inactive, 1:active)
    elif kind=='open':
      joy_state.buttons[11]=1  #d-pad, LEFT (fv.open)
    self.pub_joy.publish(joy_state)

def UpdateProcList(pm,combobox):
  combobox.clear()
  for name,proc in pm.procs.iteritems():
    combobox.addItem('{0}/{1}'.format(name,proc.pid))

if __name__=='__main__':
  def get_arg(opt_name, default):
    exists= map(lambda a:a.startswith(opt_name),sys.argv)
    if any(exists):  return sys.argv[exists.index(True)].replace(opt_name,'')
    else:  return default
  robot_code= get_arg('-robot_code=',get_arg('--robot_code=','UR3e125hzDxlpY1'))
  sim_robot_code= get_arg('-sim_robot_code=',get_arg('--sim_robot_code=','UR3eDxlpY1_SIM'))
  joy_dev= get_arg('-joy_dev=',get_arg('--joy_dev=','js0'))
  dxl_dev= get_arg('-dxl_dev=',get_arg('--dxl_dev=','USB1'))
  fullscreen= True if '-fullscreen' in sys.argv or '--fullscreen' in sys.argv else False
  is_sim= True if '-sim' in sys.argv or '--sim' in sys.argv else False

  RVIZ_CONFIG= os.environ['HOME']+'/.rviz/default.rviz'
  #Parameters:
  config={
    'URType': robot_code,
    'URType_SIM': sim_robot_code,
    'JoyUSB': joy_dev,
    'DxlUSB': dxl_dev,
    'FV_L_DEV': '/media/video_fv1',
    'FV_R_DEV': '/media/video_fv2',
    'FV_CTRL_CONFIG': '{}/data/config/fv_ctrl.yaml'.format(os.environ['HOME']),
    #'ShutdownRobotAfterUse': False,
    #'Q_INIT': [0.03572946786880493, -2.027292076741354, 1.6515636444091797, -1.1894968191729944, -1.5706136862384241, -3.1061676184283655],
    'Q_INIT': [0.03572946786880493, -2.027292076741354, 1.6515636444091797, -1.1894968191729944, -1.5706136862384241, 0.0],
    #'Q_PARK': [0.03572946786880493, -2.027292076741354, 1.6515636444091797, -1.1894968191729944, -1.5706136862384241, -3.1061676184283655],
    }
  print config

  #List of commands (name: [[command/args],'fg'/'bg']).
  cmds= {
    'roscore': ['roscore','bg'],
    'fix_usb': ['sudo /sbin/fix_usb_latency.sh tty{DxlUSB}','fg'],
    'ur_ros': ['roslaunch ay_util ur_selector.launch robot_code:={URType} jsdev:=/dev/input/{JoyUSB} dxldev:=/dev/tty{DxlUSB} with_gripper:=false','bg'],
    'ur_gripper': ['roslaunch ay_util ur_gripper_selector.launch robot_code:={URType} dxldev:=/dev/tty{DxlUSB}','bg'],
    'ur_calib': ['roslaunch ay_util ur_calib.launch robot_code:={URType}','fg'],
    'fvp_3': ['roslaunch ay_fv_extra fvp_3.launch','bg'],
    'config_fv_l': ['rosrun fingervision conf_cam2.py {FV_L_DEV} "file:CameraParams:0:`rospack find ay_fv_extra`/config/fvp_3_l.yaml"','fg'],
    'config_fv_r': ['rosrun fingervision conf_cam2.py {FV_R_DEV} "file:CameraParams:0:`rospack find ay_fv_extra`/config/fvp_3_r.yaml"','fg'],
    'realsense': ['roslaunch realsense2_camera rs_camera.launch align_depth:=true enable_pointcloud:=true depth_fps:=15 color_fps:=30 depth_width:=640 depth_height:=480 color_width:=640 color_height:=480','bg'],
    'ay_trick_ros': ['rosrun ay_trick ros_node.py','bg'],
    'rviz': ['rosrun rviz rviz -d {0}'.format(RVIZ_CONFIG),'bg'],
    }
  if is_sim:
    config['URType']= config['URType_SIM']
    for c in ('fix_usb','ur_calib','fvp_3','config_fv_l','config_fv_r','realsense'):
      cmds[c][1]= None
  for key in cmds.iterkeys():
    if isinstance(cmds[key][0],str):
      cmds[key][0]= cmds[key][0].format(**config).split(' ')

  pm= TProcessManagerJoy()
  run_cmd= lambda name: pm.RunBGProcess(name,cmds[name][0]) if cmds[name][1]=='bg' else\
                        pm.RunFGProcess(cmds[name][0]) if cmds[name][1]=='fg' else\
                        None
  stop_cmd= lambda name: pm.TerminateBGProcess(name)

  #List of script commands (name: [[script/args],'fg'/'bg']).
  scripts= {
    'setup': ['ur.setup "{URType}",True','fg'],
    'joy': ['j','bg'],
    #'stop_joy': ['q','fg'],
    'move_to_init': ['ct.robot.MoveToQ({Q_INIT},dt=5.0,blocking=True)','fg'],
    #'move_to_park': ['ct.robot.MoveToQ({Q_PARK},dt=5.0,blocking=True)','fg'],
    'grip_plus':  ['fv.open ct.robot.Arm, ct.robot.GripperPos()+0.015, True','fg'],
    }
  for key in scripts.iterkeys():
    if isinstance(scripts[key],list) and isinstance(scripts[key][0],str):
      scripts[key][0]= scripts[key][0].format(**config)

  run_script= lambda name: None if scripts[name] is None else\
                           pm.RunBGScript(scripts[name][0]) if scripts[name][1]=='bg' else\
                           pm.RunFGScript(scripts[name][0]) if scripts[name][1]=='fg' else\
                           None

  set_joy= lambda kind,value=None,is_active=0: pm.SetJoy(kind,value,is_active)
  #stop_joy= lambda: run_script('stop_joy')
  pm.joy_script_active= False
  start_joy= lambda: (setattr(pm,'joy_script_active',True), run_script('joy'))
  stop_joy= lambda: (setattr(pm,'joy_script_active',False), pm.SendString('q')) if pm.joy_script_active and pm.GetScriptNodeStatus()==ay_trick_msgs.msg.ROSNodeMode.PROGRAM_RUNNING else setattr(pm,'joy_script_active',False)
  def run_script_during_joy(name):
    if pm.joy_script_active:
      stop_joy()
      run_script(name)
      start_joy()
    else:
      run_script(name)

  #One-time commands (utility):
  #run_cmd('ur_calib')

  #Emergency commands:
  #pm.RunURDashboard('stop')
  #pm.RunURDashboard('unlock_protective_stop')

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


  #UI for configuring FV control parameters:
  ctrl_config= {
      #Common control parameters:
      'min_gstep': 0.0005,
      #Parameters used in fv.hold, fv.pickup2a, fv.pickup2b:
      'hold_sensitivity_slip': 0.08,  #Sensitivity of slip detection (smaller is more sensitive).
      'hold_sensitivity_oc': 0.2,  #Sensitivity of object-center-movement detection (smaller is more sensitive).
      'hold_sensitivity_oo': 0.5,  #Sensitivity of object-orientation-movement detection (smaller is more sensitive).
      'hold_sensitivity_oa': 0.4,  #Sensitivity of object-area-change detection (smaller is more sensitive).
      'pickup2a_area_drop_ratio': 0.3,  #If object area becomes smaller than this ratio, it's considered as dropped.
      'pickup2a_z_final': 0.05,  #Final height (offset from the beginning).
      'pickup2a_obj_area_filter_len': 5,  #Filter length for obj_area.
      #Parameters used in fv.openif:
      'openif_sensitivity_slip': 0.6,  #Sensitivity of slip detection (smaller is more sensitive).
      'openif_sensitivity_oc': 0.4,  #Sensitivity of object-center-movement detection (smaller is more sensitive).
      'openif_sensitivity_oo': 4.0,  #Sensitivity of object-orientation-movement detection (smaller is more sensitive).
      'openif_sensitivity_oa': 0.6,  #Sensitivity of object-area-change detection (smaller is more sensitive).
      'openif_sensitivity_force':0.9,  #Sensitivity of each force element; if the norm of force change is larger than this threshold, the point is counted as a force change point.
      'openif_nforce_threshold': 20,  #Threshold of number of force changing points to open the gripper.
      'openif_dw_grip': 0.02,  #Displacement of gripper movement.
    }
  if os.path.exists(config['FV_CTRL_CONFIG']):
    InsertDict(ctrl_config, LoadYAML(config['FV_CTRL_CONFIG']))
  def UpdateCtrlConfig(name, value):
    ctrl_config[name]= value
    SaveYAML(ctrl_config, config['FV_CTRL_CONFIG'], interactive=False)
  def AddCtrlConfigSliderWidget(widgets, name, prange):
    widgets['slider_ctrl_config_{}'.format(name)]= (
      'sliderh',{
        'range': prange,
        'value': ctrl_config[name],
        'n_labels': 3,
        'slider_style':1,
        'font_size_range': (5,6),
        'onvaluechange': lambda w,obj:UpdateCtrlConfig(name,obj.value())} )
    widgets['label_ctrl_config_{}'.format(name)]= (
      'label',{
        'text': name,
        'font_size_range': (5,6),
        'size_policy': ('minimum', 'minimum')} )
  def CtrlConfigSliderLayout(name):
    return ('label_ctrl_config_{}'.format(name),'slider_ctrl_config_{}'.format(name))


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
    #'rviz': (  #NOTE: To be replaced by RViz widget.
      #'primitive_painer',{
        #'shape': 'square',
        #'color': (0,120,240),
        #'margin': (0.1,0.1),
        #'size_policy': ('expanding', 'expanding')}),
    'rviz': (
      'rviz',{
        'config': RVIZ_CONFIG,
        'size_policy': ('expanding', 'expanding')}),
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
    'btn_init1': (
      'buttonchk',{
        'text':('(1)Robot core program','[3]Stop core program'),
        'onstatuschanged':lambda w,obj,status:(
                      obj.setEnabled(status in (pm.NO_CORE_PROGRAM,pm.POWER_OFF)),
                      obj.setChecked(status not in (pm.NO_CORE_PROGRAM,) ),
                      ),
        'onclick':(lambda w,obj:(
                      #run_cmd('roscore'),
                      #rospy.wait_for_service('/rosout/get_loggers', timeout=5.0),
                      run_cmd('fix_usb'),
                      run_cmd('ur_ros'),
                      #pm.InitNode(),//
                      #pm.StartUpdateStatusThread(),
                      pm.ConnectToURDashboard(),
                      pm.WaitForURROSRunning(True),  #Should be done after ConnectToURDashboard
                      w.widgets['rviz'].setup(),
                      pm.WaitForRobotMode(ur_dashboard_msgs.msg.RobotMode.POWER_OFF),
                     ),
                   lambda w,obj:(
                      #pm.RunURDashboard('shutdown') if config['ShutdownRobotAfterUse'] else None,
                      pm.DisconnectUR(),  #NOTE: This should be done after shutdown.
                      #pm.StopUpdateStatusThread(),
                      stop_cmd('ur_ros'),
                      #stop_cmd('roscore'),
                     ) )}),
    'btn_init2': (
      'buttonchk',{
        'text':('(2)Power on robot','[2]Power off robot'),
        'enabled': not is_sim,
        'onstatuschanged':lambda w,obj,status:(
                      obj.setEnabled(status in (pm.POWER_OFF,pm.TORQUE_ENABLED,pm.ROBOT_READY)),
                      obj.setChecked(status in (pm.TORQUE_ENABLED,pm.ROBOT_READY,pm.WAIT_REQUEST,pm.PROGRAM_RUNNING,pm.PROTECTIVE_STOP) ),
                      status==pm.ROBOT_EMERGENCY_STOP and (
                        stop_cmd('ur_gripper'),
                        pm.RunURDashboard('stop'),
                        pm.WaitForProgramRunning(False),
                        pm.RunURDashboard('power_off'),
                        ),
                      ),
        'onclick':(lambda w,obj:(
                      run_cmd('ur_gripper'),
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
                      stop_cmd('ur_gripper'),
                      pm.RunURDashboard('stop'),
                      pm.WaitForProgramRunning(False),
                      pm.RunURDashboard('power_off'),
                     ) )}),
    'btn_init3': (
      'buttonchk',{
        'text':('(3)Start program','[1]Stop program'),
        'onstatuschanged':lambda w,obj,status:(
                      obj.setEnabled(status in (pm.ROBOT_READY,pm.WAIT_REQUEST,pm.PROGRAM_RUNNING,pm.PROTECTIVE_STOP)),
                      obj.setChecked(status in (pm.WAIT_REQUEST,pm.PROGRAM_RUNNING,pm.PROTECTIVE_STOP) ),
                      status==pm.ROBOT_EMERGENCY_STOP and (
                        #stop_cmd('rviz'),
                        stop_cmd('ay_trick_ros'),
                        stop_cmd('fvp_3'),
                        ),
                      ),
        'onclick':(lambda w,obj:(
                      run_cmd('fvp_3'),
                      rospy.sleep(0.2),
                      run_cmd('config_fv_l'),
                      run_cmd('config_fv_r'),
                      run_cmd('ay_trick_ros'),
                      #run_cmd('rviz'),
                      pm.ConnectToScriptNode(),
                      pm.WaitForScriptNodeStatus(ay_trick_msgs.msg.ROSNodeMode.READY),
                      rospy.sleep(0.2),
                      run_script('setup'),
                     ),
                   lambda w,obj:(
                      #stop_cmd('rviz'),
                      stop_cmd('ay_trick_ros'),
                      stop_cmd('fvp_3'),
                     ) )}),
    'btn_reset_estop': (
      'button',{
        'text': 'Reset E-Stop',
        'enabled': not is_sim,
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
        'enabled': not is_sim,
        'onstatuschanged':lambda w,obj,status:(
                      obj.setEnabled(status in (pm.ROBOT_EMERGENCY_STOP,pm.POWER_OFF)) ),
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj: pm.RunURDashboard('shutdown'), }),
    'btn_exit': (
      'button',{
        'text': 'Exit',
        'onstatuschanged':lambda w,obj,status:(
                      obj.setEnabled(status in (pm.NO_CORE_PROGRAM,pm.ROBOT_EMERGENCY_STOP)) ),
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj: w.close(), }),
    #'btn_shutdown_pc': (
      #'button',{
        #'text': 'Shutdown PC',
        #'onstatuschanged':lambda w,obj,status:(
                      #obj.setEnabled(status in (pm.NO_CORE_PROGRAM,pm.ROBOT_EMERGENCY_STOP)) ),
        #'size_policy': ('expanding', 'fixed'),
        #'onclick': lambda w,obj: run_cmd('shutdown_pc'), }),
    }
  layout_init= (
    'grid',None,(
      ('btn_init1',0,0), ('btn_init2',0,1),
      ('btn_init3',1,0), (('boxv',None,('btn_reset_estop','btn_shutdown_ur','btn_exit')),1,1),
      ))

  widgets_joy= {
    'btn_activate': (
      'buttonchk',{
        'text':('Activate','Deactivate'),
        'size_policy': ('expanding', 'fixed'),
        #'font_size_range': (8,24),
        'onstatuschanged':lambda w,obj,status:(
                      obj.setChecked(False) if status not in (pm.WAIT_REQUEST,pm.PROGRAM_RUNNING) else None,
                      stop_joy() if status not in (pm.WAIT_REQUEST,pm.PROGRAM_RUNNING) else None ),
        'onclick':(lambda w,obj:(
                      start_joy(),
                     ),
                   lambda w,obj:(
                      stop_joy(),
                     ) )}),
    'btn_init_pose': (
      'button',{
        'text': 'Init Pose',
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj:run_script_during_joy('move_to_init'), }),
    'btn_push': (
      'buttonchk',{
        'text':('Push','Stop'),
        'enabled': not is_sim,
        'size_policy': ('expanding', 'fixed'),
        'onclick':(lambda w,obj:set_joy('trackf_on',is_active=w.widgets['btn_activate'].isChecked()),
                   lambda w,obj:set_joy('trackf_off') )}),
    'btn_hold': (
      'buttonchk',{
        'text':('Hold','Stop'),
        'enabled': not is_sim,
        'size_policy': ('expanding', 'fixed'),
        'onclick':(lambda w,obj:set_joy('hold_on',is_active=w.widgets['btn_activate'].isChecked()),
                   lambda w,obj:set_joy('hold_off') )}),
    'btn_pick': (
      'buttonchk',{
        'text':('Pick','Stop'),
        'enabled': not is_sim,
        'size_policy': ('expanding', 'fixed'),
        'onclick':(lambda w,obj:set_joy('pickup_on',is_active=w.widgets['btn_activate'].isChecked()),
                   lambda w,obj:set_joy('pickup_off') )}),
    'label_pitch': (
      'label',{
        'text': 'Pitch',
        'size_policy': ('expanding', 'minimum')}),
    'joy_pitch': (
      'virtual_joystick',{
        'kind':'vbox',
        'stick_color':[128,255,128],
        'size_policy': ('minimum', 'expanding'),
        'onstickmoved': lambda w,obj:set_joy('pitch',obj.position(),is_active=w.widgets['btn_activate'].isChecked()), }),
    'label_xy': (
      'label',{
        'text': 'XY',
        'size_policy': ('expanding', 'minimum')}),
    'joy_xy': (
      'virtual_joystick',{
        'kind':'circle',
        'stick_color':[128,128,255],
        'onstickmoved': lambda w,obj:set_joy('xy',obj.position(),is_active=w.widgets['btn_activate'].isChecked()), }),
    'label_z': (
      'label',{
        'text': '  Z  ',
        'size_policy': ('expanding', 'minimum')}),
    'joy_z': (
      'virtual_joystick',{
        'kind':'vbox',
        'stick_color':[128,128,255],
        'size_policy': ('minimum', 'expanding'),
        'onstickmoved': lambda w,obj:set_joy('z',obj.position(),is_active=w.widgets['btn_activate'].isChecked()), }),
    'label_yaw': (
      'label',{
        'text': 'Yaw',
        'size_policy': ('minimum', 'minimum')}),
    'joy_yaw': (
      'virtual_joystick',{
        'kind':'hbox',
        'stick_color':[255,128,128],
        'size_policy': ('expanding','minimum'),
        'onstickmoved': lambda w,obj:set_joy('yaw',obj.position(),is_active=w.widgets['btn_activate'].isChecked()), }),
    'label_grip': (
      'label',{
        'text': 'Gripper',
        'size_policy': ('minimum', 'minimum')}),
    'joy_grip': (
      'virtual_joystick',{
        'kind':'hbox',
        'stick_color':[255,128,128],
        'size_policy': ('expanding','minimum'),
        'onstickmoved': lambda w,obj:set_joy('grip',obj.position(),is_active=w.widgets['btn_activate'].isChecked()), }),
    'btn_grip_open': (
      'button',{
        'text': '[ + ]',
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj:(
                      #run_script_during_joy('grip_plus'),
                      set_joy('open'),
                      ), }),
    }

  layout_joy= (
    'boxv',None, (
      ('grid',None,(
        ('btn_activate',0,0,1,2),
        ('label_pitch',1,0,'center'), ('label_xy',1,1,'center'), ('label_z',1,2,'center'),
        ('joy_pitch',2,0), ('joy_xy',2,1), ('joy_z',2,2),
        (('boxh',None,('label_yaw','joy_yaw')),3,0,1,3),
        (('boxh',None,('label_grip','joy_grip','btn_grip_open')),4,0,1,3),
        )),
      ('boxh',None, ('btn_init_pose',)),
      ('boxh',None, ('btn_push','btn_hold','btn_pick')),
      ))

  widgets_ctrl_config= {
    }
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'min_gstep', (0.0,0.01,0.0001))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'hold_sensitivity_slip', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'hold_sensitivity_oc', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'hold_sensitivity_oo', (0.0,4.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'hold_sensitivity_oa', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'pickup2a_area_drop_ratio', (0.0,1.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'pickup2a_z_final', (0.0,0.15,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'pickup2a_obj_area_filter_len', (1,20,1))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_sensitivity_slip', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_sensitivity_oc', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_sensitivity_oo', (0.0,4.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_sensitivity_oa', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_sensitivity_force', (0.0,4.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_nforce_threshold', (1,100,1))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_dw_grip', (0.0,0.1,0.001))

  layout_ctrl_config= (
    'boxv',None, (
        ('boxh',None, CtrlConfigSliderLayout('min_gstep') ),
        ('boxh',None, CtrlConfigSliderLayout('hold_sensitivity_slip') ),
        ('boxh',None, CtrlConfigSliderLayout('hold_sensitivity_oc') ),
        ('boxh',None, CtrlConfigSliderLayout('hold_sensitivity_oo') ),
        ('boxh',None, CtrlConfigSliderLayout('hold_sensitivity_oa') ),
        ('boxh',None, CtrlConfigSliderLayout('pickup2a_area_drop_ratio') ),
        ('boxh',None, CtrlConfigSliderLayout('pickup2a_z_final') ),
        ('boxh',None, CtrlConfigSliderLayout('pickup2a_obj_area_filter_len') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_sensitivity_slip') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_sensitivity_oc') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_sensitivity_oo') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_sensitivity_oa') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_sensitivity_force') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_nforce_threshold') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_dw_grip') ),
      ))

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
    #'btn_start_r': ('duplicate', 'btn_start'),
    #'btn_stop_r': ('duplicate', 'btn_stop'),
    }
  layout_recovery= (
    'boxv',None,(
      'chkbx_safety_to_recover',
      'btn_revoery',
      ))

  widgets_debug= {
    'btn_rviz': (
      'buttonchk',{
        'text':('RViz','Stop RViz'),
        'font_size_range': (8,24),
        'onclick':(lambda w,obj:(
                      run_cmd('rviz'),
                     ),
                   lambda w,obj:(
                      stop_cmd('rviz'),
                     ) )}),
    'label_ur': (
      'label',{
        'text': 'UR: ',
        'font_size_range': (8,24),
        'size_policy': ('minimum', 'minimum')}),
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
    'label_processes': (
      'label',{
        'text': 'Processes: ',
        'font_size_range': (8,24),
        'size_policy': ('minimum', 'minimum')}),
    'combobox_procs': (
      'combobox',{
        'options':('(Process_name/PID)',),
        'font_size_range': (8,24),
        'size_adjust_policy': 'all_contents',
        'onactivated': lambda w,obj:None}),
    'btn_update_proc_list': (
      'button',{
        'text': 'Update',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: (UpdateProcList(pm,w.widgets['combobox_procs']),
                                  #w.widgets['combobox_procs'].resize(w.widgets['combobox_procs'].sizeHint())
                                  ) }),
    'btn_terminate_proc': (
      'button',{
        'text': 'Terminate',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.TerminateBGProcess(str(w.widgets['combobox_procs'].currentText()).split('/')[0]), }),
    'btn_kill_proc': (
      'button',{
        'text': 'Kill',
        'font_size_range': (8,24),
        'size_policy': ('minimum', 'fixed'),
        'onclick': lambda w,obj: pm.KillBGProcess(str(w.widgets['combobox_procs'].currentText()).split('/')[0]), }),
    'btn_dbg_exit': (
      'button',{
        'text': 'Exit',
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj: w.close(), }),
    }
  layout_debug= (
    'boxv',None,(
      ('boxv',None, ('btn_rviz',)),
      ('boxh',None, ('label_ur', ('boxv',None, (
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
                        ))
                     )),
      ('boxh',None, ('label_processes', ('boxv',None, (
                        'combobox_procs',
                        ('boxh',None, ('btn_update_proc_list','btn_terminate_proc','btn_kill_proc')),
                        ('boxh',None, ('btn_dbg_exit',)),
                        ))
                     )),
      ))

  layout_main= (
    'boxv',None,(
      'status_signal_bar1',
      ('boxh',None, (
        ('boxv',None, ('rviz','status_textbox')),
        ('boxv',None, (
        ('tab','maintab',(
          ('Initialize',layout_init),
          ('Joy',layout_joy),
          ('Control Config',layout_ctrl_config),
          ('Recovery',layout_recovery),
          ('Debug',layout_debug),
          )),'spacer_cmn1')),
        )),
      'status_signal_bar2',
      ))

  app= InitPanelApp()
  win_size= (1000,800)
  if fullscreen:  #NOTE: fullscreen mode will work only with Qt5.
    print 'Screen size:', app.screens()[0].size()
    screen_size= app.screens()[0].size()
    win_size= (screen_size.width(),screen_size.height())
  panel= TSimplePanel('Robot Operation Panel', size=win_size, font_height_scale=300.0)
  panel.AddWidgets(widgets_common)
  panel.AddWidgets(widgets_init)
  panel.AddWidgets(widgets_joy)
  panel.AddWidgets(widgets_ctrl_config)
  panel.AddWidgets(widgets_recovery)
  panel.AddWidgets(widgets_debug)
  panel.Construct(layout_main)
  #for tab in panel.layouts['maintab'].tab:
    #tab.setFont(QtGui.QFont('', 24))
  panel.layouts['maintab'].tabs.setFont(QtGui.QFont('', 18))
  #panel.showFullScreen()
  #panel.showMaximized()

  #Since the onstatuschanged signal is emitted from TProcessManager,
  #we connect the onstatuschanged slots of panel to it.
  for w_name, (w_type, w_param) in panel.widgets_in.iteritems():
    if 'onstatuschanged' in w_param and w_param['onstatuschanged'] is not None:
      pm.onstatuschanged.connect(lambda status,w_param=w_param,w_name=w_name: w_param['onstatuschanged'](panel,panel.widgets[w_name],status))

  #ROS system initialization.
  run_cmd('roscore')
  pm.InitNode()
  rospy.wait_for_service('/rosout/get_loggers', timeout=5.0)
  run_cmd('fix_usb')
  pm.StartVirtualJoyStick()
  if not is_sim:  pm.StartUpdateStatusThread()
  panel.close_callback= lambda event: (
      pm.TerminateAllBGProcesses(),  #including roscore
      pm.StopUpdateStatusThread(),
      #stop_cmd('roscore'),
      True)[-1]

  RunPanelApp()
