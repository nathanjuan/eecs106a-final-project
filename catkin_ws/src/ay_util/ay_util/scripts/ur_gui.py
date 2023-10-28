#!/usr/bin/python
#\file    ur_gui.py
#\brief   UR GUI control panel.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.25, 2018
#\version 0.2
#\date    Jun.24, 2019
#         Updated for UR3e, UR5e
import roslib; roslib.load_manifest('ay_py')
from ay_py.tool.py_gui import RunTerminalTab

if __name__=='__main__':
  robot_list= [
    'UR3', 'UR3DxlG', 'UR3ThG', 'UR3DxlpY1', 'UR3_SIM', 'UR3DxlG_SIM', 'UR3ThG_SIM', 'UR3DxlpY1_SIM',
    'UR3e','UR3eDxlG','UR3eThG','UR3eDxlpY1','UR3e_SIM','UR3eDxlG_SIM','UR3eThG_SIM','UR3eDxlpY1_SIM',
    'UR5e','UR5eDxlG','UR5eThG','UR5eDxlpY1','UR5e_SIM','UR5eDxlG_SIM','UR5eThG_SIM','UR5eDxlpY1_SIM',
    'Gen3','Gen3ThG','Gen3DxlO3',
    ]
  E= 'Enter'
  widgets= [
    ('main',[
      ('Init',(':all','ros',E)),
      ('Exit',':close') ]),
    ('roscore',[
      (':pair', ('roscore',['roscore',E]),
                ('kill',['C-c']) )  ]),
    ('URType',':cmb',robot_list),
    ('JoyUSB',':radio',['js0','js1']),
    ('DxlUSB',':radio',['USB0','USB1','USB2','USB3']),
    ('Dynamixel',[
      ('fix_usb',['rosrun ay_util fix_usb_latency.sh tty{DxlUSB}',E]),
      ]),
    ('System',[
      (':pair', ('run',['roslaunch ay_util ur_selector.launch robot_code:={URType} jsdev:=/dev/input/{JoyUSB} dxldev:=/dev/tty{DxlUSB}',E]),
                ('kill',['C-c']) ),
      ('UR calib',['roslaunch ay_util ur_calib.launch robot_code:={URType}',E]),
      ('Rboot dxlg',['roslaunch ay_util ur_gripper_reboot.launch robot_code:={URType} dxldev:=/dev/tty{DxlUSB}',E]),
      ]),
    ('UR',[
      ('dashboard_gui',['rosrun ay_util ur_dashboard_gui.py',E]),
      ]),
    #('Mikata',[
      #('survo-off',['rosrun ay_py mikata_off.py',E]),
      #('reboot',['rosrun ay_py mikata_reboot.py',E])  ]),
    ('Monitor-joy',[
      (':pair', ('echo-joy',['rostopic echo /joy',E]),
                ('kill',['C-c']) )  ]),
    ('rviz',[
      (':pair', ('rviz',['rviz',E]),
                ('kill',['C-c']) )  ]),
    ('time',[
      (':pair', ('time',['rosrun ay_vision disp_rostime',E]),
                ('kill',['C-c']) )  ]),
    #('m100',[
      #(':pair', ('start',['roslaunch ay_3dvision sentis_tof_m100_s.launch',E]),
                #('kill',['C-c']) )  ]),
    #('pose_est',[
      #(':pair', ('start',['roslaunch ay_3dvision rt_pose_estimator_m100.launch',E]),
                #('kill',['C-c']) )  ]),
    #('PiID',':radio',['pi13','pi14','pi15','local01','local12']),
    #('aypiX',[
      #(':pair', ('stream',['ssh ayg@ay{PiID} "./stream.sh"',E]),
                #('stop',[E]) ),
      #('reboot',['ssh ayg@ay{PiID} "sudo reboot"',E]),
      #('shutdown',['ssh ayg@ay{PiID} "sudo halt -p"',E])  ]),
    #('aypiX-2',[
      #('config',['ssh ayg@ay{PiID} "./conf_elp.sh"',E]),  ]),
    #('local',[
      #('config(elp)',['cd ~/ros_ws/ay_tools/fingervision/tools',E,'./conf_elp.sh {PiID}',E]),
      #('config(asahi)',['cd ~/ros_ws/ay_tools/fingervision/tools',E,'./conf_asahi.sh {PiID}',E]),  ]),
    #('fingervision',[
      #(':pair', ('start',['roslaunch ay_fv_extra fv_{PiID}.launch',E]),
                #('kill',['C-c']) )  ]),
    ('fingervision',[
      (':pair', ('fvp_3',['roslaunch ay_fv_extra fvp_3.launch',E]),
                ('kill',['C-c']) ),
      ]),
    ('fv_util',[
      ('config(L)',['rosrun fingervision conf_cam2.py /media/video_fv1 "file:CameraParams:0:`rospack find ay_fv_extra`/config/fvp_3_l.yaml"',E]),
      ('config(R)',['rosrun fingervision conf_cam2.py /media/video_fv2 "file:CameraParams:0:`rospack find ay_fv_extra`/config/fvp_3_r.yaml"',E]),
      ]),
    #('aypi13-no3',[
      #(':pair', ('stream',['ssh aypi13 "./stream_no3.sh"',E]),
                #('stop',[E]) )  ]),
    #('monitor11-no3',[
      #(':pair', ('run',['~/prg/testl/cv/capture.out "http://aypi13:8082/?action=stream&dummy=file.mjpg"',E]),
                #('kill',['C-c']) )  ]),
    #('aypi10',[
      #(':pair', ('stream1',['ssh ayg@aypi10 "./stream1.sh"',E]),
                #('stop',[E]) ),
      ##(':pair', ('stream2',['ssh ayg@aypi10 "./stream.sh"',E]),
                ##('stop',[E]) ),
      #('reboot',['ssh ayg@aypi10 "sudo reboot"',E]),
      #('shutdown',['ssh ayg@aypi10 "sudo halt -p"',E])  ]),
    #('aypi10-2',[
      #('config1',['ssh ayg@aypi10 "./conf_elp1.sh"',E]),
      ##('config2',['ssh ayg@aypi10 "./conf_elp.sh"',E])
      #]),
    #('monitor10',[
      #(':pair', ('run',['~/prg/testl/cv/capture.out "http://aypi10:8080/?action=stream&dummy=file.mjpg"',E]),
                #('kill',['C-c']) )  ]),
    ('segm_obj',[
      (':pair', ('start',['roslaunch ay_vision segm_obj3.launch',E]),
                ('kill',['C-c']) )  ]),
    #('fv10',[
      #(':pair', ('start',['roslaunch ay_fv_extra fv_pi10.launch',E]),
                #('kill',['C-c']) )  ]),
    ('JoyStickDemo',[
      (':pair', ('start(real)',['rosrun ay_trick direct_run.py "robot \'{URType}\'" "fv.fv \'on\'" "viz \'\'" j',E]),
                ('quit',['q',E]) ),
      (':pair', ('start(k-sim)',['rosrun ay_trick direct_run.py "robot \'urs\'" j',E]),
                ('quit',['q',E]) )  ]),
    #('aypi3',[
      #(':pair', ('stream',['ssh hm@aypi3 "./stream2.sh"',E]),
                #('stop',[E]) ),
      #('reboot',['ssh hm@aypi3 "sudo reboot"',E])  ]),
    #('monitor1',[
      #(':pair', ('run',['~/prg/testl/cv/capture.out "http://aypi3:8080/?action=stream&dummy=file.mjpg"',E]),
                #('kill',['C-c']) )  ]),
    #('monitor2',[
      #(':pair', ('run',['~/prg/testl/cv/capture.out "http://aypi3:8081/?action=stream&dummy=file.mjpg"',E]),
                #('kill',['C-c']) )  ]),
    ]
  exit_command= [E,'C-c']
  RunTerminalTab('UR Launcher',widgets,exit_command,size=(1000,400))
