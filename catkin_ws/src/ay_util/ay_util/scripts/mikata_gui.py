#!/usr/bin/python
#\file    mikata_gui.py
#\brief   Mikata Arm GUI control panel.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Feb.06, 2018
#\version 0.2
#\date    Sep.02, 2018
#         Modified to use mikata_driver (controller as ROS node).
#\date    Apr.28, 2022
#         Modified for a new demo.
import roslib; roslib.load_manifest('ay_py')
from ay_py.tool.py_gui import RunTerminalTab

if __name__=='__main__':
  E= 'Enter'
  widgets= [
    ('main',[
      ('Init',(':all','ros',E)),
      ('Exit',':close') ]),
    ('roscore',[
      (':pair', ('roscore',['roscore',E]),
                ('kill',['C-c']) )  ]),
    ('JoyUSB',':radio',['js0','js1']),
    ('MikataUSB',':radio',['USB0','USB1']),
    ('System',[
      (':pair', ('system',['roslaunch ay_util mikata_rot_real2.launch jsdev:=/dev/input/{JoyUSB} dev:=/dev/tty{MikataUSB}',E]),
                ('kill',['C-c']) )  ]),
    ('Mikata',[
      ('fix_usb',['sudo /sbin/fix_usb_latency.sh tty{MikataUSB}',E]),
      ('survo-off',['rosrun ay_py mikata2_off.py',E]),
      ('reboot',['rosrun ay_py mikata2_reboot.py',E])  ]),
    ('Monitor-joy',[
      (':pair', ('echo-joy',['rostopic echo /joy',E]),
                ('kill',['C-c']) )  ]),
    ('rviz',[
      (':pair', ('rviz',['rviz',E]),
                ('kill',['C-c']) )  ]),
    #('time',[
      #(':pair', ('time',['rosrun ay_vision disp_rostime',E]),
                #('kill',['C-c']) )  ]),
    ##('m100',[
      ##(':pair', ('start',['roslaunch ay_3dvision sentis_tof_m100_s.launch',E]),
                ##('kill',['C-c']) )  ]),
    ##('pose_est',[
      ##(':pair', ('start',['roslaunch ay_3dvision rt_pose_estimator_m100.launch',E]),
                ##('kill',['C-c']) )  ]),
    #('aypi11',[
      #(':pair', ('stream',['ssh ayg@aypi11 "./stream.sh"',E]),
                #('stop',[E]) ),
      #('config',['ssh ayg@aypi11 "./conf_elp.sh"',E]),
      #('reboot',['ssh ayg@aypi11 "sudo reboot"',E]),
      #('shutdown',['ssh ayg@aypi11 "sudo halt -p"',E])  ]),
    #('fv11',[
      #(':pair', ('start',['roslaunch ay_fv_extra fv_pi11.launch',E]),
                #('kill',['C-c']) )  ]),
    ('fingervision',[
      (':pair', ('fvp',['roslaunch ay_fv_extra fvp_mikata.launch',E]),
                ('kill',['C-c']) ),
      ]),
    ('fv_util',[
      ('config',['rosrun fingervision conf_cam2.py /media/video_fv1 "file:CameraParams:0:`rospack find ay_fv_extra`/config/fvp_mikata.yaml"',E]),
      ]),
    ##('aypi11-no3',[
      ##(':pair', ('stream',['ssh aypi11 "./stream_no3.sh"',E]),
                ##('stop',[E]) )  ]),
    ##('monitor11-no3',[
      ##(':pair', ('run',['~/prg/testl/cv/capture.out "http://aypi11:8082/?action=stream&dummy=file.mjpg"',E]),
                ##('kill',['C-c']) )  ]),
    #('aypi10',[
      #(':pair', ('stream',['ssh ay@aypi10 "./stream1.sh"',E]),
                #('stop',[E]) ),
      #('reboot',['ssh ay@aypi10 "sudo reboot"',E]),
      #('shutdown',['ssh ay@aypi10 "sudo halt -p"',E])  ]),
    #('monitor10',[
      #(':pair', ('run',['~/prg/testl/cv/capture.out "http://aypi10:8080/?action=stream&dummy=file.mjpg"',E]),
                #('kill',['C-c']) )  ]),
    #('segm_obj',[
      #(':pair', ('start',['roslaunch ay_vision segm_obj2.launch',E]),
                #('kill',['C-c']) )  ]),
    ('JoyStickDemo',[
      (':pair', ('start',['rosrun ay_trick direct_run.py "mikata.setup \'mikata\',True" j',E]),
                ('quit',['q',E]) )  ]),
    ##('aypi3',[
      ##(':pair', ('stream',['ssh hm@aypi3 "./stream2.sh"',E]),
                ##('stop',[E]) ),
      ##('reboot',['ssh hm@aypi3 "sudo reboot"',E])  ]),
    ##('monitor1',[
      ##(':pair', ('run',['~/prg/testl/cv/capture.out "http://aypi3:8080/?action=stream&dummy=file.mjpg"',E]),
                ##('kill',['C-c']) )  ]),
    ##('monitor2',[
      ##(':pair', ('run',['~/prg/testl/cv/capture.out "http://aypi3:8081/?action=stream&dummy=file.mjpg"',E]),
                ##('kill',['C-c']) )  ]),
    ]
  exit_command= [E,'C-c']
  RunTerminalTab('Mikata Launcher',widgets,exit_command)
