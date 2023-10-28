#!/usr/bin/python
#\file    rs_sim.py
#\brief   Simulate RealSense from a log directory.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.03, 2021
import sys
import roslib; roslib.load_manifest('ay_py')
import rospy
import tf
import six.moves.cPickle as pickle
import os
import numpy as np
import copy
import gzip
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
#import multiprocessing as mp
import threading
import Queue
from ay_py.core import CPrint, TContainer, TContainerCore, TransformLeftInv
from ay_py.ros.pointcloud import DepthRGBImgsToPointCloud, DepthImgToPointCloud
from ay_py.ros.rs import ReconstructRS

def LoadLoop(files, queue_req, queue_res):
  i_file= 0
  try:
    while True:
      if rospy.is_shutdown():  return
      while True:
        try:
          req= queue_req.get(block=False)
          if req:  break
          if not req:  return
        except Queue.Empty:
          pass
        if rospy.is_shutdown():  return
        rospy.sleep(1e-2)

      filepath= files[i_file%len(files)]
      i_file+= 1

      if filepath.endswith('.dat.gz'):
        with gzip.open(filepath,'rb') as fp:
          data= pickle.load(fp)
      else:
        with open(filepath,'rb') as fp:
          data= pickle.load(fp)
      CPrint(2,'Opened data file:',filepath)
      if isinstance(data,dict):
        if 'obs' in data and 'rs' in data['obs'] and isinstance(data['obs'].rs,TContainerCore):
          rs= data['obs'].rs
        else:
          for k,v in data.iteritems():
            if 'rs' in v and isinstance(v.rs,TContainerCore):
              rs= v.rs
      elif isinstance(data,TContainerCore):
        rs= data
      else:
        msg= 'Data file does not store rs data.'
        CPrint(4,msg)
        raise Exception(msg)
      has_rgb= rs.stamp_rgb is not None
      has_depth= rs.stamp_depth is not None
      disp_img= lambda img: '{}({})'.format(img,rs['stamp_{}'.format(img)].stamp) if rs['stamp_{}'.format(img)] is not None else 'no-{}'.format(img)
      if has_depth:  CPrint(2,'Found rs data:',disp_img('rgb'),disp_img('depth'))
      else:          CPrint(4,'No depth data:',disp_img('rgb'),disp_img('depth'))
      if 'frame' not in rs:  rs.frame= 'tool0' if 'arm' in rs else 'base_link'

      if not has_depth:  continue
      ReconstructRS(rs, parent_filepath=filepath, generate_rgb_from_depth=True)
      step= min(rs.img_depth.shape[0]//240,rs.img_depth.shape[1]//320)
      pc_msg= DepthRGBImgsToPointCloud(rs.img_depth, rs.img_rgb, rs.proj_mat, copy.deepcopy(rs.msg_depth_header), xstep=step, ystep=step)
      #pc_msg= DepthImgToPointCloud(rs.img_depth, rs.proj_mat, copy.deepcopy(rs.msg_depth_header), xstep=step, ystep=step)
      queue_res.put((rs,pc_msg))
  except Exception as e:
    CPrint(4,e)
    raise e
  finally:
    queue_res.put((TContainer(),None))

def SearchDataFiles(filename):
  if os.path.isabs(filename):
    filepath= filename
  else:
    filepath= os.path.join(os.environ['HOME'],'data',filename)
  if os.path.isdir(filepath):
    files= [os.path.join(filepath,f) for f in sorted(os.listdir(filepath))
            if not os.path.isdir(os.path.join(filepath,f))]
    return files
  else:
    return [filepath]

if __name__=='__main__':
  rospy.init_node('ros_node')
  rospy.sleep(0.1)
  datadir= rospy.get_param('~datadir', 'rs_sim_default/')
  dt_latch= rospy.get_param('~latch', 3.0)
  hz= rospy.get_param('~hz', 15.0)
  verb= rospy.get_param('~verb', False)
  #Transform from camera_link to camera_color_optical_frame of RS.
  #lcam_x_colf= rospy.get_param('~lcam_x_colf', [0.0, 0.015, 0.0, -0.5, 0.49999999999755174, -0.5, 0.5000000000024483])

  files= SearchDataFiles(datadir)
  #queue_load_req,queue_load_res= mp.Queue(),mp.Queue()
  queue_load_req,queue_load_res= Queue.Queue(),Queue.Queue()
  #proc_load_loop= mp.Process(target=LoadLoop, args=(files,queue_load_req,queue_load_res))
  #proc_load_loop.start()
  th_load_loop= threading.Thread(name='LoadLoop', target=LoadLoop, args=(files,queue_load_req,queue_load_res))
  th_load_loop.start()
  queue_load_req.put(True)

  br= tf.TransformBroadcaster()
  def br_tf(lcam_x_colf):
    br.sendTransform(lcam_x_colf[:3],lcam_x_colf[3:], rospy.Time.now(), 'camera_color_optical_frame', 'camera_link')

  rate_adjuster= rospy.Rate(hz)
  cvbridge= CvBridge()

  pc_pub= rospy.Publisher('/camera/depth/color/points', sensor_msgs.msg.PointCloud2, queue_size=1)
  depth_pub= rospy.Publisher('/camera/aligned_depth_to_color/image_raw', sensor_msgs.msg.Image, queue_size=1)
  rgb_pub= rospy.Publisher('/camera/color/image_raw', sensor_msgs.msg.Image, queue_size=1)
  cam_info_pub= rospy.Publisher('/camera/aligned_depth_to_color/camera_info', sensor_msgs.msg.CameraInfo, queue_size=1)

  try:
    rs,pc_msg= None,None
    while not rospy.is_shutdown():
      while not rospy.is_shutdown():
        try:
          rs_new,pc_msg_new= queue_load_res.get(block=False)
        except Queue.Empty:
          rs_new,pc_msg_new= None,None
        if rs_new is not None or (rs_new is None and rs is not None):  break
        rospy.sleep(0.005)

      if rs_new is not None:
        rs,pc_msg= rs_new,pc_msg_new
        #print 'DEBUG,2', rs.img_rgb
        #print 'DEBUG,2.5', rs.img_rgb.shape
        rgb_msg= cvbridge.cv2_to_imgmsg(rs.img_rgb, encoding='bgr8')
        rgb_msg.header= rs.msg_rgb_header
        #print 'DEBUG,3', rs.img_depth
        #print 'DEBUG,3.5', rs.img_depth.shape
        depth_msg= cvbridge.cv2_to_imgmsg(rs.img_depth, encoding='passthrough')  #'16UC1'
        depth_msg.header= rs.msg_depth_header
        #print 'DEBUG,3.7', cvbridge.imgmsg_to_cv2(depth_msg, '16UC1').shape

        #print 'DEBUG,4'
        cam_info_msg= sensor_msgs.msg.CameraInfo()
        cam_info_msg.header= rs.msg_rgb_header
        cam_info_msg.height= rgb_msg.height
        cam_info_msg.width= rgb_msg.width
        cam_info_msg.P= rs.proj_mat.ravel()
        #NOTE: We do not provide the following elements:
        #float64[] D
        #float64[9] K
        #float64[9] R
        #uint32 binning_x
        #uint32 binning_y
        #sensor_msgs/RegionOfInterest roi

        #Transform from camera_link to camera_color_optical_frame of RS.
        lcam_x_colf= TransformLeftInv(rs.lw_x_camera_link,rs.lx)

        if verb:
          for key,value in rs.iteritems():
            print '{key}[{type}]= {value}'.format(key=key,type=type(value),value=value)

      t_latch_start= rospy.Time.now()
      while (rospy.Time.now()-t_latch_start).to_sec()<dt_latch and not rospy.is_shutdown():
        stamp= rospy.Time.now()
        pc_msg.header.stamp= stamp
        depth_msg.header.stamp= stamp
        rgb_msg.header.stamp= stamp
        cam_info_msg.header.stamp= stamp

        br_tf(lcam_x_colf)
        pc_pub.publish(pc_msg)
        depth_pub.publish(depth_msg)
        rgb_pub.publish(rgb_msg)
        cam_info_pub.publish(cam_info_msg)
        rate_adjuster.sleep()
        if rs_new is None:  break
      if rs_new is not None:
        queue_load_req.put(True)
  except KeyboardInterrupt:
    queue_load_req.put(False)
  finally:
    queue_load_req.put(False)
    #proc_load_loop.terminate()
    th_load_loop.join()
