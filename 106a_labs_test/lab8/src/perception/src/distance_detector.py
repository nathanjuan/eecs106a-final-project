#!/usr/bin/env python

import rospy
# import cv2
import numpy as np
from sensor_msgs.msg import Image
# from sensor_msgs.msg import CameraInfo # For camera intrinsic parameters
# from cv_bridge import CvBridge
# import matplotlib.pyplot as plt
# import os
# import time
import tf
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header

class DistanceDetector:
    def __init__(self):
        rospy.init_node('distance_detector', anonymous=True)
        print("node distance detector init'ed")

        self.object_point_sub = rospy.Subscriber("/goal_point", Point, self.object_point_callback)
        # self.depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)

        self.object_point = None

        self.tf_listener = tf.TransformListener()  # Create a TransformListener object

        self.distance_pub = rospy.Publisher('distance', Header, queue_size=10)

        self.obj_point_pub = rospy.Publisher("obj_point", PointStamped, queue_size=10)
        self.ar_point_pub = rospy.Publisher('ar_point', PointStamped, queue_size=10)

        rospy.spin()
    def object_point_callback(self, msg):
        print("object poiunt recieved")
        self.object_point = msg
        self.find_distance()

    def find_distance(self):
        self.obj_point_pub.publish(PointStamped(header=Header(stamp=rospy.Time(), frame_id="/base_footprint"), point=self.object_point))
        self.tf_listener.waitForTransform("ar_marker_0", "base_footprint", rospy.Time(), rospy.Duration(10.0))
        point_in_tag_frame = self.tf_listener.transformPoint("ar_marker_0", PointStamped(header=Header(stamp=rospy.Time(), frame_id="/base_footprint"), point=self.object_point))
        self.ar_point_pub.publish(point_in_tag_frame)
        # print("ar tag 0 point: ", point_in_tag_frame.point)
        object_point = np.array([self.object_point.x, self.object_point.y, self.object_point.z])
        # print('object distance: ', np.linalg.norm(object_point))
        ar_object_np = np.array([point_in_tag_frame.point.x, point_in_tag_frame.point.y, point_in_tag_frame.point.z])
        # print('ar-object distance: ', np.linalg.norm(ar_object_np))
    

if __name__ == '__main__':
    DistanceDetector()