#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo # For camera intrinsic parameters
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import os
import time
import tf
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from sklearn.cluster import KMeans


PLOTS_DIR = os.path.join(os.getcwd(), 'plots')

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)

        self.bridge = CvBridge()

        self.cv_color_image = None
        self.cv_depth_image = None

        self.color_image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_image_callback)
        self.depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)


        self.tf_listener = tf.TransformListener()  # Create a TransformListener object

        self.point_pub = rospy.Publisher("goal_point", Point, queue_size=10)
        self.image_pub = rospy.Publisher('detected_cup', Image, queue_size=10)
        self.marker_pub = rospy.Publisher('marker_array_topic', MarkerArray, queue_size=1)
        self.goal_mask_pub = rospy.Publisher('goal_mask', Image, queue_size=10)


        rospy.spin()

    def camera_info_callback(self, msg):
        # TODO: Extract the intrinsic parameters from the CameraInfo message
        # print("got cam nfo")
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def pixel_to_point(self, u, v, depth):
        depth = depth * 1.05
        # TODO: Use the camera intrinsics to convert pixel coordinates to real-world coordinates
        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        Z = depth
        return X, Y, Z

    def color_image_callback(self, msg):
        # print("color img")
        try:
            # Convert the ROS Image message to an OpenCV image (BGR8 format)
            self.cv_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # If we have both color and depth images, process them
            if self.cv_depth_image is not None:
                self.process_images()

        except Exception as e:
            print("Error:", e)

    def depth_image_callback(self, msg):
        # print("depth image")
        try:
            # Convert the ROS Image message to an OpenCV image (16UC1 format)
            self.cv_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

        except Exception as e:
            print("Error:", e)

    def rgb_to_hsv(self, rgb_threshold):
        # Convert the RGB numpy array to an HSV numpy array.
        hsv_threshold = cv2.cvtColor(np.uint8([[rgb_threshold]]), cv2.COLOR_RGB2HSV)[0][0]
        return hsv_threshold

    def get_global_marker(self, x_c,y_c,z_c, i, color):
        self.tf_listener.waitForTransform("base_link", "camera_link", rospy.Time(), rospy.Duration(10.0))
        point_base = self.tf_listener.transformPoint("base_link", PointStamped(header=Header(stamp=rospy.Time(), frame_id="/camera_link"), point=Point(x_c, y_c, z_c)))
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.id = i
        marker.action = Marker.ADD
        marker.pose.position.x = point_base.point.x
        marker.pose.position.y = point_base.point.y
        if color == 'b':
            marker.type = Marker.CYLINDER
            marker.color.b = 1
            marker.pose.position.z = 0.005 / 2 #
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.124
            marker.scale.y = 0.124
            marker.scale.z = 0.005
            marker.color.a = 1.0  # Alpha must be non-zero
        elif color == 'r' or color == 'g':
            marker.type = Marker.CUBE
            marker.pose.position.z = max(0.060325 / 2, point_base.point.z + 0.060325 / 2) #
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.028575
            marker.scale.y = 0.028575
            marker.scale.z = 0.060325
            marker.color.a = 1.0  # Alpha must be non-zero
            if color == 'r':
                marker.color.r = 1
            elif color == 'g':
                marker.color.g = 1
        
        return marker
        
    def get_centroids(self, x_coords, y_coords, num_clusters=2, epsilon=1e-3):
        points = np.concatenate((x_coords.reshape(-1, 1), y_coords.reshape(-1,1)), axis=-1)
        kmeans_model = KMeans(n_clusters=num_clusters, random_state=1).fit(points)
        centroids = kmeans_model.cluster_centers_
        if np.sum((centroids[0] - centroids[1])**2) < epsilon:
            return np.array([np.mean(centroids, axis=0)])
        else:
            return centroids



    def process_images(self):
        # print("got image")
        # Convert the color image to HSV color space
        hsv = cv2.cvtColor(self.cv_color_image, cv2.COLOR_BGR2HSV)
        # TODO: Define range for cup color in HSV
        # NOTE: You can visualize how this is performing by viewing the result of the segmentation in rviz
        lower_rgb = np.array([50, 40, 47])
        upper_rgb = np.array([170, 0, 255])

        # Convert RGB thresholds to HSV
        #Magenta - works well
        # lower_hsv = np.array([140, 50, 50])#self.rgb_to_hsv(lower_rgb)
        # upper_hsv = np.array([160, 255, 255])#self.rgb_to_hsv(upper_rgb)
        #Red - works ok it detects some skin
        # lower_hsv = np.array([-10, 140, 140])#self.rgb_to_hsv(lower_rgb)
        # upper_hsv = np.array([10, 255, 255])#self.rgb_to_hsv(upper_rgb)
        #Green - 
        bounds = dict()
        bounds['r'] = (np.array([-10, 140, 140]), np.array([10, 255, 255]))
        bounds['g'] = (np.array([20, 90, 90]), np.array([50, 255, 255]))
        bounds['b'] = (np.array([100, 90, 90]), np.array([140, 255, 255])) #need to test
        num_blocks = 0
        marker_array = MarkerArray()

        for color in bounds.keys():
            lower_hsv, upper_hsv = bounds[color]

            # TODO: Threshold the image to get only cup colors
            # HINT: Lookup np.where() or cv2.inRange()
            mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
            if color == 'b':
                mask_img = self.bridge.cv2_to_imgmsg(mask, "mono8")
                # print(ros_image)
                self.goal_mask_pub.publish(mask_img)
            #np.where(hsv > .1 and hsv < 0.3, 1, 0)

            # TODO: Get the coordinates of the cup points on the mask
            # HINT: Lookup np.nonzero() or np.where()
            y_coords, x_coords = np.where(mask == 255)#np.nonzero(mask)

            # If there are no detected points, exit
            if len(x_coords) == 0 or len(y_coords) == 0:
                print("No {} points detected".format(color))
                continue

            centroids = self.get_centroids(x_coords / mask.shape[0], y_coords / mask.shape[1])
            centroids[:, 0] *= mask.shape[0]
            centroids[:, 1] *= mask.shape[1]
            for i in range(centroids.shape[0]):

                center_x, center_y = centroids[i].astype(int)

                # Fetch the depth value at the center
                depth = self.cv_depth_image[center_y, center_x]
                # print('test 0')
                # print(self.fx, self.fy, self.cx, self.cy)
                if self.fx and self.fy and self.cx and self.cy:
                    camera_x, camera_y, camera_z = self.pixel_to_point(center_x, center_y, depth)
                    camera_link_x, camera_link_y, camera_link_z = camera_z, -camera_x, -camera_y
                    # Convert from mm to m
                    camera_link_x /= 1000
                    camera_link_y /= 1000
                    camera_link_z /= 1000
                    # Convert the (X, Y, Z) coordinates from camera frame to odom frame
                    try:
                        # self.tf_listener.waitForTransform("base_footprint", "camera_link", rospy.Time(), rospy.Duration(10.0))
                        # point_odom = self.tf_listener.transformPoint("base_footprint", PointStamped(header=Header(stamp=rospy.Time(), frame_id="/camera_link"), point=Point(camera_link_x, camera_link_y, camera_link_z)))
                        # X_odom, Y_odom, Z_odom = point_odom.point.x, point_odom.point.y, point_odom.point.z
                        # print("Real-world coordinates in camera frame: (X, Y, Z) = ({:.2f}m, {:.2f}m, {:.2f}m)".format(camera_link_x, camera_link_y, camera_link_z))

                        # print("Publishing goal point: ", camera_link_x, camera_link_y, camera_link_z)
                        # Publish the transformed point
                        self.point_pub.publish(Point(camera_link_x, camera_link_y, camera_link_z))

                        # Overlay cup points on color image for visualization
                        cup_img = self.cv_color_image.copy()
                        cup_img[y_coords, x_coords] = [0, 0, 255]  # Highlight cup points in red
                        cv2.circle(cup_img, (center_x, center_y), 5, [0, 255, 0], -1)  # Draw green circle at center
                        
                        # Convert to ROS Image message and publish
                        ros_image = self.bridge.cv2_to_imgmsg(cup_img, "bgr8")
                        # print(ros_image)
                        self.image_pub.publish(ros_image)
                    
                        marker_array.markers.append(self.get_global_marker(camera_link_x, camera_link_y, camera_link_z, num_blocks, color))
                        num_blocks += 1

                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                        print("TF Error: " + e)
                        return
        self.marker_pub.publish(marker_array)


if __name__ == '__main__':
    # print("ran script")
    ObjectDetector()
