#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

def rgb_callback(data):
    try:
        # Convert ROS image message to OpenCV format
        bridge = CvBridge()
        rgb_image = bridge.imgmsg_to_cv2(data, "bgr8")

        # Display the RGB image using matplotlib
        plt.figure()
        plt.imshow(cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB))
        plt.title("RGB Image")
        plt.axis("off")
        plt.show()

    except Exception as e:
        rospy.logerr(e)

def depth_callback(data):
    try:
        # Convert ROS image message to OpenCV format
        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

        # Display the depth image using matplotlib
        plt.figure()
        plt.imshow(depth_image, cmap="gray")
        plt.title("Depth Image")
        plt.axis("off")
        plt.show()

    except Exception as e:
        rospy.logerr(e)

if __name__ == "__main__":
    rospy.init_node("image_stream_viewer")

    rgb_sub = rospy.Subscriber(
        "/hsrb/head_rgbd_sensor/rgb/image_raw", Image, rgb_callback, queue_size=1
    )
    depth_sub = rospy.Subscriber(
        "/hsrb/head_rgbd_sensor/depth_registered/image_raw", Image, depth_callback, queue_size=1
    )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
