#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2016 Toyota Motor Corporation
"""Speak Object Weight Sample"""

import math
# import os
import sys
from std_msgs.msg import Bool
# import actionlib
import numpy as np
from scipy import signal

import time 
# from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import WrenchStamped
# import tf
# import geometry_msgs.msg
# from  tf.transformations import euler_from_quaternion
# from geometry_msgs.msg import Quaternion

import rospy
from sensor_msgs.msg import JointState
from tmc_control_msgs.msg import (
    GripperApplyEffortAction,
    GripperApplyEffortGoal
)
from tmc_manipulation_msgs.srv import (
    SafeJointChange,
    SafeJointChangeRequest
)
from tmc_msgs.msg import (
    TalkRequestAction,
    TalkRequestGoal,
    Voice
)


_CONNECTION_TIMEOUT = 10.0


def compute_difference(pre_data_list, post_data_list,initial,post):
    if (len(pre_data_list) != len(post_data_list)):
        raise ValueError('Argument lists differ in length')
   
    
    angle=np.degrees(initial)
    
    # Applying transformation based on rotation about Z axis
    x_new=post_data_list[0]*math.cos(angle)-post_data_list[1]*math.sin(angle)
    y_new=post_data_list[0]*math.sin(angle)+post_data_list[1]*math.cos(angle)
    z_new=post_data_list[2]

    x_old=pre_data_list[0]
    y_old=pre_data_list[1]
    z_old=pre_data_list[2]

    # Calcurate square sum of difference
    result=math.sqrt(math.pow(x_old-x_new,2)+math.pow(y_old-y_new,2)+math.pow(z_old - z_new,2))
    return result

class ForceSensorCapture(object):
    """Subscribe and hold force sensor data"""

    def __init__(self):
        self._force_data_x = 0.0
        self._force_data_y = 0.0
        self._force_data_z = 0.0
        self.wrist_roll_initial_pos=0.0
        self.wrist_roll_post_pos=0.0
        self.wrist_roll_angle=0.0
        # Subscribe force torque sensor data from HSRB
        
        # Here we are using compensated data
        ft_sensor_topic = '/hsrb/wrist_wrench/compensated'
        self._wrist_wrench_sub = rospy.Subscriber(
            ft_sensor_topic, WrenchStamped, self.__ft_sensor_cb)
        self.wrist_roll_sub=rospy.Subscriber('/hsrb/joint_states',JointState,self.__angle_cb)

        # Wait for connection
        try:
            rospy.wait_for_message(ft_sensor_topic, WrenchStamped,
                                   timeout=_CONNECTION_TIMEOUT)
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)
    
    def __angle_cb(self,data):
        self.wrist_roll_angle=data.position[-1]


    def get_current_force(self):
        return [self._force_data_x, self._force_data_y, self._force_data_z]
    # Added separately
    def get_current_angle(self):
        return self.wrist_roll_angle

    def __ft_sensor_cb(self, data):
        
        # Getting force as three components
        self._force_data_x = data.wrench.force.x
        self._force_data_y = data.wrench.force.y
        self._force_data_z = data.wrench.force.z
        
        # Applying low pass filter for force values

        FX = []
        FY = []
        FZ = []

        FX.append(self._force_data_x)
        FY.append(self._force_data_y)
        FZ.append(self._force_data_z)

        # Sampling frequency
        fs = 124.95 
        
        # Cut-off frequency
        fc = 55

        w = fc / (fs/2)

        # Using function for lowpass butterworth filter
        b,a = signal.butter(2,w,'low')
        
        self._force_data_x = signal.lfilter(b, a, FX) #Forward filter
        self._force_data_y = signal.lfilter(b, a, FY)
        self._force_data_z = signal.lfilter(b, a, FZ)

        # Reference for lowpass filter: https://answers.ros.org/question/312833/how-do-i-implement-a-low-pass-filter-to-reduce-the-noise-coming-from-a-topic-that-is-publishing-a-wrenchstamped-msg-type-using-a-python-script/



def calculate_force():

    pub = rospy.Publisher('force',Bool, queue_size = 10)
    # Start force sensor capture
    force_sensor_capture = ForceSensorCapture()

    # Get initial data of force sensor
    pre_force_list = force_sensor_capture.get_current_force()
    
    pre_angle=force_sensor_capture.get_current_angle()


    # Wait until force sensor data become stable
    rospy.sleep(1.0)
    
    # Getting current force
    post_force_list = force_sensor_capture.get_current_force()

    post_angle=force_sensor_capture.get_current_angle()



    force_difference = compute_difference(pre_force_list, post_force_list,pre_angle,post_angle)
   

    rospy.Rate(10)
    while not rospy.is_shutdown():
        # Getting new force sensor reading
        post_force_list = force_sensor_capture.get_current_force()
        
        # Getting current angle
        post_angle=force_sensor_capture.get_current_angle()


        # Computing difference from initial and new force sensor readings

        force_difference = compute_difference(pre_force_list, post_force_list,pre_angle,post_angle)
        if force_difference > 45:
            pub.publish(True)
        else:
            pub.publish(False)
        print(force_difference)
        rospy.sleep(0.1)


if __name__ == '__main__':
    rospy.init_node('hsrb_get_force')
    calculate_force()
    rospy.spin()
