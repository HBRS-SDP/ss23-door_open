#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time

class ForceToVelocityNode:
    def __init__(self):
        rospy.init_node('force_to_velocity_node')

        self.linear_velocity = -0.05
        self.angular_velocity = 0.0
        self.linear_velocity_y = 0.0
        self.angular_published = False

        self.force_threshold = -30.0
        self.force_another_threshold = -50
        self.direction = None # earlier is_left
        self.in_loop = False
        self.trend_value_count = 0
        self.force_trend_values = []
        self.sub_force = rospy.Subscriber('/max_force', Float32, self.force_callback)
        self.pub_cmd_vel = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
        rospy.on_shutdown(self.shutdown)

    def check_force_trend(self, tolerance_threshold = 5, min_streak_length = 3):
        print(self.force_trend_values)
        dir = None
        streak = 0
        max_streak = 0
        is_trend_decreasing = False
        
        for i in range(1, len(self.force_trend_values)):
            if self.force_trend_values[i] <= self.force_trend_values[i - 1] + tolerance_threshold:
                streak += 1
                max_streak = max(max_streak, streak)
                if streak >= min_streak_length:
                    is_trend_decreasing = True
            else:
                streak = 0
        # print(max_streak)

        if is_trend_decreasing == True:
            dir = 'Left'
        else:
            dir = 'Right'
        
        print(dir)
        return dir

    def decide_direction_of_movement(self, force_angle):
        if self.trend_value_count <= 50: # TODO: change this to checking length of list and clean list after done
            # self.linear_velocity_y = -0.01 # pass this back to callback
            self.force_trend_values.append(force_angle)
            self.trend_value_count += 1
            # print("decide direction", self.force_trend_values)
        else:
            self.direction  = self.check_force_trend()

        
        # if self.trend_value_set == True:
        #    self.direction  = check_force_trend(self.force_trend_values)


    def force_callback(self, force_msg):

        if force_msg.data > -30.0 and self.in_loop == False:
            # print("first loop")
            self.linear_velocity = -0.01
            self.linear_velocity_y = 0.0

        else:
            # print("we in else")
            self.in_loop = True
            if force_msg.data > 15.0:
                self.in_loop = False
            if self.direction == None:
                self.linear_velocity_y = -0.01
                self.decide_direction_of_movement(force_msg.data)
            

            self.linear_velocity = 0.0

            if self.direction == 'Right':# and force_msg.data > -33.0:
                # self.in_loop = True
                # if force_msg.data > 15.0:
                #     self.in_loop = False
                print("going right")
                print(self.direction)
                self.linear_velocity_y = -0.01
            if self.direction == 'Left':
                self.linear_velocity_y = 0.0
                # print("In else")
                # if force_msg.data < -33.0: 
                # self.in_loop = True
                # if force_msg.data > 15.0:
                #     self.in_loop = False
                print("going left")
                self.linear_velocity_y = 0.01
                self.linear_velocity = 0.0
    #    if force_msg.data < self.force_threshold:
    #        self.angular_published = True
    #        print("here")
    #        #self.angular_velocity = -0.1  # Set angular velocity only once
    #        self.linear_velocity = 0.0
    #        self.linear_velocity_y = -0.03
    #        if force_msg.data < (self.force_threshold - 15):
    #            self.linear_velocity_y = 0.03
    #    else:
    #        print("loop")
    #        self.angular_velocity = 0.0
    #        self.linear_velocity_y = 0.0
    #        self.linear_velocity = -0.05

       

        # cmd_vel_msg = Twist()
        # cmd_vel_msg.linear.x = -0.1#self.linear_velocity
        # cmd_vel_msg.linear.y = 0.0#self.linear_velocity_y
        # cmd_vel_msg.angular.z = 0.0#self.angular_velocity

        # self.pub_cmd_vel.publish(cmd_vel_msg)

    def shutdown(self):
        cmd_vel_msg = Twist()
        self.pub_cmd_vel.publish(cmd_vel_msg)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = self.linear_velocity
            cmd_vel_msg.linear.y = self.linear_velocity_y
            cmd_vel_msg.angular.z = self.angular_velocity

            self.pub_cmd_vel.publish(cmd_vel_msg)

            rate.sleep()

if __name__ == '__main__':
    try:
        node = ForceToVelocityNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
             
