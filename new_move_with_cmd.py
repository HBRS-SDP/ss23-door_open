#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
import time

class ForceToVelocityNode:
    def __init__(self):
        rospy.init_node('force_to_velocity_node')

        self.linear_velocity_x = -0.05
        self.angular_velocity = 0.0
        self.linear_velocity_y = 0.0
        self.force_angle = 0.0
        self.angular_published = False

        self.force_threshold = -30.0
        self.force_another_threshold = -50
        self.direction = None # earlier is_left
        self.in_loop = False
        self.trend_value_count = 0
        self.force_trend_values = []
        self.sub_force = rospy.Subscriber('/force_angles', Float32, self.force_callback)
        self.pub_cmd_vel = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
        self.force_feedback_sub = rospy.Subscriber('force', Bool, self.get_force_feedback)
        rospy.on_shutdown(self.shutdown)

    def get_force_feedback(self, msg):
        if msg.data:
            self.push_door_motion()
        else:
            self.pull_door_motion()


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

    def pull_door_motion(self):
        if self.force_angle.data > -30.0 and self.in_loop == False:
            # print("first loop")
            self.linear_velocity_x = -0.01
            self.linear_velocity_y = 0.0

        else:
            # print("we in else")
            self.in_loop = True
            if self.force_angle.data > 15.0:
                self.in_loop = False
            if self.direction == None:
                self.linear_velocity_y = -0.01
                self.decide_direction_of_movement(self.force_angle.data)
            

            self.linear_velocity_x = 0.0

            if self.direction == 'Right':# and self.force_angle.data > -33.0:
                # self.in_loop = True
                # if self.force_angle.data > 15.0:
                #     self.in_loop = False
                print("going right")
                print(self.direction)
                self.linear_velocity_y = -0.01
            if self.direction == 'Left':
                self.linear_velocity_y = 0.0
                print("going left")
                self.linear_velocity_y = 0.01
                self.linear_velocity_x = 0.0

    def push_door_motion(self):
        self.linear_velocity_x = 0.01
        self.linear_velocity_y = 0.0

        time.sleep(2)

        self.gripper_controller.open()
        rospy.loginfo('Door Handle released')

        self.moveToNeutral()

        time.sleep(2)


        self.linear_velocity = 0.0
        self.linear_velocity_y = 0.1

        time.sleep(2)


        self.linear_velocity = 0.1
        self.linear_velocity_y = 0.0

    def force_callback(self, force_angle):
        print(force_angle)

        self.force_angle=force_angle


    def shutdown(self):
        cmd_vel_msg = Twist()
        self.pub_cmd_vel.publish(cmd_vel_msg)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            self.pull_door_motion()
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = self.linear_velocity_x
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
             
