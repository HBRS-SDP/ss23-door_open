#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
import control_msgs.msg
import trajectory_msgs.msg
import controller_manager_msgs.srv
import time
import numpy as np
#refactor
from mas_hsr_gripper_controller.gripper_controller import GripperController
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
import actionlib




class ForceToVelocityNode:
    def __init__(self):
        rospy.init_node('force_to_velocity_node')
        #######################################3change vel
        self.linear_velocity_x = -0.00
        self.angular_velocity = 0.0
        self.linear_velocity_y = 0.0
        self.force_angle = 0.0
        self.angular_published = False

        self.door_open_status = False
        self.previous_x_position = None

        self.force_threshold = -30.0
        self.force_another_threshold = -50
        self.direction = None # earlier is_left
        self.in_loop = False
        self.trend_value_count = 0
        self.force_trend_values = []
        self.sub_force = rospy.Subscriber('/force_angles', Float64, self.force_callback)
        self.pub_cmd_vel = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
        self.force_feedback_sub = rospy.Subscriber('force_threshold', Bool, self.get_force_feedback)
        self.odom_sub = rospy.Subscriber('/hsrb/odom', Odometry, self.get_odom_data)
        self.gripper_controller = GripperController()
        self.move_arm_client = actionlib.SimpleActionClient("move_arm_server", MoveArmAction)
        rospy.loginfo('[pickup] Waiting for %s server', "move_arm_server")
                #initialising the action client for pouring
        self.action_cli = actionlib.SimpleActionClient(
            '/hsrb/arm_trajectory_controller/follow_joint_trajectory',
            control_msgs.msg.FollowJointTrajectoryAction)
        # wait for the action server to establish connection
        self.action_cli.wait_for_server()
        
        self.move_arm_client.wait_for_server() 
        print("All good!!")
        rospy.on_shutdown(self.shutdown)

    def get_force_feedback(self, msg):
        # start_odom = self.odom_pose_x
        # if abs(start_odom)-abs(self.odom_pose_x)

 

        print(msg.data)
        # if msg.data:
        self.push_door_motion()
            
        # else:
        #     print('dfhdh')
        # self.pull_door_motion()

    def get_odom_data(self,msg):
        self.odom_pose_x = msg.pose.pose.position.x


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
        if self.force_angle > -30.0 and self.in_loop == False:
            # print("first loop")
            self.linear_velocity_x = -0.01
            self.linear_velocity_y = 0.0

        else:
            # print("we in else")
            self.in_loop = True
            if self.force_angle > 15.0:
                self.in_loop = False
            if self.direction == None:
                self.linear_velocity_y = -0.01
                self.decide_direction_of_movement(self.force_angle)
            

            self.linear_velocity_x = 0.0
            self.direction = 'Right'

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

    def moveToNeutral(self):
        move_arm_goal = MoveArmGoal()
        move_arm_goal.goal_type = MoveArmGoal.NAMED_TARGET
        move_arm_goal.named_target = "neutral"
        self.move_arm_client.send_goal(move_arm_goal)
        self.move_arm_client.wait_for_result()
        self.move_arm_client.get_result()
        rospy.loginfo("Back to neutral position")
        
        rospy.sleep(5)
    
    def move_to_home_position(self):
        print("Moving to home position")

        # open gripper by defaults
        self.gripper_controller.open()

        # Go to a position from which door will be pushed
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()

        p.positions= [0.0, 0.0, 0.0, -1.5, 0]
        p.velocities = [0, 0, 0, 0, 0]
        p.time_from_start = rospy.Duration(1)
        traj.points = [p]
        goal.trajectory = traj
        self.action_cli.send_goal(goal)
        self.action_cli.wait_for_result()
        rospy.loginfo('Door pushing position ready')

    def push_door_motion(self):
        self.move_to_home_position()
        self.linear_velocity_x = 0.0
        self.linear_velocity_y = +0.05
        time.sleep(3)
        self.linear_velocity_x = 0.1
        self.linear_velocity_y = 0.00
        time.sleep(1)
        self.linear_velocity_x = 0.01
        current_x_position = self.odom_pose_x
       
        if self.previous_x_position is not None:
            distance_x = abs(abs(current_x_position) - abs(self.previous_x_position))
            print(distance_x,self.door_open_status)
            #  print(current_x_position,self.door_open_status)
            if distance_x >= 0.1 and not self.door_open_status :
                rospy.loginfo("Flag raised! X-position difference is greater than 0.5 meter.")
                self.door_open_status  = True
                self.linear_velocity_x = 0.0
                self.linear_velocity_y = 0.0
            elif distance_x < 0.1:
                rospy.loginfo("Flag reset. X-position difference is less than 0.5 meter.")
                self.door_open_status  = False
                self.linear_velocity_x = 0.05
                self.linear_velocity_y = 0.0
        else:
            self.previous_x_position = current_x_position
         

        self.gripper_controller.open()
        rospy.loginfo('Door Handle released')

        # self.moveToNeutral()

    def force_callback(self, force_angle):
        self.force_angle=force_angle.data


    def shutdown(self):
        cmd_vel_msg = Twist()
        self.pub_cmd_vel.publish(cmd_vel_msg)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            # self.pull_door_motion()
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
             
