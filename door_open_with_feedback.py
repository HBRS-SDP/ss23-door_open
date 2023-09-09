#!/usr/bin/env python

import sys
import rospy
import actionlib
import random
import numpy as np
from std_msgs.msg import Bool,String
import time
import control_msgs.msg
import trajectory_msgs.msg
import controller_manager_msgs.srv
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool

from mdr_pickup_action.msg import PickupAction, PickupGoal
from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mas_hsr_gripper_controller.gripper_controller import GripperController

class DoorOpen :
    def __init__(self):
        rospy.init_node("DoorOpen")

        # intialize gripper controllerr
        self.gripper_controller = GripperController()   
        self.action_cli = actionlib.SimpleActionClient(
            '/hsrb/arm_trajectory_controller/follow_joint_trajectory',
            control_msgs.msg.FollowJointTrajectoryAction)
        # wait for the action server to establish connection
        self.action_cli.wait_for_server()
        rospy.loginfo("Connected to server for executing pouring")

        self.speak=1
        self.direction_multiplier = 1
        self.wrist_direction = None
        self.say_pub = rospy.Publisher('/say', String, latch=True, queue_size=1)
        self.pub  = rospy.Publisher('Handle_unlatched', Bool, queue_size=10)
        self.goal = control_msgs.msg.FollowJointTrajectoryGoal()
        self.traj = trajectory_msgs.msg.JointTrajectory()
        self.traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        self.p = trajectory_msgs.msg.JointTrajectoryPoint()
        #receive torques
        self.torque_sub = rospy.Subscriber('/hsrb/wrist_wrench/compensated', Bool, self.save_torque)
        self.torque_val = 0
        self.force_feedback_sub = rospy.Subscriber('force_threshold', Bool, self.get_force_feedback)


        self.pub_cmd_vel = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)


        #initialising the client for moving arm to neutral position
        try:
            self.move_arm_client = actionlib.SimpleActionClient("move_arm_server", MoveArmAction)
            rospy.loginfo('[pickup] Waiting for %s server', "move_arm_server")
            self.move_arm_client.wait_for_server()
        except Exception as exc:
            rospy.logerr('[pickup] %s server does not seem to respond: %s',
                        "move_arm_server", str(exc))
        print("All good!!")
    def get_force_feedback(self, msg):
        if msg.data and self.speak:
            self.direction_multiplier = 1
            rospy.logerr('[door-open]Cannot pull. Force feedback exceeds threshold. Trying other direction...')
            self.speak=0
            self.say("Cannot pull. Force feedback exceeds threshold.")

    def save_torque(self,msg):
        self.torque_val=msg.wrench.torque.x

    def get_door_handle_allignment(self):
        #decide clockwise or anticlockwise rotation
        self.p.positions= [0.35, -0.42, 0.0, -1.00, np.round(np.deg2rad(-55), 2)]
        self.p.velocities = [0, 0, 0, 0, 0]
        self.p.time_from_start = rospy.Duration(1)
        self.traj.points = [self.p]
        self.goal.trajectory = self.traj
        self.action_cli.send_goal(self.goal)
        self.action_cli.wait_for_result()
        time.sleep(1)
        if self.torque_val>0.5:
            self.p.positions= [0.35, -0.42, 0.0, -1.00, np.round(np.deg2rad(-135), 2)]
            self.p.time_from_start = rospy.Duration(1)
            self.traj.points = [self.p]
            self.goal.trajectory = self.traj
            self.action_cli.send_goal(self.goal)
            print(self.action_cli.wait_for_result())
            #Anti-clockwise wrist rotation
            self.wrist_direction='acw'
        else:
            #Clockwise wrist rotation
            self.wrist_direction='cw'
        self.say(str(self.wrist_direction))


    def say(self, sentence):
        say_msg = String()
        say_msg.data = sentence
        self.say_pub.publish(say_msg)

    def moveToNeutral(self):
        move_arm_goal = MoveArmGoal()
        move_arm_goal.goal_type = MoveArmGoal.NAMED_TARGET
        move_arm_goal.named_target = "neutral"
        self.move_arm_client.send_goal(move_arm_goal)
        self.move_arm_client.wait_for_result()
        self.move_arm_client.get_result()
        rospy.loginfo("Back to neutral position")
        
        rospy.sleep(5)


    def one_func(self):
        self.speak=1
        # open gripper by default
        self.say("Through the door")
        self.gripper_controller.open()
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()


        angles= list(range(0, -100, -15))
        inRadians= np.deg2rad(angles)
        wrist_roll_angles= np.round(inRadians, 2)
        for i in wrist_roll_angles: 
            p.positions= [0.35, -0.42, 0.0, -1.00, i]
            p.velocities = [0, 0, 0, 0, 0]
            p.time_from_start = rospy.Duration(1)
            traj.points = [p]
            goal.trajectory = traj
            self.action_cli.send_goal(goal)
            self.action_cli.wait_for_result()

        # close gripper arm
        self.gripper_controller.close()
        rospy.loginfo('Door Handle Grasped')

        handle = self.get_door_handle_allignment()
        
        ## second stage 
        if self.wrist_direction == 'acw':
            angles= list(range(-135, -145, -15))
        elif self.wrist_direction == 'cw':
            angles= list(range(-55, -45, 15))
        inRadians= np.deg2rad(angles)
        wrist_roll_angles= np.round(inRadians, 2)
        for i in wrist_roll_angles: 
            p.positions= [0.31, -0.42, 0.0, -1.00, i]
            p.velocities = [0, 0, 0, 0, 0]
            p.time_from_start = rospy.Duration(1)
            traj.points = [p]
            goal.trajectory = traj
            self.action_cli.send_goal(goal)
            self.action_cli.wait_for_result()       

        # close gripper arm
        self.gripper_controller.close()
        rospy.loginfo('Door Handle Unlatched')
        rospy.Rate(10)

        # while not rospy.is_shutdown():
            
        #     rospy.sleep(0.1)
        rospy.loginfo('Received force feedback')
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = -0.1
        self.pub_cmd_vel.publish(cmd_vel_msg)
        time.sleep(1)
        cmd_vel_msg.linear.x = 0.0
        self.pub_cmd_vel.publish(cmd_vel_msg)

        


        # now move back a bit
        #self.movebackwards()
        #rospy.loginfo('Moved back a bit')

        
def main():
    door_open= DoorOpen()
    door_open.one_func()
    rospy.spin()
    


if __name__== "__main__" :
    main()
