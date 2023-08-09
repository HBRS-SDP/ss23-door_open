#!/usr/bin/env python

import sys
import rospy
import actionlib
import random
import numpy as np
from std_msgs.msg import Bool,String

import control_msgs.msg
import trajectory_msgs.msg
import controller_manager_msgs.srv
from geometry_msgs.msg import PoseStamped

from mdr_pickup_action.msg import PickupAction, PickupGoal
from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mas_hsr_gripper_controller.gripper_controller import GripperController

class pickAndPour :
    def __init__(self):
        rospy.init_node("pickAndPour")

        # intialize gripper controllerr
        self.gripper_controller = GripperController()

        #initialising the action client for picking
        self.client_pick = actionlib.SimpleActionClient('pickup_server', PickupAction)
        rospy.loginfo('waiting for server')
        self.client_pick.wait_for_server()
        rospy.loginfo('Done waiting for server')

        #initialising the action client for moving sideways
        try:
            self.move_base_client = actionlib.SimpleActionClient("move_base_server", MoveBaseAction)
            rospy.loginfo('[pickup] Waiting for %s server', "move_base_server")
            self.move_base_client.wait_for_server()
        except Exception as exc:
            rospy.logerr('[pickup] %s server does not seem to respond: %s',
                         "move_base_server", str(exc))     

        #initialising the action client for pouring
        self.action_cli = actionlib.SimpleActionClient(
            '/hsrb/arm_trajectory_controller/follow_joint_trajectory',
            control_msgs.msg.FollowJointTrajectoryAction)
        # wait for the action server to establish connection
        self.action_cli.wait_for_server()
        rospy.loginfo("Connected to server for executing pouring")

        self.speak=1
        self.direction_multiplier = 1
        self.say_pub = rospy.Publisher('/say', String, latch=True, queue_size=1)
        #initialising the client for moving arm to neutral position
        try:
            self.move_arm_client = actionlib.SimpleActionClient("move_arm_server", MoveArmAction)
            rospy.loginfo('[pickup] Waiting for %s server', "move_arm_server")
            self.move_arm_client.wait_for_server()
        except Exception as exc:
            rospy.logerr('[pickup] %s server does not seem to respond: %s',
                        "move_arm_server", str(exc))
    
    def get_force_feedback(self, msg):
        if msg.data and self.speak:
            self.direction_multiplier = 1
            rospy.logerr('[door-open]Cannot pull. Force feedback exceeds threshold. Trying other direction...')
            self.speak=0
            self.say("Cannot pull. Force feedback exceeds threshold.")

            

    def pick(self):
        #min_height = 0.82 
        #max_height = 0.95
        #pickup_height = random.uniform(min_height, max_height)

        goal = PickupGoal()
        #goal.context = PickupGoal.CONTEXT_MOVING
        goal.pose.header.frame_id = 'base_link'
        goal.pose.header.stamp = rospy.Time.now()

        # goal.pose.pose.position.x = 0.418
        # goal.pose.pose.position.y = 0.078
        # goal.pose.pose.position.z = 0.842
        # goal.pose.pose.position.z = 0.95
        
        # goal.pose.pose.orientation.x = 0.758
        # goal.pose.pose.orientation.y = 0.000
        # goal.pose.pose.orientation.z = 0.652
        # goal.pose.pose.orientation.w = 0.000

        goal.pose.pose.position.x = 0.0
        goal.pose.pose.position.y = 0.0
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 0.0

        rospy.loginfo('sending goal')
        self.client_pick.send_goal(goal)
        self.client_pick.wait_for_result()
        rospy.loginfo('got result')
        
        rospy.loginfo(self.client_pick.get_result())
        rospy.sleep(5)

    def say(self, sentence):
        say_msg = String()
        say_msg.data = sentence
        self.say_pub.publish(say_msg)
    
    def movebackwards(self):
        val = np.linspace(0,3,5)
        aligned_base_pose = PoseStamped()
        aligned_base_pose.header.frame_id = 'base_link'
        aligned_base_pose.header.stamp = rospy.Time.now()
        for i in val:
            print(i)
            print(aligned_base_pose.pose.position.y)
            aligned_base_pose.pose.position.x = -round((i/10),2)
            aligned_base_pose.pose.position.y = -round((i/10)**2,2)
            aligned_base_pose.pose.position.z = 0.
            aligned_base_pose.pose.orientation.x = -0.15
            aligned_base_pose.pose.orientation.y = 0.1
            aligned_base_pose.pose.orientation.z = 0.05
            aligned_base_pose.pose.orientation.w = 0.924

            move_base_goal = MoveBaseGoal()
            move_base_goal.goal_type = MoveBaseGoal.POSE
            move_base_goal.pose = aligned_base_pose
            self.move_base_client.send_goal(move_base_goal)
            self.move_base_client.wait_for_result()
            self.move_base_client.get_result()
            print(self.move_base_client.get_result())
            # rospy.sleep(5)


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


        #angles from 0 to -90 degress to achieve the pouring action
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

        # unlatching process

        # first stage
        # angles from -100 to -60 degress to achieve the pouring action
        angles= list(range(-90, -55, 15))
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
        
        # second stage 
        angles= list(range(-55, -45, 15))
        inRadians= np.deg2rad(angles)
        wrist_roll_angles= np.round(inRadians, 2)
        for i in wrist_roll_angles: 
            p.positions= [0.32, -0.42, 0.0, -1.00, i]
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
        while not rospy.is_shutdown():
            self.force_feedback_sub = rospy.Subscriber('force', Bool, self.get_force_feedback)
            rospy.sleep(0.1)
        # now move back a bit
        #self.movebackwards()
        #rospy.loginfo('Moved back a bit')

        
def main():
    pick_pour= pickAndPour()
    # pick_pour.pick()
    # pick_pour.moveSideWays()
    # pick_pour.moveToNeutral()
    pick_pour.one_func()


if __name__== "__main__" :
    main()
