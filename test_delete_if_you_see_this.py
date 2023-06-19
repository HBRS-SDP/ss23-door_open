#!/usr/bin/env python

import sys
import rospy
import actionlib
import random
import numpy as np

import control_msgs.msg
import trajectory_msgs.msg
import controller_manager_msgs.srv
from geometry_msgs.msg import PoseStamped

from mdr_pickup_action.msg import PickupAction, PickupGoal
from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal





class pickAndPour :
    def __init__(self):
        rospy.init_node("pickAndPour")

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
       
        #initialising the client for moving arm to neutral position
        try:
            self.move_arm_client = actionlib.SimpleActionClient("move_arm_server", MoveArmAction)
            rospy.loginfo('[pickup] Waiting for %s server', "move_arm_server")
            self.move_arm_client.wait_for_server()
        except Exception as exc:
            rospy.logerr('[pickup] %s server does not seem to respond: %s',
                        "move_arm_server", str(exc))

    def pick(self):
        #min_height = 0.82 
        #max_height = 0.95
        #pickup_height = random.uniform(min_height, max_height)

        goal = PickupGoal()
        #goal.context = PickupGoal.CONTEXT_MOVING
        goal.pose.header.frame_id = 'base_link'
        goal.pose.header.stamp = rospy.Time.now()

        goal.pose.pose.position.x = 0.418
        goal.pose.pose.position.y = 0.0
        goal.pose.pose.position.z = 0.842
        #goal.pose.pose.position.z = 0.95
        
        goal.pose.pose.orientation.x = 0.758
        goal.pose.pose.orientation.y = 0.000
        goal.pose.pose.orientation.z = 0.652
        goal.pose.pose.orientation.w = 0.000

        rospy.loginfo('sending goal')
        self.client_pick.send_goal(goal)
        self.client_pick.wait_for_result()
        rospy.loginfo('got result')
        
        rospy.loginfo(self.client_pick.get_result())
        rospy.sleep(5)

def main():
    pick_pour= pickAndPour()
#    pick_pour.pour()
   # pick_pour.moveSideWays()
   # pick_pour.moveToNeutral()
    pick_pour.pick()


if __name__== "__main__" :
    door = pickAndPour()
    main()

