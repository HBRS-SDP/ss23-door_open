#! /usr/bin/env python
from __future__ import print_function

import rospy
import roslib
import actionlib
from geometry_msgs.msg import PoseStamped

from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal




if __name__ == '__main__':
    rospy.init_node('move_arm_client_test')

    client = actionlib.SimpleActionClient('/move_arm_server', MoveArmAction)
    client.wait_for_server()

    goal = MoveArmGoal()
    correct_input = True
    goal.goal_type = MoveArmGoal.END_EFFECTOR_POSE


    pose = PoseStamped()
    pose.header.frame_id = 'base_link'

    pose.pose.position.x = -0.511
    pose.pose.position.y = -0.376
    pose.pose.position.z = 0.893

    #pose.pose.orientation.x = 0.556
    #pose.pose.orientation.y = 0.258
    #pose.pose.orientation.z = 0.716
    #pose.pose.orientation.w = 0.333

    goal.end_effector_pose = pose
    goal.dmp_name = ''
    print(goal)

    if correct_input:
        timeout = 30.0
        rospy.loginfo('Sending action lib goal to move_arm_server')
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
        print(client.get_result())
