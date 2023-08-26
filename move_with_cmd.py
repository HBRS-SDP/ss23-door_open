#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class ForceToVelocityNode:
    def __init__(self):
        rospy.init_node('force_to_velocity_node')
        
        self.linear_velocity = -0.05
        self.angular_velocity = 0.0
        self.linear_velocity_y = 0.0
        self.angular_published = False

        self.force_threshold = -30

        self.sub_force = rospy.Subscriber('/max_force', Float32, self.force_callback)
        self.pub_cmd_vel = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
        rospy.on_shutdown(self.shutdown)

    def force_callback(self, force_msg):
       if force_msg.data < self.force_threshold:
           self.angular_published = True
           print("here")
           #self.angular_velocity = -0.1  # Set angular velocity only once
           self.linear_velocity = 0.0
           self.linear_velocity_y = -0.03
           if force_msg.data < (self.force_threshold - 15):
               self.linear_velocity_y = 0.03
       else:
           print("loop")
           self.angular_velocity = 0.0
           self.linear_velocity_y = 0.0
           self.linear_velocity = -0.05
            
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
