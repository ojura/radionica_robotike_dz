#!/usr/bin/env python3.8

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('spiral_controller')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# parameters
start_linear_speed = 0
speed_increment = 0.15
angle_speed = 2.0

i = 0

rate = rospy.Rate(1.0)
while not rospy.is_shutdown():
    
    msg = Twist()
    msg.linear.x = start_linear_speed + i
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = angle_speed
    
    print(f"Sending {msg}")
    pub.publish(msg)
    i += speed_increment
    rate.sleep()

"""
Observations
- `start_linear_speed` determines how initial circle will be after spirals start to form
- proportion of `angle_speed` and `speed_increment` determines the density of the lines
"""

