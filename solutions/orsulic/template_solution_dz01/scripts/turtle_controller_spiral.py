#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist 

if __name__ == "__main__":
    rospy.init_node('turtle_controller_spiral')

    rate = rospy.Rate(20)
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

    t0 = rospy.Time.now()

    while not rospy.is_shutdown():
        t = (rospy.Time.now() - t0).to_sec()

        current_velocity = Twist()
        current_velocity.linear.x = 2
        current_velocity.angular.z = 0.8

        publisher.publish(current_velocity)
        rate.sleep()