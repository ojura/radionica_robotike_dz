#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Point
from turtlesim.msg import Pose



def calulate_and_publish_turtle_velocity():
    current_velocity = Twist()
    current_velocity.linear.x = 2
    current_velocity.angular.z = 0.8

    publisher.publish(current_velocity)


def callback_turtlesim_pose(turtle_pose_msg):
    global turtle_pose
    turtle_pose = turtle_pose_msg
    calulate_and_publish_turtle_velocity()


def callback_mouse_position(mouse_position_msg):
    global mouse_position
    mouse_position = mouse_position_msg
    calulate_and_publish_turtle_velocity()

if __name__ == "__main__":
    rospy.init_node('kontroler_kornjace')

    rate = rospy.Rate(20)
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    turtle_pose = None
    mouse_position = None
    rospy.Subscriber('mouse_position', Point, callback_position)
    rospy.Subscriber('turtle1/pose', Pose, callback_turtlesim_pose)
    rospy.spin()