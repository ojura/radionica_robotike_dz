#!/usr/bin/env python3.8

from math import sqrt, atan2, pow

import rospy

from geometry_msgs.msg import Point, Twist
from turtlesim.msg import Pose


class TurtlesimMouseTracker:
    """
    Tracker which navigates turtle to follow mouse input.
    """

    def __init__(self):
        self.mouse_sub = rospy.Subscriber(
            "/mouse_position", Point, self.mouse_position_callback)
        self.goal_pose = Point()

        self.turtle_pose_sub = rospy.Subscriber(
            "/turtle1/pose", Pose, self.turtle_pose_callback)
        self.pose = Pose()

        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.turtle_windows_coord_in_screen = Point()
        self.turtle_windows_coord_in_screen.x = 29
        self.turtle_windows_coord_in_screen.y = 706

    def turtle_pose_callback(self, msg):
        self.pose = msg

    def mouse_position_callback(self, msg):
        self.goal_pose.x = msg.x - self.turtle_windows_coord_in_screen.x
        # Turtle and screen coordinate systems have oposite y
        self.goal_pose.y = -(msg.y - self.turtle_windows_coord_in_screen.y)

        # Transform pixels to pose values
        self.goal_pose.x = self.goal_pose.x / 500 * 11.088
        self.goal_pose.y = self.goal_pose.y / 500 * 11.088

    def get_distance(self, goal_x, goal_y):
        return sqrt(pow(goal_x - self.pose.x, 2) + (pow(goal_y - self.pose.y, 2)))

    def get_ang_distance(self, goal_x, goal_y):
        return atan2(goal_y - self.pose.y, goal_x - self.pose.x) - self.pose.theta

    def run(self):
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            linear_vel = self.get_distance(self.goal_pose.x, self.goal_pose.y)
            angular_vel = self.get_ang_distance(
                self.goal_pose.x, self.goal_pose.y)

            # Dead zone not tu stutter turtle too much
            if abs(linear_vel) < 0.5:
                linear_vel = 0.0

            # Rounding to -pi, pi interval
            if angular_vel > 3.14:
                angular_vel -= 6.28
            if angular_vel < -3.14:
                angular_vel += 6.28

            # Send the turtle to the destination
            msg = Twist()
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            if abs(angular_vel) > 0.3:
                # Too big angle, it is more efficient to first satisfy angular difference only
                msg.linear.x = 0
                msg.angular.z = angular_vel * 4.0
            else:
                msg.linear.x = linear_vel * 1.0
                msg.angular.z = angular_vel * 2.0

            self.cmd_pub.publish(msg)
            self.rate.sleep()


def main():
    rospy.init_node('turtlesim_mouse_tracker', anonymous=True)
    tutlesim_mouse_tracker = TurtlesimMouseTracker()
    tutlesim_mouse_tracker.run()


if __name__ == '__main__':
    main()
