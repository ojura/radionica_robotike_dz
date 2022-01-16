#!/usr/bin/env python3

from pynput.mouse import Controller

import rospy
from geometry_msgs.msg import Point

class MousePositionPublisher:

    def __init__(self):
        self.position_pub = rospy.Publisher("mouse_position", Point, queue_size=1)
        self.position_msg = Point()
        self.mouse = Controller()

    def publish_position(self, unused_event):
        curr_position = self.mouse.position
        self.position_msg.x = curr_position[0]
        self.position_msg.y = curr_position[1]
        self.position_pub.publish(self.position_msg)

    def run(self):
        rospy.Timer(rospy.Duration(1 / 10), self.publish_position)
        rospy.spin()

def main():
  rospy.init_node('mouse_position_publisher', anonymous=True)
  mouse_position_publisher = MousePositionPublisher()
  mouse_position_publisher.run()

if __name__ == '__main__':
    main()
