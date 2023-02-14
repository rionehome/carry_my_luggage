#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

LINEAR_SPEED = 1.5 # m/s
ANGULAR_SPEED = 0.5 # m/s

class Move():
    def __init__(self):
        self.pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)
        self.sub = rospy.Subscriber("/move", String, self.callback)
    
    def callback(self, msg):
        twist = Twist()

        if msg.data == "forward": # go forward
            twist.linear.x = LINEAR_SPEED
            twist.angular.z = 0
        elif msg.data == "right": # turn right
            twist.linear.x = 0
            twist.angular.z = -1 * ANGULAR_SPEED
        elif msg.data == "left": # turn left
            twist.linear.x = 0
            twist.angular.z = ANGULAR_SPEED

        start_time = time.time()

        while time.time() - start_time < 2:
            self.pub.publish(twist)
            rospy.Rate(30).sleep()

if __name__ == '__main__':
    rospy.init_node("move")
    move = Move()
    while not rospy.is_shutdown():
        rospy.Rate(10).sleep()