#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from carry_my_luggage.msg import MoveAction
import time

LINEAR_SPEED = 0.15 # m/s
ANGULAR_SPEED = 0.5 # m/s

class Move():
    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
        self.sub = rospy.Subscriber("/move", MoveAction, self.callback)
    
    def callback(self, msg):
        twist = Twist()

        twist.linear.x = msg.linear_speed
        if msg.direction == "left":
            twist.angular.z = msg.angle_speed
        elif msg.direction == "right":
            twist.angular.z = - 1 * msg.angle_speed
        # if msg.direction == "stop":
        #     twist.linear.x = 0
        #     twist.angular.z = 0
        # elif msg.direction == "forward": # go forward
        #     twist.linear.x = msg.speed
        #     twist.angular.z = 0
        # elif msg.direction == "right": # turn right
        #     twist.linear.x = 0
        #     twist.angular.z = -1 * msg.speed
        # elif msg.direction == "left": # turn left
        #     twist.linear.x = 0
        #     twist.angular.z = msg.speed
        # elif msg.distance == "long":
        #     twist.linear.x = msg.speed
        #     twist.angular.z = 0
        # elif msg.distance == "short":
        #     twist.linear.x = -1 * msg.speed
        #     twist.angular.z = 0
        # elif msg.distance == "normal":
        #     twist.linear.x = msg.speed
        #     twist.angular.z = 0
        #normalのときは何もしない
        

        start_time = time.time()
        target_time = msg.time

        while time.time() - start_time < target_time:
            self.pub.publish(twist)
            rospy.Rate(30).sleep()

if __name__ == '__main__':
    rospy.init_node("move")
    move = Move()
    while not rospy.is_shutdown():
        rospy.Rate(10).sleep()