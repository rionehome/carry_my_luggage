#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Float32
from carry_my_luggage.msg import ArmAction, MoveAction, LidarData
import time
import sys
from math import pi

STOP_DISTANCE = 1.0 + 0.15 # m
LINEAR_SPEED = 0.15 # m/s
ANGULAR_SPEED = 0.75 # m/s

class CarryMyLuggage():
    def __init__(self):
        rospy.init_node("main")
        # for robot movement
        self.move_pub = rospy.Publisher("/move", MoveAction, queue_size=1)

        # for robot arm manipulation
        self.arm_pub = rospy.Publisher("/arm", ArmAction, queue_size=1)

        # for audio
        self.audio_pub = rospy.Publisher("/audio", String, queue_size=1)
    
    def main(self):
        # wait for nodes
        time.sleep(3)

        # lidar test
        while True:
            lidarData = rospy.wait_for_message('/lidar', LidarData)
            distance = lidarData.distance
            print(distance)
            mn = min(distance)
            mn_index = distance.index(mn)
            mx = max(distance)
            mx_index = distance.index(mx)
            print("min:", mn, mn_index)
            print("max", mx, mx_index)

            if mn_index > 1 and mn_index < 11:
                m = MoveAction()
                m.direction = "left"
                m.speed = ANGULAR_SPEED
                m.time = 0.1
                self.move_pub.publish(m)
                print("you are atleft")
            elif mn_index > 11 and mn_index <= 21:
                m = MoveAction()
                m.direction = "right"
                m.speed = ANGULAR_SPEED
                m.time = 0.1
                self.move_pub.publish(m)
                print("you are at right")
            elif mn > 0.8:
                m = MoveAction()
                m.direction = "forward"
                m.speed = LINEAR_SPEED
                m.time = 0.1
                self.move_pub.publish(m)
                print("please don't leave me~~~")

            
        exit(0)
        while True:
            distance = rospy.wait_for_message('/lidar', Float32)

            if distance.data > STOP_DISTANCE and distance.data < STOP_DISTANCE * 1.5:
                m = MoveAction()
                m.direction = "forward"
                m.speed = LINEAR_SPEED / 2
                m.time = 0.1
                self.move_pub.publish(m)
            elif distance.data > STOP_DISTANCE:
                m = MoveAction()
                m.direction = "forward"
                m.speed = LINEAR_SPEED
                m.time = 0.1
                self.move_pub.publish(m)
            else:
                break

        exit(0)

        # audio test
        self.audio_pub.publish("テスト")

        # robot arm test
        armAction = ArmAction()
        armAction.joint = [0, pi / 4, 0, 0]
        armAction.gripper = "open"
        # armAction.time = 6 # オプショナル。大きな角度を移動する場合に指定
        self.arm_pub.publish(armAction)

        time.sleep(3)
        sys.exit(0)

if __name__ == '__main__':
    carryMyLuggage = CarryMyLuggage()
    carryMyLuggage.main()