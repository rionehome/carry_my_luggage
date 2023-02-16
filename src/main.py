#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from carry_my_luggage.msg import ArmAction
import time
import sys
from math import pi

class CarryMyLuggage():
    def __init__(self):
        rospy.init_node("main")
        # for robot movement
        self.move_pub = rospy.Publisher("/move", String, queue_size=1)

        # for robot arm manipulation
        self.arm_pub = rospy.Publisher("/arm", ArmAction, queue_size=1)

        # for audio
        self.audio_pub = rospy.Publisher("/audio", String, queue_size=1)
    
    def main(self):
        # wait for nodes
        time.sleep(3)

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