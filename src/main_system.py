#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from carry_my_luggage.msg import Detect
from carry_my_luggage.srv import HandDirection, IsMeaning, SpeechToText, TextToSpeech

WIDTH = 640
HEIGHT = 480


class MainSystem:
    def __init__(self):
        rospy.init_node("main_system")

        # control
        self.control_vel_pub = rospy.Publisher("/control_system/cmd_vel", Twist, queue_size=1)

        # image
        self.person_detect_distance = "far"
        self.person_detect_direction = "middle"

        # hand direction
        self.image_hand_detect_client = rospy.ServiceProxy("/image_system/hand_direction", HandDirection)

        # person_detect
        self.image_person_detect_switch_pub = rospy.Publisher(
            "/image_system/person_detect/switch", String, queue_size=1
        )
        rospy.Subscriber("/image_system/person_detect/result", Detect, self.image_person_detect_result_callback)
        self.person_detect_count = 0
        self.person_detect_direction = []
        self.person_detect_distance = []
        self.person_detect_xmid = []
        self.person_detect_ymid = []
        self.person_detect_width = []
        self.person_detect_height = []

        # paperbag detect
        # paperbag
        self.image_paperbag_detect_switch_pub = rospy.Publisher(
            "/image_system/paperbag_detect/switch", String, queue_size=1
        )
        rospy.Subscriber(
            "/image_system/paperbag_detect/paperbag/result", Detect, self.image_paperbag_detect_paperbag_result_callback
        )
        self.paperbag_count = 0
        self.paperbag_direction = []
        self.paperbag_distance = []
        self.paperbag_detect_xmid = []
        self.paperbag_detect_ymid = []
        self.paperbag_detect_width = []
        self.paperbag_detect_height = []

        # holding
        rospy.Subscriber(
            "/image_system/paperbag_detect/holding/result", Detect, self.image_paperbag_detect_holding_result_callback
        )
        self.holding_count = 0
        self.holding_direction = []
        self.holding_distance = []
        self.holding_detect_xmid = []
        self.holding_detect_ymid = []
        self.holding_detect_width = []
        self.holding_detect_height = []

        # audio
        self.audio_tts_client = rospy.ServiceProxy("/audio_system/text_to_speech", TextToSpeech)
        self.audio_stt_client = rospy.ServiceProxy("/audio_system/speech_to_text", SpeechToText)
        self.audio_is_meaning_client = rospy.ServiceProxy("/audio_system/is_meaning", IsMeaning)

    def main(self):
        rospy.loginfo("carry_my_luggage start!")
        rospy.loginfo("Sleeping for 6 seconds")
        rospy.loginfo("Please wait...")

        time.sleep(3)
        self.audio_tts_client("ちょっとまっててね")
        time.sleep(3)

        self.audio_tts_client("どちらの紙袋を取りたいか指で指してください")

        i = self.image_hand_detect_client(90)
        i = i.direction

        # 返ってくるiの値は指を指した人から見た方向
        # ロボットから見た方向を発話する
        if i == "right":
            self.audio_tts_client("左ですね")
            self.audio_tts_client("左の紙袋を掴みます")
        elif i == "left":
            self.audio_tts_client("右ですね")
            self.audio_tts_client("右の紙袋を掴みます")

        while True:
            self.image_paperbag_detect_switch_pub.publish("on")

        while True:
            t = Twist()
            self.image_person_detect_switch_pub.publish("on")

            count = self.person_detect_count

            if count == 1:

                direction = self.person_detect_direction[0]
                height = self.person_detect_height[0]
                xmid = self.person_detect_xmid[0]

                # if direction == "right":
                #     rospy.loginfo("right")
                #     t.angular.z = - 0.7
                # elif direction == "middle":
                #     rospy.loginfo("middle")
                # elif direction == "left":
                #     rospy.loginfo("left")
                #     t.angular.z = 0.7

                if xmid > (WIDTH / 2) - 20 and xmid < (WIDTH / 2) + 20:
                    pass
                elif xmid < WIDTH / 2:
                    t.angular.z = 0.7
                elif xmid > WIDTH / 2:
                    t.angular.z = -0.7

                if height >= 460:
                    t.linear.x = 0
                elif height < 460 and height >= 410:
                    t.linear.x = 0.1
                elif height < 410:
                    t.linear.x = 0.15
            elif count > 1:
                max_height = max(self.person_detect_height)
                index = self.person_detect_height.index(max_height)

                height = self.person_detect_height[index]
                xmid = self.person_detect_xmid[index]

                if xmid > (WIDTH / 2) - 20 and xmid < (WIDTH / 2) + 20:
                    pass
                elif xmid < WIDTH / 2:
                    t.angular.z = 0.7
                elif xmid > WIDTH / 2:
                    t.angular.z = -0.7

                if height >= 460:
                    t.linear.x = 0
                elif height < 460 and height >= 410:
                    t.linear.x = 0.1
                elif height < 410:
                    t.linear.x = 0.15
            else:
                t.linear.x = 0
                t.angular.z = 0

            self.control_vel_pub.publish(t)

            rospy.Rate(10).sleep()
            # i = self.person_detect_distance
            # j = self.person_detect_direction

            # t = Twist()

            # if i != "close":
            #     t.linear.x = 0.1
            # else:
            #     t.linear.x = 0

            # if j == "right":
            #     t.angular.z = - 0.6
            # elif j == "left":
            #     t.angular.z = 0.6

            # self.control_vel_pub.publish(t)
            # rospy.Rate(10).sleep

        self.audio_tts_client("どちらに紙袋があるか指で示してください")

        i = self.image_hand_detect_client(90)
        i = i.direction

        # OPから見た方向
        if i == "Right":
            self.audio_tts_client("右ですね")
        elif i == "Left":
            self.audio_tts_client("左ですね")

        rospy.loginfo("carry_my_luggage finish!")

    def image_person_detect_distance_callback(self, msg):
        self.person_detect_distance = msg.data

    def image_person_detect_direction_callback(self, msg):
        self.person_detect_direction = msg.data

    def image_person_detect_result_callback(self, msg):
        self.person_detect_count = msg.count
        self.person_detect_direction = msg.direction
        self.person_detect_distance = msg.distance
        self.person_detect_xmid = msg.xmid
        self.person_detect_ymid = msg.ymid
        self.person_detect_width = msg.width
        self.person_detect_height = msg.height

    def image_paperbag_detect_paperbag_result_callback(self, msg):
        self.paperbag_count = msg.count
        self.paperbag_direction = msg.direction
        self.paperbag_distance = msg.distance
        self.paperbag_detect_xmid = msg.xmid
        self.paperbag_detect_ymid = msg.ymid
        self.paperbag_detect_width = msg.width
        self.paperbag_detect_height = msg.height

    def image_paperbag_detect_holding_result_callback(self, msg):
        self.holding_count = msg.count
        self.holding_direction = msg.direction
        self.holding_distance = msg.distance
        self.holding_detect_xmid = msg.xmid
        self.holding_detect_ymid = msg.ymid
        self.holding_detect_width = msg.width
        self.holding_detect_height = msg.height


if __name__ == "__main__":
    try:
        mainSystem = MainSystem()
        mainSystem.main()

    except KeyboardInterrupt:
        pass
