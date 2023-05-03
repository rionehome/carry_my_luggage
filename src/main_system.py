#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from carry_my_luggage.msg import Detect
from carry_my_luggage.srv import HandDirection, IsMeaning, MoveArm, SpeechToText, TextToSpeech

WIDTH = 640
HEIGHT = 480


class MainSystem:
    def __init__(self):
        rospy.init_node("main_system")

        # control
        self.control_vel_pub = rospy.Publisher("/control_system/cmd_vel", Twist, queue_size=1)
        self.control_arm = rospy.ServiceProxy("/move_arm", MoveArm)

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
        self.person_count = 0
        self.person_direction = []
        self.person_distance = []
        self.person_xmid = []
        self.person_ymid = []
        self.person_width = []
        self.person_height = []

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
        self.paperbag_xmid = []
        self.paperbag_ymid = []
        self.paperbag_width = []
        self.paperbag_height = []

        # holding
        rospy.Subscriber(
            "/image_system/paperbag_detect/holding/result", Detect, self.image_paperbag_detect_holding_result_callback
        )
        self.holding_count = 0
        self.holding_direction = []
        self.holding_distance = []
        self.holding_xmid = []
        self.holding_ymid = []
        self.holding_width = []
        self.holding_height = []

        # audio
        self.audio_tts_client = rospy.ServiceProxy("/audio_system/text_to_speech", TextToSpeech)
        self.audio_stt_client = rospy.ServiceProxy("/audio_system/speech_to_text", SpeechToText)
        self.audio_is_meaning_client = rospy.ServiceProxy("/audio_system/is_meaning", IsMeaning)

    def main(self):
        rospy.loginfo("carry_my_luggage start!")
        rospy.loginfo("Sleeping for 6 seconds")
        rospy.loginfo("Please wait...")

        time.sleep(3)
        self.audio_tts_client("Please wait for seconds")
        time.sleep(3)

        rospy.wait_for_service("/move_arm")

        self.audio_tts_client("Please point which paperbag you want to pick")

        i = self.image_hand_detect_client(60)
        direction = i.direction

        # ロボットから見た方向を発話する
        if direction == "left":
            self.audio_tts_client("You chose left")
            self.audio_tts_client("I will grab left paperbag")
        elif direction == "right":
            self.audio_tts_client("You chose right")
            self.audio_tts_client("I will grab right paperbag")

        did_turn = False
        paperbag_timer = 0

        while True:
            t = Twist()

            self.image_paperbag_detect_switch_pub.publish("on")

            p_count = self.paperbag_count
            p_width = self.paperbag_width
            p_direction = self.paperbag_direction
            h_count = self.holding_count
            h_direction = self.holding_direction
            h_width = self.holding_width

            if h_count >= 2 and direction in h_direction and did_turn == False:
                if direction == "right":
                    t.angular.z = -0.6
                elif direction == "left":
                    t.angular.z = 0.6
                else:
                    t.angular.z = 0
                    did_turn = True

                self.control_vel_pub.publish(t)
            elif p_count >= 2:
                max_width = 0
                for i in range(p_count):
                    if max_width < p_width[i]:
                        max_width = p_width[i]
                        direction = p_direction[i]

                if max_width > 450:
                    t.linear.x = 0
                else:
                    t.linear.x = 0.12

                self.control_vel_pub.publish(t)

            elif p_count == 1:
                max_width = p_width[0]
                d = p_direction[0]

                if max_width > 450:
                    t.linear.x = 0
                else:
                    t.linear.x = 0.12

                if d == "right":
                    t.angular.z = -0.7
                elif d == "left":
                    t.angular.z = 0.7
                else:
                    t.angular.z = 0

                if max_width > 450 and d == "middle":
                    if paperbag_timer % 180:
                        self.image_paperbag_detect_switch_pub.publish("off")
                        break

                    paperbag_timer += 1

                self.control_vel_pub.publish(t)

                # rospy.loginfo(max_width)

        # 初期位置
        self.control_arm(0, 0, 0, 0)
        # じゅんびして〜
        self.control_arm(23, 5, 10, 4)
        # のばしてぇーの〜
        self.control_arm(35, 7, 10, 4)
        # とじるっ！
        self.control_arm(35, 7, 10, 2)
        # ひっかける！
        self.control_arm(37, 7, 30, 2)
        # もちあげてからの〜
        self.control_arm(30, 20, 30, 2)

        # wait for hooking paperbag manually
        time.sleep(10)

        persondetect_timer = 0

        while True:
            t = Twist()
            self.image_person_detect_switch_pub.publish("on")

            count = self.person_count

            if count == 1:

                direction = self.person_direction[0]
                height = self.person_height[0]
                xmid = self.person_xmid[0]

                # if direction == "right":
                #     rospy.loginfo("right")
                #     t.angular.z = - 0.7
                # elif direction == "middle":
                #     rospy.loginfo("middle")
                # elif direction == "left":
                #     rospy.loginfo("left")
                #     t.angular.z = 0.7

                if xmid > (WIDTH / 2) - 15 and xmid < (WIDTH / 2) + 15:
                    pass
                elif xmid < WIDTH / 2:
                    t.angular.z = 0.5
                elif xmid > WIDTH / 2:
                    t.angular.z = -0.5

                if height >= 460:
                    t.linear.x = 0
                elif height < 460 and height >= 410:
                    t.linear.x = 0.06
                elif height < 410:
                    t.linear.x = 0.12

            elif count > 1:
                max_height = max(self.person_height)
                index = self.person_height.index(max_height)

                height = self.person_height[index]
                xmid = self.person_xmid[index]

                if xmid > (WIDTH / 2) - 20 and xmid < (WIDTH / 2) + 20:
                    pass
                elif xmid < WIDTH / 2:
                    t.angular.z = 0.7
                elif xmid > WIDTH / 2:
                    t.angular.z = -0.7

                if height >= 400:
                    t.linear.x = 0
                elif height < 400 and height >= 350:
                    t.linear.x = 0.1
                elif height < 350:
                    t.linear.x = 0.15

                if xmid > (WIDTH / 2) - 20 and xmid < (WIDTH / 2) + 20 and height >= 400:
                    if persondetect_timer % 180 == 0:
                        self.audio_tts_client("Did you arrive at parking?")
                        i = self.audio_stt_client()
                        if i.res == "yes" or i.res == "Yes":
                            break

                        persondetect_timer = 0

                    persondetect_timer += 1
            else:
                t.linear.x = 0
                t.angular.z = 0

            self.control_vel_pub.publish(t)

            rospy.Rate(10).sleep()

        rospy.loginfo("carry_my_luggage finish!")

    def image_person_detect_distance_callback(self, msg):
        self.person_detect_distance = msg.data

    def image_person_detect_direction_callback(self, msg):
        self.person_detect_direction = msg.data

    def image_person_detect_result_callback(self, msg):
        self.person_count = msg.count
        self.person_direction = msg.direction
        self.person_distance = msg.distance
        self.person_xmid = msg.xmid
        self.person_ymid = msg.ymid
        self.person_width = msg.width
        self.person_height = msg.height

    def image_paperbag_detect_paperbag_result_callback(self, msg):
        self.paperbag_count = msg.count
        self.paperbag_direction = msg.direction
        self.paperbag_distance = msg.distance
        self.paperbag_xmid = msg.xmid
        self.paperbag_ymid = msg.ymid
        self.paperbag_width = msg.width
        self.paperbag_height = msg.height

    def image_paperbag_detect_holding_result_callback(self, msg):
        self.holding_count = msg.count
        self.holding_direction = msg.direction
        self.holding_distance = msg.distance
        self.holding_xmid = msg.xmid
        self.holding_ymid = msg.ymid
        self.holding_width = msg.width
        self.holding_height = msg.height


if __name__ == "__main__":
    try:
        mainSystem = MainSystem()
        mainSystem.main()

    except KeyboardInterrupt:
        pass
