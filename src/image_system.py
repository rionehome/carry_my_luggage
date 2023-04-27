#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import cv2
import matplotlib
import rospy
import torch
from std_msgs.msg import String

from carry_my_luggage.srv import HandDirection
from hand_direction import get_direction

WIDTH = 640
HEIGHT = 480


class ImageSystem:
    def __init__(self):
        matplotlib.use("Agg")  # fix weird Gtk 2 error
        rospy.init_node("image_system")

        self.is_person_detect_on = False

        self.person_detect_model = torch.hub.load("ultralytics/yolov5", "yolov5s")

        self.hand_direction_srv = rospy.Service(
            "/image_system/hand_direction", HandDirection, self.hand_direction_callback
        )

        self.person_detect_switch_sub = rospy.Subscriber(
            "/image_system/person_detect/switch", String, self.person_detect_switch_callback
        )

        self.person_detect_direction_pub = rospy.Publisher(
            "/image_system/person_detect/direction", String, queue_size=1
        )

        self.person_detect_distance_pub = rospy.Publisher("/image_system/person_detect/distance", String, queue_size=1)

    def hand_direction_callback(self, msg):
        rospy.loginfo("image_system: detecting hand direction")
        return get_direction(msg.n)

    def person_detect_switch_callback(self, msg):
        if msg.data == "on" and self.is_person_detect_on == False:
            self.is_person_detect_on = True
            self.cap = cv2.VideoCapture(0)
        elif msg.data == "off" and self.is_person_detect_on == True:
            self.is_person_detect_on = False
            self.cap.release()
            cv2.destroyAllWindows()

        if self.is_person_detect_on:
            self.person_detect()

    def person_detect(self):
        ret, img = self.cap.read()  # 画像の大きさを取得するために1度だけ最初によびだす。

        # 人が写っていない前提で初期化する
        robo_p_dis = 3  # ロボットと人との距離感覚
        robo_p_drct = 3  # ロボットと人との方向感覚

        c = 0
        heigh = 0  # カメラから取得した画像の高さを保持
        width = 0  # カメラから取得した画像の幅を保持

        person_count = 0  # 人が写っているかどうかを判定するための変数

        if self.is_person_detect_on:
            ret, img = self.cap.read()
            result = self.person_detect_model(img)

            if c == 0:
                height, width, _ = img.shape[:3]
                print("高さ=" + str(height))
                print("幅=" + str(width))
                c += 1

            # 推論結果を取得
            obj = result.pandas().xyxy[0]

            # 人が写っているかを調べる
            for i in range(len(obj)):
                if obj.name[i] == "person":
                    person_count += 1
                    break

            # 人が一人も写っていないとき
            if person_count == 0:

                # 探す指示を距離3方向3として与える
                robo_p_dis = 3
                robo_p_drct = 3

            # 人が写っているとき
            if person_count == 1:

                # バウンディングボックスの情報を取得
                for i in range(len(obj)):

                    # 0番目の人(オペレータを想定している)について
                    # ロボットから見たときの距離と方向について
                    if obj.name[i] == "person" and i == 0:

                        # 人のときだけ計算することで無駄な計算を削減する。
                        xmin = obj.xmin[i]
                        ymin = obj.ymin[i]
                        xmax = obj.xmax[i]
                        ymax = obj.ymax[i]

                        w = xmax - xmin  # 矩形の幅
                        h = ymax - ymin  # 矩形の高さ
                        c_x = (xmax + xmin) / 2  # 矩形の中心のx座標
                        c_y = (ymax + ymin) / 2  # 矩形の中心のy座標

                        if c_x >= 0 and c_x <= WIDTH * (1 / 3):
                            print("left")
                            self.person_detect_direction_pub.publish("left")
                        elif c_x < WIDTH and c_x <= WIDTH * (2 / 3):
                            print("center")
                            self.person_detect_direction_pub.publish("center")
                        else:
                            print("right")
                            self.person_detect_direction_pub.publish("right")

                        if h >= 450:
                            print("close")
                            self.person_detect_distance_pub.publish("close")
                        elif h > 350 and h < 450:
                            print("middle")
                            self.person_detect_distance_pub.publish("middle")
                        elif h <= 350:
                            print("far")
                            self.person_detect_distance_pub.publish("far")

            person_count = 0  # 判定するための変数を初期化する

            # バウンディングボックスを描画
            result.render()
            cv2.imshow("result", result.ims[0])
            if cv2.waitKey(1) & 0xFF == ord("q"):
                return


if __name__ == "__main__":
    try:
        imageSystem = ImageSystem()
        rospy.spin()

    except KeyboardInterrupt:
        pass
