#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import rospy
import cv2
import torch
from carry_my_luggage.msg import PersonDetect
from std_msgs.msg import String
from finger_direction import get_direction
import time


class Camera:
    def __init__(self):
        self.person_pub = rospy.Publisher("/person", PersonDetect, queue_size=1)
        self.finger_pub = rospy.Publisher("/finger_res", String, queue_size=1)
        self.sub = rospy.Subscriber("/switch_camera", String, self.cb)
        self.switch = "person"

    def cb(self, message):
        self.switch = message.data

    def person_detect(self):
        model = torch.hub.load("ultralytics/yolov5", "yolov5s")

        cap = cv2.VideoCapture(0)

        # 人が写っていない前提で初期化する
        robo_p_dis = 3  # ロボットと人と の距離感覚
        robo_p_drct = 3  # ロボットと人との方向感覚
        c = 0
        heigh = 0  # カメラから取得した画像の高さを保持
        width = 0  # カメラから取得した画像の幅を保持

        person_count = 0  # 人が写っているかどうかを判定するための変数

        while True:
            # cap = cv2.VideoCapture(0)
            if self.switch == "finger":
                cap.release()
                # cv2.destroyAllWindows()
                res = String()
                res.data = get_direction(10)
                self.finger_pub.publish(res)
                return

            elif self.switch == "realsence":
                return
            elif cap == None:
                cap = cv2.VideoCapture(0)

            ret, img = cap.read()
            result = model(img)

            if c == 0:
                height, width, _ = img.shape[:3]
                # print("高さ=" + str(height))
                # print("幅=" + str(width ))
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
                        name = obj.name[i]
                        xmin = obj.xmin[i]
                        ymin = obj.ymin[i]
                        xmax = obj.xmax[i]
                        ymax = obj.ymax[i]

                        # print("name =", name, "xmin =", xmin, "ymin =", ymin, "xmax =", ymax, "ymin =", ymax)

                        w = xmax - xmin  # 矩形の幅
                        h = ymax - ymin  # 矩形の高さ
                        c_x = (xmax + xmin) / 2  # 矩形の中心のx座標
                        c_y = (ymax + ymin) / 2  # 矩形の中心のy座標

                        if w < 350:
                            # print(str(i) + "番目の人が遠い")
                            robo_p_dis = 0  # ロボットは人が中央に来るまで前に進む

                        elif w >= 350 and w <= 380:
                            # print(str(i) + "番目の人が中央の距離")
                            robo_p_dis = 1  # ロボットはそのまま

                        elif w > 380:
                            # print(str(i) + "番目の人が近い")
                            robo_p_dis = 2  # ロボットは人が中央に来るまで後ろに下がる

                        if c_x < width / 3:
                            # print(str(i) + "番目の人が左にいる")
                            robo_p_drct = 0  # ロボットは人が中央に来るまで左回りする

                        elif c_x > width / 3 and c_x < width * 2 / 3:
                            # print(str(i) + "番目の人が中央の方向")
                            robo_p_drct = 1  # ロボットはそのまま

                        elif c_x > width * 2 / 3:
                            # print(str(i) + "番目の人が右にいる")
                            robo_p_drct = 2  # ロボットは人が中央に来るまで右回りする

                    # print("距離:" + str(robo_p_dis) + "、方向:" + str(robo_p_drct))

                    # print("名前" + str(name) + "幅:" + str(w) + ", 高さ:" + str(h))

            # 距離と方向をPublishしてほしい。
            p = PersonDetect()
            p.robo_p_dis = robo_p_dis
            p.robo_p_drct = robo_p_drct  # 改良してから変更する
            p.p_exist = person_count
            self.person_pub.publish(p)
            # 距離と方向をPublishした

            person_count = 0  # 判定するための変数を初期化する

            # バウンディングボックスを描画
            result.render()
            cv2.imshow("result", result.ims[0])

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break


if __name__ == "__main__":
    rospy.init_node("person")
    camera = Camera()
    while not rospy.is_shutdown():
        camera.person_detect()
        rospy.Rate(10).sleep()
