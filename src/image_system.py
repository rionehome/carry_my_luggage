#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import cv2
import matplotlib
import rospy
import torch
from std_msgs.msg import String

from carry_my_luggage.msg import Detect
from carry_my_luggage.srv import HandDirection
from hand_direction import get_direction

WIDTH = 640
HEIGHT = 480


class ImageSystem:
    def __init__(self):
        matplotlib.use("Agg")  # fix weird Gtk 2 error
        rospy.init_node("image_system")

        # hand detection
        self.hand_direction_srv = rospy.Service(
            "/image_system/hand_direction", HandDirection, self.hand_direction_callback
        )

        # person detect
        self.is_person_detect_on = False
        self.person_detect_model = torch.hub.load("ultralytics/yolov5", "yolov5s")
        self.person_detect_switch_sub = rospy.Subscriber(
            "/image_system/person_detect/switch", String, self.person_detect_switch_callback
        )
        self.person_detect_result_pub = rospy.Publisher("/image_system/person_detect/result", Detect, queue_size=1)

        # paperbag detect
        self.is_paperbag_detect_on = False
        self.paperbag_detect_model = torch.hub.load(
            "/home/ri-one/yolov5/",
            "custom",
            path="/home/ri-one/catkin_ws/src/carry_my_luggage/22sbest_pprbg.pt",
            source="local",
            force_reload=True,
        )
        self.person_detect_switch_sub = rospy.Subscriber(
            "/image_system/paperbag_detect/switch", String, self.paperbag_detect_switch_callback
        )
        self.paperbag_detect_paperbag_result_pub = rospy.Publisher(
            "/image_system/paperbag_detect/paperbag/result", Detect, queue_size=1
        )
        self.paperbag_detect_holding_result_pub = rospy.Publisher(
            "/image_system/paperbag_detect/holding/result", Detect, queue_size=1
        )

    def hand_direction_callback(self, msg):
        rospy.loginfo("image_system: Detecting hand direction")

        return get_direction(msg.n)

    def person_detect_switch_callback(self, msg):
        if msg.data == "on" and self.is_person_detect_on == False:
            rospy.loginfo("image_system: Turning on person_detect")

            self.is_person_detect_on = True
            self.cap = cv2.VideoCapture(0)
        elif msg.data == "off" and self.is_person_detect_on == True:
            rospy.loginfo("image_system: Turning off person_detect")

            self.is_person_detect_on = False
            self.cap.release()
            cv2.destroyAllWindows()

        if self.is_person_detect_on:
            self.person_detect()

    def person_detect(self):
        ret, img = self.cap.read()  # 画像の大きさを取得するために1度だけ最初によびだす。

        p_count = 0  # 人が写っているかどうかを判定するための変数
        p_direction = []
        p_distance = []
        p_xmid = []
        p_ymid = []
        p_width = []
        p_height = []

        ret, img = self.cap.read()
        result = self.person_detect_model(img)

        # 推論結果を取得
        obj = result.pandas().xyxy[0]

        # 人が写っているかを調べる
        for i in range(len(obj)):
            if obj.name[i] == "person":
                p_count += 1

                xmin = obj.xmin[i]
                ymin = obj.ymin[i]
                xmax = obj.xmax[i]
                ymax = obj.ymax[i]

                width = xmax - xmin  # 矩形の幅
                height = ymax - ymin  # 矩形の高さ
                xmid = (xmax + xmin) / 2  # 矩形の中心のx座標
                ymid = (ymax + ymin) / 2  # 矩形の中心のy座標

                p_width.append(int(width))
                p_height.append(int(height))
                p_xmid.append(int(xmid))
                p_ymid.append(int(ymid))

                if xmid >= 0 and xmid <= WIDTH * (1 / 3):
                    p_direction.append("left")
                elif xmid < WIDTH and xmid <= WIDTH * (2 / 3):
                    p_direction.append("middle")
                elif xmid > WIDTH * (2 / 3) and xmid < WIDTH:
                    p_direction.append("right")

                if height >= 380:
                    p_distance.append("close")
                elif height > 360 and height < 380:
                    p_distance.append("middle")
                elif height <= 360:
                    p_distance.append("far")

        d = Detect()
        d.count = p_count
        d.direction = p_direction
        d.distance = p_distance
        d.xmid = p_xmid
        d.ymid = p_ymid
        d.width = p_width
        d.height = p_height
        self.person_detect_result_pub.publish(d)

        # バウンディングボックスを描画
        result.render()
        cv2.imshow("result", result.ims[0])
        if cv2.waitKey(1) & 0xFF == ord("q"):
            return

    def paperbag_detect_switch_callback(self, msg):
        if msg.data == "on" and self.is_paperbag_detect_on == False:
            if self.is_person_detect_on:
                self.cap.release()
                cv2.destroyAllWindows()

            rospy.loginfo("image_system: Turning on paperbag_detect")

            self.is_paperbag_detect_on = True
            self.cap = cv2.VideoCapture(2)
        elif msg.data == "off" and self.is_paperbag_detect_on == True:
            rospy.loginfo("image_system: Turning off paperbag_detect")

            self.is_paperbag_detect_on = False
            self.cap.release()
            cv2.destroyAllWindows()

        if self.is_paperbag_detect_on:
            self.paperbag_detect()

    def paperbag_detect(self):
        # p for paperbag
        p_count = 0
        p_direction = []
        p_distance = []
        p_xmid = []
        p_ymid = []
        p_width = []
        p_height = []

        # h for holding
        h_count = 0
        h_direction = []
        h_distance = []
        h_xmid = []
        h_ymid = []
        h_width = []
        h_height = []

        ret, img = self.cap.read()
        result = self.paperbag_detect_model(img)

        # 推論結果を取得
        obj = result.pandas().xyxy[0]

        # 紙袋が写っているかを調べる holding
        for i in range(len(obj)):
            if obj.name[i] == "paper_bag":
                p_count += 1

                xmin = obj.xmin[i]
                ymin = obj.ymin[i]
                xmax = obj.xmax[i]
                ymax = obj.ymax[i]

                width = xmax - xmin  # 矩形の幅
                height = ymax - ymin  # 矩形の高さ
                xmid = (xmin + xmax) / 2
                ymid = (ymin + ymax) / 2

                p_width.append(int(width))
                p_height.append(int(height))
                p_xmid.append(int(xmid))
                p_ymid.append(int(ymid))

                if xmid > 0 and xmid <= WIDTH * (1 / 3):
                    p_direction.append("left")
                elif xmid > WIDTH * (1 / 3) and xmid <= WIDTH * (2 / 3):
                    p_direction.append("middle")
                elif xmid > WIDTH * (2 / 3) and xmid < WIDTH:
                    p_direction.append("right")

            if obj.name[i] == "holding":
                h_count += 1

                xmin = obj.xmin[i]
                ymin = obj.ymin[i]
                xmax = obj.xmax[i]
                ymax = obj.ymax[i]

                width = xmax - xmin  # 矩形の幅
                height = ymax - ymin  # 矩形の高さ
                xmid = (xmin + xmax) / 2
                ymid = (ymin + ymax) / 2

                h_width.append(int(width))
                h_height.append(int(height))
                h_xmid.append(int(xmid))
                h_ymid.append(int(ymid))

                if xmid > 0 and xmid <= WIDTH * (1 / 3):
                    h_direction.append("left")
                elif xmid > WIDTH * (1 / 3) and xmid <= WIDTH * (2 / 3):
                    h_direction.append("middle")
                elif xmid > WIDTH * (2 / 3) and xmid < WIDTH:
                    h_direction.append("right")

        d = Detect()
        d.count = p_count
        d.direction = p_direction
        d.distance = p_distance
        d.xmid = p_xmid
        d.ymid = p_ymid
        d.width = p_width
        d.height = p_height
        self.paperbag_detect_paperbag_result_pub.publish(d)

        d = Detect()
        d.count = h_count
        d.direction = h_direction
        d.distance = h_distance
        d.xmid = h_xmid
        d.ymid = h_ymid
        d.width = h_width
        d.height = h_height
        self.paperbag_detect_holding_result_pub.publish(d)

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
