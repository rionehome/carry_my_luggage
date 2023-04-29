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

        self.hand_direction_srv = rospy.Service(
            "/image_system/hand_direction", HandDirection, self.hand_direction_callback
        )

        self.is_person_detect_on = False

        self.person_detect_model = torch.hub.load("ultralytics/yolov5", "yolov5s")

        self.person_detect_switch_sub = rospy.Subscriber(
            "/image_system/person_detect/switch", String, self.person_detect_switch_callback
        )

        self.person_detect_direction_pub = rospy.Publisher(
            "/image_system/person_detect/direction", String, queue_size=1
        )

        self.person_detect_distance_pub = rospy.Publisher("/image_system/person_detect/distance", String, queue_size=1)

        self.is_paperbag_detect_on = False

        self.person_detect_switch_sub = rospy.Subscriber(
            "/image_system/paperbag_detect/switch", String, self.paperbag_detect_switch_callback
        )
        self.paperbag_detect_model = torch.hub.load(
            "/home/ri-one/yolov5/",
            "custom",
            path="/home/ri-one/catkin_ws/src/carry_my_luggage/22sbest_pprbg.pt",
            source="local",
            force_reload=True,
        )

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

    def paperbag_detect_switch_callback(self, msg):
        if msg.data == "on" and self.is_paperbag_detect_on == False:
            self.is_paperbag_detect_on = True
            self.cap = cv2.VideoCapture(0)
        elif msg.data == "off" and self.is_paperbag_detect_on == True:
            self.is_paperbag_detect_on = False
            self.cap.release()
            cv2.destroyAllWindows()

        if self.is_paperbag_detect_on:
            self.paperbag_detect()

    def paperbag_detect(self):
        # 人が写っていない前提で初期化する
        robo_p_dis = 3  # ロボットと人との距離感覚
        robo_p_drct = 3  # ロボットと人との方向感覚

        c = 0
        heigh = 0  # カメラから取得した画像の高さを保持
        width = 0  # カメラから取得した画像の幅を保持
        img_c_x = 0  # 画像のx座標の中心位置
        img_c_y = 0  # 画像のy座標の中心位置

        pprbg_count = 0  # メガネが検出されたかどうか
        put_glss = 0  # 最終的にその人がメガネをかけているかを判定する

        state = "select"  # 鞄をつかむまでの状態
        bag_rl = ""  # 左右のどちらのカバンを追いかけたか
        track_pprbg_ID = 0  # 左右の鞄のどちらを追いかけるかのID

        # select 左右の鞄のいずれかを近づく
        # get 鞄の取手を探してつかむ

        while True:
            ret, img = self.cap.read()
            result = self.paperbag_detect_model(img)

            if c == 0:
                height, width, _ = img.shape[:3]
                img_cx = width / 2
                img_cy = height / 2
                print("高さ=" + str(height))
                print("幅=" + str(width))
                c += 1

            # 推論結果を取得
            obj = result.pandas().xyxy[0]

            # 紙袋が写っているかを調べる holding
            for i in range(len(obj)):
                if obj.name[i] == "paper_bag":
                    pprbg_count += 1
                    break

            # 紙袋が写っていないとき何もしない

            # メガネが写っているとき
            if pprbg_count == 1:

                # glss_w_l = [] #検出されたメガネの大きさを保持
                # glss_idx_l = [] #検出されたメガネの添字を保持
                # 紙袋は左右に2つあり最も近いものを追いかける必要性もない

                pprbg_cx_list = []  # 左右のどちらかを追いかけるために使用

                """
                紙袋の左右の判断をする場合のみ select状態を使用
                自動的にロボットの視界から紙袋を自動的に外す場合など場合、get状態を使用
                """

                # 左右のどちらかに近づくとき
                # 鞄が2個とも視界に入っていたら
                if state == "select":

                    # 紙袋の数を数える
                    for i in range(len(obj)):
                        if obj.name[i] == "paper_bag":
                            pprbg_count += 1

                    # 左右に紙袋が2つ写っている状況
                    if pprbg_count == 2:
                        # バウンディングボックスの情報を取得
                        for i in range(len(obj)):

                            # 検出された物体が紙袋のときに
                            if obj.name[i] == "paper_bag":

                                # 写っているときだけ計算することで無駄な計算を削減する。
                                xmin = obj.xmin[i]
                                ymin = obj.ymin[i]
                                xmax = obj.xmax[i]
                                ymax = obj.ymax[i]

                                c_x = (xmin + xmax) / 2  # カバンがカメラから見て真ん中に来ているかを見る

                                pprbg_cx_list.append(c_x)  # 鞄の中心座標を追加する

                        bag_rl = "right"

                        # 左右のどちらかに決まるまで待つ
                        while bag_rl != "right" and bag_rl != "left":
                            bag_rl = ""  # 左右のどちらか決まっていない

                        # ロボットから見て左右のどちらにあるか
                        if bag_rl == "right":
                            track_pprbg_ID = pprbg_cx_list.index(max(pprbg_cx_list))
                        elif bag_rl == "left":
                            track_pprbg_ID = pprbg_cx_list.index(min(pprbg_cx_list))

                        print(str(i + 1) + "番目、" + "、中心x:" + str(c_x))

                    # 紙袋が1つだけになったとき(視界から紙袋が消えた)
                    elif pprbg_count == 1:
                        state = "get"  # とってを掴みにいく

                # つかむ状態のとき視界には鞄は1つだけでないとおかしい
                if state == "get":

                    pprbg_count = 0
                    hl_count = 0

                    # 紙袋の数を数える
                    for i in range(len(obj)):
                        if obj.name[i] == "paper_bag":
                            pprbg_count += 1

                    # 紙袋の取手の数を数える
                    for i in range(len(obj)):
                        if obj.name[i] == "holding":
                            hl_count += 1

                    # 紙袋と取手が1ずつ検出されたとき
                    if pprbg_count == 1 and hl_count == 1:

                        pprbg_w = 0  # 紙袋の幅を取得
                        pprbg_cx = 0  # 紙袋の幅を取得

                        # 紙袋と取手を含むバウンディングボックスの情報を取得
                        for i in range(len(obj)):

                            # 紙袋を引き続き追いかける
                            if obj.name[i] == "paper_bag":

                                # 写っているときだけ計算することで無駄な計算を削減する。
                                xmin = obj.xmin[i]
                                ymin = obj.ymin[i]
                                xmax = obj.xmax[i]
                                ymax = obj.ymax[i]

                                pprbg_w = xmax - xmin
                                pprbg_cx = (xmin + xmax) / 2

                                delta_pprbg_cx = int(pprbg_cx - img_cx)
                                # print("中心からのずれ")
                                # print("delta_pprbg_cx =" + str(delta_pprbg_cx))

                                """
                                微調整をする場合、
                                if pprbg_w > width/2 - 50 and pprbg_w < width/2 + 50:を残す

                                しない場合
                                #紙袋から離れているときに、紙袋に接近する
                                else:
                                    print("紙袋がほどよい距離と方向にあるように調整する。")
                                """
                                # 紙袋に十分近づいているときに、
                                if pprbg_w > width / 2 - 50 and pprbg_w < width / 2 + 50:
                                    print("紙袋が視界の中央に来るように調整する。")

                                    # 鞄が視界の中央に来るように位置調整を行う。
                                    if delta_pprbg_cx < -40:
                                        robo_p_drct = 4  # 微調整用_左
                                        print("左にある")

                                    elif delta_pprbg_cx > 40:
                                        robo_p_drct = 6  # 微調整用_右
                                        print("右にある")

                                    else:
                                        robo_p_drct = 5  # 微調整用_中央
                                        print("中央にある")

                                # 紙袋から離れているときに、紙袋に接近する
                                else:
                                    print("紙袋がほどよい距離と方向にあるように調整する。")

                                    if pprbg_w < width / 2 + 100:
                                        # print(str(i) + "番目の人が遠い")
                                        robo_p_dis = 0  # ロボットは人が中央に来るまで前に進む

                                    elif pprbg_w >= width / 2 - 100 and pprbg_w <= width / 2 + 100:
                                        # print(str(i) + "番目の人が中央の距離")
                                        robo_p_dis = 1  # ロボットはそのまま

                                    elif pprbg_w > width / 2 - 100:
                                        # print(str(i) + "番目の人が近い")
                                        robo_p_dis = 2  # ロボットは人が中央に来るまで後ろに下がる

                                    if pprbg_cx < width / 3:
                                        # print(str(i) + "番目の人が左にいる")
                                        robo_p_drct = 0  # 大規模調整_左

                                    elif pprbg_cx > width / 3 and pprbg_cx < width * 2 / 3:
                                        # print(str(i) + "番目の人が中央の方向")
                                        robo_p_drct = 1  # _大規模調整_中央

                                    elif pprbg_cx > width * 2 / 3:
                                        # print(str(i) + "番目の人が右にいる")
                                        robo_p_drct = 2  # 大規模調整_右

                                    print("robo_p_dis=" + str(robo_p_dis))
                                    print("robo_p_drct=" + str(robo_p_drct))

                print(state)

            pprbg_count = 0  # 紙袋の数を数える変数を初期化

            """
            ここで、距離と方向をPublishしてほしい。
            変数の値と意味
            [robo_p_dis]
            0:遠い、1:中央、2:近い
            3:紙袋が存在しない
            [robo_p_drct]
            0:大きく遠い、1:大まかに中央、2:大きく近い
            3:紙袋が存在しない
            大まかに中央を更に細かく3つに分類
            4:小さく遠い、5:狭く中央、6:小さく近い
            """

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
