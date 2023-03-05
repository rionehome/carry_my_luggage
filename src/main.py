#!/usr/bin/env python
# -*- coding: utf-8 -*-

from ast import Str
import rospy
from std_msgs.msg import String, Float32
from carry_my_luggage.msg import MoveAction, LidarData, PersonDetect
from carry_my_luggage.srv import Camera_msg, MoveArm, SpeechToText, isMeaning

import time
import sys
import os
from math import pi
print(sys.version.split()[0])


STOP_DISTANCE = 1.0 + 0.15 # m
LINEAR_SPEED = 0.05 # m/s
ANGULAR_SPEED = 0.75  # m/s

class CarryMyLuggage():
    def __init__(self):
        rospy.init_node("main")
        # for robot movement
        self.move_pub = rospy.Publisher("/move", MoveAction, queue_size=1)

        # for robot arm manipulation
        rospy.wait_for_service("/move_arm")
        # for audio
        self.audio_pub = rospy.Publisher("/audio", String, queue_size=1)

        # for camera
        self.switch_pub = rospy.Publisher("/switch_camera", String, queue_size=1)
        self.finger_sub = rospy.Subscriber("/finger_res", String, self.finger_cb)
        self.finger_res = ""
        # for speechToText

        self.speechToText = rospy.ServiceProxy("/speechToText", SpeechToText )

        # for isMeaning

        self.isMeaning = rospy.ServiceProxy("/isMeaning", isMeaning )
        
        #self.sub = rospy.Subscriber("/person", PersonDetect, self.callback)
    
    def finger_cb(self, message):
        self.finger_res = message.data

    def go_near(self, move_mode="front", approach_distance=0.8, lidar_ignore="no"):
        global_direction = "forward"
        global_linear_speed = LINEAR_SPEED #対象に合わせて、速度を変える
        global_distance = "normal"
        #self.audio_pub.publish("おはよ") #audio.pyを動かす時に、引数として発言させたいものを入れる
        
        #Yolo information
        while True:
            print("go_near Function is runnning")
        
            #(制御)車の前についたタイミングの話
            #車の前についたらgo_near()を終了させる
            # rospy.wait_for_service("/speechToText")
            # text = self.speechToText(True, 4, False, True, -1, "")
            # if reach_near_car == True: #Trueのとき、この関数を終了する
            #     return
            
            #lidar information
            lidarData = rospy.wait_for_message('/lidar', LidarData) #lidar.pyから一つのデータが送られてくるまで待つ
            distance = lidarData.distance
            
            print(distance)
            mn = min(distance)
            mn_index = distance.index(mn)
            mx = max(distance)
            mx_index = distance.index(mx)
            print("min:", mn, mn_index)
            print("max", mx, mx_index)
            front_back = lidarData.front_back
            left_right = lidarData.left_right
            
            
            switch = String()
            switch.data = "person"
            self.switch_pub.publish(switch)
            detectData = rospy.wait_for_message('/person', PersonDetect)
            p_direction = detectData.robo_p_drct
            p_distance = detectData.robo_p_dis
            
            #command select
            c = MoveAction()
            c.distance = "forward"
            c.direction = "normal"
            c.time = 0.1
            c.linear_speed = 0.0
            c.angle_speed = 0.0
            
            if lidar_ignore == "ignore_back":
                mn = min(distance[0:5], distance[18:len(distance)])
            
            if mn < approach_distance: #止まる（Turtlebotからの距離が近い）
                if p_distance == "normal" or p_distance == "short":
                    print("I can get close here.")
                    self.audio_pub.publish("これ以上近づけません")
                    time.sleep(2)
                    global_direction = "stop" 
                    c.direction = "stop"
                    c.angle_speed = 0.0
                    c.linear_speed = 0.0
                    c.distance = "stop"
                    self.move_pub.publish(c)
                    #break
                    return
                elif left_right == "right":
                    c.direction = "left"
                else:
                    c.direction = "right"
                c.angle_speed = ANGULAR_SPEED   #@@@@@@ここも変えて<-これを検索する
                global_direction = left_right
                
                if move_mode == "front":
                    if front_back == "back":
                        c.linear_speed = LINEAR_SPEED 
                    else:
                        c.linear_speed = 0.0
                    c.distance = "stop"
                    global_distance = "stop"
                    self.move_pub.publish(c)
                else:
                    if front_back == "front":
                        c.linear_speed = -LINEAR_SPEED
                    else:
                        c.linear_speed = 0.0
                    c.distance = "stop"
                    global_distance = "stop"
                    self.move_pub.publish(c)
                # count_time += 1
                
            # elif global_distance == "stop":
            #     c.distance = "forward"
            #     c.direction = global_direction
            #     c.angle_speed = LINEAR_SPEED #@@@@@@ここも変えて
            #     c.linear_speed = 0.0
            #     self.move_pub.publish(c)
            #     time.sleep(count_time)
            #     global_direction = "forward"
            #     global_distance = "normal"
                
            elif p_direction == 0:
                if global_direction != "left":
                    print("you are left side so I turn left")
                    # self.audio_pub.publish("たーんれふと")
                    global_direction = "left"
                c.direction = "left"
                # c.angle_speed = ANGULAR_SPEED + global_linear_speed * 2 
                c.angle_speed = ANGULAR_SPEED
            elif p_direction == 2:
                if global_direction != "right":
                    print("you are right side so I turn right")
                    # self.audio_pub.publish("たーんらいと")
                    global_direction = "right"
                c.direction = "right"
                # c.angle_speed = ANGULAR_SPEED + global_linear_speed * 2 
                c.angle_speed = ANGULAR_SPEED
            elif p_direction== 1:
                if global_direction != "forward":
                    print("you are good")
                    # self.audio_pub.publish("かくどいいね")
                    global_direction = "forward"
                c.direction = "forward"

            if mn < approach_distance:
                c.linear_speed = 0.0

            elif p_distance == 0:
                if global_distance != "long":
                    self.audio_pub.publish("とおい")
                    print("angle but you have long distance.")
                    global_distance = "long"
                c.distance = "long"
                if global_linear_speed < 0.5:
                    global_linear_speed += 0.02
                # if global_linear_speed < 1:
                #     global_linear_speed += 0.02
                c.linear_speed = global_linear_speed
                print(c.linear_speed)

            elif p_distance == 2:
                if global_distance != "short":
                    self.audio_pub.publish("ちかい")
                    print("angle but you have short distance.")
                    global_distance = "short"
                c.distance = "short"
                global_linear_speed = 0.1
                # if global_linear_speed >= 0.1:
                    # global_linear_speed -= 0.7
                c.linear_speed = 0.05
                print(c.linear_speed)

            elif p_distance == 1:
                if global_distance != "normal":
                    self.audio_pub.publish("よい")
                    print("angle and distance.")
                    global_distance = "normal"
                c.distance = "normal"
                if global_linear_speed >= LINEAR_SPEED:
                    global_linear_speed -= 0.05
                c.linear_speed = global_linear_speed
                print(c.linear_speed)

            print("GLOBAL LINEAR : " + str(global_linear_speed))
            
            if move_mode == "back":
                c.linear_speed *= -1
                c.angle_speed *= -1
            self.move_pub.publish(c)
    
    def main(self): #@←これをまだできていないコードの部分のチェックマークとする
        # wait for nodes
        time.sleep(3)
        self.audio_pub.publish("テスト")
        # rospy.wait_for_service("/isMeaning")
        # # im = isMeaning()
        # res = self.isMeaning("test",["test"])
        # print("RES : " + str(res.res))

        ser = rospy.ServiceProxy("move_arm", MoveArm)
        #/ 初期位置 #/←このマークをarmの行動と426f8c5b8a23b9392a1040456f0db5f194816966 する
        res = ser(0, 0, 0, 0)
        print(res.res)


# OPに近づく（fingerで角度を識別でできる距離まで）
        #人間との距離が近づいた時点で止まる（引数から設定できるようにする  　#approach_distanceを指を認識できる距離に設定しておく
        self.go_near(move_mode="front", approach_distance=0.5) #move_modeは正面をTurtlebotのどちらにするか, approach_distanceは最終的に止まる距離を示す
# OPが指差したカバンを探す
        switch = String()
        switch.data = "finger"
        self.audio_pub.publish("カメラの中に映るように紙袋の場所を指差してください")
        time.sleep(2)
        self.switch_pub.publish(switch)
        detectData = rospy.wait_for_message('/finger_res', String)
        fingerDirection = detectData.data
        time.sleep(2)

        print(fingerDirection)

        self.audio_pub.publish(("右" if fingerDirection == "Right" else "左") +"に向かって紙袋を探します。")
        time.sleep(3)
        switch.data = "realsence"
        self.switch_pub.publish(switch)
        self.audio_pub.publish("カメラを紙袋のカメラに切り替えました。")
        time.sleep(3)

        #@(画像)Subscriberで今カバンを検知できているかを受け取る
        #@(制御)bag_detect.pyで、紙袋を検知しているかを変数bag_existとする.存在したらFalse,存在しなかったらTrueとする


        self.audio_pub.publish("紙袋を探しています。")
        self.audio_pub.publish("かみぶくろをはっけんしました。紙袋に向かって進みます")

        bag_exist = False #test
        while bag_exist:
            m = MoveAction()
            m.time = 0.1
            m.linear_speed = 0.0
            m.distance = "normal"
            m.direction = fingerDirection
            m.angle_speed = -1 * ANGULAR_SPEED #カメラの向きが逆のため、マイナスにする
            self.move_pub.publish(m)
            #@(制御)おそらく、ここでbag_existの状態を更新する

        switch.data = "person"
        self.switch_pub.publish(switch)
        rospy.wait_for_message('/person', PersonDetect)
        time.sleep(1)

        
#カバンの持ち手が持てるように移動する
        #publishする前後と回転の方向を逆にする（カメラの向きに移動するため）(move_mode)
        #@(画像)YoLoでカバンの輪っかに対面する場所に認識させる
        #カバンとの距離を決めておいた距離にする(approach_distance) 
        self.go_near(move_mode="back", approach_distance=1.0)
        self.audio_pub.publish("かばんの場所に到着しました。かばんを掴みます。")#test
        time.sleep(3)



        #@(制御)lidarの認識点として、Turtlebotの後側の認識点をやめる（カバンを壁として認識することの無いようにするため）
        
        
# カバンをつかむ
        #原先輩のプログラムを参考に、armを動かすプログラムを適応させる
        #/ じゅんびして〜
        res = ser(23, 5, 10, 4)
        print(res.res)

        #/ のばしてぇーの〜
        res = ser(35, 7, 10, 4)
        print(res.res)

        #/ とじるっ！
        res = ser(35, 7, 10, 2)
        print(res.res)

        #/ ひっかける！
        res = ser(37, 7, 30, 2)
        print(res.res)

        #/ もちあげてからの〜
        res = ser(30, 20, 30, 2)
        print(res.res)

        self.audio_pub.publish("カバンをつかみました。オペレーターに向かって進みます。")
        time.sleep(3)
        
# OPに向かって進む（アリーナの外に出るため壁が近くてもぶつから内容に移動できなければならない）
        self.go_near("front", 0.8, "ignore-back")#この時点では人間を追いかけ続ける必要がある
        
        
# OPが車に向かって移動するので、ついていく
        self.audio_pub.publish("くるまのまえについたらおしえてください")    





        self.audio_pub.publish("車の前についたことを確認できたのでかばんを渡します。")




# カバンを渡す
        #原先輩のプログラムを参考に、armを動かすプログラムを適応させる
        # おろす
        res = ser(35, 7, 10, 2)
        print(res.res)

        # ほんではなす
        res = ser(35, 7, 10, 4)
        print(res.res)

        # 初期位置
        res = ser(0, 0, 0, 0)
        print(res.res)
        
        self.audio_pub.publish("かばんをわたしました")#test

# スタート位置に戻る
        reach_near_car = False #Falseに戻さないとgo_near関数はすぐに実行終了する
        #@(画像)(制御)スタート位置までの目印等
        self.audio_pub.publish("すたーとちにもどります")#test
        
# プログラムを終了する
        m = MoveAction()
        m.time = 0.1
        m.angle_speed = 0.0
        m.linear_speed = 0.0
        m.direction = "forward"
        m.distance = "normal"
        self.move_pub.publish(m)
        self.audio_pub.publish("実行終了しました。")


if __name__ == '__main__':
    carryMyLuggage = CarryMyLuggage()
    carryMyLuggage.main()
    
    
    # MoveArm
    # MoveArm.x: 目標座標のx(cm)
    # MoveArm.y: 目標座標のy(cm)
    # MoveArm.dig: 手先の角度(度)
    # MoveArm.grip: 握り具合0〜4の整数値, 0->close, 4->open
    # 
    # MoveArm.res: rosserviceの戻り値, 0->正常終了, -1->エラー終了