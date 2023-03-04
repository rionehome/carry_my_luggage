#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Float32
from carry_my_luggage.msg import MoveAction, LidarData, PersonDetect
from carry_my_luggage.srv import Camera_msg, MoveArm, SpeechToText, isMeaning

import time
import sys
import os
from math import pi

STOP_DISTANCE = 1.0 + 0.15 # m
LINEAR_SPEED = 0.15 # m/s
ANGULAR_SPEED = 0.75  # m/s

reach_near_car = False
class CarryMyLuggage():
    def __init__(self):
        rospy.init_node("main")426f8c5b8a23b9392a1040456f0db5f194816966 
        # for robot movement
        self.move_pub = rospy.Publisher("/move", MoveAction, queue_size=1)

        # for robot arm manipulation
        rospy.wait_for_service("/move_arm")
        # for audio
        self.audio_pub = rospy.Publisher("/audio", String, queue_size=1)

        # for camera
        self.camera_ser = rospy.ServiceProxy("/camera", Camera_msg)
        # for speechToText

        self.speechToText = rospy.ServiceProxy("/speechToText", SpeechToText )

        # for isMeaning

        self.isMeaning = rospy.ServiceProxy("/isMeaning", isMeaning )
        
        #self.sub = rospy.Subscriber("/person", PersonDetect, self.callback)
        
    def go_near(self):
        global_direction = "forward"
        global_linear_speed = LINEAR_SPEED #対象に合わせて、速度を変える
        global_angle_speed = ANGULAR_SPEED #これは使いみち無いかも
        global_distance = "normal"
        #self.audio_pub.publish("おはよ") #audio.pyを動かす時に、引数として発言させたいものを入れる
        
        #Yolo information
        while True:
            print("go_near Function is runnning")
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
            detectData = rospy.wait_for_message('/person', PersonDetect)
            p_direction = detectData.robo_p_drct
            p_distance = detectData.robo_p_dis
            
            #command select^
            c = MoveAction()
            c.distance = "forward"
            c.direction = "stop"
            c.distance = "normal"
            c.time = 0.1
            c.linear_speed = 0.0
            c.angle_speed = 0.0
            c.direction = "normal"
            if mn < 1.0:#止まる（Turtlebotからの距離が近い）
                if global_direction != "stop":
                    print("I can get close here")
                    self.audio_pub.publish("これ以上近づけません")
                    time.sleep(2)
                    global_direction = "stop" 
                    break
                c.direction = "stop"
                c.angle_speed = 0.0
                c.linear_speed = 0.0
                c.distance = "long"
                
                
                #止まることを最優先するため、初期値で設定している
            elif p_direction == 0:
                if global_direction != "left":
                    print("you are left side so I turn left")
                    self.audio_pub.publish("たーんれふと")
                    global_direction = "left"
                c.direction = "left"
                c.angle_speed = ANGULAR_SPEED + global_linear_speed * 2 
            elif p_direction == 2:
                if global_direction != "right":
                    print("you are right side so I turn right")
                    self.audio_pub.publish("たーんらいと")
                    global_direction = "right"
                c.direction = "right"
                c.angle_speed = ANGULAR_SPEED + global_linear_speed * 2 
            elif p_direction== 1:
                if global_direction != "forward":
                    print("you are good")
                    self.audio_pub.publish("かくどいいね")
                    global_direction = "forward"
                c.direction = "forward"

            if mn < 1.0:
                c.linear_speed = 0.0

            elif p_distance == 0:
                if global_distance != "long":
                    self.audio_pub.publish("かくどはいいが、きょりがとおい")
                    print("angle but you have long distance.")
                    global_distance = "long"
                c.distance = "long"
                if global_linear_speed < 0.5:
                    global_linear_speed += 0.07
                if global_linear_speed < 1:
                    global_linear_speed += 0.02
                c.linear_speed = global_linear_speed
                print(c.linear_speed)

            elif p_distance == 2:
                if global_distance != "short":
                    self.audio_pub.publish("かくどはいいが、きょりがちかい")
                    print("angle but you have short distance.")
                    global_distance = "short"
                c.distance = "short"
                global_linear_speed = 0.1
                # if global_linear_speed >= 0.1:
                    # global_linear_speed -= 0.7
                c.linear_speed = 0.05
                print(c.linear_speed)
                self.audio_pub.publish("あああああああ")

            elif p_distance == 1:
                if global_distance != "normal":
                    self.audio_pub.publish("かくどもきょりもいいかんじ")
                    print("angle and distance.")
                    global_distance = "normal"
                c.distance = "normal"
                if global_linear_speed >= LINEAR_SPEED:
                    global_linear_speed -= 0.05
                c.linear_speed = global_linear_speed
                print(c.linear_speed)

            print("GLOBAL LINEAR : " + str(global_linear_speed))
            self.move_pub.publish(c)

    
    def main(self): #@←これをまだできていないコードの部分のチェックマークとする
        # wait for nodes
        time.sleep(3)
        self.audio_pub.publish("メインかんすうをじっこうちゅう")
        print("音声を実行しました。")

        ser = rospy.ServiceProxy("move_arm", MoveArm)
        #/ 初期位置 #/←このマークをarmの行動と426f8c5b8a23b9392a1040456f0db5f194816966 する
        res = ser(0, 0, 0, 0)
        print(res.res)

# OPに近づく（fingerで角度を識別でできる距離まで）
        #人間との距離が近づいた時点で止まる（引数から設定できるようにする
        self.go_near(move_mode="front", approach_distance=1.0) #move_modeは正面をTurtlebotのどちらにするか, approach_distanceは最終的に止まる距離を示す

        #approach_distanceを指を認識できる距離に設定しておく
        rospy.wait_for_service("/speechToText")
        text = self.speechToText(True, 4, False, True, -1, "")
        
# OPが指差したカバンを探す
        rospy.wait_for_service("/camera")
        fingerDirection = self.camera_ser("finger", 5)
        print("FINGER DIRECTION : " + fingerDirection.res)

        # fingerDirection =  get_direction(5)
        # print(fingerDirection)
        
        #@(制御)カメラを紙袋を検出する方に切り替える(紙袋を検出して、それに向かって動くpythonファイルを実行する)
        #@(画像)YoLoで紙袋を認識するpythonファイルを作って、仮に紙袋が認識できるとき"ある",できないとき"ない"とするd

        #thorough@(制御)(画像)#test
        yolo_send_message = "ある"#test
        self.audio_pub.publish("かみぶくろみつけた")#test

        while yolo_send_message != "ある":
            m = MoveAction()
            m.time = 0.1
            m.angle_speed = 0.0
            m.linear_speed = 0.0
            m.distance = "normal"
            
            if fingerDirection == "right":
                m.direction = "right"
                m.angle_speed = ANGULAR_SPEED
            elif fingerDirection == "left":
                m.direction = "left"
                m.angle_speed = ANGULAR_SPEED
                
            self.move_pub.publish(m)
        
        
#カバンの持ち手が持てるように移動する
        #publishする前後と回転の方向を逆にする（カメラの向きに移動するため）(move_mode)
        #@(画像)YoLoでカバンの輪っかに対面する場所に認識させる
        #カバンとの距離を決めておいた距離にする(approach_distance) 
        self.go_near(move_mode="back", approach_distance=1.0)
        
        self.audio_pub.publish("かばんへうごいた")#test

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

        self.audio_pub.publish("きゃっちばっぐ")
        
# OPに向かって進む（アリーナの外に出るため壁が近くてもぶつから内容に移動できなければならない）
        self.go_near()#この時点では人間を追いかけ続ける必要がある
        
        
# OPが車に向かって移動するので、ついていく
        self.audio_pub.publish("くるまのまえについたらおしえてください")
        #車の前についたらgo_near()を終了させる
        reach_near_car = True

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
        
        self.audio_pub.publish("かばんをわたした")#test

# スタート位置に戻る
        reach_near_car = False #Falseに戻さないとgo_near関数はすぐに実行終了する
        #@(画像)(制御)スタート位置までの目印等
        self.audio_pub.publish("すたーとちにもどる")#test
        
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