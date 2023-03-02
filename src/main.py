#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Float32
from carry_my_luggage.msg import ArmAction, MoveAction, LidarData, PersonDetect
import time
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from math import pi
# from hand_detect.finger_direction import get_direction

STOP_DISTANCE = 1.0 + 0.15 # m
LINEAR_SPEED = 0.15 # m/s
ANGULAR_SPEED = 0.75 # m/s

global_direction = "forward"
global_linear_speed = LINEAR_SPEED #対象に合わせて、速度を変える
global_angle_speed = ANGULAR_SPEED #これは使いみち無いかも
global_distance = "normal"




class CarryMyLuggage():
    def __init__(self):
        rospy.init_node("main")
        # for robot movement
        self.move_pub = rospy.Publisher("/move", MoveAction, queue_size=1)

        # for robot arm manipulation
        self.arm_pub = rospy.Publisher("/arm", ArmAction, queue_size=1)

        # for audio
        self.audio_pub = rospy.Publisher("/audio", String, queue_size=1)
        
        #self.sub = rospy.Subscriber("/person", PersonDetect, self.callback)
        
    def go_near(self):
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
        #self.audio_pub.publish("おはよ") #audio.pyを動かす時に、引数として発言させたいものを入れる
        
        #Yolo information
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
        while True:
            if mn < 0.35:#止まる（Turtlebotからの距離が近い）
                if global_direction != "stop":
                    print("I can get close here")
                    self.audio_pub.publish("これ以上近づけません")
                    global_direction = "stop" 
                    break
                c.direction = "stop"
                c.angle_speed = 0.0
                
                
                #止まることを最優先するため、初期値で設定している
            elif p_direction == 0:
                if global_direction != "left":
                    print("you are left side so I turn left")
                    self.audio_pub.publish("たーんれふと")
                    global_direction = "left"
                c.direction = "left"
                c.angle_speed = ANGULAR_SPEED
            elif p_direction == 2:
                if global_direction != "right":
                    print("you are right side so I turn right")
                    self.audio_pub.publish("たーんらいと")
                    global_direction = "right"
                c.direction = "right"
                c.angle_speed = ANGULAR_SPEED
            elif p_direction== 1:
                if global_direction != "forward":
                    print("you are good")
                    self.audio_pub.publish("かくどいいね")
                    global_direction = "forward"
                c.direction = "forward"

            if mn < 0.35:
                c.linear_speed = 0.0
            elif p_distance == 0:
                if global_distance != "long":
                    self.audio_pub.publish("かくどはいいが、きょりがとおい")
                    print("angle but you have long distance.")
                    global_distance = "long"
                c.distance = "long"
                global_linear_speed = global_linear_speed * 1.25
                c.linear_speed = global_linear_speed
            elif p_distance == 2:
                if global_distance != "short":
                    self.audio_pub.publish("かくどはいいが、きょりがちかい")
                    print("angle but you have short distance.")
                    global_distance = "short"
                c.distance = "short"
                global_linear_speed = global_linear_speed * 1.25
                c.linear_speed = global_linear_speed
            elif p_distance == 1:
                if global_distance != "normal":
                    self.audio_pub.publish("かくどもきょりもいいかんじ")
                    print("angle and distance.")
                    global_distance = "normal"
                c.distance = "normal"
                c.linear_speed = global_linear_speed

        self.move_pub.publish(c)
    
    def main(self):
        # wait for nodes
        time.sleep(3)

        # OPに近づく（fingerで角度を識別でできる距離まで）
        self.go_near()
        #fingerDirection =  get_direction(5)
        #print(fingerDirection)
        
        # OPが指差したカバンを探す
        
        
        # カバンの前まで移動
        
        
        # カバンをつかむ
        
        
        # OPに向かって進む（アリーナの外に出るため壁が近くてもぶつから内容に移動できなければならない）
        
        
        # OPが車に向かって移動するので、ついていく
        

        # カバンを渡す
        
        
        # スタート位置に戻る
        
        
        # プログラムを終了する
        
            
            

            
            
"""
            exit(0)
            if mn_index > 1 and mn_index < 11:
                m = MoveAction()
                m.direction = "left"
                m.speed = ANGULAR_SPEED
                m.time = 0.1
                self.move_pub.publish(m)
                print("you are at left")
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
                
            self.audio_pub.publish("テストしていますよーーー")
            personData = rospy.Subscriber("/person", PersonDetect)

            
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
        
                
    def armfinishmove(self):
        armAction = ArmAction()
        armAction.joint = [2/(3 * pi), 2/(3*pi), 2/(3*pi), 2/(3*pi)]
        armAction.gripper = "open"
        armAction.time = 0
        carryMyLuggage.arm_pub.publish(armAction)
        # armAction.time = 6 # オプショナル。大きな角度を移動する場合に指定
        armAction.joint = [2/(5 * pi), 2/(5*pi), 2/(5*pi), 2/(5*pi)]
        armAction.gripper = "close"
        armAction.time = 6
        carryMyLuggage.arm_pub.publish(armAction)
"""


if __name__ == '__main__':
    carryMyLuggage = CarryMyLuggage()
    
    # try:
    carryMyLuggage.main()
        
    # except KeyboardInterrupt: #ctrl+cの時に、行う処理
    #     carryMyLuggage.armfinishmove()
    #     raise