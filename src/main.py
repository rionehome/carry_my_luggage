#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Float32
from carry_my_luggage.msg import ArmAction, MoveAction, LidarData, PersonDetect
import time
import sys
from math import pi

STOP_DISTANCE = 1.0 + 0.15 # m
LINEAR_SPEED = 0.15 # m/s
ANGULAR_SPEED = 0.75 # m/s



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
    
    def main(self):
        # wait for nodes
        time.sleep(3)

        global_direction = "forward"
        global_speed = LINEAR_SPEED #対象に合わせて、速度を変える
        global_distance = "normal"
        
        while True:
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
            
            #command select
            c = MoveAction()
            #c.distance = "forward"
            c.time = 0.1
            c.speed = 0.0
            #c.direction = "normal"
            
            if mn < 0.3:#止まる（Turtlebotからの距離が近い）
                if global_direction != "stop":
                    print("I can get close here")
                    self.audio_pub.publish("これ以上近づけません")
                    global_direction = "stop" 
                c.direction = "stop"
                #止まることを最優先するため、初期値で設定している
            elif p_direction == 0:
                if global_direction != "left":
                    print("you are left side so I turn left")
                    self.audio_pub.publish("たーんれふと")
                    global_direction = "left"
                c.direction = "left"
                c.speed = ANGULAR_SPEED
            elif p_direction == 2:
                if global_direction != "right":
                    print("you are right side so I turn right")
                    self.audio_pub.publish("たーんらいと")
                    global_direction = "right"
                c.direction = "right"
                c.speed = ANGULAR_SPEED
            elif p_direction== 1:
                if global_direction != "forward":
                    print("you are good")
                    self.audio_pub.publish("いいね")
                    global_direction = "forward"
                c.direction = "forward"
                if p_distance == 0:
                    if global_distance != "long":
                        self.audio_pub.publish("かくどはいいけどきょりがとおい")
                        print("angle but you have long distance.")
                        global_distance = "long"
                    c.distance = "long"
                    c.speed = global_speed * 1.1
                elif p_distance == 2:
                    if global_distance != "short":
                        self.audio_pub.publish("かくどはいいけどきょりがちかい")
                        print("angle but you have short distance.")
                        global_distance = "short"
                    c.distance = "short"
                    c.speed = global_speed * 1.1
                elif p_distance == 1:
                    if global_distance != "normal":
                        self.audio_pub.publish("かくどもきょりもいいかんじ")
                        print("angle and distance.")
                        global_distance = "normal"
                    c.distance = "normal"
                    c.speed = global_speed
                
            self.move_pub.publish(c)
            
            """exit(0)
            if msg.robo_p_drct == 0:
                c.direction = "left"
                c.speed = ANGULAR_SPEED*0.5
                print("you are left side so I turned left")
            elif msg.robo_p_drct == 1:
                c.direction = "forward"
                c.speed = LINEAR_SPEED*1.5
                print("you are good angle")
            elif msg.robo_p_drct == 2:
                c.direction = "right"
                c.speed = ANGULAR_SPEED*0.5
                print("you are right side so I turn right")
            elif msg.robo_p_dis == 0:
                c.distance = "long"
                c.speed = LINEAR_SPEED * 1.5
                print("you have long distance")
            #距離が普通の場合、今の速度をキープするようにする
            elif msg.robo_p_dis == 1:
                c.distance = "normal"
                c.speed = LINEAR_SPEED
                print("distance is normal!")
            #距離が短い場合、スピードが０の場合、離れるようにする
            #距離が短い場合、スピードがある場合、速度を遅くする
            elif msg.robo_p_dis == 2:
                c.distance = "short"
                c.speed = -LINEAR_SPEED*1.5
                print("you have short distance")
                
            self.move_pub.publish(c)
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
"""
            
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
        
        exit(0)
        # robot arm test
        armAction = ArmAction()
        armAction.joint = [2/(3 * pi), 2/(3*pi), 2/(3*pi), 2/(3*pi)]
        armAction.gripper = "open"
        # armAction.time = 6 # オプショナル。大きな角度を移動する場合に指定
        carryMyLuggage.arm_pub.publish(armAction)
        armAction.joint = [2/(5 * pi), 2/(5*pi), 2/(5*pi), 2/(5*pi)]
        carryMyLuggage.arm_pub.publish(armAction)

            
                
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
        def callback(self, msg):
                    
                #＜とりあえず書いてみてほしいと言われたものを書いてみる＞

                #人の距離（遠い・中くらい・近い）の３段階のデータが送られてくるので、
                #それに対応して、行動するような動きをするもの
                #距離が遠い時に、スピードが０ならば、発進するようにする距離が遠くて、スピードがあるならば、速度を上げるようにする
            
            c = MoveAction()
            c.time = 0.01
            c.speed = 0 #なんとなく値を初期化
            
            if msg.robo_p_drct == 0:
                c.direction = "left"
                c.speed = ANGULAR_SPEED*0.5
                print("you are left side so I turned left")
            elif msg.robo_p_drct == 1:
                c.direction = "forward"
                c.speed = LINEAR_SPEED*1.5
                print("you are good angle")
            elif msg.robo_p_drct == 2:
                c.direction = "right"
                c.speed = ANGULAR_SPEED*0.5
                print("you are right side so I turn right")
            elif msg.robo_p_dis == 0:
                c.distance = "long"
                c.speed = LINEAR_SPEED * 1.5
                print("you have long distance")
            #距離が普通の場合、今の速度をキープするようにする
            elif msg.robo_p_dis == 1:
                c.distance = "normal"
                c.speed = LINEAR_SPEED
                print("distance is normal!")
            #距離が短い場合、スピードが０の場合、離れるようにする
            #距離が短い場合、スピードがある場合、速度を遅くする
            elif msg.robo_p_dis == 2:
                c.distance = "short"
                c.speed = -LINEAR_SPEED*1.5
                print("you have short distance")
                
            self.move_pub.publish(c)
            """


if __name__ == '__main__':
    carryMyLuggage = CarryMyLuggage()
    
    # try:
    carryMyLuggage.main()
        
    # except KeyboardInterrupt: #ctrl+cの時に、行う処理
    #     carryMyLuggage.armfinishmove()
    #     raise