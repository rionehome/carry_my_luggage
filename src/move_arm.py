#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from carry_my_luggage.msg import ArmAction
from carry_my_luggage.srv import MoveArm
import numpy as np
from math import pi, radians, sqrt
import time
import traceback

class Take_a_Bag():
    def __init__(self):
        rospy.init_node("shigoto_shimasu")
        self.arm_pub = rospy.Publisher("/arm", ArmAction, queue_size=1)
        self.ser = rospy.Service("/move_arm", MoveArm, self.main)
        rospy.spin()
    
    def main(self, message):
        #Linkの長さ 
        L1 = 13
        L2 = 15
        L3 = 15
        #アーム手先座標
        Px = message.x
        Py = message.y
        # L3の角度指定。(x軸とのなす角)
        tht = radians(message.deg)
        
        grip = message.grip
        if grip < 0 or grip > 4:
            rospy.loginfo("gripper: Not 0-4")
            return -1
        
        Px = Px - L3*np.cos(tht)
        Py = Py - L3*np.sin(tht)
        try:
            th1 = np.arccos((Px*Px+Py*Py+L1*L1-L2*L2)/(2*L1*sqrt(Px*Px+Py*Py)))+np.arctan(Py/Px)
            th2 = np.arctan((Py-L1*np.sin(th1))/(Px-L1*np.cos(th1)))-th1
            th3 = tht - th1 -th2
        except:
            rospy.loginfo(traceback.print_exc())
            return -1
        
        # 逆運動学の角度からロボットの角度に変換
        th1 = -th1 + pi/2
        th2 = -th2 - pi/2
        th3 = -th3
        
        # 角度オーバー
        if th1 > pi/2 or th1 < -pi/2:
            th1 = None
        if th2 > pi/2 or th2 < -pi/2:
            th2 = None
        if th1 == None or th2 == None or th3 == None:
            rospy.loginfo(traceback.print_exc())
            return -1
        
        #init
        if message.x == 0 and message.y == 0:
            th1 = -pi/4
            th2 = pi/4
            th3 = pi/4
            grip = 4
        
        armAction = ArmAction()
        armAction.joint = [0, th1, th2, th3]
        armAction.gripper = grip
        self.arm_pub.publish(armAction)
        time.sleep(3)
        
        return 0
    
    
if __name__ == '__main__':
    take = Take_a_Bag()