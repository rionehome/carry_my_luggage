#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time

import numpy as np
import rospy

from carry_my_luggage.msg import ArmAction
from carry_my_luggage.srv import MoveArm


class Take_a_Bag:
    def __init__(self):
        rospy.init_node("shigoto_shimasu")
        self.arm_pub = rospy.Publisher("/arm", ArmAction, queue_size=1)
        self.ser = rospy.Service("/move_arm", MoveArm, self.main)
        rospy.spin()

    def main(self, message):
        # Linkの長さ(固定値)
        L1 = 13
        L2 = 15
        L3 = 15
        # アーム手先座標
        Px = message.x
        Py = message.y
        # L3の角度指定(x軸とのなす角), deg -> rad
        tht = math.radians(message.deg)

        pi = math.pi
        # 付け根の角度
        th0 = 0

        # init
        if message.x == 0 and message.y == 0:
            th1 = -pi / 4
            th2 = pi / 4
            th3 = pi / 4
            grip = 4

            # 以下の設定でアームを動かす
            armAction = ArmAction()
            armAction.joint = [th0, th1, th2, th3]
            armAction.gripper = grip
            self.arm_pub.publish(armAction)
            time.sleep(3)  # アームのデフォルト動作時間が3秒

            return 0

        # gripの設定
        grip = message.grip
        if grip < 0 or grip > 4:
            rospy.loginfo("move_arm: grip is Not 0-4")
            return -1

        # 3link -> 2link
        tPx = Px - L3 * np.cos(tht)
        tPy = Py - L3 * np.sin(tht)

        # 逆運動学で計算
        try:
            c = math.sqrt((tPx * tPx) + (tPy * tPy))
            # への字の解
            th2 = -math.acos((c * c - L1 * L1 - L2 * L2) / (2 * L1 * L2))
            th1 = math.atan2(tPy, tPx) + math.acos((c * c + L1 * L1 - L2 * L2) / (2 * L1 * c))
            if abs(math.pi / 2 - th1) > math.pi / 2:
                # 逆への字の解
                th2 = math.acos((c * c - L1 * L1 - L2 * L2) / (2 * L1 * L2))
                th1 = math.atan2(tPy, tPx) - math.acos((c * c + L1 * L1 - L2 * L2) / (2 * L1 * c))
            th3 = tht - th1 - th2

        except:
            # 主に目標座標がアームの可動域外になった場合
            rospy.loginfo("move_arm: No solution")
            return -1

        # 念のため、順運動学で検算
        x = L1 * math.cos(th1) + L2 * math.cos(th1 + th2) + L3 * math.cos(th1 + th2 + th3)
        y = L1 * math.sin(th1) + L2 * math.sin(th1 + th2) + L3 * math.sin(th1 + th2 + th3)
        if abs(Px - x) > 1.0 or abs(Py - y) > 1.0:
            rospy.loginfo("move_arm: No solution")
            return -1

        # 逆運動学の角度からロボットの角度に変換
        th1 = -th1 + pi / 2
        th2 = -th2 - pi / 2
        th3 = -th3

        # ロボットが安全に動作する角度をオーバー
        if abs(th1) > math.pi / 2 or abs(th2) > math.pi / 2 or abs(th3) > math.pi / 2:
            rospy.loginfo("move_arm: No realistic solution")
            return -1

        # 以下の設定でアームを動かす
        armAction = ArmAction()
        armAction.joint = [th0, th1, th2, th3]
        armAction.gripper = grip
        self.arm_pub.publish(armAction)
        time.sleep(3)  # アームのデフォルト動作時間が3秒

        return 0


if __name__ == "__main__":
    take = Take_a_Bag()
