#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from carry_my_luggage.msg import ArmAction
import numpy as np
from numpy import sin,cos
from math import pi
import time

def Lx(l):
    Li = np.matrix((
        ( 1., 0., l),
        ( 0., 1., 0.),
        ( 0., 0., 1.)
    ))
    return Li

def Rz(th):
    R = np.matrix((
        (cos(th), -sin(th), 0.),
        (sin(th),  cos(th), 0.),
        (0., 0., 1.)
    ))
    return R

def dRz(th):
    dR = np.matrix( (
        (-sin(th),-cos(th),0.),
        ( cos(th),-sin(th),0.),
        (0.,0.,0.)
    ))
    return dR

def arm2_ik(X, L, th):
    [x, y] = X
    [l1, l2, l3] = L
    [th1, th2, th3] = th
    X = np.matrix(( ( np.array(x) ), ( np.array(y) )))
    th1 =  np.radians(th1)
    th2 =  np.radians(th2)
    th3 =  np.radians(th3)

    for j in range(100):
        x = np.array([[0.],[0.],[1.]] )
        P = np.matrix((
            (1.,0.,0.),
            (0.,1.,0.),
            ))

        r = P * Rz(th1) * Lx(l1) * Rz(th2) * Lx(l2) *  Rz(th3) * Lx(l3) * x

        J1 = dRz(th1) * Lx(l1) * x
        J2 = Rz(th1) * Lx(l1) * dRz(th2) * Lx(l2) * x
        J3 = Rz(th1) * Lx(l1) * Rz(th2) * Lx(l2) * dRz(th3) * Lx(l3) * x

        JJ = np.c_[J1,J2,J3]
        J = P * JJ
        invJ = J.T * np.linalg.inv(J * J.T)
        
        dx = X.T - r
        th = 0.1 * invJ * dx
    
        th1 = th1 + th[0,0]
        th2 = th2 + th[1,0]
        th3 = th3

    return th1, th2, th3

def arm2_fk(L, th):
    [l1, l2, l3] = L
    [th1, th2, th3] = th
    vec = np.array([[0.],[0.],[1.]] )
    (x1, y1, z1) = Rz(th1) * Lx(l1) * vec
    (x2, y2, z2) = Rz(th1) * Lx(l1) * Rz(th2) * Lx(l2) * vec
    (x3, y3, z3) = Rz(th1) * Lx(l1) * Rz(th2) * Lx(l2) * Rz(th3) * Lx(l3) * vec
    return x1, y1, x2, y2, x3, y3

def vec2argdr(v1, v2):
    v12_cross = np.cross(v1, v2)
    theta = np.arcsin(np.linalg.norm(v12_cross) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
    return np.sign(v12_cross) * theta

def vec_tra(x, y):
    return np.array([float(x), float(y)])

def sentaisho(x_1, y_1, th1):
    A = np.matrix((
        ( np.cos(2 * th1), np.sin(2 * th1)),
        ( np.sin(2 * th1), -np.cos(2 * th1)),
    ))
    v1 = np.array([[x_1],[y_1]])
    v2 = A * v1
    return float(v2[0]), float(v2[1])


#引数 3次元配列:各リンクの長さ[cm]、2次元配列:手先の位置[cm]、3次元配列:初期角度[rad]
def inv_kine_arg(L, X, th_rad):
    th_deg = [np.rad2deg(th_rad[0]), np.rad2deg(th_rad[1]), np.rad2deg(th_rad[2])]

    th = arm2_ik(X, L, th_deg)

    (x1, y1, x2, y2, x3, y3) = arm2_fk(L, th)

    th_2 = float(np.arctan(y3/x3))

    x1_2, y1_2 = sentaisho(float(x1), float(y1), th_2)
    x2_2, y2_2 = sentaisho(float(x2), float(y2), th_2)

    th_list = [np.deg2rad(th[0]), np.deg2rad(th[1]), np.deg2rad(th[2]), 0, 0, 0]

    th_list[3] = vec2argdr(np.array([1, 0]), vec_tra(x1_2, y1_2))
    th_list[4] = vec2argdr(vec_tra(x1_2, y1_2), vec_tra(x2_2, y2_2))
    th_list[5] = vec2argdr(vec_tra(x2_2, y2_2), vec_tra(x3, y3))

    print(th)
    print(th_list[3])
    print(th_list[4])
    print(th_list[5])
    
    return th_list    


class Take_a_Bag():
    def __init__(self):
        rospy.init_node("shigoto_shimasu")
        self.arm_pub = rospy.Publisher("/arm", ArmAction, queue_size=1)
        self.pub = rospy.Publisher("/take_a_bug", String, queue_size=1)
        self.sub = rospy.Subscriber("/take_a_bug", String, self.cb)
        
    
    def cb(self, message):
        if message != "onegai":
            pass
        else:
            self.main()
    
    def main(self):
        # L = [13.0, 13.0, 15.0]
        # X = [10, 10]
        l = [-pi/4, pi/4, pi/4]
        # th_rad = [l[0]+pi/2, -l[1], -l[2]]
        # a = inv_kine_arg(L, X, th_rad)
        # b = [a[3], a[4], a[5]]
        
        armAction = ArmAction()
        # init
        armAction.joint = [0, l[0], l[1], l[2]]
        armAction.gripper = "open"
        self.arm_pub.publish(armAction)
        time.sleep(3)
        
        # l[0] = l[0] + b[0]
        # l[1] = l[1] + b[1]
        # l[2] = l[2] + b[2]
        
        # init
        # armAction.joint = [0, b[0]-pi/4, b[1], b[2]]
        # armAction.gripper = "open"
        # self.arm_pub.publish(armAction)
        # time.sleep(3)
        # exit(0)
        # 準備して〜
        armAction.joint = [0, 0, pi/3, -pi/3]
        #armAction.gripper = "open"
        self.arm_pub.publish(armAction)
        time.sleep(3)
        
        # のばして〜
        armAction.joint = [0, pi/3, -pi/3, -pi/12]
        #armAction.gripper = "open"
        self.arm_pub.publish(armAction)
        time.sleep(3)
        
        # とじるっ！
        armAction.joint = [0, pi/3, -pi/3, -pi/6]
        armAction.gripper = "close"
        self.arm_pub.publish(armAction)
        time.sleep(3)
        
        # とじるっ！
        armAction.joint = [0, pi/4, -pi/4, -pi/6]
        armAction.gripper = "close"
        self.arm_pub.publish(armAction)
        
    
    
if __name__ == '__main__':
    #try:
        take = Take_a_Bag()
        # operation check
        time.sleep(0.5)
        take.main()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
        
    # except:
    #     print("error: take a bag")
    #     pass