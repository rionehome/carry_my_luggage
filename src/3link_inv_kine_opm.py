#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#理論(ヤコビ逆行列による数値解の逆運動学)
#HatenaBlog, クワマイでもできる, 数値計算で逆運動学を解く
#https://kuwamai.hatenablog.com/entry/2019/10/05/211232, 2022年12月28日.

#実装(ヤコビ逆行列による数値解の逆運動学)
#アルゴリズム情報, 【Python】2リンクマニピュレータの逆運動学(収束計算), 
#https://algorithm.joho.info/programming/python/inverse-kinematics-convergence-calculation/, 2022年12月28日.

import numpy as np
from numpy import sin,cos
import matplotlib.pyplot as plt
import math
import rospy

class Inv_cac():
    
    def __init__(self):
        self.inv_cac_pub = rospy.Publisher("/inv_cac", Inv_cac, queue_size=1)

        
                
    # 並進行列(x軸方向に並進)
    def Lx(self, l):
        Li = np.matrix((
            ( 1., 0., l),
            ( 0., 1., 0.),
            ( 0., 0., 1.)
        ))
        return Li

    # 回転行列(z軸周りに回転)
    def Rz(self, th):
        R = np.matrix((
            (cos(th), -sin(th), 0.),
            (sin(th),  cos(th), 0.),
            (0., 0., 1.)
        ))
        return R

    # 座標変換行列の微係数(z軸周り)
    def dRz(self, th):
        dR = np.matrix( (
            (-sin(th),-cos(th),0.),
            ( cos(th),-sin(th),0.),
            (0.,0.,0.)
        ))
        return dR

    def graph_option(self):
        fn = "Times New Roman"
        # グラフ表示の設定
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.tick_params(labelsize=13)
        return fn   

    # グラフの描画
    def plot(self, x, y, fn):
        #fn = "Times New Roman"
        # グラフ表示の設定
        #fig = plt.figure()
        #ax = fig.add_subplot(111)
        #ax.tick_params(labelsize=13)                    # 軸のフォントサイズ
        plt.xlabel("$x [m]$", fontsize=20, fontname=fn)
        plt.ylabel("$y [m]$", fontsize=20, fontname=fn)
        plt.plot(x, y,"-g",lw=5,label="link")           # リンクの描画
        plt.plot(x, y,"or",lw=5, ms=10,label="joint")   # 関節の描画
        #plt.xlim(-1.2,1.2)
        #plt.ylim(-1.2,1.2)
        #plt.xlim(-30, 30)
        #plt.ylim(-30, 30)
        plt.grid()
        plt.legend(fontsize=20) # 凡例
        #plt.show()




    def arm2_ik(self, X, L, th):
        #[x, y, theta] = X
        [x, y] = X
        [l1, l2, l3] = L     #[l1, l2] = L
        [th1, th2, th3] = th #[th1, th2] = th
        #X = np.matrix(( ( np.array(x) ), ( np.array(y) ),  ( np.array(theta) )))
        X = np.matrix(( ( np.array(x) ), ( np.array(y) )))
        th1 =  np.radians(th1)  # 仮りの解1
        th2 =  np.radians(th2)  # 仮りの解2
        th3 =  np.radians(th3)  # 仮りの解3 (追加)

        
        # 収束計算を100回繰り返す
        for j in range(100):
            # 原点座標(縦ベクトル)
            x = np.array([[0.],[0.],[1.]] )

            P = np.matrix((
                (1.,0.,0.), #x座標
                (0.,1.,0.), #y座標
                ))

            # 現在の手先位置を求める
            #変数と関数の名前を同じにしないこと! list' object is not callable   print(L(1))
            #この場合では引数Lと関数L()が同じ名前になっている
            #x, y座標について関節を1つずつたどりながら、回転と並進の座標変換を行っている
            #Xg = P * Rz(th1) * Lx(l1) * Rz(th2) * Lx(l2) * x
            r = P * Rz(th1) * Lx(l1) * Rz(th2) * Lx(l2) *  Rz(th3) * Lx(l3) * x #改変

            # ヤコビ行列を求める
            J1 = dRz(th1) * Lx(l1) * x                                       #θ_1について偏微分
            J2 = Rz(th1) * Lx(l1) * dRz(th2) * Lx(l2) * x                    #θ_2について偏微分
            J3 = Rz(th1) * Lx(l1) * Rz(th2) * Lx(l2) * dRz(th3) * Lx(l3) * x #θ_3について偏微分 (追加)

            JJ = np.c_[J1,J2,J3] #JJ = np.c_[J1,J2]  # 3つの列ベクトルを連結する
            J = P * JJ                            #ヤコビ行列
            invJ = J.T * np.linalg.inv(J * J.T)      #ヤコビ行列の逆行列 (疑似行列も対応)
            
            dx = X.T - r             #位置の変位量 (サイトのミス 現在のベクトルを転置すること。)
            th = 0.1 * invJ * dx      #逆運動学の式 0.05は刻み幅

            th1 = th1 + th[0,0]
            th2 = th2 + th[1,0]
            #th3 = th3 + th[2,0] #追加
            th3 = th3

            #print(r)
            
            #print("x座標=" + str(r[0]) + ", y座標=" + str(r[1]))
        return th1, th2, th3 #改変

    # 順運動学の計算
    def arm2_fk(self, L, th):
        [l1, l2, l3] = L     #[l1, l2] = L
        [th1, th2, th3] = th #[th1, th2] = th
        vec = np.array([[0.],[0.],[1.]] )
        (x1, y1, z1) = Rz(th1) * Lx(l1) * vec               # 第1関節の位置
        (x2, y2, z2) = Rz(th1) * Lx(l1) * Rz(th2) * Lx(l2) * vec  # 第2関節の位置
        (x3, y3, z3) = Rz(th1) * Lx(l1) * Rz(th2) * Lx(l2) * Rz(th3) * Lx(l3) * vec  # 第3関節の位置 (追加)
        return x1, y1, x2, y2, x3, y3

    #2つのベクトルのなす角を求める
    #角度の向きが分からない非推奨
    def vec2arg(self, vec1, vec2):
        #2つのベクトルの大きさを求める

        #print("ベクトル1") 
        #print(vec1)

        absvec1=np.linalg.norm(vec1)
        absvec2=np.linalg.norm(vec2)


        inner=np.inner(vec1,vec2) #2つベクトルの内積

        cos_theta=inner/(absvec1*absvec2) #2つのベクトルのcos(theta)
        print("成す角の余弦=")
        print(cos_theta)


        theta=math.acos(cos_theta) #2つのベクトルのなす角(theta)
        print(theta)
        

        return theta
        #print('angle='+str(round(theta,2))+'deg')


    #外積で角度と向きの両方を求める。
    #引数2つの2次元ベクトルnumpy配列型
    def vec2argdr(self, v1, v2):

        v12_cross = np.cross(v1, v2) #2つのベクトルの外積
        #print("符号=" + str(np.sign(v12_cross)))
        theta = np.arcsin(np.linalg.norm(v12_cross) / (np.linalg.norm(v1) * np.linalg.norm(v2))) #角度

        return np.sign(v12_cross) * theta


    #np配列の角括弧を外し、計算できる形に直す。
    def vec_tra(self, x, y):
        return np.array([float(x), float(y)])


    #式の参考サイト
    #FC2, 直線に関する対称移動行列の導出
    #http://blog8192.blog.fc2.com/blog-entry-63.html, 2023年2月18日.

    #引数は何れもfloat型
    def sentaisho(self, x_1, y_1, th1):
        A = np.matrix((
            ( np.cos(2 * th1), np.sin(2 * th1)),
            ( np.sin(2 * th1), -np.cos(2 * th1)),
        )) #2行2列の正方行列

        v1 = np.array([[x_1],[y_1]]) #2行1列の縦ベクトル

        v2 = A * v1 #内積を計算 結果も2行1列の縦ベクトル

        #print("行列=" + str(A))
        #print("縦ベクトル=" + str(v1))
        #print("結果ベクトル=" + str(v2))

        return float(v2[0]), float(v2[1])


    #引数 3次元配列:各リンクの長さ[cm]、2次元配列:手先の位置[cm]、3次元配列:初期角度[rad]
    def inv_kine_arg(self, L, X, th_rad):
        # パラメータ
        #L = [0.4, 0.6, 0.2] #L = [0.5, 0.5] # リンク1, 2, 3の長さ (改変)
        #X = [0.7, 0.5] # 手先の目標位置(x,y)
        #L = [12.8, 13.6, 12.6]
        #X = [30, 20]

        #X = [0.6, 0.7, 90] # 手先の目標位置(x,y)
        #th_deg = [0, 90, 0]  #th = [20, 20]  # 初期関節角度(仮の解) (改変)
        th_deg = [np.rad2deg(th_rad[0]), np.rad2deg(th_rad[1]), np.rad2deg(th_rad[2])]

        # 逆運動学の計算
        th = arm2_ik(X, L, th_deg)

        #print("関節角度は、")
        #print(th)

        # 順運動学の計算
        (x1, y1, x2, y2, x3, y3) = arm2_fk(L, th) #改変

        th_2 = float(np.arctan(y3/x3)) #手先と根元を通る直線の傾きの逆正接で角度を求める。

        x1_2, y1_2 = sentaisho(float(x1), float(y1), th_2) #収束演算で得られた解の線対象な解を求める
        x2_2, y2_2 = sentaisho(float(x2), float(y2), th_2)

        #前3つ元の角度、後ろ3つ線対称に写像した部分の角度
        th_list = [np.deg2rad(th[0]), np.deg2rad(th[1]), np.deg2rad(th[2]), 0, 0, 0]

        #x, yの各値は、1*1のnumpy行列型であるため、float()関数を用いてスカラーに変換している。
        th_list[3] = vec2argdr(np.array([1, 0]), vec_tra(x1_2, y1_2))
        th_list[4] = vec2argdr(vec_tra(x1_2, y1_2), vec_tra(x2_2, y2_2))
        th_list[5] = vec2argdr(vec_tra(x2_2, y2_2), vec_tra(x3, y3))

        print(th_list)

        return th_list



    # メイン
    def main(self):
        # パラメータ
        #L = [0.4, 0.6, 0.2] #L = [0.5, 0.5] # リンク1, 2, 3の長さ (改変)
        #X = [0.7, 0.5] # 手先の目標位置(x,y)
        L = [12.8, 13.6, 12.6]
        X = [30, 20]

        #X = [0.6, 0.7, 90] # 手先の目標位置(x,y)
        th = [0, 90, 0]  #th = [20, 20]  # 初期関節角度(仮の解) (改変)

        # 逆運動学の計算
        th = arm2_ik(X, L, th)

        #print("関節角度は、")
        #print(th)

        # 順運動学の計算
        (x1, y1, x2, y2, x3, y3) = arm2_fk(L, th) #改変

        #print(str(np.rad2deg(th[0])) + "度")

        th_2 = float(np.arctan(y3/x3)) #手先と根元を通る直線の傾きの逆正接で角度を求める。

        x1_2, y1_2 = sentaisho(float(x1), float(y1), th_2) #収束演算で得られた解の線対象な解を求める
        x2_2, y2_2 = sentaisho(float(x2), float(y2), th_2)

        fn = graph_option()

        # ロボットアームの描画
        x_1 = (0, x1, x2, x3) #改変
        y_1 = (0, y1, y2, y3) #改変
        plot(x_1, y_1, fn)

        x_2 = (0, x1_2, x2_2, x3) #改変
        y_2 = (0, y1_2, y2_2, y3) #改変
        plot(x_2, y_2, fn)

        plt.axis('equal')
        plt.show()

        #前3つ元の角度、後ろ3つ線対称に写像した部分の角度
        th_list = [np.deg2rad(th[0]), np.deg2rad(th[1]), np.deg2rad(th[2]), 0, 0, 0]

        #x, yの各値は、1*1のnumpy行列型であるため、float()関数を用いてスカラーに変換している。
        th_list[3] = vec2argdr(np.array([1, 0]), vec_tra(x1_2, y1_2))
        th_list[4] = vec2argdr(vec_tra(x1_2, y1_2), vec_tra(x2_2, y2_2))
        #th_list[1][1] = vec2argdr(vec_tra(x2_2, y2_2), vec_tra(x1_2, y1_2))
        th_list[5] = vec2argdr(vec_tra(x2_2, y2_2), vec_tra(x3, y3))
        #th_list[1][2] = vec2argdr(vec_tra(x3, y3), vec_tra(x2_2, y2_2))

        #for i in range(3, len(th_list)):
        #    print(str(i-2) + "番目の関節は" + str(np.rad2deg(th_list[i])) + "度")
        

        """
        theta1 = vec2arg(np.array([1, 0]), np.array([float(x1), (y1)]))
        theta2 = vec2arg(np.array([float(x1), float(y1)]), np.array([float(x2)-float(x1), float(y2)-float(y1)]))
        theta3 = vec2arg(np.array([float(x2)-float(x1), float(y2)-float(y1)]), np.array([float(x3)-float(x2), float(y3)-float(y2)]))
        """

        print(th_list)

        #print("関節1=" + str(th[0]) + "、関節2=" + str(th[1]) + "、関節3=" + str(th[2]))
        #plot(x, y)




if __name__ == '__main__':
    L = [12.8, 13.6, 12.6]
    X = [30, 20]
    th_rad = [0, np.pi/2, 0]
    inv_kine_arg(L, X, th_rad)

    
    