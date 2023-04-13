#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from carry_my_luggage.srv import MoveArm

rospy.wait_for_service("/move_arm")

ser = rospy.ServiceProxy("/move_arm", MoveArm)

# MoveArm
# MoveArm.x: 目標座標のx(cm)
# MoveArm.y: 目標座標のy(cm)
# MoveArm.dig: 手先の角度(度)
# MoveArm.grip: 握り具合0〜4の整数値, 0->close, 4->open
#
# MoveArm.res: rosserviceの戻り値, 0->正常終了, -1->エラー終了

# 初期位置
res = ser(0, 0, 0, 0)
print(res.res)

# じゅんびして〜
res = ser(23, 5, 10, 4)
print(res.res)

# のばしてぇーの〜
res = ser(35, 7, 10, 4)
print(res.res)

# とじるっ！
res = ser(35, 7, 10, 2)
print(res.res)

# ひっかける！
res = ser(37, 7, 30, 2)
print(res.res)

# もちあげてからの〜
res = ser(30, 20, 30, 2)
print(res.res)

# おろす
res = ser(35, 7, 10, 2)
print(res.res)

# ほんではなす
res = ser(35, 7, 10, 4)
print(res.res)

# 初期位置
res = ser(0, 0, 0, 0)
print(res.res)
