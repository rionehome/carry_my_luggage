#!/usr/bin/env python3

import math

import rospy
from sensor_msgs.msg import LaserScan

from carry_my_luggage.msg import LidarData

PIZZA = 24


class Lidar:
    def __init__(self):
        self.pub = rospy.Publisher("/control_system/lidar", LidarData, queue_size=1)
        self.main()

    def remove_inf(self, ranges):
        f_ranges = []
        for i in ranges:
            if not math.isinf(i):
                f_ranges.append(i)

        return f_ranges

    def get_distance(self):
        scan = rospy.wait_for_message("/scan", LaserScan)
        # rospy.loginfo(scan)

        distance = []
        ranges = self.remove_inf(scan.ranges)
        pizza = PIZZA
        dots = int(math.floor(len(ranges) / pizza))
        # print(dots)
        # print(len(ranges))

        for i in range(pizza):
            start = dots * i
            end = start + dots
            r = ranges[start:end]
            a = sum(r) / len(r)
            distance.append(a)

        # rospy.loginfo(distance)

        return distance

    def get_direction(self, ranges):
        mn = min(ranges)
        mn_index = ranges.index(mn)

        if mn_index <= 5 or mn_index >= 18:
            front_back = "front"
        else:
            front_back = "back"

        if mn_index <= 11:
            left_right = "left"
        else:
            left_right = "right"

        direction = [front_back, left_right]

        return direction

    def main(self):
        while not rospy.is_shutdown():
            l = LidarData()
            l.distance = self.get_distance()
            direction = self.get_direction(l.distance)
            l.front_back = direction[0]
            l.left_right = direction[1]
            # print(direction)
            self.pub.publish(l)
            rospy.Rate(10).sleep()


if __name__ == "__main__":
    rospy.init_node("lidar")
    lidar = Lidar()
