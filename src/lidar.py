#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from carry_my_luggage.msg import LidarData
import math

PIZZA = 24

class Lidar():
    def __init__(self):
        self.pub = rospy.Publisher('/lidar', LidarData, queue_size=1)
    
    def remove_inf(self, ranges):
        f_ranges = []

        for i in ranges:
            if not math.isinf(i):
                f_ranges.append(i)

        return f_ranges
    
    def average_ranges(self, ranges):
        return sum(ranges) / len(ranges)
    
    def get_distance(self):
        scan = rospy.wait_for_message('/scan', LaserScan)
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
            distance.append(self.average_ranges(ranges[start:end]))

        # rospy.loginfo(distance)

        return distance

if __name__ == '__main__':
    rospy.init_node("lidar")
    lidar = Lidar()
    while not rospy.is_shutdown():
        l = LidarData()
        l.distance = lidar.get_distance()
        lidar.pub.publish(l)
        rospy.Rate(10).sleep()