#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

class Lidar():
    def __init__(self):
        self.pub = rospy.Publisher('/lidar', Float32, queue_size=1)
    
    def get_average_ranges(self, ranges):
        f_ranges = []

        for i in ranges:
            if i != float("inf"):
                f_ranges.append(i)

        return sum(f_ranges) / len(f_ranges)
    
    def get_distance(self, direction):
        scan = rospy.wait_for_message('/scan', LaserScan)

        if direction == "forward":
            distance = self.get_average_ranges(scan.ranges[0:48] + scan.ranges[1147 - 48 - 1:1147])
        elif direction == "right":
            distance = self.get_average_ranges(scan.ranges[858 - 48:858 + 48])
        elif direction == "left":
            distance = self.get_average_ranges(scan.ranges[286 - 48:286+48])
        else:
            rospy.loginfo("get_distance: You must specify direction")
            return
        
        return distance

if __name__ == '__main__':
    rospy.init_node("lidar")
    lidar = Lidar()
    while not rospy.is_shutdown():
        d = lidar.get_distance("forward")
        lidar.pub.publish(Float32(d))
        rospy.Rate(10).sleep()