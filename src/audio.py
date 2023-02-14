#!/usr/bin/env python
# use python2

import rospy
from std_msgs.msg import String

class Audio():
    def __init__(self):
        self.sub = rospy.Subscriber("/audio", String, self.callback)
    
    def callback(self, msg):
        """
        print(msg.data)
        
        text to speach here
        """
        pass

if __name__ == '__main__':
    rospy.init_node("audio")
    audio = Audio()
    while not rospy.is_shutdown():
        rospy.Rate(10).sleep()