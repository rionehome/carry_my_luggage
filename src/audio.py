#!/usr/bin/env python
# use python2

import rospy
from std_msgs.msg import String
from ..speech_and_NLP.src.tools.text_to_speech.textToSpeech import textToSpeech

class Audio():
    def __init__(self):
        self.sub = rospy.Subscriber("/audio", String, self.callback)
    
    def callback(self, msg):
        textToSpeech(msg.data)

if __name__ == '__main__':
    rospy.init_node("audio")
    audio = Audio()
    
    while not rospy.is_shutdown():
        rospy.Rate(10).sleep()