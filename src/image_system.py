#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import rospy

from carry_my_luggage.srv import HandDirection
from finger_direction import get_direction


class ImageSystem:
    def __init__(self):
        rospy.init_node("image_system")
        self.hand_direction_srv = rospy.Service(
            "/image_system/hand_direction", HandDirection, self.hand_direction_callback
        )

    def hand_direction_callback(self, msg):
        rospy.loginfo("image_system: detecting hand direction")
        return get_direction(msg.n)


if __name__ == "__main__":
    try:
        imageSystem = ImageSystem()
        rospy.spin()

    except KeyboardInterrupt:
        pass
