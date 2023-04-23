#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import rospy


class ImageSystem:
    def __init__(self):
        rospy.init_node("image_system")


if __name__ == "__main__":
    try:
        imageSystem = ImageSystem()
        rospy.spin()

    except KeyboardInterrupt:
        pass
