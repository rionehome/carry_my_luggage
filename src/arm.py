#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
from carry_my_luggage.msg import ArmAction
import sys
from math import pi

# names of group and joint
arm_planning_group = "arm"
arm_joint_names = ["joint1", "joint2", "joint3", "joint4"]
gripper_planning_group = "gripper"
gripper_joint_names = ["gripper"]


class Arm:
    def __init__(self):
        rospy.init_node("arm")

        self.sub = rospy.Subscriber("/arm", ArmAction, self.callback)

        rospy.wait_for_service("/goal_joint_space_path")
        rospy.wait_for_service("/goal_tool_control")

    def move_arm(self, position, planning_time=3.0):
        service = rospy.ServiceProxy("/goal_joint_space_path", SetJointPosition)
        request = SetJointPositionRequest()

        request.planning_group = arm_planning_group
        request.joint_position.joint_name = arm_joint_names
        request.joint_position.position = position
        request.joint_position.max_accelerations_scaling_factor = 1.0
        request.joint_position.max_velocity_scaling_factor = 1.0
        request.path_time = planning_time

        service(request)

    def move_gripper(self, action, planning_time=4.0):
        service = rospy.ServiceProxy("/goal_tool_control", SetJointPosition)
        request = SetJointPositionRequest()

        # position 0.01 - -0.01
        if action == 0:
            position = [-0.01]
        elif action == 1:
            position = [-0.005]
        elif action == 2:
            position = [0]
        elif action == 3:
            position = [0.005]
        elif action == 4:
            position = [0.01]
        else:
            sys.exit("You must specify arm's action with 0-4")

        request.planning_group = gripper_planning_group
        request.joint_position.joint_name = gripper_joint_names
        request.joint_position.position = position
        request.joint_position.max_accelerations_scaling_factor = 1.0
        request.joint_position.max_velocity_scaling_factor = 1.0
        request.path_time = planning_time

        service(request)

    def callback(self, msg):
        if msg.time != 0.0:
            self.move_arm(msg.joint, planning_time=msg.time)
            self.move_gripper(msg.gripper, planning_time=msg.time)
        else:
            self.move_arm(msg.joint)
            self.move_gripper(msg.gripper)


if __name__ == "__main__":
    arm = Arm()
    while not rospy.is_shutdown():
        rospy.Rate(10).sleep()
