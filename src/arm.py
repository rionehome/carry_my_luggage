#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, String
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
import sys
import time
from math import pi

# names of group and joint
arm_planning_group = 'arm'
arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
gripper_planning_group = 'gripper'
gripper_joint_names = ['gripper']

class Arm():
    def __init__(self):
        rospy.init_node("arm")

        self.joint_sub = rospy.Subscriber("/arm_joint", Float32MultiArray, self.joint_callback)
        self.gripper_sub = rospy.Subscriber("/arm_gripper", String, self.gripper_callback)

        rospy.wait_for_service('/goal_joint_space_path')
        rospy.wait_for_service('/goal_tool_control')
    
    def move_arm(self, position, planning_time=3.0):
        service = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
        request = SetJointPositionRequest()
        
        request.planning_group = arm_planning_group
        request.joint_position.joint_name = arm_joint_names
        request.joint_position.position = position
        request.joint_position.max_accelerations_scaling_factor = 1.0
        request.joint_position.max_velocity_scaling_factor = 1.0
        request.path_time = planning_time

        service(request)
    
    def move_gripper(self, action, planning_time=4.0):
        service = rospy.ServiceProxy('/goal_tool_control', SetJointPosition)
        request = SetJointPositionRequest()

        # position 0.01 - -0.01
        if action == "open":
            position = [0.007]
        elif action == "close":
            position = [-0.007]
        elif action == "init":
            position = [0.01]
        else:
            sys.exit("You must specify arm's action with open or close")
      
        request.planning_group = gripper_planning_group
        request.joint_position.joint_name = gripper_joint_names
        request.joint_position.position = position
        request.joint_position.max_accelerations_scaling_factor = 1.0
        request.joint_position.max_velocity_scaling_factor = 1.0
        request.path_time = planning_time

        service(request)

        # time.sleep(3)
        # rate = rospy.Rate(1)
        # rate.sleep()
        # rospy.sleep(3)

    def joint_callback(self, msg):
        pass

    def gripper_callback(self, msg):

        if msg.data == "open":
            rospy.loginfo("arm: Openning gripper")
            self.move_gripper("open")
        elif msg.data == "close":
            rospy.loginfo("arm: Closing gripper")
            self.move_gripper("close")
        elif msg.data == "init":
            self.move_gripper("init")


if __name__ == '__main__':
    arm = Arm()
    while not rospy.is_shutdown():
        rospy.Rate(10).sleep()