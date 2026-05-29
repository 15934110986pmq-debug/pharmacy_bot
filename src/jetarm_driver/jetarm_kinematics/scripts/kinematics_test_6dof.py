#!/usr/bin/env python3
import sys
import rospy
from jetarm_sdk import controller_client
from math import radians


class KinematicsTestNode:
    def __init__(self):
        rospy.init_node("kinematics_test", log_level=rospy.INFO)
        self.controller = controller_client.JointControllerClient(wait_for_service=None)
        self.ik = rospy.ServiceProxy("/kinematics/set_pose_target")


if __name__ == "__main__":
    node = KinematicsTestNode()
    rospy.spin()
