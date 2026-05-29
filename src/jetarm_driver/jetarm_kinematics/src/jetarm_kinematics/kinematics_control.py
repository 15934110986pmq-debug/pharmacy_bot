#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/03/21
# @author:aiden
# 机械臂运动学调用(call robotic arm kinematics)
import rospy
import jetarm_kinematics.transform as transform 
from hiwonder_interfaces.srv import SetRobotPose, GetRobotPose, SetJointValue

def get_current_pose():
    res = rospy.ServiceProxy('/kinematics/get_current_pose', GetRobotPose, persistent=True)()
    return res.success, res.solution, res.pose

def set_pose_target(position, pitch, pitch_range=[-180, 180], resolution=1):
    '''
    给定坐标和俯仰角，返回逆运动学解(Given the coordinates and pitch angle, and return the inverse kinematics solution)
    position: 目标位置，列表形式[x, y, z]，单位m(the target position in the form of list[x, y, z], in the unit of m)
    pitch: 目标俯仰角，单位度，范围-180~180(the target pitch angle in degrees, ranging from -180 to 180)
    pitch_range: 如果在目标俯仰角找不到解，则在这个范围内寻找解(if a solution cannot be found at the target pitch angle, search for a solution within this range)
    resolution: pitch_range范围角度的分辨率(resolution of pitch_range)
    return: 调用是否成功， 舵机的目标位置， 当前舵机的位置， 机械臂的目标姿态， 最优解所有舵机转动的变化量(Whether the call is successful, the target position of the servo, the current position of the servo, the target pose of the robotic arm, and the changes in the rotation of all servos in the optimal solution)
    '''
    res = rospy.ServiceProxy('/kinematics/set_pose_target', SetRobotPose, persistent=True)(position, pitch, pitch_range, resolution)
    return [res.success, list(res.pulse), list(res.current_pulse), list(res.rpy), res.min_variation]

def set_joint_value_target(joint_value):
    '''
    给定每个舵机的转动角度，返回机械臂到达的目标位置姿态(Given the rotation angles of each servo, return the target position and pose that the robotic arm reaches)
    joint_value: 每个舵机转动的角度，列表形式[joint1, joint2, joint3, joint4, joint5]，单位脉宽(The rotation angle of each servo, in the form of a list [joint1, joint2, joint3, joint4, joint5], in units of pulse width)
    return: 目标位置的3D坐标和位姿，格式geometry_msgs/Pose(the 3D coordinates and pose of the target position, in the format of geometry_msgs/Pose)
    '''
    return rospy.ServiceProxy('/kinematics/set_joint_value_target', SetJointValue, persistent=True)(joint_value)
    
if __name__ == "__main__":
    # 初始化节点(initialize node)
    rospy.init_node('kinematics_controller', anonymous=True)
    res = set_pose_target([0, -0.126, 0.097], 66, [-67, -66], 1)
    print('ik', res)
    if res[1] != []:
        res = set_joint_value_target(res[1])
        print('fk', res)

