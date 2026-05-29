#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/03/01
# @author:aiden
# 用来测试夹取和放置(used for testing the grasping and placing)

import rospy
import actionlib
from geometry_msgs.msg import Point
from hiwonder_interfaces.msg import MultiRawIdPosDur
from hiwonder_interfaces.msg import MoveAction, MoveGoal

place_position = [
        [0.175, -0.08, 0.015],
        [0.175, -0.08, 0.050],
        [0.123, -0.077, 0.005],
        [0.123, -0.077, 0.047]
        ]

rospy.init_node('pick_place_demo', log_level=rospy.ERROR)
joints_pub = rospy.Publisher('/controllers/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
action_client = actionlib.SimpleActionClient('/grasp', MoveAction)  # 夹取动作(grasping action)
action_client.wait_for_server()

action_finish = True
def done_callback(state, result):
    '''
    action完成时回调(callback upon completion)
    :param state:
    :param result:
    :return:
    '''
    global action_finish

    rospy.loginfo('state:%f'%state)
    # 如果机械臂到达不了指定位置也会返回True，所有需要结合完成百分比来确认机械臂是否搬运了(If the robotic arm cannot reach the specified position, it will still return True, so it's necessary to combine it with the completion percentage to confirm whether the robotic arm has performed the transfer)
    rospy.loginfo('complete:%s'%result.result.complete)
    action_finish = True

def active_callback():
    '''
    action开始时回调(callback at the beginning)
    :return:
    '''
    global action_finish 
    action_finish = False
    rospy.loginfo('start move')

def feedback_callback(msg):
    '''
    action执行中回调(callback during execution)
    :param msg:
    :return:
    '''
    rospy.loginfo('finish action: {:.2%}'.format(msg.percent)) 

def pick():
    goal = MoveGoal()
    goal.grasp.mode = 'pick'  # 夹取模式(grasping mode)
    p = Point()
    p.y = -0.02
    p.x = 0.204 
    p.z = 0.01 
    # 物体坐标(object coordinate)
    goal.grasp.position = p
    
    # 夹取时的姿态角(grasping pose angle)
    goal.grasp.roll = 90
    goal.grasp.align_angle = 44
    # 夹取时靠近的方向和距离(the direction and distance when approaching for grasping)
    goal.grasp.grasp_approach.z = 0.02

    # 夹取后后撤的方向和距离(the direction and distance for withdrawing after grasping)
    goal.grasp.grasp_retreat.z = 0.02

    # 夹取前后夹持器的开合(the opening and closing of the gripper before and after grasping)
    goal.grasp.grasp_posture = 480
    goal.grasp.pre_grasp_posture = 200  
    print(goal)
    action_client.send_goal(goal, done_callback, active_callback, feedback_callback)

def place(num):
    goal = MoveGoal()
    goal.grasp.mode = 'place'  # 夹取模式(grasping mode)

    # 物体坐标(object coordinate)
    goal.grasp.position.x = place_position[num-1][0]
    goal.grasp.position.y = place_position[num-1][1]
    goal.grasp.position.z = place_position[num-1][2] 

    # 夹取时的姿态角(grasping pose angle)
    goal.grasp.roll = 90

    # 夹取时靠近的方向和距离(direction and proximity during grasping)
    goal.grasp.grasp_approach.z = 0.02

    # 夹取后后撤的方向和距离(the direction and distance for withdrawing after grasping)
    goal.grasp.grasp_retreat.z = 0.04

    # 夹取前后夹持器的开合(the opening and closing of the gripper before and after grasping)
    goal.grasp.grasp_posture = 100
    goal.grasp.pre_grasp_posture = 400   
    action_client.send_goal(goal, done_callback, active_callback, feedback_callback)

while True:
    if action_finish:
        # pick()
        # rospy.sleep(0.2)
        mode = int(input('input mode(1 for pick 2 for place others break):'))
        if mode == 1:
            pick()
        elif mode == 2:
            num = int(input('input num(1 2 3 4):'))
            place(num)
        else:
            break
    else:
        rospy.sleep(0.01)
