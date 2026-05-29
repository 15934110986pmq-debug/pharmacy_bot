#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/02/15
# @author:aiden
# 路径规划节点, 以actionlib形式调用(call the path planning nodes in the form of actionlib)
import rospy
import actionlib
import grasp_trajectory
from jetarm_sdk import common
from actionlib_msgs.msg import GoalID
from std_srvs.srv import Empty, EmptyResponse
from hiwonder_interfaces.msg import Grasp, MoveFeedback, MoveResult, MoveAction, MultiRawIdPosDur, RawIdPosDur

class GraspNode:
    def __init__(self, name):
        # 初始化节点(initialize nodes)
        rospy.init_node(name, log_level=rospy.DEBUG)
        self.name = name
        self.cancel = False
        
        joints_pub = rospy.Publisher('/controllers/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        gripper_pub = rospy.Publisher('/controllers/id_pos_dur', RawIdPosDur, queue_size=1)
        
        self.grasp = grasp_trajectory.Grasp(joints_pub, gripper_pub)

        cancel_sub = rospy.Subscriber('/%s/cancel'%self.name, GoalID, self.callback)

        rospy.Service('~update_param', Empty, self.update_param)
        self.action_server = actionlib.SimpleActionServer('/%s'%self.name, MoveAction, self.execute, False)

        self.action_server.start()

        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down")

    def update_param(self, msg):
        # 误差补偿(error compensation)
        self.grasp.update_param()
        common.loginfo('%s update param'%self.name)
        
        return EmptyResponse()

    def execute(self, goal):
        rospy.loginfo("run goal")
        self.cancel = False
        percent = MoveFeedback()
        res = MoveResult()
        result = self.grasp.set_target(goal.grasp)
        
        i = 0
        rate = rospy.Rate(10)
        if result != []:  # 可以达到(can achieve)
            action_queue = self.grasp.get_queue()
            queue_len = len(action_queue)
            while not rospy.is_shutdown() and i < queue_len:  # 循环遍历动作(iterate through actions in a loop)
                t = action_queue[i][1]
                if t is not None:
                    action_queue[i][0](t)
                else:
                    action_queue[i][0]()
                i += 1
                percent.percent = i/float(queue_len)
                self.action_server.publish_feedback(percent)  # 发布完成度(publish the completion status)
                if self.cancel:  # 中途取消(cancel midway)
                    rospy.loginfo('##########cancel##########')
                    break
                rate.sleep()
            if self.cancel:  # 中途取消(cancel midway)
                res.result.complete = False
            else:
                # 返回到达目标位置时的位姿(retrieve the pose at the target location upon arrival)
                res.result.complete = True
                res.result.grasp_posture.r = self.grasp.target2[3][0]
                res.result.grasp_posture.p = self.grasp.target2[3][1]
                res.result.grasp_posture.y = self.grasp.target2[3][2]
            self.action_server.set_succeeded(res)
        else:  # 无法到达(cannot achieve)
            percent.percent = 0
            self.action_server.publish_feedback(percent)
            res.result.complete = True
            self.action_server.set_succeeded(res)

    def callback(self, msg):
        self.cancel = True

if __name__ == '__main__':
    GraspNode('grasp')
