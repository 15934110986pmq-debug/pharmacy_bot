#!/usr/bin/env python3
# encoding: utf-8
import sys
import rospy
import signal
from hiwonder_interfaces.msg import SerialServoMove
from jetarm_kinematics.kinematics_control import set_pose_target
from jetarm_kinematics.inverse_kinematics import get_ik, get_position_ik, set_link, get_link, set_joint_range, get_joint_range
#导入动作组驱动函数

class path_planning:
    def __init__(self, name):
        # 初始化节点
        rospy.init_node(name, log_level=rospy.INFO)
        self.name = name
        self.running = True 
        self.bus_servo_data_detection = False
        self.servo = 0
        # 初始化机械臂运动状态
        self.start = True   
        rospy.wait_for_service('/kinematics/set_pose_target')
        # 发布总线舵机话题
        self.bus_servo_pub = rospy.Publisher('/jetarm_sdk/serial_servo/move', SerialServoMove, queue_size=1)
        # 接收总线舵机话题
        self.bus_servo_sub = rospy.Subscriber('/jetarm_sdk/serial_servo/move', SerialServoMove, self.bus_servo_data_callback)
        #调用程序中断函数
        signal.signal(signal.SIGINT, self.shutdown)
        rospy.sleep(0.2)
        self.run()
        
    #程序中断函数，用于停止程序
    def shutdown(self, signum, frame):
        self.running = False
        
    def bus_servo_data_callback(self,msg):
        if msg.servo_id == self.servo: #判断该话题的ID是否为空
            self.bus_servo_data_detection = True
            self.servo = 0
            
    def bus_servo_controls(self,id=0,
                       position=0,
                       duration=0.0):
                       
        #bus_servo_data =[]
        # 设置总线舵机消息类型
        data = SerialServoMove()
        data.servo_id = id #总线舵机ID    
        data.position = position #总线舵机角度[0-1000]
        data.duration = duration #总线舵机运行时间
        self.bus_servo_pub.publish(data) #发布数据
        
    def bus_servo_move(self,servo_list=[]):
        for i in servo_list:
            while True:
                if i != []:
                    if self.bus_servo_data_detection:
                        self.bus_servo_data_detection = False
                        break
                        
                    self.servo = i[0]
                    self.bus_servo_controls(id =i[0],position =int(i[1]),duration=1000) #发布数据
                    rospy.sleep(0.01)
                else:
                    break
    
    #路径规划
    def movement_0(self,x,y,z,pitch,t=1000):
        #是否可以规划机械臂运动到设置的坐标
        target = set_pose_target([x,y,z],pitch, [-90, 90], 1)
        if target[1] != []:  # 可以达到
            servo_data = target[1]
            print("舵机角度",servo_data)
            #转动
            self.bus_servo_move([[1,servo_data[0]]]) 
            rospy.sleep(t/1000.0)
        else:
            self.start = False
            print("无法运行到此位置")
            
    def movement_1(self,x,y,z,pitch,t=1000):
        #是否可以规划机械臂运动到设置的坐标
        target = set_pose_target([x,y,z],pitch, [-90, 90], 1)
        if target[1] != []:  # 可以达到
            servo_data = target[1]
            print("舵机角度",servo_data)
            #转动
            self.bus_servo_move([[1,servo_data[0]],[2,servo_data[1]],[3,servo_data[2]],[4,servo_data[3]]]) 
            rospy.sleep(t/1000.0)
        else: 
            self.start = False
            print("无法运行到此位置")

    def run(self):
        #初始化机械臂
        self.bus_servo_move([[1,500],[2,610],[3,70],[4,140]]) 
        rospy.sleep(2)
        while self.running:
            #运行路径规划程序
            if self.start:
                print("云台运动")
                self.movement_0(0.2,-0.1,0.05,90)
                rospy.sleep(2)
                print("动作1")
                self.movement_1(0.2,-0.1,0.05,90)
                rospy.sleep(2)
                print("动作2")
                self.movement_1(0.2,-0.1,0.005,90)
                rospy.sleep(2)
                self.running = False
            else:
                self.running = False
            rospy.sleep(0.01)
            
        self.bus_servo_move([[1,500],[2,610],[3,70],[4,140]]) 
        rospy.sleep(1)
            
if __name__ == '__main__':
    path_planning('path_planning')
