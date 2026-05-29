#!/usr/bin/env python3
# encoding: utf-8
# Date:2021/08/12
import rospy
import signal
import jetarm_kinematics.transform as transform
from jetarm_kinematics.forward_kinematics import ForwardKinematics
from jetarm_kinematics.inverse_kinematics import get_ik, get_position_ik, set_link, get_link, set_joint_range, get_joint_range
from hiwonder_interfaces.msg import SerialServoMove

from jetarm_sdk import bus_servo_control
from hiwonder_interfaces.msg import MultiRawIdPosDur

rospy.init_node('fk_demo', anonymous=True) #初始化节点
fk = ForwardKinematics(debug=True)
print("舵机脉宽值：")
servo_list = [500,400,300,400,500]
print(servo_list)  

print("舵机脉宽值转为弧度：")
pulse = transform.pulse2angle(servo_list)  # 舵机脉宽值转为弧度
print(pulse)

res = fk.get_fk(pulse)  #获取运动学正解
print('正运动学解-坐标：', res[0])
print('正运动学解-四元数：', res[1])
print('转换四元数成rpy：', transform.qua2rpy(res[1]))

bus_servo_data_detection = False
bus_servo_id_detection = False
j = 1
# 总线舵机数据回调函数
def bus_servo_data_callback(msg):
    global bus_servo_data_detection,j
    #print(msg)
    if msg.servo_id == j:
        j+=1
    if msg.servo_id == 5: #判断该话题的ID是否为空
        bus_servo_data_detection = True

def bus_servo_controls(id=0,
                       position=0,
                       duration=0.0):
                       
    #bus_servo_data =[]
    # 设置总线舵机消息类型
    data = SerialServoMove()
    data.servo_id = id #总线舵机ID    
    data.position = position #总线舵机角度[0-1000]
    data.duration = duration #总线舵机运行时间
    bus_servo_pub.publish(data) #发布数据

# 发布总线舵机话题
bus_servo_pub = rospy.Publisher('/jetarm_sdk/serial_servo/move', SerialServoMove, queue_size=1)
# 接收总线舵机话题
bus_servo_sub = rospy.Subscriber('/jetarm_sdk/serial_servo/move', SerialServoMove, bus_servo_data_callback)
while True:
    rospy.wait_for_service('/kinematics/set_pose_target')
    bus_servo_controls(id =j,position =servo_list[j-1],duration=500) #发布数据
    rospy.sleep(0.25)
    if bus_servo_data_detection:
        break
