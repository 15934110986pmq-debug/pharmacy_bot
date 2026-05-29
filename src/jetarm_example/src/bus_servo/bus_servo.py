#!/usr/bin/env python3
# encoding: utf-8
# Date:2021/08/12
# STM32总线舵机控制例程
import rospy
import signal
from hiwonder_interfaces.msg import SerialServoMove
                                    

# 总线舵机数据检测
bus_servo_data_detection = False
running = True

def shutdown(signum, frame):
    global running
    running = False
    rospy.loginfo('shutdown')
    rospy.signal_shutdown('shutdown')
    
# 总线舵机数据回调函数
def bus_servo_data_callback(msg):
    global bus_servo_data_detection
    print(msg)
    if msg.servo_id != []: #判断该话题的ID是否为空
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


if __name__ == '__main__':
    rospy.init_node('bus_servo_demo', anonymous=True) #初始化节点
    signal.signal(signal.SIGINT, shutdown)
    rospy.wait_for_service('/jetarm_sdk/get_loggers')
    #发布总线舵机话题
    bus_servo_pub = rospy.Publisher('/jetarm_sdk/serial_servo/move', SerialServoMove, queue_size=1)
    #接收总线舵机话题
    bus_servo_sub = rospy.Subscriber('/jetarm_sdk/serial_servo/move', SerialServoMove, bus_servo_data_callback)
    while running:
        bus_servo_controls(id =4,position =500,duration=500) #发布数据
        rospy.sleep(0.25) # 运行时间
        bus_servo_controls(id =4,position =200,duration=500) #发布数据
        rospy.sleep(0.25)# 运行时间
