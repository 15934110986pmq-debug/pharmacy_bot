#!/usr/bin/env python3
# encoding: utf-8
# Date:2021/08/12
# STM32蜂鸣器控制例程
import rospy
import signal
from hiwonder_interfaces.msg import Buzzer

#蜂鸣器数据检测
buzzer_data_detection = False
running = True

def shutdown(signum, frame):
    global running
    running = False
    rospy.loginfo('shutdown')
    rospy.signal_shutdown('shutdown')
    
# 蜂鸣器数据回调函数
def buzzer_data_callback(msg):
    global buzzer_data_detection
    if msg.freq != 0: #判断该话题的频率（freq）是否为0
        print(msg)
        buzzer_data_detection = True



if __name__ == '__main__':
    rospy.init_node('buzzer_demo', anonymous=True) #初始化节点
    signal.signal(signal.SIGINT, shutdown)
    rospy.wait_for_service('/jetarm_sdk/get_loggers')
    #发布蜂鸣器话题
    buzzer_pub = rospy.Publisher('/jetarm_sdk/set_buzzer', Buzzer, queue_size=1)
    #接收蜂鸣器话题
    buzzer_sub = rospy.Subscriber('/jetarm_sdk/set_buzzer', Buzzer, buzzer_data_callback)
    #设置蜂鸣器消息类型
    buzzer_data = Buzzer()
    buzzer_data.freq = 3000 #蜂鸣器频率
    buzzer_data.on_ticks = 100 #响的时间
    buzzer_data.off_ticks = 100 #响的间隔
    buzzer_data.repeat = 10 #重复的次数
    while running:
        buzzer_pub.publish(buzzer_data) #发布蜂鸣器数据
        rospy.sleep(1) #延时，防止一直发布数据
        if buzzer_data_detection:
            break
