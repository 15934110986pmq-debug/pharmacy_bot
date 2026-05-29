#!/usr/bin/env python3
# encoding: utf-8
# Date:2021/08/12
# STM32LED控制例程
import rospy
import signal
from hiwonder_interfaces.msg import Led

# LED数据检测
led_data_detection = False
running = True

def shutdown(signum, frame):
    global running
    running = False
    rospy.loginfo('shutdown')
    rospy.signal_shutdown('shutdown')
    
# LED数据回调函数
def led_data_callback(msg):
    global led_data_detection
    if msg.on_ticks != 0: #判断该话题的启动时间是否为0
        print(msg)
        led_data_detection = True



if __name__ == '__main__':
    rospy.init_node('led_demo', anonymous=True) #初始化节点
    signal.signal(signal.SIGINT, shutdown)
    rospy.wait_for_service('/jetarm_sdk/get_loggers')
    # 发布LED话题
    led_pub = rospy.Publisher('/jetarm_sdk/set_led', Led, queue_size=1)
    # 接收LED话题
    led_sub = rospy.Subscriber('/jetarm_sdk/set_led', Led, led_data_callback)
    # 设置LED消息类型
    led_data = Led()
    led_data.brightness = 1  #LED ID
    led_data.on_ticks = 200  #亮的时间
    led_data.off_ticks= 200 #亮的间隔
    led_data.repeat = 10#重复的次数
    while running:
        led_pub.publish(led_data) #发布LED数据
        rospy.sleep(1) # 延时，防止一直发布数据
        if led_data_detection:
            break
