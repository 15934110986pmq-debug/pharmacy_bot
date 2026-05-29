#!/usr/bin/python3
# coding=utf8
# Date:2021/07/24
# 串口舵机代码控制例程(interface servo code control routine)
import math
import rospy
import signal
from hiwonder_interfaces.msg import RawIdPosDur

running = True
def shutdown(signum, frame):
    global running

    running = False
    rospy.loginfo('shutdown')
    rospy.signal_shutdown('shutdown')

signal.signal(signal.SIGINT, shutdown)

def set_grasp(pub, duration, pos_s):
    msg = RawIdPosDur()
    msg.id = 10
    msg.position = pos_s
    msg.duration = duration
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('servo_control_example', anonymous=True)

    # 发布舵机位置(publish servo position)
    grasp_pub = rospy.Publisher('/controllers/set_joint', JointMove, queue_size=1)
    while running:
        try:
            # 参数1:发布句柄(parameter1: publish handle)
            # 参数2:运行时间(ms)(parameter2: running times(ms))
            # 参数3:位置(parameter3: position)
            set_grasp(grasp_pub, 500, 200)
            rospy.sleep(1)
            set_grasp(grasp_pub, 500, 500)
            rospy.sleep(1)
        except Exception as e:
            print(e)
            break
