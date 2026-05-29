#!/usr/bin/env python3
# encoding: utf-8
import cv2
import rospy
import signal
import numpy as np
from sensor_msgs.msg import Image

class color_space:
    def __init__(self, name):
        # 初始化节点
        rospy.init_node(name, log_level=rospy.INFO)
        self.name = name
        self.image = None
        self.image_bgr = None
        self.image_test = None
        self.running = True
        self.image_sub = None
        source_image_topic = rospy.get_param('~source_image_topic', '/camera/image_raw')
        #启动程序中断函数
        signal.signal(signal.SIGINT, self.shutdown)

        # 检测图像发布
        self.image_sub = rospy.Subscriber(source_image_topic, Image, self.image_callback)
        rospy.sleep(0.2)
        self.run()
        
    #程序中断函数，用于停止程序
    def shutdown(self, signum, frame):
        self.running = False
        
    #处理ROS节点数据
    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                           buffer=ros_image.data)  # 将ros格式图像消息转化为opencv格式

        self.image = rgb_image
        #RGB转BGR
        self.image_bgr = cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR)
        #RGB转LAB
        self.image_test = cv2.cvtColor(self.image, cv2.COLOR_RGB2LAB)

    def run(self):
        while self.running:
            if self.image is not None and self.image_bgr is not None and self.image_test is not None:
                #展示图像
                cv2.imshow('RGB', self.image)
                cv2.imshow('BGR', self.image_bgr)
                cv2.imshow('TEST', self.image_test)
                cv2.waitKey(1)
            rospy.sleep(0.01)
if __name__ == '__main__':
    color_space('color_space')
