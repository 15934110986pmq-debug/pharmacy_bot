#!/usr/bin/env python3
# encoding: utf-8
import cv2
import rospy
import signal
import numpy as np
from sensor_msgs.msg import Image


class color_threshold:
    def __init__(self, name):
        # 初始化节点
        rospy.init_node(name, log_level=rospy.INFO)
        self.name = name
        self.image = None
        self.image_test = None
        self.running = True
        self.image_sub = None
        source_image_topic = rospy.get_param('~source_image_topic', '/camera/image_raw')
        #设置需要识别的名字和阈值
        self.color_threshold = {"blue":[(0,0,0),(255,255,104)],
                                "red":[(0,149,108),(255,255,255)],
                                "green":[(0,0,138),(255,130,255)]}
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
        #将图像的颜色空间转换成LAB
        self.image= cv2.cvtColor(rgb_image, cv2.COLOR_RGB2LAB)
        #设置颜色阈值并进行二值化
        self.image_test = cv2.inRange(self.image,self.color_threshold['red'][0],self.color_threshold['red'][1])

    def run(self):
        while self.running:
            if self.image is not None and self.image_test is not None:
                #展示画面
                cv2.imshow('TEST', self.image)
                #展示识别到的画面
                cv2.imshow('TEST2', self.image_test)
                cv2.waitKey(1)
if __name__ == '__main__':
    color_threshold('color_threshold')
