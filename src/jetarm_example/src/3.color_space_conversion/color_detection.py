#!/usr/bin/env python3
# encoding: utf-8
import cv2
import rospy
import signal
import math
import numpy as np
from sensor_msgs.msg import Image

class color_detection:
    def __init__(self, name):
        # 初始化节点
        rospy.init_node(name, log_level=rospy.INFO)
        self.name = name
        self.image = None
        self.image_bgr = None
        self.image_test = None
        self.running = True
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
        #RGB转LAB
        self.image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2LAB)
        #RGB转BGR
        self.image_bgr = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR) 
    
    def erode_and_dilate(self, binary, kernel=3):
        # 腐蚀膨胀
        element = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel, kernel))
        eroded = cv2.erode(binary, element)  # 腐蚀
        dilated = cv2.dilate(eroded, element)  # 膨胀
        return dilated

    #颜色识别函数
    def color_detection(self,color):
        #将颜色空间转换为LAB
        self.image_test = cv2.GaussianBlur(self.image_bgr, (3, 3), 3)
        self.image_test = cv2.cvtColor(self.image_test, cv2.COLOR_BGR2LAB)
        #将颜色阈值填入，并输出识别后的二值化图像
        self.image_test = cv2.inRange(self.image_test,
                                       self.color_threshold[color][0],
                                       self.color_threshold[color][1])
        self.image_test = self.erode_and_dilate(self.image_test)
        # 找出所有轮廓
        contours = cv2.findContours(self.image_test, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[-2]  
        # 遍历轮廓
        c = max(contours, key = cv2.contourArea)
        area = math.fabs(cv2.contourArea(c))  
        if area >= 2000:
            print("检测到",color)

    def run(self):
        while self.running:
            try:
                if self.image is not None:
                    #设置所需识别的所有颜色及其阈值
                    self.color_detection("red")
                if self.image_bgr is not None and self.image_test is not None:
                    #展示识别效果
                    cv2.imshow('RGB', self.image_bgr)
                    cv2.imshow('color_detection', self.image_test)
                    cv2.waitKey(1)

            except Exception as e:
                print("未检测到所需识别的颜色，请将色块放置到相机视野内。")
if __name__ == '__main__':
    color_detection('color_detection')
