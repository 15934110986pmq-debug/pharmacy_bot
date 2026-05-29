#!/usr/bin/env python3
# encoding: utf-8
import cv2
import sys
import rospy
import signal
import numpy as np
from sensor_msgs.msg import Image
sys.path.append('/home/ubuntu/jetarm/src/jetarm_example/src/Simple_library')

import color_detection_base
class pixel_coordinate:
    def __init__(self, name):
        # 初始化节点
        rospy.init_node(name, log_level=rospy.INFO)
        self.name = name
        self.image = None
        self.image_rgb = None
        self.image_test = None
        self.running = True
        source_image_topic = rospy.get_param('~source_image_topic', '/camera/image_raw')
        self.color = rospy.get_param('~color', 'red')
        # 初始化颜色识别类
        self.color_detection = color_detection_base.color_detection()
        # 启动程序中断函数
        signal.signal(signal.SIGINT, self.shutdown)
        # 检测图像发布
        self.image_sub = rospy.Subscriber(source_image_topic, Image, self.image_callback)
        rospy.sleep(0.2)
        self.run()
    # 程序中断函数，用于停止程序
    def shutdown(self, signum, frame):
        self.running = False
        
    # 处理ROS节点数据
    def image_callback(self, ros_image):
        # 将ros格式图像消息转化为opencv格式
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,buffer=ros_image.data)
        # 将颜色空间转换成LAB 
        self.image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2LAB)
        # 将颜色空间转换成BGR
        self.image_bgr = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

    
    def run(self):
        while self.running:
            try:
                if self.image is not None and self.image_bgr is not None :
                    # 启动颜色识别得到识别后的图像和像素坐标               
                    self.image_test= self.color_detection.color_detection(self.color,self.image_bgr)
                    # 计算识别到的轮廓
                    contours = cv2.findContours(self.image_test, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[-2]  # 找出所有轮廓
                    # 找出最大轮廓
                    c = max(contours, key = cv2.contourArea)
                    # 根据轮廓大小判断是否进行下一步处理
                    rect = cv2.minAreaRect(c)  # 获取最小外接矩形
                    corners = np.int0(cv2.boxPoints(rect))  # 获取最小外接矩形的四个角点
                    x, y = rect[0][0],rect[0][1]
                    # 打印像素坐标
                    print("像素坐标为:","x:",x,"y:",y)
                    # 展示
                    cv2.imshow('BGR', self.image_bgr)
                    cv2.imshow('color_detection', self.image_test)
                    cv2.waitKey(1)
            except Exception as e:
                 print("未检测到所需识别的颜色，请将色块放置到相机视野内。")
if __name__ == '__main__':
    pixel_coordinate('pixel_coordinate_calculation')
