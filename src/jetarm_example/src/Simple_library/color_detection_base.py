#!/usr/bin/env python3
# encoding: utf-8
import cv2
import math
import numpy as np

class color_detection:
    def __init__(self):
        self.image = None
        self.image_rgb = None
        self.image_test = None
        #设置图像宽高
        self.size = {"height":240,"width":320}
        #设置颜色阈值
        self.color_threshold = {"blue":[(0,0,0),(255,255,104)],
                                "red":[(0,149,108),(255,255,255)],
                                "green":[(0,0,138),(255,130,255)]}


    def erode_and_dilate(self, binary, kernel=3):
        # 腐蚀膨胀
        element = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel, kernel))
        eroded = cv2.erode(binary, element)  # 腐蚀
        dilated = cv2.dilate(eroded, element)  # 膨胀

        return dilated
        
    # 颜色识别函数
    def color_detection(self,color,image_bgr):
         # 得到图像的宽高
         img_h, img_w = image_bgr.shape[:2]
         # 高斯模糊
         self.image_test = cv2.GaussianBlur(image_bgr, (3, 3), 3)
         # 转换颜色空间
         self.image_test = cv2.cvtColor(self.image_test, cv2.COLOR_BGR2LAB)
         # 根据阈值识别颜色
         self.image_test = cv2.inRange(self.image_test,
                                       self.color_threshold[color][0],
                                       self.color_threshold[color][1])
         # 腐蚀膨胀
         self.image_test = self.erode_and_dilate(self.image_test)

             
         return self.image_test 

if __name__ == '__main__':
    color_detection()
