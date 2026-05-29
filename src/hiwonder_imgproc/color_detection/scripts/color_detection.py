#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/02/02
# @author:aiden
# 识别颜色以及计算物体的位置信息(recognize color and calculate the position information of object)
import cv2
import math
import numpy as np
from position_change_detect import position_reorder

class ColorDetection:
    def __init__(self, config, color_list, distance, get_surface=True):
        '''
        :param config: 包含lab阈值，roi等(include lab threshold value, roi, etc.)
        :param color_list: 需要识别的颜色列表(the color list to be recognized)
        :param distance: 对比上一次和当前的位置，相差距离小于此值则相应的颜色标签不变(if the difference in distance between the previous and current positions is less than this value, the corresponding color label remains unchanged)
        '''
        self.get_surface = get_surface
        self.lab_data = config['lab']
        self.color_list = color_list
        self.distance = distance

        # 前一次的物体信息列表，用来和当前比较以确认颜色标签是否需要改变(a list of previous object information used to compare with the current one to confirm whether the color label needs to be changed)
        self.last_object_info_list = []

        # roi区域，先缩放再截取(roi area, scale first then crop)
        self.roi = [config['roi']['height'][0],
                    config['roi']['height'][1],
                    config['roi']['width'][0],
                    config['roi']['width'][1]]

        # min_area < 物体的颜色面积 < max_area(min_area < color area of object < max_area)
        self.min_area = config['area']['min_area']
        self.max_area = config['area']['max_area']

        # 图像缩放比例系数(0, 1)，加速识别(image scaling factor (0, 1) to accelerate recognition)
        self.size = config['image_proc_size']
        
        self.perspective_transformation_matrix = np.array(config['perspective_transformation_matrix']).reshape((3, 3))

    def update_config(self, config):
        '''
        更新参数(update parameter)
        :param lab_data:
        :return:
        '''
        self.lab_data = config['lab']
        #self.roi = [config['roi']['height'][0],
        #            config['roi']['height'][1],
        #            config['roi']['width'][0],
        #            config['roi']['width'][1]]
        #self.min_area = config['area']['min_area']
        #self.max_area = config['area']['max_area']
        #self.size = config['image_proc_size']

    def update_color(self, color_list):
        '''
        更新颜色列表(update color list)
        :param color_list:
        :return:
        '''
        self.color_list = color_list

    def adaptive_threshold(self, gray_image):
        # 用自适应阈值先进行分割, 过滤掉侧面(perform segmentation using adaptive thresholding, then filter out the side views)
        # cv2.ADAPTIVE_THRESH_MEAN_C： 邻域所有像素点的权重值是一致的(the weight values of all neighboring pixels are uniform)
        # cv2.ADAPTIVE_THRESH_GAUSSIAN _C ： 与邻域各个像素点到中心点的距离有关，通过高斯方程得到各个点的权重值(the weight of each pixel in the neighborhood is determined by its distance from the center point, calculated using the Gaussian equation)
        binary = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 5)

        return binary
    
    def canny_proc(self, bgr_image):
        # 边缘提取，用来进一步分割(当两个相同颜色物体靠在一起时，只能靠边缘去区分)(edge detection, used for further segmentation (when two objects of the same color are adjacent, they can only be distinguished by their edges))
        mask = cv2.Canny(bgr_image, 20, 60, 3, L2gradient=True)
        mask = 255 - cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)))  # 加粗边界，黑白反转(thicken edges, and invert black and white)
        
        return mask
    
    def erode_and_dilate(self, binary, kernel=3):
        # 腐蚀膨胀(erode and dilate)
        element = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel, kernel))
        eroded = cv2.erode(binary, element)  # 腐蚀(erode)
        dilated = cv2.dilate(eroded, element)  # 膨胀(dilate)

        return dilated

    def point_remapped(self, point, now, new, data_type=float):
        """
        将一个点的坐标从一个图片尺寸映射的新的图片上(map the coordinate of one point from an image size to a new one)
        :param point: 点的坐标(coordinate of point)
        :param now: 现在图片的尺寸(size of current image)
        :param new: 新的图片尺寸(size of new image)
        :return: 新的点坐标(new point coordinate)
        """
        x, y = point
        now_w, now_h = now
        new_w, new_h = new
        new_x = x * new_w / now_w
        new_y = y * new_h / now_h
        
        return data_type(new_x), data_type(new_y)

    def get_top_surface(self, rgb_image):
        # 为了只提取物体最上层表面(to extract only the uppermost surface of the object)
        image_gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        image_mb = cv2.medianBlur(image_gray, 3)  # 中值滤波(median filtering)
        binary = self.adaptive_threshold(image_mb)  # 阈值自适应(adaptive thresholding)

        image_gs = cv2.GaussianBlur(rgb_image, (5, 5), 5)  # 高斯模糊去噪(Gaussian blur for noise reduction)
        mask = self.canny_proc(image_gs)  # 边缘检测(edge detection)

        mask1 = cv2.bitwise_and(binary, mask)  # 合并两个提取出来的图像，保留他们共有的地方(merge two extracted images, preserving their common areas)
        roi_image_mask = cv2.bitwise_and(rgb_image, rgb_image, mask=mask1)  # 和原图做遮罩，保留需要识别的区域(create a mask with the original image to retain the area to be identified)

        return roi_image_mask

    def detect(self, bgr_image):
        '''
        颜色检测(color detection)
        :param image: 要进行颜色检测的原图像，格式为bgr，即opencv格式(the original image for color detection, formatted in BGR (OpenCV format))
        :return: 返回原图像和检测的物体的信息(return the original image along with the information of the detected objects)
        '''
        try:
            img_h, img_w = bgr_image.shape[:2]  # 获取原图大小(obtain the size of the original image)
            
            # bgr_image = cv2.warpPerspective(bgr_image, self.perspective_transformation_matrix, (img_w, img_h), flags=cv2.INTER_LINEAR)
            
            image_resize = cv2.resize(bgr_image, (self.size['width'], self.size['height']), interpolation=cv2.INTER_NEAREST)  # 图像缩放, 加快图像处理速度, 不能太小，否则会丢失大量细节(Image scaling to speed up image processing, but not too small, otherwise a large amount of detail will be lost)

            # 计算缩放后的roi(calculate the scaled rio)
            roi = [int(self.roi[0]*self.size['height']),
                   int(self.roi[1]*self.size['height']),
                   int(self.roi[2]*self.size['width']),
                   int(self.roi[3]*self.size['width'])]

            # 在原图上框出roi(frame the roi on the original image)
            cv2.rectangle(bgr_image, (int(img_w*self.roi[2]), int(img_h*self.roi[0])),
                           (int(img_w*self.roi[3]), int(img_h*self.roi[1])), (0, 255, 255), 2, cv2.LINE_8, 0)

            roi_image = image_resize[roi[0]:roi[1], roi[2]:roi[3]]  # 截取roi(crop the roi)

            if self.get_surface:
                roi_image_mask = self.get_top_surface(roi_image)
            else:
                roi_image_mask = roi_image

            image_gb = cv2.GaussianBlur(roi_image_mask, (3, 3), 3)
            image_lab = cv2.cvtColor(image_gb, cv2.COLOR_BGR2LAB)  # bgr空间转lab空间，方便提取颜色(convert from BGR color space to LAB color space for easier color extraction)
            # 物体信息列表:颜色, 位置, 大小, 角度(object information list: color, position, size, angle)
            object_info_list = []
            for color in self.color_list:  # 遍历颜色列表(iterate through color list)
                if color in self.lab_data:  # 如果要识别的颜色在lab里有(if the color to be recognized exists in LAB space)
                    index = 0  # 颜色标签号(color label number)
                    
                    lower = tuple(self.lab_data[color]['min'])
                    upper = tuple(self.lab_data[color]['max'])
                    
                    binary = cv2.inRange(image_lab, lower, upper)  # 二值化(binarization)
                    dilated = self.erode_and_dilate(binary)  # 腐蚀膨胀(erode and dilate)
                    contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出所有轮廓(find out all contours)
                    for c in contours:  # 历遍所有轮廓(iterate through all contours)
                        area = math.fabs(cv2.contourArea(c))  # 计算轮廓面积(calculate contour area)
                        if self.min_area <= area <= self.max_area:  # 根据面积筛选符合的轮廓(filter contours based on area)
                            rect = cv2.minAreaRect(c)  # 获取最小外接矩形(obtain the minimum bounding rectangle)
                            corners = np.int0(cv2.boxPoints(rect))  # 获取最小外接矩形的四个角点(obtain the four corner points of the minimum bounding rectangle)
                            # print(corners)
                            for j in range(4):
                                corners[j, 0], corners[j, 1] = self.point_remapped([corners[j, 0] + roi[2], corners[j, 1] + roi[0]],
                                                                              [self.size['width'], self.size['height']], [img_w, img_h], data_type=int)  # 点映射到原图大小(mapping points to the original image size)
                            x, y = self.point_remapped([rect[0][0] + roi[2], rect[0][1] + roi[0]],
                                                                              [self.size['width'], self.size['height']], [img_w, img_h], data_type=int)  # 点映射到原图大小(mapping points to the original image size)
                            width, height = self.point_remapped([rect[1][0], rect[1][1]],
                                                                              [self.size['width'], self.size['height']], [img_w, img_h], data_type=int)  # 点映射到原图大小(mapping points to the original image size)
                            index += 1 # 序号递增(incremental numbering)
                            color_index = color + str(index)  # 颜色+数字作为标签(color+number as the tag)
                            position = [x, y]  # 获取矩形中心(get rectangle center)
                            size = [width, height]  # 获取矩形大小(get rectangle size)
                            angle = int(round(rect[2]))  # 矩形角度(rectangle angle)
                            
                            # dst = np.dot(self.perspective_transformation_matrix, np.array([[position[0]], [position[1]], [1]]))
                            # print(dst, dst[0, 0]/dst[2, 0], dst[1, 0]/dst[2, 0])
                            # cv2.circle(warped, (int(dst[0, 0]/dst[2, 0]), int(dst[1, 0]/dst[2, 0])), 5, (255, 0, 0), -1) 
                            
                            # 颜色, 位置, 大小, 角度(color, position, size, angle)
                            object_info_list.extend([[color_index, position, size, angle]])

                            cv2.drawContours(bgr_image, [corners], -1, (0, 255, 255), 2, cv2.LINE_AA)  # 绘制矩形轮廓(draw the contour of the rectangle)

            # cv2.imshow('warped', warped)
            reorder_object_info_list = object_info_list
            if object_info_list != []:
                if self.last_object_info_list != []:
                    # 对比上一次的物体的位置来重新排序(reorder based on the position of the previous object)
                    reorder_object_info_list = position_reorder(object_info_list, self.last_object_info_list, self.distance)
            if reorder_object_info_list != []:
                for point in reorder_object_info_list:  # 绘制数字标签(draw digital tag)
                    cv2.putText(bgr_image, point[0], (point[1][0] - 4*len(point[0]), point[1][1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 2)

            self.last_object_info_list = reorder_object_info_list
            
            return bgr_image, reorder_object_info_list  # 返回原图像和物体的信息(return the original image and object information)
        except BaseException as e:
            print('color detect error:', e)
            return bgr_image, [] 
