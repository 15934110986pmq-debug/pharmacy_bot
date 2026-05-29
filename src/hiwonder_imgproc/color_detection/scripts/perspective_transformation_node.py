#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/02/21
# @author:aiden
# 订阅摄像头图像，检测白色识别区域进行透视变换处理，发布处理后的图像(Subscribe to the camera image, detect the white recognition area for perspective transformation, process the image, and publish the processed image)
import cv2
import yaml
import rospy
import signal
import threading
import numpy as np
import pandas as pd

from sensor_msgs.msg import Image
from std_srvs.srv import Empty, EmptyResponse

from jetarm_sdk import common

CONFIG_NAME = '/config'
class PerspectiveTransformationNode:
    def __init__(self, name):
        # 初始化节点(initialize node)
        rospy.init_node(name, log_level=rospy.INFO)
        self.name = name

        self.lock = threading.RLock()

        self.config = rospy.get_param(CONFIG_NAME)
        self.lab_data = self.config['lab']

        # roi区域，先缩放再截取(roi area, scale first then crop)
        self.roi = [self.config['roi']['height'][0],
                    self.config['roi']['height'][1],
                    self.config['roi']['width'][0],
                    self.config['roi']['width'][1]]

        # 图像缩放比例系数(0, 1)，加速识别(image scaling factor (0, 1) to accelerate recognition)
        self.size = self.config['image_proc_size']
        
        self.src_points = None
        scale = self.config['white_area_world_size']['height']/self.config['white_area_world_size']['width']
        width = self.config['white_area_pixel_size']['width']
        camera_resolution = self.config['camera_resolution']
        camera_width = camera_resolution['width']
        camera_height = camera_resolution['height']
        self.dst_points = np.float32(
            [[int((camera_width - width)/2), int((camera_height + width*scale)/2)],
             [int((camera_width - width)/2), int((camera_height - width*scale)/2)],
             [int((camera_width + width)/2), int((camera_height - width*scale)/2)],
             [int((camera_width + width)/2), int((camera_height + width*scale)/2)]])

        self.left_top_x = []
        self.left_top_y = []
        self.right_top_x = []
        self.right_top_y = []
        self.left_down_x = []
        self.left_down_y = []
        self.right_down_x = []
        self.right_down_y = []
        
        self.yaw = 0
        self.count = 0
        self.save = True
        self.bgr_image = None
        self.running = True
        self.image_sub = None
        self.tag_sub = None
        self.start_calibration = False

        signal.signal(signal.SIGINT, self.shutdown)

        self.source_image_topic = rospy.get_param('~source_image_topic', '/camera/image_raw')
        # 获取参数(get parameter)
        self.debug = rospy.get_param('~debug')

        # 检测图像发布(detect image publish)
        self.image_pub = rospy.Publisher('~image_result', Image, queue_size=1)

        rospy.Service('~enter', Empty, self.enter_func)
        rospy.Service('~exit', Empty, self.exit_func)
        rospy.Service('~start', Empty, self.start_func)
        rospy.Service('~stop', Empty, self.stop_func)
        rospy.Service('~save', Empty, self.save_func)
        rospy.Service('~update_param', Empty, self.update_param)

        rospy.sleep(0.2)
        
        if self.debug: 
            self.enter_func(None)
            self.start_func(None)
        
        common.loginfo("%s init finish"%self.name)
        self.image_proc()

    def shutdown(self, signum, frame):
        self.running = False

    def update_param(self, msg):
        self.config = rospy.get_param(CONFIG_NAME)
        self.lab_data = self.config['lab']

        # roi区域，先缩放再截取(roi area, scale first then crop)
        self.roi = [self.config['roi']['height'][0],
                    self.config['roi']['height'][1],
                    self.config['roi']['width'][0],
                    self.config['roi']['width'][1]]

        # 图像缩放比例系数(0, 1)，加速识别(image scaling factor (0, 1) to accelerate recognition)
        self.size = self.config['image_proc_size']
        
        scale = self.config['white_area_world_size']['height']/self.config['white_area_world_size']['width']
        width = self.config['white_area_pixel_size']['width']
        camera_resolution = self.config['camera_resolution']
        camera_width = camera_resolution['width']
        camera_height = camera_resolution['height']
        self.dst_points = np.float32(
            [[int((camera_width - width)/2), int((camera_height + width*scale)/2)],
             [int((camera_width - width)/2), int((camera_height - width*scale)/2)],
             [int((camera_width + width)/2), int((camera_height - width*scale)/2)],
             [int((camera_width + width)/2), int((camera_height + width*scale)/2)]])
        common.loginfo('%s update param'%self.name)

        return EmptyResponse()

    def enter_func(self, msg):
        if self.image_sub is None:
            #camera_name = rospy.get_param('/camera/camera_name', 'camera')  # 获取参数(obtain parameter)
            #image_topic = rospy.get_param('/camera/image_topic', 'image_rect_color')  # 获取参数(obtain parameter)
            #self.image_sub = rospy.Subscriber('/{}/{}'.format(camera_name, image_topic), Image, self.image_callback)  # 订阅原始图像(subscribe the original image)
            self.image_sub = rospy.Subscriber(self.source_image_topic, Image, self.image_callback)
        common.loginfo("%s enter"%self.name)

        return EmptyResponse()

    # 注销订阅(unsubscribe from the subscription)
    def exit_func(self, msg):
        if self.image_sub is not None:
            self.image_sub.unregister()
            self.image_sub = None
        common.loginfo('%s exit'%self.name)

        return EmptyResponse()

    # 开启检测(start detection)
    def start_func(self, msg):
        with self.lock:
            self.count = 0
            self.save = True
            self.src_points = None
            self.start_calibration = True
        common.loginfo("%s start"%self.name)

        return EmptyResponse()

    # 停止检测(stop detection)
    def stop_func(self, msg):
        with self.lock:
            self.start_calibration = False
            self.src_points = None
        common.loginfo("%s stop"%self.name)

        return EmptyResponse()
    
    def save_func(self, msg):
        with self.lock:
            self.save = True
        
        return EmptyResponse()

    def average_process(self, data):
        data1 = pd.DataFrame(data)
        data2 = data1.copy()
        u = data2.mean()
        std = data2.std()

        data_c = data1[np.abs(data1 - u) <= std]

        return round(data_c.mean()[0])

    def save_yaml_data(self, data, yaml_file):
        f = open(yaml_file, 'w', encoding='utf-8')
        yaml.dump(data, f)
        rospy.set_param(CONFIG_NAME, data)
        rospy.ServiceProxy('/update_config', Empty)

        f.close()

    def image_proc(self):
        while self.running:
            if self.bgr_image is not None:
                warped_image = self.bgr_image.copy()
                if self.start_calibration:  # 如果开启检测(if start detection)
                    bgr_image = self.bgr_image.copy()
                    img_h, img_w = bgr_image.shape[:2]  # 获取原图大小(get original image size)
                    image_resize = cv2.resize(bgr_image, (self.size['width'], self.size['height']), interpolation=cv2.INTER_NEAREST)  # 图像缩放, 加快图像处理速度, 不能太小，否则会丢失大量细节(Image scaling to speed up image processing, but not too small, otherwise a large amount of detail will be lost)

                    # 计算缩放后的roi(calculate the scaled roi)
                    roi = [int(self.roi[0]*self.size['height']),
                           int(self.roi[1]*self.size['height']),
                           int(self.roi[2]*self.size['width']),
                           int(self.roi[3]*self.size['width'])]

                    roi_image = image_resize[roi[0]:roi[1], roi[2]:roi[3]]  # 截取roi(crop roi)
              
                    image_lab = cv2.cvtColor(roi_image, cv2.COLOR_BGR2LAB)
                    lower = self.lab_data['white']['min']
                    upper = self.lab_data['white']['max']
                    binary = cv2.inRange(image_lab, tuple(lower), tuple(upper))
                    
                    element = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
                    eroded = cv2.erode(binary, element)  # 腐蚀(erode)
                    dilated = 255 - cv2.dilate(eroded, element)  # 膨胀(dilate)
                    
                    contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出所有轮廓(find out all contours)
                    max_contour = common.get_area_max_contour(contours, 10)[0]
                    approx = cv2.approxPolyDP(max_contour, 75, True)
                    for j in range(4):
                        approx[:, 0][j, 0], approx[:, 0][j, 1] = common.point_remapped([approx[:, 0][j, 0] + roi[2], approx[:, 0][j, 1] + roi[0]],
                                                                      [self.size['width'], self.size['height']], [img_w, img_h])  # 点映射到原图大小(map points to the original image size)
                    
                    cv2.polylines(bgr_image, [approx], True, (255, 0, 255), 2)

                    corners = np.argpartition(approx[..., 0][:, 0], -2)
                    left = approx[corners[:2]][:, 0]
                    right = approx[corners[-2:]][:, 0]
                    if left[0, 1] < left[1, 1]:
                        left_top = left[0, :]
                        left_down = left[1, :]
                    else:
                        left_top = left[1, :]
                        left_down = left[0, :]

                    if right[0, 1] < right[1, 1]:
                        right_top = right[0, :]
                        right_down = right[1, :]
                    else:
                        right_top = right[1, :]
                        right_down = right[0, :]
                    
                    if len(approx) == 4:
                        self.count += 1
                    if self.count > 10:
                        self.right_top_x.append(right_top[0])
                        self.right_top_y.append(right_top[1])
                        self.left_top_x.append(left_top[0])
                        self.left_top_y.append(left_top[1])
                        self.left_down_x.append(left_down[0])
                        self.left_down_y.append(left_down[1])
                        self.right_down_x.append(right_down[0])
                        self.right_down_y.append(right_down[1])
                    if self.count > 15:
                        right_top_point = [self.average_process(self.right_top_x), self.average_process(self.right_top_y)]
                        right_down_point = [self.average_process(self.right_down_x), self.average_process(self.right_down_y)]
                        left_top_point = [self.average_process(self.left_top_x), self.average_process(self.left_top_y)]
                        left_down_point = [self.average_process(self.left_down_x), self.average_process(self.left_down_y)]
                        # left_down
                        # left_up
                        # right_up
                        # right_down
                        self.src_points = np.float32(
                            [left_down_point,
                            left_top_point,
                            right_top_point,
                            right_down_point])
                        
                        self.count = 0
                        self.save = True
                        self.start_calibration = False
                        common.loginfo('finish calibration')
                    if self.debug:
                        cv2.imshow('bgr_image', bgr_image)
                else:
                    if self.src_points is not None:
                        warped_image, matrix, matrix_inv = common.perspective_transform(self.bgr_image, self.src_points, self.dst_points, True)
                        print(self.dst_points[1, :].astype(int).tolist())
                        print(self.dst_points[3, :].tolist())
                        cv2.rectangle(warped_image, tuple(self.dst_points[0, :].astype(int).tolist()), tuple(self.dst_points[2, :].astype(int).tolist()), (255, 255, 0), 2)
                        if self.save:
                            self.config['perspective_transformation_matrix'] = matrix.tolist()
                            self.config['perspective_transformation_matrix_inv'] = matrix_inv.tolist()
                            self.save_yaml_data(self.config, '/home/ubuntu/jetarm/src/hiwonder_imgproc/color_detection/config/config.yaml') 
                            self.save = False
                            common.loginfo('save config')
                    rospy.sleep(0.01)
                ros_image = common.cv2_image2ros(warped_image, self.name)
                self.image_pub.publish(ros_image)  # 发布图像(publish image)
                
                if self.debug:
                    cv2.imshow('warped_image', warped_image)
                    cv2.waitKey(1)
            else:
                rospy.sleep(0.01)

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                           buffer=ros_image.data)  # 将ros格式图像消息转化为opencv格式(convert the image information from the ros format to opencv format)
        self.bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

if __name__ == '__main__':
    PerspectiveTransformationNode('perspective_transformation')
