#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/02/04
# @author:aiden
# 订阅摄像头图像，进行图像处理，发布相应的物体姿态信息和图像(Subscribe to camera images, process the images, and publish corresponding object pose information and images)
import cv2
import rospy
import signal
import numpy as np

from sensor_msgs.msg import Image
from std_srvs.srv import Empty, EmptyResponse
from hiwonder_interfaces.msg import ObjectInfo, ObjectsInfo

from jetarm_sdk import common
from color_detection import ColorDetection

CONFIG_NAME = '/config'
class ColorDetectionNode:
    def __init__(self, name):
        # 初始化节点(initialize node)
        rospy.init_node(name, log_level=rospy.INFO)
        self.name = name

        self.image = None
        self.running = True
        self.image_sub = None
        self.start_detect = False
        signal.signal(signal.SIGINT, self.shutdown)

        # 获取参数(get parameter)
        config = rospy.get_param(CONFIG_NAME)
        color_list = rospy.get_param('~color_list')
        distance = rospy.get_param('~distance')
        self.debug = rospy.get_param('~debug')
        self.source_image_topic = rospy.get_param('~source_image_topic', '/camera/image_raw')

        # 实例化颜色识别类(instantiate the color recognition class)
        self.config = config
        self.detect = ColorDetection(config, color_list, distance)

        # 检测图像发布(detect image publish)
        self.image_pub = rospy.Publisher('~image_result', Image, queue_size=1)

        # 物体位姿发布(publish object pose)
        self.color_info_pub = rospy.Publisher('/object/pixel_coords', ObjectsInfo, queue_size=1)

        rospy.Service('~enter', Empty, self.enter_func)
        rospy.Service('~exit', Empty, self.exit_func)
        rospy.Service('~start', Empty, self.start_func)
        rospy.Service('~stop', Empty, self.stop_func)
        rospy.Service('~update_color', Empty, self.update_color)
        rospy.Service('~update_param', Empty, self.update_param)

        rospy.sleep(0.2)
        common.loginfo("%s init finish"%self.name)
        if self.debug:
            self.enter_func(None)
            self.start_func(None)

        self.enable_display = rospy.get_param('~enable_display', False)
        self.image_proc()

    def shutdown(self, signum, frame):
        self.running = False
    
    # 开启订阅(start subscribing)
    def enter_func(self, msg):
        # 获取参数(get parameter)
        self.image = None
        if self.image_sub is None:
            # camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/camera_info')
            #self.image_sub = rospy.Subscriber('/%s/image_rect_color'%self.camera['camera_name'], Image, self.image_callback)  # 订阅校准后的图像(subscribe to the calibrated image)
            self.image_sub = rospy.Subscriber(self.source_image_topic, Image, self.image_callback)
        common.loginfo("%s enter"%self.name)

        return EmptyResponse()

    # 注销订阅(unsubscribe from the subscription)
    def exit_func(self, msg):
        if self.image_sub is not None:
            self.image_sub.unregister()
            self.image_sub = None
        self.image = None
        self.start_detect = False
        common.loginfo('%s exit'%self.name)

        return EmptyResponse()

    # 开启检测(start detection)
    def start_func(self, msg):
        self.start_detect = True
        common.loginfo("%s start"%self.name)

        return EmptyResponse()

    # 停止检测(stop detection)
    def stop_func(self, msg):
        self.start_detect = False
        common.loginfo("%s stop"%self.name)

        return EmptyResponse()

    # 更新颜色列表(update color list)
    def update_color(self, msg):
        color_list = rospy.get_param('~color_list')
        self.detect.update_color(color_list)
        common.loginfo('update color list')

        return EmptyResponse()

    # 更新config参数(update config parameter)
    def update_param(self, msg):
        config = rospy.get_param(CONFIG_NAME)
        self.detect.update_config(config)
        common.loginfo('%s update param'%self.name)

        return EmptyResponse()

    def image_proc(self):
        while self.running:
            if self.image is not None:
                image = self.image.copy()
                if self.start_detect:  # 如果开启检测(if detection is started)
                    self.config['lab'] = rospy.get_param('/config/lab')
                    self.detect.update_config(self.config)
                    frame_result, poses = self.detect.detect(image)  # 颜色检测(color detection)
                    if poses != []:
                        colors_info = []
                        for p in poses:
                            color_info = ObjectInfo()
                            color_info.label = p[0]
                            color_info.center.x = p[1][0]
                            color_info.center.y = p[1][1]
                            color_info.size.width = p[2][0]
                            color_info.size.height = p[2][1]
                            color_info.yaw = p[3]
                            color_info.height = 0.03
                            colors_info.append(color_info)
                        self.color_info_pub.publish(colors_info)  # 发布位姿(publish position)
                else:
                    frame_result = image
                    rospy.sleep(0.01)

                if self.enable_display:
                    cv2.imshow('color_detection', frame_result)
                    cv2.waitKey(1)

                ros_image = common.cv2_image2ros(frame_result, self.name) # opencv格式转为ros(convert opencv format to ros)
                self.image_pub.publish(ros_image)  # 发布图像(publish image)
            else:
                rospy.sleep(0.01)

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                           buffer=ros_image.data)  # 将ros格式图像消息转化为opencv格式(convert the image information from ros format to opencv format)
        self.image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

if __name__ == '__main__':
    ColorDetectionNode('color_detection')
