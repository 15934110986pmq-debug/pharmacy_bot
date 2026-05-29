#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# yolov5目标检测(yolov5 target detection)
import os
import cv2
import rospy
import signal
import numpy as np
import hiwonder_sdk.fps as fps
from hiwonder_sdk import common
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from std_srvs.srv import Empty, EmptyResponse
from yolov5_trt import YoLov5TRT, colors, plot_one_box
from hiwonder_interfaces.msg import ObjectInfo, ObjectsInfo

CONFIG_NAME = '/config'
MODE_PATH = os.path.split(os.path.realpath(__file__))[0]

class Yolov5Node:
    def __init__(self, name):
        rospy.init_node(name)
        
        self.name = name
        self.image_sub = None
        self.start_detect = False
        self.bgr_image = None
        self.running = True

        signal.signal(signal.SIGINT, self.shutdown)

        self.fps = fps.FPS()  # fps计算器(fps calculator)

        # 获取参数(get parameter)
        engine = rospy.get_param('~engine')
        lib = rospy.get_param('~lib')
        conf_thresh = rospy.get_param('~conf_thresh', 0.8)
        self.classes = rospy.get_param('~classes')
        config = rospy.get_param(CONFIG_NAME)
        self.roi = [config['roi']['height'][0],
                    config['roi']['height'][1],
                    config['roi']['width'][0],
                    config['roi']['width'][1]]

        self.yolov5 = YoLov5TRT(os.path.join(MODE_PATH, engine), os.path.join(MODE_PATH, lib), self.classes, conf_thresh)
        
        rospy.Service('~enter', Empty, self.enter_func)
        rospy.Service('~exit', Empty, self.exit_func)
        rospy.Service('~start', Empty, self.start_srv_callback)  # 进入玩法(enter the game)
        rospy.Service('~stop', Empty, self.stop_srv_callback)  # 退出玩法(exit the game)
        rospy.Service('~update_param', Empty, self.update_param)
        self.camera = rospy.get_param('/camera')
        
        self.object_pub = rospy.Publisher('/object/pixel_coords', ObjectsInfo, queue_size=1)
        self.result_image_pub = rospy.Publisher('~image_result', Image, queue_size=1)
        rospy.set_param('~init_finish', True)
        
        self.image_proc()

    def update_param(self, msg):
        config = rospy.get_param(CONFIG_NAME)
        self.roi = [config['roi']['height'][0],
                    config['roi']['height'][1],
                    config['roi']['width'][0],
                    config['roi']['width'][1]]        
        common.loginfo('%s update param'%self.name)

        return EmptyResponse()

    # 开启订阅(start subscribing)
    def enter_func(self, msg):
        self.bgr_image = None
        if self.image_sub is None:
            self.image_sub = rospy.Subscriber('/%s/image_rect_color'%self.camera['camera_name'], Image, self.image_callback, queue_size=1)
        common.loginfo("%s enter"%self.name)

        return EmptyResponse()

    # 注销订阅(unsubscribe from the subscription)
    def exit_func(self, msg):
        if self.image_sub is not None:
            self.image_sub.unregister()
            self.image_sub = None
        self.bgr_image = None
        common.loginfo('%s exit'%self.name)

        return EmptyResponse()

    def start_srv_callback(self, msg):
        common.loginfo("%s start"%self.name)

        self.start_detect = True

        return EmptyResponse()

    def stop_srv_callback(self, msg):
        common.loginfo('%s stop'%self.name)

        self.start_detect = False

        return EmptyResponse()

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the custom image information to image)
        self.bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
   
    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def image_proc(self):
        while self.running:
            if self.bgr_image is not None:
                image = self.bgr_image
                result_image = image.copy()
                try:
                    if self.start_detect:
                        objects_info = []
                        img_h, img_w = image.shape[:2]  # 获取原图大小(get original image size)
                        
                        roi = [int(self.roi[0]*img_h),
                               int(self.roi[1]*img_h),
                               int(self.roi[2]*img_w),
                               int(self.roi[3]*img_w)]
                        image_mask = np.zeros([img_h, img_w], np.uint8)
                        image_mask[roi[0]:roi[1], roi[2]:roi[3]] = 255
                        image_roi = cv2.bitwise_and(image, image, mask=image_mask)
                        
                        boxes, scores, classid = self.yolov5.infer(image_roi)
                        for box, cls_conf, cls_id in zip(boxes, scores, classid):
                            color = colors(cls_id, True)
                            
                            object_info = ObjectInfo()
                            object_info.label = self.classes[cls_id]
                            box = box.astype(int)
                            object_info.center.x = int((box[0] + box[2])/2) 
                            object_info.center.y = int((box[1] + box[3])/2)
                            object_info.size.width = abs(box[0] - box[2]) 
                            object_info.size.height = abs(box[1] - box[3])
                            object_info.height = 0.04
                            objects_info.append(object_info)
                            
                            plot_one_box(
                            box,
                            result_image,
                            color=color,
                            label="{}:{:.2f}".format(
                                self.classes[cls_id], cls_conf
                            ),
                        )
                        self.object_pub.publish(objects_info)
                        # self.fps.update()
                        # result_image = self.fps.show_fps(result_image)
                except BaseException as e:
                    print(e)
                self.result_image_pub.publish(common.cv2_image2ros(result_image, frame_id='yolov5'))
            else:
                rospy.sleep(0.01)
        self.yolov5.destroy() 
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    node = Yolov5Node('yolov5')
