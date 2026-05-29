#!/usr/bin/env python3
# coding: utf8

import os
import sys
import cv2
import time
import rospy
import queue
import numpy as np
from yolov5_onnx import YOLOV5
from sensor_msgs.msg import Image
from hiwonder_interfaces.msg import MultiRawIdPosDur
from jetarm_sdk import bus_servo_control



TRT_INPUT_SIZE = 160
TRT_NUM_CLASSES = 2
TRT_CLASS_NAMES = ("mask", "nomask", "mask_incorrect")
COLORS = ((0, 0, 255), (255, 0, 0), (0, 255, 0))


class FacemaskNode:
    def __init__(self):
        rospy.init_node('facemask_node')

        # 建立Yolo实例(establish a YOLO instance)
        weights = '/home/ubuntu/weights/face_mask/face_mask.onnx'
        self.yolov5 = YOLOV5(weights, TRT_CLASS_NAMES, 0.90)

        self.servos_pub = rospy.Publisher('/controllers/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        rospy.sleep(2)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 700), (3, 85), (4, 350), (5, 500), (10, 200)))
        rospy.sleep(1)

        # 订阅相机图像话题(subscribe camera image topic)
        self.image_queue = queue.Queue(maxsize=2)
        source_image_topic = rospy.get_param('~source_image_topic', 'camera/image_rect_color')
        self.image_sub = rospy.Subscriber(source_image_topic, Image, self.image_callback, queue_size=1)
    
    def image_callback(self, ros_image: Image):
        #rospy.logdebug('Received an image! ')
        try:
            self.image_queue.put_nowait(ros_image) # 将图片压入队列(push the image into the queue)
        except Exception as e:
            pass

    def image_process(self):
        ros_image = self.image_queue.get(block=True) # 从队列里面取出画面(retrieve the frame from the queue)

        # 将画面转为 opencv 格式(convert the screen to opencv format)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        result_image = np.copy(rgb_image)

        try:
            # outputs = self.yolov5.detect(rgb_image) # 对画面进行识别(recognize screen)
            # 后处理, 将原始输出转换为边界框,进行 NMS 阈值处理等(post-processing: Convert the raw output to bounding boxes, perform Non-Maximum Suppression (NMS), and apply thresholding)
            boxes, confs, classes = self.yolov5.inference(rgb_image) 

            for box, cls_id, cls_conf in zip(boxes, classes, confs):
                x1 = box[0] 
                y1 = box[1] 
                x2 = box[2] 
                y2 = box[3] 

                # 结果画面中显示是否戴口罩(display whether the person is wearing a mask in the result image)
                cv2.putText(result_image, 
                            TRT_CLASS_NAMES[cls_id] + " " + str(float(cls_conf))[:4],
                            (int(x1), int(y1) - 5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLORS[cls_id], 2)
                # 结果画面中框出口罩(box out the masks in the result image)
                cv2.rectangle(result_image, 
                              (int(x1), int(y1)), (int(x2), int(y2)), 
                              COLORS[cls_id], 3)

                rospy.loginfo((cls_id, float(cls_conf), x1, x2, y1, y2))
        except Exception as e:
            rospy.logerr(str(e))
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow("image", result_image)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        facemask_node = FacemaskNode()
        while not rospy.is_shutdown():
            facemask_node.image_process()
    except Exception as e:
        rospy.logerr(str(e))

