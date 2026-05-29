#!/usr/bin/env python3
# coding: utf8

import os
import cv2
import sys
import threading
import rospy
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image as RosImage
from hiwonder_interfaces.msg import MultiRawIdPosDur
from vision_utils import box_center, distance
from jetarm_sdk import bus_servo_control, pid
from utils import show_faces, mp_face_location
from vision_utils import fps
import gc




class FaceTracker:
    def __init__(self):
        self.face_detector = mp.solutions.face_detection.FaceDetection(
            min_detection_confidence=0.5,
        )
        self.pid_yaw = pid.PID(25.5, 0, 6.2)
        self.pid_pitch = pid.PID(15.5, 0, 6.2)
        self.detected_face = 0 
        self.yaw = 500
        self.pitch = 350

    def proc(self, source_image, result_image):
        results = self.face_detector.process(source_image)
        boxes, keypoints = mp_face_location(results, source_image)
        o_h, o_w = source_image.shape[:2]

        if len(boxes) > 0:
            self.detected_face += 1 
            self.detected_face = min(self.detected_face, 20) # 让计数总是不大于20(ensure that the count is never greater than 20)

            # 连续 5 帧识别到了人脸就开始追踪, 避免误识别(start tracking if a face is detected in five consecutive frames to avoid false positives)
            if self.detected_face >= 5:
                center = [box_center(box) for box in boxes] # 计算所有人脸的中心坐标(calculate the center coordinate of all human faces)
                dist = [distance(c, (o_w / 2, o_h / 2)) for c in center] # 计算所有人脸中心坐标到画面中心的距离(calculate the distance from the center of each detected face to the center of the screen)
                face = min(zip(boxes, center, dist), key=lambda k: k[2]) # 找出到画面中心距离最小的人脸(identify the face with the minimum distance to the center of the screen)

                # 计算要追踪的人脸距画面中心的x轴距离(0~1)。(calculate the x-axis distance (0~1) of the face to be tracked from the center of the screen)
                c_x, c_y = face[1]
                dist_x = c_x / o_w
                dist_y = c_y / o_h

                if abs(dist_y - 0.5) > 0.01:
                    self.pid_pitch.SetPoint = 0.5
                    self.pid_pitch.update(dist_y) # 更新俯仰角 pid 控制器(update the pitch angle PID controller)
                    self.pitch = min(max(self.pitch + self.pid_pitch.output, 100), 740)  # 获取新的俯仰角并限制运动范围(retrieve the new pitch angle and limit the range of motion)
                else:
                    self.pid_pitch.clear()

                if abs(dist_x - 0.5) > 0.01:
                    self.pid_yaw.SetPoint = 0.5
                    self.pid_yaw.update(dist_x) # 更新偏航角 pid 控制器(update the yaw angle PID controller)
                    self.yaw = min(max(self.yaw + self.pid_yaw.output, 0),  1000)  # 获取新的偏航角并限制运动范围(retrieve the new pitch angle and limit the range of motion)
                else:
                    self.pid_yaw.clear()

        else: # 这里是没有识别到人脸的处理(here is the processing for when no face is detected)
            gc.collect()
            if self.detected_face > 0:
                self.detected_face -= 1
            else:
                self.pid_pitch.clear()
                self.pid_yaw.clear()

        result_image = show_faces(source_image, result_image, boxes, keypoints) # 在画面中显示识别到的人脸和脸部关键点(display the detected faces and facial key points on the screen)
        return result_image, (self.pitch, self.yaw)


class ObjectTrackingNode:
    def __init__(self, node_name, log_level=rospy.INFO):
        rospy.init_node(node_name, anonymous=True, log_level=log_level)

        self.fps = fps.FPS()
        self.thread = None

        self.servos_pub = rospy.Publisher('/controllers/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        rospy.sleep(2)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 700), (3, 85), (4, 350), (5, 500), (10, 200)))
        rospy.sleep(1)

        self.tracker = FaceTracker()
        self.source_image_topic = rospy.get_param('~source_image_topic', '/camera/image_raw')
        self.image_sub = rospy.Subscriber(self.source_image_topic, RosImage, self.image_callback, queue_size=10)
        rospy.loginfo("已开启人脸追踪")


    def image_callback(self, ros_image: RosImage):
        # rospy.logdebug('Received an image! ')
        # 将ros格式图像转换为opencv格式(convert the rod format image to opencv format)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        result_image = np.copy(rgb_image)

        result_image, p_y = self.tracker.proc(rgb_image, result_image)
        if p_y is not None:
            bus_servo_control.set_servos(self.servos_pub, 30, ((1, p_y[1]), (4, p_y[0])))

        # 计算帧率及发布结果图像(calculate the frame rate and publish the resulting image)
        #self.fps.update()
        #result_image = self.fps.show_fps(result_image)
        cv2.imshow("face_tracking", cv2.resize(cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR), (320, 240)))
        cv2.waitKey(1)


if __name__ == "__main__":
    node = ObjectTrackingNode("face_tracking", log_level=rospy.DEBUG)
    rospy.spin()

