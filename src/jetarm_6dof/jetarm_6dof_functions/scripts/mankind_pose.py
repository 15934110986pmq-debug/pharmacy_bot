#!/usr/bin/env python3
"""
这个程序实现了人体骨架识别(this program implements human skeleton recognition)
运行现象：桌面显示识别结果画面， 显示人体骨架连线(runtime behavior: the desktop displays the recognition result screen, showing the human skeleton lines)
"""
import os
import cv2
import rospy
from vision_utils import fps
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from hiwonder_interfaces.msg import MultiRawIdPosDur
from jetarm_sdk import bus_servo_control


class PoseNode:
    def __init__(self):
        rospy.init_node("mankind_pose_node", anonymous=True)
        # 实例化一个肢体识别器(instantiate a limb recognizer)
        self.pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=0,
            min_detection_confidence=0.8,
            min_tracking_confidence=0.2
        )
        self.drawing = mp.solutions.drawing_utils # 结果绘制工具(result drawing tool)
        self.fps = fps.FPS() # 帧率计数器(frame rate calculator)

        self.servos_pub = rospy.Publisher('/controllers/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        rospy.sleep(3)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 700), (3, 85), (4, 350), (5, 500), (10, 200)))
        rospy.sleep(2)

        # 订阅图像(subscribe image)
        source_image_topic = rospy.get_param('~source_image_topic', '/camera/image_raw')
        self.image_sub = rospy.Subscriber(source_image_topic, Image, self.image_callback, queue_size=1)

    def image_callback(self, ros_image):
        # rospy.logdebug('Received an image! ')
        # 将 ros 格式画面转为 opencv 格式(convert the ros format image to opencv format)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面(original RGB image)
        rgb_image = cv2.flip(rgb_image, 1)  # 镜像画面, 这样可以正对屏幕和相机看效果(mirror image, aligned with the screen and camera for better visualization)
        result_image = cv2.resize(rgb_image, (int(ros_image.width * 1.6), int(ros_image.height * 1.6))) # 将画面复制一份作为结果，结果绘制在这上面(duplicate the image as the result canvas, and draw the results on it)
        rgb_image = cv2.resize(rgb_image, (int(ros_image.width / 2), int(ros_image.height / 2))) 
        results = self.pose.process(rgb_image)  # 进行识别(perform recognition)
        self.drawing.draw_landmarks(result_image, results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS) # 画出各关节及连线(draw the joints and lines connecting them)
        self.fps.update() # 计算帧率(calculate frame rate)
        result_image = self.fps.show_fps(result_image) # 在结果画面上显示帧率(display the frame rate in the result screen)
        cv2.imshow('image', cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)


if __name__ == "__main__":
    try:
        pose_node = PoseNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

