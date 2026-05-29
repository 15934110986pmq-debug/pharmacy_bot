#!/usr/bin/env python3
# coding: utf8

import os
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image as RosImage
from hiwonder_interfaces.msg import MultiRawIdPosDur
from vision_utils import fps, colors, get_area_max_contour
from jetarm_sdk import pid, bus_servo_control



class ColorTracker:
    def __init__(self, target_color):
        self.target_color = target_color
        self.pid_yaw = pid.PID(55.5, 0, 1.2)
        self.pid_pitch = pid.PID(45.5, 0, 1.2)
        self.yaw = 500
        self.pitch = 350
    
    def proc (self, source_image, result_image, color_ranges):
        h, w = source_image.shape[:2]
        color = color_ranges[self.target_color]

        img = cv2.resize(source_image, (160, 120))
        img_blur = cv2.GaussianBlur(img, (3, 3), 3) # 高斯模糊(Gaussian blur)
        img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB) # 转换到 LAB 空间(convert to LAB space)
        mask = cv2.inRange(img_lab, tuple(color['min']), tuple(color['max'])) # 二值化(binarization)

        # 平滑边缘，去除小块，合并靠近的块(smooth the edges, remove small patches, and merge neighboring patches)
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

        # 找出最大轮廓(find out the largest contour)
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        max_contour_area = get_area_max_contour(contours, 10)

        # 如果有符合要求的轮廓(if there are contours meet the requirement)
        if max_contour_area is not None:
            (center_x, center_y), radius = cv2.minEnclosingCircle(max_contour_area[0]) # 最小外接圆(the minimum bounding circle)

            # 圈出识别的的要追踪的色块(outline the tracked color block for recognition)
            circle_color = colors.rgb[self.target_color] if self.target_color in colors.rgb else (0x55, 0x55, 0x55)
            cv2.circle(result_image, (int(center_x * 4), int(center_y * 4)), int(radius * 4), circle_color, 2)

            center_x = center_x * 4 / w
            if abs(center_x - 0.5) > 0.02: # 相差范围小于一定值就不用再动了(if the difference range is smaller than a certain value, there's no need to move anymore)
                self.pid_yaw.SetPoint = 0.5 # 我们的目标是要让色块在画面的中心, 就是整个画面的像素宽度的 1/2 位置(our goal is to position the color block at the center of the frame, which corresponds to half the pixel width of the entire frame)
                self.pid_yaw.update(center_x)
                self.yaw = min(max(self.yaw + self.pid_yaw.output, 0), 1000)
            else:
                self.pid_yaw.clear() # 如果已经到达中心了就复位一下 pid 控制器(reset the PID controller if the center has been reached)

            center_y = center_y * 4/ h
            if abs(center_y - 0.5) > 0.02:
                self.pid_pitch.SetPoint = 0.5
                self.pid_pitch.update(center_y)
                self.pitch = min(max(self.pitch + self.pid_pitch.output, 100), 740)
            else:
                self.pid_pitch.clear()
            # rospy.loginfo("x:{:.2f}\ty:{:.2f}".format(self.x , self.y))
            return (result_image, (self.pitch, self.yaw))
        else:
            return (result_image, None)


class ObjectTrackingNode:
    def __init__(self, node_name, log_level=rospy.INFO):
        rospy.init_node(node_name, anonymous=True, log_level=log_level)

        self.tracker = None
        self.fps = fps.FPS()
        self.color_ranges = rospy.get_param('/config/lab', {})
        self.servos_pub = rospy.Publisher('/controllers/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        rospy.sleep(3)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 700), (3, 85), (4, 350), (5, 500), (10, 200)))
        rospy.sleep(2)
        self.target_color = rospy.get_param('~target_color', 'red')
        rospy.loginfo("正在设置将要追踪的目标认识为" + self.target_color)
        self.tracker = ColorTracker(self.target_color)
        self.source_image_topic = rospy.get_param('~source_image_topic', '/camera/image_raw')
        rospy.loginfo("订阅原图像节点 " + self.target_color)
        self.image_sub = rospy.Subscriber(self.source_image_topic, RosImage, self.image_callback, queue_size=1)

    def image_callback(self, ros_image: RosImage):
        # rospy.logdebug('Received an image! ')
        # 将ros格式图像转换为opencv格式(convert the ros format image to opencv format)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        result_image = np.copy(rgb_image)

        if self.tracker is not None:
            color_ranges = rospy.get_param('/config/lab', self.color_ranges)
            result_image, p_y = self.tracker.proc(rgb_image, result_image, color_ranges)
            if p_y is not None:
                bus_servo_control.set_servos(self.servos_pub, 30, ((1, p_y[1]), (4, p_y[0])))
        # 计算帧率及发布结果图像(calculate the frame rate and publish the resulting image)
        self.fps.update()
        result_image = self.fps.show_fps(result_image)
        cv2.imshow("color_tracking", cv2.resize(cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR), (320, 240))) # 显示小尺寸会更快更节省资源 , 想要大把resize去掉(Display smaller sizes would be faster and more resource-efficient, if you want to remove resizing altogether, that's fine)
        cv2.waitKey(1)


if __name__ == "__main__":
    node = ObjectTrackingNode("color_tracking", log_level=rospy.DEBUG)
    rospy.spin()
