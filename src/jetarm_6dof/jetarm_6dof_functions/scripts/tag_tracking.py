#!/usr/bin/env python3
import os
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from vision_utils import fps, draw_tags
from hiwonder_interfaces.msg import MultiRawIdPosDur
from dt_apriltags import Detector
from jetarm_sdk import bus_servo_control, pid


class TagTrackingNode:
    def __init__(self):
        rospy.init_node('tag_tracking_node')

        self.pid_yaw = pid.PID(20.5, 0, 7.5)
        self.pid_pitch = pid.PID(17.0, 0, 7.5)
        self.yaw = 500
        self.pitch = 350

        self.tracker = None
        self.enable_select = False

        self.at_detector = Detector(searchpath=['apriltags'], 
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)

        self.fps = fps.FPS() # 帧率统计器(frame rate counter)
        self.servos_pub = rospy.Publisher('/controllers/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        rospy.sleep(3)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 700), (3, 85), (4, 350), (5, 500), (10, 200)))
        rospy.sleep(2)

        # 订阅相机图像话题(subscribe camera image topic)
        source_image_topic = rospy.get_param('~source_image_topic', '/usb_cam/image_rect_color')
        rospy.loginfo("订阅原图像节点 " + source_image_topic)
        self.image_sub = rospy.Subscriber(source_image_topic, Image, self.image_callback, queue_size=2)


    def image_callback(self, ros_image):
        #rospy.logdebug('Received an image! ')
        # 将画面转为 opencv 格式(convert the screen to opencv format)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        result_image = np.copy(rgb_image)
        factor = 4
        #rgb_image = cv2.resize(rgb_image, (int(rgb_image[1] / factor), int(rgb_image[0] / factor)))
        tags = self.at_detector.detect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY), False, None, 0.025)
        tags = sorted(tags, key=lambda tag: tag.tag_id) # 貌似出来就是升序排列的不需要手动进行排列(if the result is already sorted in ascending order, manual sorting is not necessary)
        draw_tags(result_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))
        if len(tags) > 0 and tags[0].tag_id == 1:
            center_x, center_y = tags[0].center
            center_x = center_x / rgb_image.shape[1]
            if abs(center_x - 0.5) > 0.02: # 相差范围小于一定值就不用再动了(if the difference range is smaller than a certain value, there's no need to move anymore)
                self.pid_yaw.SetPoint = 0.5 # 我们的目标是要让色块在画面的中心, 就是整个画面的像素宽度的 1/2 位置(our goal is to position the color block at the center of the frame, which corresponds to half the pixel width of the entire frame)
                self.pid_yaw.update(center_x)
                self.yaw = min(max(self.yaw + self.pid_yaw.output, 0), 1000)
            else:
                self.pid_yaw.clear() # 如果已经到达中心了就复位一下 pid 控制器(reset the PID controller if the center has been reached)

            center_y = center_y / rgb_image.shape[0]
            if abs(center_y - 0.5) > 0.02:
                self.pid_pitch.SetPoint = 0.5
                self.pid_pitch.update(center_y)
                self.pitch = min(max(self.pitch + self.pid_pitch.output, 100), 740)
            else:
                self.pid_pitch.clear()
            bus_servo_control.set_servos(self.servos_pub, 50, ((1, self.yaw), (4, self.pitch)))

        self.fps.update()
        self.fps.show_fps(result_image)
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow("tag_tracking", result_image)
        key = cv2.waitKey(1)


if __name__ == '__main__':
    try:
        tag_tracking = TagTrackingNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

