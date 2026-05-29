#!/usr/bin/env python3
import os
import cv2
import rospy
import queue
import numpy as np
from sensor_msgs.msg import Image
from vision_utils import fps, box_center
from hiwonder_interfaces.msg import MultiRawIdPosDur
from jetarm_sdk import pid, bus_servo_control


class KCFTrackingNode:
    def __init__(self):
        rospy.init_node('kcf_node')

        self.pid_yaw = pid.PID(15.5, 0, 3.5)
        self.pid_pitch = pid.PID(15.5, 0, 3.5)
        self.yaw = 500
        self.pitch = 350

        self.tracker = None
        self.enable_select = False
        self.fps = fps.FPS() # 帧率统计器(frame rate counter)
        self.servos_pub = rospy.Publisher('/controllers/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        rospy.sleep(3)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 700), (3, 85), (4, 350), (5, 500), (10, 200)))
        rospy.sleep(2)

        # 订阅相机图像话题(subscribe camera image topic)
        self.image_queue = queue.Queue(maxsize=2)
        source_image_topic = rospy.get_param('~source_image_topic', 'camera/image_rect_color')
        rospy.loginfo("订阅原图像节点 " + source_image_topic)
        self.image_sub = rospy.Subscriber(source_image_topic, Image, self.image_callback, queue_size=2)


    def image_callback(self, ros_image):
        #rospy.logdebug('Received an image! ')
        # 将画面转为 opencv 格式(convert image to opencv format)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        result_image = np.copy(rgb_image)
        factor = 4
        rgb_image = cv2.resize(rgb_image, (int(ros_image.width / factor), int(ros_image.height / factor)))

        try:
            if self.tracker is None:
                if self.enable_select:
                    roi = cv2.selectROI("image", cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR), False)
                    roi =  tuple(int(i / factor)for i in roi)
                    if roi:
                        param = cv2.TrackerKCF.Params()
                        param.detect_thresh = 0.2
                        self.tracker = cv2.TrackerKCF_create(param)
                        self.tracker.init(rgb_image, roi)
            else:
                status, box = self.tracker.update(rgb_image)
                if status:
                    # rospy.loginfo(str(box))
                    p1 = int(box[0] * factor), int(box[1] * factor)
                    p2 = p1[0] + int(box[2] * factor), p1[1] + int(box[3] * factor)
                    cv2.rectangle(result_image, p1, p2, (255, 255, 0), 2)
                    center_x, center_y = (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2
                    # print(center_x,  center_y, result_image.shape[:2], rgb_image.shape[:2])

                    center_x = center_x / result_image.shape[1]
                    if abs(center_x - 0.5) > 0.02: # 相差范围小于一定值就不用再动了(if the difference range is smaller than a certain value, there's no need to move anymore)
                        self.pid_yaw.SetPoint = 0.5 # 我们的目标是要让色块在画面的中心, 就是整个画面的像素宽度的 1/2 位置(our goal is to position the color block at the center of the frame, which corresponds to half the pixel width of the entire frame)
                        self.pid_yaw.update(center_x)
                        self.yaw = min(max(self.yaw + self.pid_yaw.output, 0), 1000)
                    else:
                        self.pid_yaw.clear() # 如果已经到达中心了就复位一下 pid 控制器(reset the PID controller if the center has been reached)

                    center_y = center_y / result_image.shape[0]
                    if abs(center_y - 0.5) > 0.02:
                        self.pid_pitch.SetPoint = 0.5
                        self.pid_pitch.update(center_y)
                        self.pitch = min(max(self.pitch + self.pid_pitch.output, 100), 740)
                    else:
                        self.pid_pitch.clear()
                    bus_servo_control.set_servos(self.servos_pub, 30, ((1, self.yaw), (4, self.pitch)))
                    # rospy.loginfo("x:{:.2f}\ty:{:.2f}".format(self.x , self.y))

        except Exception as e:
            rospy.logerr(str(e))


        self.fps.update()
        self.fps.show_fps(result_image)
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow("image", result_image)

        key = cv2.waitKey(1)
        if key == ord('s'): # 按下s开始选择追踪目标(press 's' to start selecting the tracking target)
            self.tracker = None
            self.enable_select = True


if __name__ == '__main__':
    try:
        kcf_tracking = KCFTrackingNode()
        print("在画面窗口按下s开始选择追踪目标")
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

