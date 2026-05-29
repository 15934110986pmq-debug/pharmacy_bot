#!/usr/bin/env python3
# coding: utf8

import os
import sys
import rospy
import numpy as np
import threading
import cv2
import yaml
from sensor_msgs.msg import Image as RosImage, CameraInfo
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation
from handeye_calibration import get_cam2hand
from vision_utils import fps
from vision_utils.apriltag_detector import Detector as TagDetector
import kine7
import roslib.packages
from jetarm_sdk import controller_client
from jetarm_kinematics import kine7
from math import radians


class AutoCalibration:
    def __init__(self, node_name, log_level=rospy.INFO):
        rospy.init_node(node_name, anonymous=True, log_level=log_level)

        self.calibration_step = 0
        self.tags = []
        self.positions = []
        self.tag_count = 0
        self.tag = None
        self.endpoint = None

        self.K = None
        self.D = None

        self.kine = kine7.Kinematics()
        self.joint = controller_client.JointControllerClient()
        self.tag_detector = TagDetector(tag_size=0.033)
        self.camera_pose = None

        self.target_position = None
        self.target_position_count = 0

        self.lock = threading.RLock()
        self.fps = fps.FPS()    # 帧率统计器(frame rate counter)
        self.thread = None

        # 订阅(subscribe)
        source_image_topic = rospy.get_param('~source_image_topic', '/usb_cam/image_rect_color')
        camera_info_topic = rospy.get_param('~camera_info_topic', '/usb_cam/camera_info')
        self.endpoint_pose_sub = rospy.Subscriber('/kinematics/endpoint_pose', Pose, self.endpoint_update_callback, queue_size=10)
        self.image_sub = rospy.Subscriber(source_image_topic, RosImage, self.image_callback, queue_size=1)
        self.camera_info_sub = rospy.Subscriber(camera_info_topic, CameraInfo, self.camera_info_callback)

        rospy.sleep(1)
        rospy.loginfo("开始进行标定")
        self.thread = threading.Thread(target=self.calibration_proc)
        self.thread.start()

    def go_home(self, duration=1.5):
        solve = self.kine.ikine_euler((0.0, 0.12, 0.12), (radians(-165), radians(0.0), radians(0.0)))
        self.joint.set_joints(solve[1], duration)
        rospy.sleep(duration)

    def camera_info_callback(self, msg):
        with self.lock:
            K = np.matrix(msg.K).reshape(1, -1, 3)
            D = np.array(msg.D)
            new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (640, 480), 0, (640, 480))
            self.K, self.D = np.matrix(new_K), np.zeros((5, 1))


    def endpoint_update_callback(self, msg):
        position = [msg.position.x, msg.position.y, msg.position.z]
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        r = Rotation.from_quat(quat)
        # rospy.loginfo("xyz:" + str(position) + "    rpy:" + str(r.as_euler('xyz', degrees=True)))
        self.endpoint =  [position, r.as_euler('xyz', degrees=False).tolist()]


    def image_callback(self, ros_image: RosImage):
        # rospy.logdebug('Received an image! ')
        # 将ros格式图像转换为opencv格式(convert the ros format image to opencv format)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        result_image = np.copy(rgb_image)
        if self.K is not None and self.D is not None:
            if self.calibration_step == 1:
                h, w = rgb_image.shape[:2]
                n_k, _ = cv2.getOptimalNewCameraMatrix(self.K, self.D, (640, 480), 0)
                tags = self.tag_detector.detect(rgb_image, n_k, self.D, scale=1)
                result_image = self.tag_detector.draw(result_image, tags)
                if len(tags) > 0 and tags[0]['id'] == 1:
                    self.tag_count += 1
                    self.tag = self.tag_detector.get_pose(tags[0])
                    if self.tag_count >= 50:
                        self.calibration_step += 1
                        print(self.tag)
                        print(self.endpoint)
                        print()
                        print()
                        print()
        else:
            rospy.logerr("相机内参为None, 请确保相机节点已正常开启")

        # 计算帧率及发布结果图像(calculate the frame rate and publish the resulting image)
        self.fps.update()
        result_image = self.fps.show_fps(result_image)
        cv2.imshow("result", cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)


    def calibration_proc(self):
        self.tags = []
        self.positions = []
        self.go_home()

        # 第一点(the first point)
        self.tag = None
        self.tag_count = 0
        self.calibration_step = 0
        self.joint.set_servo(1, 500, 1.5)
        self.joint.set_servo(2, 690, 1.5)
        self.joint.set_servo(3, 50, 1.5)
        self.joint.set_servo(4, 500, 1.5)
        self.joint.set_servo(5, 120, 1.5)
        self.joint.set_servo(6, 500, 1.5)
        rospy.sleep(2)
        self.calibration_step = 1
        while self.calibration_step == 1:
            rospy.sleep(0.1)
        self.positions.append(self.endpoint)
        self.tags.append(self.tag)

        # 第二点(the second point)
        self.tag = None
        self.tag_count = 0
        self.calibration_step = 0
        self.joint.set_servo(1, 500, 1.5)
        self.joint.set_servo(2, 520, 1.5)
        self.joint.set_servo(3, 110, 1.5)
        self.joint.set_servo(4, 500, 1.5)
        self.joint.set_servo(5, 120, 1.5)
        self.joint.set_servo(6, 500, 1.5)
        rospy.sleep(2)
        self.calibration_step = 1
        while self.calibration_step == 1:
            rospy.sleep(0.2)
        self.positions.append(self.endpoint)
        self.tags.append(self.tag)

        # 第三点(the third point)
        self.tag = None
        self.tag_count = 0
        self.calibration_step = 0
        self.joint.set_servo(1, 703, 1.5)
        self.joint.set_servo(2, 420, 1.5)
        self.joint.set_servo(3, 150, 1.5)
        self.joint.set_servo(4, 350, 1.5)
        self.joint.set_servo(5, 125, 1.5)
        self.joint.set_servo(6, 500, 1.5)
        rospy.sleep(2)
        self.calibration_step = 1
        while self.calibration_step == 1:
            rospy.sleep(0.2)
        self.positions.append(self.endpoint)
        self.tags.append(self.tag)

        # 第四点(the fourth point)
        self.tag = None
        self.tag_count = 0
        self.calibration_step = 0
        self.joint.set_servo(1, 340, 1.5)
        self.joint.set_servo(2, 450, 1.5)
        self.joint.set_servo(3, 140, 1.5)
        self.joint.set_servo(4, 575, 1.5)
        self.joint.set_servo(5, 120, 1.5)
        self.joint.set_servo(6, 500, 1.5)
        rospy.sleep(2)
        self.calibration_step = 1
        while self.calibration_step == 1:
            rospy.sleep(0.2)
        self.positions.append(self.endpoint)
        self.tags.append(self.tag)

        print(self.positions, self.tags)
        ps = []
        for p, e in self.positions:
            p.extend(e)
            ps.append(p)
        ts = []
        print("tags", self.tags)
        for p, e in self.tags:
            p.extend(e)
            ts.append(p)
        ch = get_cam2hand(ps, ts).tolist()

        rospy.set_param('extrinsics', ch)
        camera_type = rospy.get_param("camera_type", "GEMINI")
        path = roslib.packages.get_pkg_dir('jetarm_peripherals')
        with open(os.path.join(path, 'config/extrinsics_%s.yaml'%camera_type), 'w') as file:
            yaml.dump(ch, file)

        self.go_home()
        rospy.loginfo("Finished")
        rospy.signal_shutdown("finished")
        sys.exit(0)

if __name__ == "__main__":
    node = AutoCalibration("auto_calibration", log_level=rospy.DEBUG)
    rospy.spin()
