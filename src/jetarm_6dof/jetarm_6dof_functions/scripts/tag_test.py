#!/usr/bin/python3

import math
import threading
import cv2
import rospy
import numpy as np
from apriltag import apriltag
from sensor_msgs.msg import CameraInfo, Image
from vision_utils import fps, point_remapped
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose
from spatialmath import SE3
from jetarm_kinematics import kinematics_client
from jetarm_sdk import controller_client, sdk_client
import time

AXIS = np.float32([[0,    0,    0],
                   [0.02, 0,    0], 
                   [0,    0.02, 0], 
                   [0,    0,    0.02]])

CIRCLE = np.float32([[0.005 * math.cos(math.radians(i)), 0.005 * math.sin(math.radians(i)), 0] for i in range(360)])
AXIS = np.append(AXIS, CIRCLE, axis=0)


def draw(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1,2)
    cv2.drawContours(img, [imgpts[4:]],-1,(255, 255, 0), -1)
    cv2.line(img, tuple(imgpts[0]), tuple(imgpts[1]),(255, 0, 0),3)
    cv2.line(img, tuple(imgpts[0]), tuple(imgpts[2]),(0, 255, 0),3)
    cv2.line(img, tuple(imgpts[0]), tuple(imgpts[3]),(0, 0, 255),3)
    return img


class TagNode:
    def __init__(self, name):
        rospy.init_node(name)

        self.kine = kinematics_client.KinematicsClient()
        self.joint = controller_client.JointControllerClient()
        self.sdk = sdk_client.JetArmSDKClient()

        self.lock = threading.Lock()
        self.tag_detector = apriltag("tag36h11")
        self.fps = fps.FPS()
        self.camera_intrinsic = None
        self.dist_coeffs = None
        self.camera_pose = None
        self.step = 0
        self.stamp = time.time()
        self.target_pose = None
        self.moving_tag = None
        self.error_msg = None

        self.proc_width = 320
        self.proc_height = 240
        self.width = 640
        self.height = 480
        self.tag_width = rospy.get_param('~tag_width', 0.033)
        self.objp = np.array([[-self.tag_width/2, -self.tag_width/2,  0],
                              [ self.tag_width/2, -self.tag_width/2,  0],
                              [-self.tag_width/2,  self.tag_width/2,  0],
                              [ self.tag_width/2,  self.tag_width/2,  0],
                              [ 0,        0,        0]], dtype=np.float64)


        # 订阅相机图像话题, 订阅相机参数话题(subscribe camera image topic, subscribe camera parameter topic)
        self.source_topic = rospy.get_param('~source_topic', '/camera/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/camera_info')

        self.image_sub = rospy.Subscriber(self.source_topic, Image, self.image_callback, queue_size=1)
        self.image_sub = rospy.Subscriber("/kinematics/camera_pose", Pose, self.camera_pose_callback, queue_size=1)
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)

        rospy.sleep(1)
    

    def camera_pose_callback(self, msg):
        t = SE3(msg.position.x, msg.position.y, msg.position.z)
        r = SE3.RPY(Rotation.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]).as_euler('xyz', degrees=False))
        self.camera_pose = t * r
    
    def camera_info_callback(self, msg):
        with self.lock:
            K = np.matrix(msg.K).reshape(1, -1, 3)
            D = np.array(msg.D)
            new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (640, 480), 0, (640, 480))
            self.camera_interinsic, self.dist_coeffs = np.matrix(new_K), np.zeros((5, 1))

 
    def image_callback(self, ros_image: Image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the custom image information to image)
        result_image = np.copy(rgb_image)
        height, width = rgb_image.shape[:2]
        try:
            with self.lock:
                    if self.camera_intrinsic is not None and self.dist_coeffs is not None:
                        gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY) # 将 RGB 图片转为灰度图(convert the RGB image to grey-scale picture)
                        gray = cv2.resize(gray, (self.proc_width, self.proc_height)) # 缩放尺寸减少计算量(reduce computational load by decreasing the size of the scale)
                        new_height, new_width = gray.shape[:2]
                        try:
                            detections = self.tag_detector.detect(gray)
                        except Exception as e:
                            detections = () # 没有找到二维码(QR code not found)
                        for detection in detections:
                            corners =[point_remapped(p, (new_width, new_height), (width, height)) for p in detection['lb-rb-rt-lt']]
                            center = point_remapped(detection['center'], (new_width, new_height), (width, height))
                            # 画出四角(draw the four corners)
                            for p in corners:
                                cv2.circle(result_image, (int(p[0]), int(p[1])), 5, (0, 0, 255), -1)
                            cv2.circle(result_image, (int(center[0]), int(center[1])), 8, (0, 255, 0), -1)
                            cv2.putText(result_image, "ID:%d"%detection['id'], (int(center[0]-25), int(center[1]-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

                            center = point_remapped(detection['center'], (self.proc_width, self.proc_height), (self.width, self.height))
                            lb, rb, rt, lt =[point_remapped(p, (self.proc_width, self.proc_height), (self.width, self.height)) for p in detection['lb-rb-rt-lt']]
                            corners = np.array([lb, rb, lt, rt, center]).reshape(5, -1)
                            # n_k, _ = cv2.getOptimalNewCameraMatrix(self.camera_intrinsic, self.dist_coeffs, (640, 480), 0)
                            # ret, rvecs, tvecs = cv2.solvePnP(self.objp, corners, n_k, self.dist_coeffs)
                            ret, rvecs, tvecs = cv2.solvePnP(self.objp, corners, self.camera_intrinsic, self.dist_coeffs)
                            print(rvecs, tvecs)
                            euler = Rotation.from_rotvec(rvecs.reshape((3, ))).as_euler('xyz')
                            p = SE3(tvecs.reshape((3,))) * SE3.RPY(euler)
                            p = self.camera_pose * p
                            print(p.t, [math.degrees(i) for i in p.eul()])
                    else:
                        rospy.logerr("相机内参为None, 请确保相机节点已正常开启")
        except Exception as e:
            rospy.logerr(str(e))

        self.fps.update()
        self.fps.show_fps(result_image)
        cv2.imshow('image', cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)



if __name__ == '__main__':
    try:
        tag_node = TagNode('tag_disp')
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))


