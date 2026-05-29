#!/usr/bin/env python3
# encoding: utf-8
import cv2
import sys
import math
import rospy
import signal
import numpy as np
from sensor_msgs.msg import Image as RosImage, CameraInfo
from vision_utils import xyz_quat_to_mat, xyz_euler_to_mat, xyz_rot_to_mat, mat_to_xyz_euler, pixels_to_world, draw_tags, distance, fps, extristric_plane_shift
sys.path.append('/home/ubuntu/jetarm/src/jetarm_example/src/Simple_library')
import color_detection_base

CONFIG_NAME='/config'
class coordinate_transformation:
    def __init__(self, name):
        # 初始化节点
        rospy.init_node(name, log_level=rospy.INFO)
        self.name = name
        self.image_bgr = None
        self.image_test = None
        self.K = None
        self.running = True
        self.height = 0.03
        self.width = 520
        source_image_topic = rospy.get_param('~source_image_topic', '/camera/image_raw')
        camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/image_raw')
        self.color = rospy.get_param('~color', 'red')
        config = rospy.get_param(CONFIG_NAME)
        tvec, rmat = config['extristric']
        tvec, rmat = extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.030)
        self.extristric = tvec, rmat
        white_area_center = config['white_area_pose_world']
        self.white_area_center = white_area_center
        # 初始化颜色识别类
        self.color_detection = color_detection_base.color_detection()
        # 启动程序中断函数
        signal.signal(signal.SIGINT, self.shutdown)
        # 检测图像发布
        self.image_sub = rospy.Subscriber(source_image_topic, RosImage, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(camera_info_topic, CameraInfo, self.camera_info_callback)
        rospy.sleep(0.2)
        self.run()
    # 程序中断函数，用于停止程序
    def shutdown(self, signum, frame):
        self.running = False

    # 处理ROS节点数据
    def image_callback(self, ros_image):
        # 将ros格式图像消息转化为opencv格式
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,buffer=ros_image.data)
        # 将图像颜色空间转换成BGR
        self.image_bgr = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    def camera_info_callback(self, msg):
        #print(msg)  
        self.K = np.matrix(msg.K).reshape(1, -1, 3)
    def color_coordinate(self):
        # 计算识别到的轮廓
        contours = cv2.findContours(self.image_test, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[-2]
        # 找出最大轮廓
        c = max(contours, key = cv2.contourArea)
        # 计算轮廓面积
        area = math.fabs(cv2.contourArea(c))  
        # 根据轮廓大小判断是否进行下一步处理
        rect = cv2.minAreaRect(c)  # 获取最小外接矩形
        corners = np.int0(cv2.boxPoints(rect))  # 获取最小外接矩形的四个角点
        x, y = rect[0][0],rect[0][1]
        # 打印像素坐标
        print("像素坐标为:","x:",x,"y:",y)

        return x,y
    
    def run(self):
        while self.running:
            try:
                if self.image_bgr is not None :
                    # 启动颜色识别得到识别后的图像   
                    self.image_test= self.color_detection.color_detection(self.color,self.image_bgr)
                    # 得到像素坐标   
                    x,y = self.color_coordinate()
                    projection_matrix = np.row_stack((np.column_stack((self.extristric[1],self.extristric[0])), np.array([[0, 0, 0, 1]])))
                    #print(self.K)
                    world_pose = pixels_to_world([[x,y]], self.K, projection_matrix)[0]
                    world_pose[1] = -world_pose[1]
                    world_pose[2] = 0.03
                    world_pose = np.matmul(self.white_area_center, xyz_euler_to_mat(world_pose, (0, 0, 0)))
                    world_pose[2] = 0.03
                    pose_t, _ = mat_to_xyz_euler(world_pose)
                    print("实际坐标为：",pose_t)
                    # 展示
                    cv2.imshow('BGR', self.image_bgr)
                    cv2.imshow('color_detection', self.image_test)
                    cv2.waitKey(1)
            except Exception as e:
                 print("未检测到所需识别的颜色，请将色块放置到相机视野内。")
if __name__ == '__main__':
    coordinate_transformation('coordinate_transformation')
