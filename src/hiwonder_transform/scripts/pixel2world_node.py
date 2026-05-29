#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/02/2
# @author:aiden
# 识别颜色以及计算物体的位置信息(recognize color and calculate the position information of object)

import cv2
import rospy
import numpy as np
import calculate_grasp_yaw
from jetarm_sdk import common
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Empty, EmptyResponse
from hiwonder_interfaces.srv import SetTarget, GetTarget, GetRobotPose
from hiwonder_interfaces.msg import PixelPosition, ObjectsInfo, ObjectPose
import transforms3d as tfs
from vision_utils import xyz_quat_to_mat, xyz_euler_to_mat

CONFIG_NAME = '/config'
COLOR_DETECTION_NODE_NAME = '/color_detection'

class Pixel2World:
    def __init__(self, name):
        # 初始化节点(initialize node)
        rospy.init_node(name, log_level=rospy.INFO)
        self.name = name
        
        self.image = None
        self.target = None
        self.detect_labels = []
        
        self.cam_height = 0.184 # 相机高度(camera height)
        self.tag2cam_y = 0
        self.TAG2BASE_X = 0 
        self.TAG2BASE_Y = 0.21 - 0.038/2 + 0.025/2
        self.K = None
        self.D = None

        self.enable_display = rospy.get_param('~enable_display', False)
        self.debug = rospy.get_param('~debug', False)
        config = rospy.get_param(CONFIG_NAME)
        self.world_width = config['white_area_world_size']['width']
        self.world_height = config['white_area_world_size']['height']
        self.width = config['white_area_pixel_size']['width']
        
        camera_resolution = config['camera_resolution']
        self.camera_width = camera_resolution['width']
        self.camera_height = camera_resolution['height']
        self.perspective_transformation_matrix = np.array(config['perspective_transformation_matrix'])
        self.hand2cam_transformation_matrix = np.array(config['hand2cam_transformation_matrix'])

        # tag_center = config['apriltag_center']
        # warped_point = np.dot(self.perspective_transformation_matrix, np.array([[tag_center[0]], [tag_center[1]], [1]]))
        # x = warped_point[0, 0]/warped_point[2, 0]
        # y = warped_point[1, 0]/warped_point[2, 0]
        # self.tag_center_y = common.val_map(x, int((self.camera_width - self.width)/2), int((self.camera_width + self.width)/2), self.world_width/2, -self.world_width/2)
        # self.tag_center_x = common.val_map(y, int((self.camera_height - self.width*self.world_height/self.world_width)/2), int((self.camera_height + self.width*self.world_height/self.world_width)/2), self.world_height/2, -self.world_height/2)

        color_info_sub = rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.object_info_callback)
        camera_info_sub = rospy.Subscriber('/rgbd/color/camera_info', CameraInfo, self.camera_info_callback)
        self.position_pub = rospy.Publisher('/object/world_coords', ObjectPose, queue_size=1)
        
        rospy.Service('/update_config', Empty, self.update_config)
        rospy.Service('~update_param', Empty, self.update_param)
        set_target_srv = rospy.Service('~set_target', SetTarget, self.set_target)   
        get_label_srv = rospy.Service('~get_label_list', GetTarget, self.get_label_list)
        clear_target_srv = rospy.Service('~clear_target', Empty, self.clear_target)   
        rospy.sleep(0.2)
        
        common.loginfo("%s init finish"%self.name)
        #while not rospy.is_shutdown():
        #    try:
        #        if rospy.get_param('/init_pose/init_finish'):
        #            break
        #    except:
        #        rospy.sleep(0.1)
        rospy.wait_for_service('/kinematics/get_current_pose')
        if self.debug:
            camera = rospy.get_param('/camera')
            rospy.Subscriber('/%s/image_rect_color'%camera['camera_name'], Image, self.image_callback)  # 订阅校准后的图像
            __target = SetTarget()
            __target.label = 'red1'
            self.set_target(__target)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down")
    def camera_info_callback(self, msg):
        self.K = np.matrix(msg.K).reshape(1, -1, 3)
        self.D = np.array(msg.D)
    

    # 更新config参数(update config parameter)
    def update_config(self, msg):
        rospy.ServiceProxy('/%s/update_param'%self.name, Empty)()
        rospy.sleep(0.1) 
        rospy.ServiceProxy('%s/update_param'%COLOR_DETECTION_NODE_NAME, Empty)()
        rospy.sleep(0.1)
        rospy.ServiceProxy('/perspective_transformation/update_param', Empty)()
        rospy.sleep(0.1)
        rospy.ServiceProxy('/yolov5/update_param', Empty)()
        
        common.loginfo('update config')
        
        return EmptyResponse()

    def update_param(self, msg):
        config = rospy.get_param(CONFIG_NAME)
        self.world_width = config['white_area_world_size']['width']
        self.world_height = config['white_area_world_size']['height']
        self.width = config['white_area_pixel_size']['width']
        
        camera_resolution = config['camera_resolution']
        self.camera_width = camera_resolution['width']
        self.camera_height = camera_resolution['height']
        self.perspective_transformation_matrix = np.array(config['perspective_transformation_matrix'])
        self.hand2cam_transformation_matrix = np.array(config['hand2cam_transformation_matrix'])
        
        # tag_center = config['apriltag_center']
        # warped_point = np.dot(self.perspective_transformation_matrix, np.array([[tag_center[0]], [tag_center[1]], [1]]))
        # x = warped_point[0, 0]/warped_point[2, 0]
        # y = warped_point[1, 0]/warped_point[2, 0]
        # self.tag_center_y = common.val_map(x, int((self.camera_width - self.width)/2), int((self.camera_width + self.width)/2), self.world_width/2, -self.world_width/2)
        # self.tag_center_x = common.val_map(y, int((self.camera_height - self.width*self.world_height/self.world_width)/2), int((self.camera_height + self.width*self.world_height/self.world_width)/2), self.world_height/2, -self.world_height/2)
        
        # print(self.tag_center_x, self.tag_center_y)
        common.loginfo('%s update param'%self.name)

        return EmptyResponse()

    def clear_target(self, msg):
        self.target = None
        self.detect_labels = []
        common.loginfo("clear target")

        return EmptyResponse()

    def set_target(self, msg):
        #rospy.loginfo('set target:%s'%msg.label)
        #rospy.loginfo('detect labels:%s'%detect_labels)

        # 计算摄像头的当前位置(calculate the current position of camera)
        res = rospy.ServiceProxy('/kinematics/get_current_pose', GetRobotPose)()
        R = common.qua2rot(res.pose.orientation)
        x, y, z = res.pose.position.x, res.pose.position.y, res.pose.position.z 
        T = np.row_stack((np.column_stack((R, np.array([[x], [y], [z]]))), np.array([[0, 0, 0, 1]])))        
        print(x, y, z)

        endpoint_quat = res.pose.orientation.w, res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z
        mat = xyz_quat_to_mat((x, y, z), endpoint_quat)
        t, r, _, _ = tfs.affines.decompose(self.hand2cam_transformation_matrix)
        print("cam_pose:")
        print(t)
        print(np.degrees(tfs.euler.mat2euler(r)))
        print()
        print()

        camera_pose = np.dot(mat, np.matrix(self.hand2cam_transformation_matrix))
        t, r, _, _ = tfs.affines.decompose(camera_pose)
        print("cam_pose:")
        print(t)
        print(np.degrees(tfs.euler.mat2euler(r)))
        print()
        print()
        

        print(T, self.hand2cam_transformation_matrix)
        cam_pose = np.dot(T, np.matrix(self.hand2cam_transformation_matrix))
        print("cam_pose", cam_pose)
        self.cam_height = cam_pose[2, 3] # 相机高度(camera height)
        self.tag2cam_y = self.TAG2BASE_Y + cam_pose[1, 3]        
        print(self.cam_height, self.tag2cam_y, cam_pose[1, 3], cam_pose[0, 3])
        
        common.loginfo("set target")
        self.target = msg.label

        return [True, 'set_target']

    def get_label_list(self, msg):
        common.loginfo("get label list")
        rospy.loginfo('detect labels:%s'%self.detect_labels)

        return [True, self.detect_labels]

    def pixel2world(self, point):
        # 像素坐标转为世界坐标(convert pixel coordinates to world coordinates)
        # 像素坐标转为世界坐标(convert pixel coordinates to world coordinates)

        warped_point = np.dot(self.perspective_transformation_matrix, np.array([[point[0, 0]], [point[0, 1]], [1]]))
        x = warped_point[0, 0]/warped_point[2, 0]
        y = warped_point[1, 0]/warped_point[2, 0]


        obj2tag_x = common.val_map(x, int((self.camera_width - self.width)/2), int((self.camera_width + self.width)/2), self.world_width/2, -self.world_width/2)
        obj2tag_y = common.val_map(y, int((self.camera_height - self.width*self.world_height/self.world_width)/2), int((self.camera_height + self.width*self.world_height/self.world_width)/2), self.world_height/2, -self.world_height/2)# - self.tag_center_x
        
        #print(obj2tag_x) #, self.tag_center_x)
        #print(self.tag2cam_y, obj2tag_y)
        x_world = obj2tag_x - point[0, 2]*(obj2tag_x)/self.cam_height + self.TAG2BASE_X
        y_world = obj2tag_y - point[0, 2]*(self.tag2cam_y + obj2tag_y)/self.cam_height + self.TAG2BASE_Y
        
        return x_world, -y_world   # 运动学坐标系x轴朝前，所以在这里对换下(the x-axis of the kinematic coordinate system faces forward, so swap it here)

    def object_info_callback(self, msg):
        try:
            points = {}
            for p in msg.data:
                points[p.label] = [p.center, p.size, p.yaw, p.height]
            labels = list(points.keys())
            self.detect_labels = labels
            if self.target in self.detect_labels:
                point = np.array([[points[self.target][0].x, points[self.target][0].y, points[self.target][-1]]], dtype=np.double)
                x, y = self.pixel2world(point)
                
                pose_msg = ObjectPose()
                pose_msg.label = self.target
                yaw, collision_roi = calculate_grasp_yaw.get_yaw_angle(self.image, self.target, points, [pose_msg.y, -pose_msg.x], self.pixel2world)
                pose_msg.x = -y
                pose_msg.y = x
                # yaw = None
                if yaw is None:
                    pose_msg.collision = True
                    pose_msg.yaw = 0
                else:
                    p1 = PixelPosition()
                    p1.x = collision_roi[0][0]
                    p1.y = collision_roi[0][1]
                    p2 = PixelPosition()
                    p2.x = collision_roi[1][0]
                    p2.y = collision_roi[1][1]
                    p3 = PixelPosition()
                    p3.x = collision_roi[2][0]
                    p3.y = collision_roi[2][1]
                    p4 = PixelPosition()
                    p4.x = collision_roi[3][0]
                    p4.y = collision_roi[3][1]

                    pose_msg.collision_roi = [p1, p2, p3, p4]
                    pose_msg.collision = False
                    pose_msg.yaw = -yaw  # 舵机转动方向和这个相反，所以取反(the servo rotates in the opposite direction, so take the opposite)
                if pose_msg.label is not None:
                    self.position_pub.publish(pose_msg)
            if self.enable_display and self.image is not None:
                cv2.imshow('collision detection', self.image)
                cv2.waitKey(1)
        except BaseException as e:
            print('color_info_callback error:', e)

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                           buffer=ros_image.data)  # 将ros格式图像消息转化为opencv格式(convert the ros format image information to opencv format)
        self.image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

if __name__ == '__main__':
    Pixel2World('pixel2world')
