#!/usr/bin/env python3
# coding: utf8

import cv2
import math
import time
import rospy
import numpy as np
import threading
import yaml
import threading
import actions
import heart
from utils import unregister
from jetarm_sdk import common, bus_servo_control
from sensor_msgs.msg import Image as RosImage, CameraInfo
from hiwonder_interfaces.msg import MultiRawIdPosDur
from hiwonder_interfaces.srv import GetRobotPose
from vision_utils import draw_tags, xyz_euler_to_mat,  xyz_quat_to_mat, distance, pixels_to_world, extristric_plane_shift, xyz_rot_to_mat, mat_to_xyz_euler
from scipy.spatial.transform import Rotation 
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse, Empty
from dt_apriltags import Detector
from object_sortting import ObjectSorttingNode

CONFIG_NAME='/config'

class CalibrationNode:
    def __init__(self, node_name, log_level=rospy.INFO):
        rospy.init_node(node_name, anonymous=True, log_level=log_level)

        self.calibration_step = 0
        self.tags = []
        self.pose = []
        self.tag_count = 0
        self.K = None
        self.lock = threading.RLock()
        self.thread = None
        self.err_msg = None
        self.imgpts = None
        config = rospy.get_param(CONFIG_NAME)
        if 'extristric' in config:
            tvec, rmat = config['extristric']
            self.extristric = np.array(tvec), np.array(rmat)
        else:
            self.extristric = None
        self.tag_size = config['calibration_tag_size']
        self.tag_id = config['calibration_tag_id']
        self.tag_id_2 = config['calibration_tag_id_2']
        self.hand2cam_tf_matrix = config['hand2cam_tf_matrix'], 
        self.config_file = 'config.yaml'
        self.config_path = "/home/ubuntu/jetarm/src/hiwonder_imgproc/color_detection/config/"
        self.servos_pub = rospy.Publisher("/controllers/multi_id_pos_dur", MultiRawIdPosDur, queue_size=1)

        self.at_detector = Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=4,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

        # 订阅(subscribe)
        self.image_sub = None
        self.camera_info_sub = None
        self.result_image_pub = rospy.Publisher('~image_result', RosImage, queue_size=10)

        # services
        self.enter_srv = rospy.Service('~enter', Trigger, self.enter_srv_callback)
        self.exit_srv =  rospy.Service('~exit', Trigger, self.exit_srv_callback)
        self.enable_detect_srv = rospy.Service('~start', Trigger, self.start_calibration_srv_callback)
        self.heart = heart.Heart('~heartbeat', 5, lambda e: self.exit_srv_callback(e))


    def calibration_proc(self):
        self.tags = []
        actions.go_home(self.servos_pub)
        rospy.sleep(2)

        # 手眼标定。。。。(hand-eye calibration)
        # 通过固定的结构尺寸确定(calibration through fixed structural dimensions)

        # 获取当前末端坐标(get the current end-effector coordinates)
        endpoint = rospy.ServiceProxy('/kinematics/get_current_pose', GetRobotPose)().pose 
        self.endpoint = xyz_quat_to_mat([endpoint.position.x, endpoint.position.y,  endpoint.position.z],
                                            [endpoint.orientation.w, endpoint.orientation.x, endpoint.orientation.y, endpoint.orientation.z])
            
        # 获取标签数据(get tag data)
        t = time.time()
        self.tags = []
        self.calibration_step = 1
        while self.calibration_step == 1 and time.time() - t < 10:
            rospy.sleep(0.1)

        if len(self.tags) < 30:
            self.err_msg = "Time out, calibrate failed!!!"
            rospy.sleep(3)
            self.err_msg = None
            self.calibration_step = 0
            self.thread = None
            self.tags = []
            return

        config = rospy.get_param(CONFIG_NAME)
        # 识别区域中心位置标定(calibration of the center position in the recognition area)
        # 对多次识别的数据求均值(calculate the average of multiple recognition data)
        pose = map(lambda tag: xyz_rot_to_mat(tag.pose_t, tag.pose_R), self.tags) # 将所有位姿转为4x4齐次矩阵(convert all poses to 4x4 homogeneous matrices)
        vectors = map(lambda p: p.ravel(), pose) # 将矩阵展平为向量(flatten the matrix into a vector)
        avg_pose = np.mean(list(vectors), axis=0).reshape((4, 4))  # 求均值并重组为4x4矩阵(calculate the mean and reassemble into a 4x4 matrix)
        pose_end = np.matmul(config['hand2cam_tf_matrix'], avg_pose)  # 转换到末端相对坐标(transform to end-effector relative coordinates)

        pose_world = np.matmul(self.endpoint, pose_end)  # 转换到机械臂世界坐标(transform to robotic arm world coordinates)
        config['white_area_pose_cam'] = avg_pose.tolist()  # 识别区域中心的在相机的世界坐标系中的位置, 结果存入到param中(the position of the center of the recognition area in the camera's world coordinate system is stored in the parameter 'param')
        xyz, euler = mat_to_xyz_euler(avg_pose)
        config['white_area_pose_world'] = pose_world.tolist()  # 识别区域中心的机械臂世界坐标系的位置, 结果存入到param中(the position of the center of the recognition area in the camera's world coordinate system is stored in the parameter 'param')
        axyz, aeuler = mat_to_xyz_euler(pose_world)
        print(xyz, euler, axyz, aeuler)

        # 外参标定(extrinsic calibration)
        world_points = np.array([(-self.tag_size/2, -self.tag_size/2, 0), 
                                 ( self.tag_size/2, -self.tag_size/2, 0), 
                                 ( self.tag_size/2,  self.tag_size/2, 0), 
                                 (-self.tag_size/2,  self.tag_size/2, 0)] * len(self.tags), dtype=np.float64)

        image_points = np.array(list(map(lambda tag: tag.corners, self.tags)), dtype=np.float64).reshape((-1, 2))
        retval, rvec, tvec = cv2.solvePnP(world_points, image_points, self.K, self.D)
        rmat, _ = cv2.Rodrigues(rvec)
        self.extristric = tvec, rmat
        config['extristric'] = [tvec.reshape((3)).tolist(), rmat.reshape((-1, 3)).tolist()] # 外参存入param中(the extrinsic parameters are stored in 'param')

        self.save_yaml_data(config, self.config_path + self.config_file)
     
        actions.go_home(self.servos_pub)
        self.calibration_step = 20
        rospy.sleep(3)
        self.calibration_step = 0
        self.thread = None

    
    def save_yaml_data(self, data, yaml_file):
        with open(yaml_file, 'w', encoding='utf-8') as f:
            yaml.dump(data, f)
        rospy.set_param(CONFIG_NAME, data)
        # rospy.ServiceProxy('/update_config', Empty)()
        rospy.sleep(0.1)

    def start_calibration_srv_callback(self, _):
        rospy.loginfo("开始进行标定")
        with self.lock:
            if self.image_sub is None:
                err_msg = "Please call enter service first"
                rospy.logerr(err_msg)
                return TriggerResponse(success=False, message=err_msg)
            if self.thread is None:
                self.thread = threading.Thread(target=self.calibration_proc)
                self.thread.start()
        return TriggerResponse(success=True)

    def enter_srv_callback(self, _: TriggerRequest):
        # 获取和发布图像的topic(get and publish topic of image)
        with self.lock:
            source_image_topic = rospy.get_param('~source_image_topic', '/rgbd_cam/color/image_rect_color')
            camera_info_topic = rospy.get_param('~camera_info_topic', '/rgbd_cam/color/camera_info')
            # source_image_topic = '/usb_cam/image_raw'
            unregister(self.image_sub)
            unregister(self.camera_info_sub)
            threading.Thread(target=actions.go_home, args=(self.servos_pub, 1.5)).start()
            self.image_sub = rospy.Subscriber(source_image_topic, RosImage, self.image_callback, queue_size=1)
            self.camera_info_sub = rospy.Subscriber(camera_info_topic, CameraInfo, self.camera_info_callback)
        return TriggerResponse(success=True)
    def draw_retangle(self):
        config = rospy.get_param(CONFIG_NAME)
        # 识别区域的四个角的世界坐标(the world coordinates of the four corners of the recognition area)
        white_area_cam = config['white_area_pose_cam']
        white_area_center = config['white_area_pose_world']
        self.white_area_center = white_area_center
        self.white_area_cam = white_area_cam
        white_area_height = config['white_area_world_size']['height']
        white_area_width = config['white_area_world_size']['width']
        white_area_lt = np.matmul(white_area_center, xyz_euler_to_mat((white_area_height / 2, white_area_width / 2 + 0.0, 0.0), (0, 0, 0)))
        white_area_lb = np.matmul(white_area_center, xyz_euler_to_mat((-white_area_height / 2, white_area_width / 2 + 0.0, 0.0), (0, 0, 0)))
        white_area_rb = np.matmul(white_area_center, xyz_euler_to_mat((-white_area_height / 2, -white_area_width / 2 -0.0, 0.0), (0, 0, 0)))
        white_area_rt = np.matmul(white_area_center, xyz_euler_to_mat((white_area_height / 2, -white_area_width / 2 -0.0, 0.0), (0, 0, 0)))
        endpoint = rospy.ServiceProxy('/kinematics/get_current_pose', GetRobotPose)().pose 
        self.endpoint = xyz_quat_to_mat([endpoint.position.x, endpoint.position.y,  endpoint.position.z],
                                        [endpoint.orientation.w, endpoint.orientation.x, endpoint.orientation.y, endpoint.orientation.z])
        corners_cam =  np.matmul(np.linalg.inv(np.matmul(self.endpoint, config['hand2cam_tf_matrix'])), [white_area_lt, white_area_lb, white_area_rb, white_area_rt, white_area_center])
        corners_cam = np.matmul(np.linalg.inv(white_area_cam), corners_cam)
        corners_cam = corners_cam[:, :3, 3:].reshape((-1, 3))
        tvec, rmat = config['extristric']

        # rospy.loginfo("waitting for K & D ")
        # while self.K is None or self.D is None:
            # rospy.sleep(0.5)
        # tvec, rmat = config['extristric']

        while self.K is None or self.D is None:
            rospy.sleep(0.5)

        center_imgpts, jac = cv2.projectPoints(corners_cam[-1:], np.array(rmat), np.array(tvec), self.K, self.D)
        self.center_imgpts = np.int32(center_imgpts).reshape(2)

        tvec, rmat = extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.030)
        imgpts, jac = cv2.projectPoints(corners_cam[:-1], np.array(rmat), np.array(tvec), self.K, self.D)
        self.imgpts = np.int32(imgpts).reshape(-1, 2)


    def camera_info_callback(self, msg):
        with self.lock:
            K = np.matrix(msg.K).reshape(1, -1, 3)
            D = np.array(msg.D)
            new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (640, 480), 0, (640, 480))
            self.K, self.D = np.matrix(new_K), np.zeros((5, 1))


    def exit_srv_callback(self, _: TriggerRequest):
        with self.lock:
            unregister(self.image_sub)
            unregister(self.camera_info_sub)
        return TriggerResponse(success=True)
    

    def image_callback(self, ros_image: RosImage):
        # rospy.logdebug('Received an image! ')
        # 将ros格式图像转换为opencv格式(convert the ros format image to opencv format)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        result_image = np.copy(rgb_image)
        if self.K is not None:
            tags = self.at_detector.detect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY), True, (self.K[0,0], self.K[1,1], self.K[0,2], self.K[1,2]), self.tag_size)
            result_image = draw_tags(result_image, tags)
            if self.calibration_step == 1:
                #print(len(tags), tags[0].tag_id)
                if len(tags) == 1 and (tags[0].tag_id == self.tag_id or tags[0].tag_id == self.tag_id_2):
                    self.err_msg = None
                    if len(self.tags) > 0:
                        if distance(self.tags[-1].pose_t, tags[0].pose_t) < 0.003:
                            self.tags.append(tags[0])
                        else:
                            self.tags = []
                    else:
                        self.tags.append(tags[0])
                    if len(self.tags) >= 50:
                        print("收集完成")
                        self.calibration_step = 2
                else:
                    self.tags = []
                    if self.err_msg is None:
                        self.err_msg = "Please make sure there is only one tag in the;screen and the tag id is 1 or 100"
            if self.extristric is not None:
                world_points = np.array([(-self.tag_size/2, -self.tag_size/2, 0), 
                                         ( self.tag_size/2, -self.tag_size/2, 0), 
                                         ( self.tag_size/2,  self.tag_size/2, 0), 
                                         (-self.tag_size/2,  self.tag_size/2, 0)], dtype=np.float64)
                image_points, _ = cv2.projectPoints(world_points, self.extristric[1], self.extristric[0], self.K, self.D)
                image_points = image_points.astype(np.int32).reshape((-1, 2)).tolist()
                for p in image_points:
                    cv2.circle(result_image, tuple(p), 3, (0, 0, 0), -1)
                #projection_matrix = np.row_stack((np.column_stack((self.extristric[1], self.extristric[0])), np.array([[0, 0, 0, 1]])))
                #print(pixels_to_world([tags[0].center], self.K, projection_matrix))
                #print()
                #print()
                #print()
        

        if self.err_msg is not None:
            rospy.logerr(self.err_msg)
            err_msg = self.err_msg.split(';')
            for i, m in enumerate(err_msg):
                cv2.putText(result_image, m, (5, 50 + (i * 30)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 6)
                cv2.putText(result_image, m, (5, 50 + (i * 30)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        
        if self.calibration_step != 0:
            if self.calibration_step == 20:
                msg = "Calibration finished!"
                self.draw_retangle()
                # cv2.drawContours(result_image, [self.imgpts], -1, (255, 255, 0), 2, cv2.LINE_AA) # 绘制矩形(draw rectangle)
            else:
                msg = "Calibrating..."
            cv2.putText(result_image, msg, (5, result_image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 6)
            cv2.putText(result_image, msg, (5, result_image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        
        cv2.drawContours(result_image, [self.imgpts], -1, (255, 255, 0), 2, cv2.LINE_AA) # 绘制矩形(draw rectangle)
        # 计算帧率及发布结果图像(calculate the frame rate and publish the resulting image)
        ros_image.data = result_image.tobytes()
        self.result_image_pub.publish(ros_image)


if __name__ == "__main__":
    node = CalibrationNode("calibration", log_level=rospy.INFO)
    rospy.spin()
