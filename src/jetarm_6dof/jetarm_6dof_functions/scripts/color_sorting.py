#!/usr/bin/env python3
# coding: utf8

import os
import math
import cv2
import rospy
import numpy as np
import threading
from vision_utils import xyz_quat_to_mat, xyz_euler_to_mat, xyz_rot_to_mat, mat_to_xyz_euler, pixels_to_world, distance, fps, extristric_plane_shift
from sensor_msgs.msg import Image as RosImage, CameraInfo
from hiwonder_interfaces.srv import GetRobotPose
from hiwonder_interfaces.msg import Grasp, MoveAction, MoveGoal, MultiRawIdPosDur
from jetarm_sdk import bus_servo_control
import actions
import actionlib


TARGET_POSITION = {
    "red": [0.07, 0.08],
    "green": [0.0, 0.08],
    "blue": [-0.07, 0.08],
}


CONFIG_NAME='/config'
POSITIONS_PATH='/positions'

class ColorSorttingNode():
    def __init__(self, node_name, log_level=rospy.INFO):
        rospy.init_node(node_name, anonymous=True, log_level=log_level)

        self.K = None
        self.D = None

        self.lock = threading.RLock()
        self.fps = fps.FPS()    # 帧率统计器(frame rate counter)
        self.thread = None

        self.enable_sortting = False
        self.imgpts = None
        self.center_imgpts = None
        self.roi = None
        self.pick_pitch = 80
        self.moving_step = 0
        self.status = 1
        self.count = 0
        config = rospy.get_param(CONFIG_NAME)
        self.color_ranges = config['lab']
        self.min_area = config['area']['min_area']
        self.max_area = config['area']['max_area']
        self.hand2cam_tf_matrix = config['hand2cam_tf_matrix'], 

        self.target = None
        self.target_labels = {
            "red": rospy.get_param('~red', True),
            "green": rospy.get_param('~green', True),
            "blue": rospy.get_param('~blue', True),
        }

        self.image_sub = None
        self.camera_info_sub = None
        self.servos_pub = rospy.Publisher("/controllers/multi_id_pos_dur", MultiRawIdPosDur, queue_size=1)
        self.action_client = actionlib.SimpleActionClient('/grasp', MoveAction)
        rospy.sleep(2)
        actions.go_home(self.servos_pub)
        rospy.sleep(1)

        source_image_topic = rospy.get_param('~source_image_topic', '/camera/image_raw')
        camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/camera_info')
        self.camera_info_sub = rospy.Subscriber(camera_info_topic, CameraInfo, self.camera_info_callback)

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

        rospy.loginfo("waitting for K & D ")
        while self.K is None or self.D is None:
            rospy.sleep(0.5)

        self.hand2cam_tf_matrix = config['hand2cam_tf_matrix'], 
        center_imgpts, jac = cv2.projectPoints(corners_cam[-1:], np.array(rmat), np.array(tvec), self.K, self.D)
        self.center_imgpts = np.int32(center_imgpts).reshape(2)

        tvec, rmat = extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.030)
        self.extristric = tvec, rmat
        imgpts, jac = cv2.projectPoints(corners_cam[:-1], np.array(rmat), np.array(tvec), self.K, self.D)
        self.imgpts = np.int32(imgpts).reshape(-1, 2)

        # 裁切出ROI区域(crop the ROI area)
        x_min = min(self.imgpts, key=lambda p: p[0])[0] # x轴最小值(the minimum value of the x-axis)
        x_max = max(self.imgpts, key=lambda p: p[0])[0] # x轴最大值(the maximal value of the x-axis)
        y_min = min(self.imgpts, key=lambda p: p[1])[1] # y轴最小值(the minimum value of the y-axis)
        y_max = max(self.imgpts, key=lambda p: p[1])[1] # y轴最大值(the maximal value of the y-axis)
        roi = np.maximum(np.array([y_min, y_max, x_min, x_max]), 0)
        print(roi)
        self.roi = roi
        self.image_sub = rospy.Subscriber(source_image_topic, RosImage, self.image_callback, queue_size=1)

    def camera_info_callback(self, msg):
        with self.lock:
            K = np.matrix(msg.K).reshape(1, -1, 3)
            D = np.array(msg.D)
            new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (640, 480), 0, (640, 480))
            self.K, self.D = np.matrix(new_K), np.zeros((5, 1))


    def point_remapped(self, point, now, new, data_type=float):
        """
        将一个点的坐标从一个图片尺寸映射的新的图片上(map the coordinate of one point from a picture to a new picture of different size)
        :param point: 点的坐标(coordinate of point)
        :param now: 现在图片的尺寸(size of current picture)
        :param new: 新的图片尺寸(new picture size)
        :return: 新的点坐标(new point coordinate)
        """
        x, y = point
        now_w, now_h = now
        new_w, new_h = new
        new_x = x * new_w / now_w
        new_y = y * new_h / now_h
        return data_type(new_x), data_type(new_y)

    def adaptive_threshold(self, gray_image):
        # 用自适应阈值先进行分割, 过滤掉侧面(perform segmentation using adaptive thresholding firstly, then filter out the side views)
        # cv2.ADAPTIVE_THRESH_MEAN_C： 邻域所有像素点的权重值是一致的(all pixels in the neighborhood have equal weights)
        # cv2.ADAPTIVE_THRESH_GAUSSIAN _C ： 与邻域各个像素点到中心点的距离有关，通过高斯方程得到各个点的权重值(the weight of each pixel in the neighborhood is determined by its distance from the center point, calculated using the Gaussian equation)
        binary = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 41, 7)
        cv2.imshow("BIN", binary)
        return binary
    
    def canny_proc(self, bgr_image):
        # 边缘提取，用来进一步分割(当两个相同颜色物体靠在一起时，只能靠边缘去区分)(edge detection, used for further segmentation (when two objects of the same color are adjacent, only edges can distinguish them))
        mask = cv2.Canny(bgr_image, 23, 51, 23, L2gradient=True)
        mask = 255 - cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11)))  # 加粗边界，黑白反转(thicken the borders and invert the colors (black to white and white to black))
        return mask

    def get_top_surface(self, rgb_image):
        # 为了只提取物体最上层表面(to extract only the top surface of the object)
        # 将图像转换到HSV颜色空间(convert the image to HSV color space)
        image_scale = cv2.convertScaleAbs(rgb_image, alpha=2.5, beta=0)
        cv2.imshow("sc", cv2.cvtColor(image_scale, cv2.COLOR_RGB2BGR))
        image_gray = cv2.cvtColor(image_scale, cv2.COLOR_RGB2GRAY)
        image_mb = cv2.medianBlur(image_gray, 3)  # 中值滤波(frame rate counter)
        image_gs = cv2.GaussianBlur(image_mb, (5, 5), 5)  # 高斯模糊去噪(Gaussian blur for noise reduction)
        binary = self.adaptive_threshold(image_gs)  # 阈值自适应(adaptive thresholding)
        mask = self.canny_proc(image_gs)  # 边缘检测(edge detection)
        mask1 = cv2.bitwise_and(binary, mask)  # 合并两个提取出来的图像，保留他们共有的地方(merge two extracted images, retaining their common areas)
        roi_image_mask = cv2.bitwise_and(rgb_image, rgb_image, mask=mask1)  # 和原图做遮罩，保留需要识别的区域(apply a mask to the original image to retain the area that needs to be recognized)
        return roi_image_mask
    
    def get_endpoint(self):
        endpoint = rospy.ServiceProxy('/kinematics/get_current_pose', GetRobotPose)().pose
        self.endpoint = xyz_quat_to_mat([endpoint.position.x, endpoint.position.y,  endpoint.position.z],
                                        [endpoint.orientation.w, endpoint.orientation.x, endpoint.orientation.y, endpoint.orientation.z])
    def done_callback(self, state, result): #动作执行完毕回调(callback function when the action is completed)
        rospy.loginfo("state:%f", state)
        if not result.result.complete: # 如果在移动中被取消，需要回到初始位置(if canceled while in motion, return to the initial position)
            actions.go_home(self.servos_pub)

        if self.finish_percent == 1:  # 如果完整的完成移动(if the motion is completed in full)
            if self.moving_step == 1:  # 如果完成了夹取(if complete grasping)
                # self.moving_step = 2
                self.status = 2
            elif self.moving_step == 2:  # 如果完成了放置(if complete placement)
                actions.go_home(self.servos_pub)
                self.get_endpoint()
                self.last_position = None
                self.target = None
                self.count = 0
                self.moving_step = 0

        else:  # 如果被取消或者无法到达指定位置(if canceled or unable to reach the specified position)
            actions.go_home(self.servos_pub)
            self.moving_step = 0
            self.stackup_step = 0
            self.last_position = None
    
    def place(self):
        goal = MoveGoal()
        goal.grasp.mode = 'place'
        goal.grasp.position.x = -0.005 + TARGET_POSITION[self.target[0]][0]
        goal.grasp.position.y = 0.155 + TARGET_POSITION[self.target[0]] [1]
        goal.grasp.position.z = 0.02
        goal.grasp.pitch = self.pick_pitch
        goal.grasp.align_angle = -90 #yaw #- 20/1000* 240
        goal.grasp.grasp_approach.z = 0.04 # 放置时靠近的方向和距离(direction and distance of approach during placement)
        goal.grasp.grasp_retreat.z = 0.04 # 放置后后撤的方向和距离(direction and distance of retreat after placement)
        goal.grasp.grasp_posture = 400  # 夹取前后夹持器的开合角度(the opening and closing angle of the grippers before and after grasping)
        goal.grasp.pre_grasp_posture = 600
        self.action_client.send_goal(goal, self.done_callback, self.active_callback, self.feedback_callback)
        self.moving_step = 2


    def active_callback(self): # 运动开始回调(callback function when the motion starts)
        self.start_move = True
        rospy.loginfo("start move")
    
    def feedback_callback(self, msg): # 动作执行进度回调(progress callback during action execution)
        rospy.loginfo("finish action: {:.2%}".format(msg.percent))
        self.finish_percent = msg.percent;

    def start_sortting(self, pose_t, pose_R):
        rospy.loginfo("开始搬运堆叠...")
        print(pose_t, pose_R)
        self.moving_step = 1
        goal = MoveGoal()
        goal.grasp.mode = 'pick'
        # 物体坐标(object coordinates)
        goal.grasp.position.x = pose_t[0]
        goal.grasp.position.y = pose_t[1]
        goal.grasp.position.z = 0.012
        # 夹取时的姿态角(position angle during grasping)
        goal.grasp.pitch = self.pick_pitch
        r = pose_R[2] % 90 # 将旋转角限制到 ±45°(limit the rotation angle to ±45°)
        r = r - 90 if r > 45 else (r + 90 if r < -45 else r)
        goal.grasp.align_angle = r
        #夹取时靠近的方向和距离(direction and distance of approach during grasping)
        goal.grasp.grasp_approach.z = 0.04
        #夹取后后撤方向和距离(direction and distance of retreat after grasping)
        goal.grasp.grasp_retreat.z = 0.05
        #夹取前后夹持器的开合(the opening and closing angle of the grippers before and after grasping)
        goal.grasp.grasp_posture = 570
        goal.grasp.pre_grasp_posture = 350
        self.action_client.send_goal(goal, self.done_callback, self.active_callback, self.feedback_callback) # 发送夹取请求(send grasping requirement)

    def action_starting(self, pose_t, pose_R):
        while True:
            if self.status == 1:
                self.status = 0
                self.start_sortting(pose_t,pose_R)
            elif self.status == 2:
                self.status = 0
                self.place()
            else:
                rospy.sleep(0.01)
    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        # 将ros格式图像转换为opencv格式(convert the ros format image to opencv format)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        result_image = np.copy(rgb_image)

        # # 绘制识别区域(draw recognition area)
        if self.imgpts is not None:
            cv2.drawContours(result_image, [self.imgpts], -1, (255, 255, 0), 2, cv2.LINE_AA) # 绘制矩形(draw rectangle)
            for p in self.imgpts:
                cv2.circle(result_image, tuple(p), 8, (255, 0, 0), -1)
            pass

        if self.center_imgpts is not None:
            cv2.line(result_image, (self.center_imgpts[0]-10, self.center_imgpts[1]), (self.center_imgpts[0]+10, self.center_imgpts[1]), (255, 255, 0), 2)
            cv2.line(result_image, (self.center_imgpts[0], self.center_imgpts[1]-10), (self.center_imgpts[0], self.center_imgpts[1]+10), (255, 255, 0), 2)
        # 生成识别区域的遮罩(generate a mask for the recognition area)
        target_list = []
        index = 0
        if self.roi is not None and self.moving_step == 0:
            roi_area_mask = np.zeros(shape=(ros_image.height, ros_image.width, 1), dtype=np.uint8)
            roi_area_mask = cv2.drawContours(roi_area_mask, [self.imgpts], -1, 255, cv2.FILLED)
            rgb_image = cv2.bitwise_and(rgb_image, rgb_image, mask=roi_area_mask)  # 和原图做遮罩，保留需要识别的区域(apply a mask to the original image to retain the area that needs to be recognized)
            roi_img = rgb_image[self.roi[0]:self.roi[1], self.roi[2]:self.roi[3]]
            roi_img = self.get_top_surface(roi_img)
            cv2.imshow("roi", cv2.cvtColor(roi_img, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)
            #result_image[0:int(roi_img.shape[0]), 0:int(roi_img.shape[1])] = roi_img
            image_lab = cv2.cvtColor(roi_img, cv2.COLOR_RGB2LAB) # 转换到 LAB 空间(convert to LAB space)
            self.color_ranges = rospy.get_param('/config/lab', self.color_ranges)
            img_h, img_w = rgb_image.shape[:2]
            for color_name in ['red', 'green', 'blue']:
                color = self.color_ranges[color_name]
                mask = cv2.inRange(image_lab, tuple(color['min']), tuple(color['max']))   # 二值化(binarization)
                # 平滑边缘，去除小块，合并靠近的块(smooth the edges, remove small patches, and merge neighboring patches)
                eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出所有轮廓(find out all contours)
                contours_area = map(lambda c: (math.fabs(cv2.contourArea(c)), c), contours)  # 计算轮廓面积(calculate contour area)
                contours = map(lambda a_c: a_c[1], filter(lambda a: self.min_area <= a[0] <= self.max_area, contours_area))
                for c in contours:
                    # cv2.drawContours(result_image, c, -1, (255, 255, 0), 2, cv2.LINE_AA) # 绘制轮廓(draw contour)
                    rect = cv2.minAreaRect(c)  # 获取最小外接矩形(get the minimum bounding rectangle)
                    (center_x, center_y), r = cv2.minEnclosingCircle(c)
                    center_x, center_y = self.roi[2] + center_x , self.roi[0] + center_y
                    # center_x, center_y = self.roi[2] + rect[0][0], self.roi[0] + rect[0][1] 
                    cv2.circle(result_image, (int(center_x), int(center_y)), 8, (0, 0, 0), -1)
                    corners = list(map(lambda p: (self.roi[2] + p[0], self.roi[0] + p[1]), cv2.boxPoints(rect))) # 获取最小外接矩形的四个角点, 转换回原始图的坐标(get the four angular points of minimum bounding rectangle, convert to the coordinate of original image)
                    cv2.drawContours(result_image, [np.intp(corners)], -1, (0, 255, 255), 2, cv2.LINE_AA)  # 绘制矩形轮廓(draw rectangle contour)
                    index += 1 # 序号递增(increment the index number)
                    angle = int(round(rect[2]))  # 矩形角度(rectangle angle)
                    target_list.append([color_name, index, (center_x, center_y), angle])

        if self.moving_step == 0:
            for target in target_list:
                if self.target_labels[target[0]]:
                   # 颜色处理(color processing)
                   if self.target is not None:
                       if self.target[0] == target[0]:
                           self.count += 1
                       else:
                           self.count = 0
                   self.target = target
                   if self.count > 50:
                       projection_matrix = np.row_stack((np.column_stack((self.extristric[1], self.extristric[0])), np.array([[0, 0, 0, 1]])))
                       world_pose = pixels_to_world([target[2]], self.K, projection_matrix)[0]
                       world_pose[1] = -world_pose[1]
                       world_pose[2] = 0.03
                       world_pose = np.matmul(self.white_area_center, xyz_euler_to_mat(world_pose, (0, 0, 0)))
                       world_pose[2] = 0.03
                       print(world_pose)
                       #pose_end = np.matmul(self.hand2cam_tf_matrix, world_pose) # 转换的末端相对坐标(relative coordinates of the converted end)
                       #pose_world = np.matmul(self.endpoint, pose_end) # 转换到机械臂世界坐标(convert to the robotic arm's world coordinates)
                       pose_t, _ = mat_to_xyz_euler(world_pose)
                       pose_t[2] = 0.010
                       params = rospy.get_param(os.path.join(POSITIONS_PATH, 'color_sortting'))
                       for i in range(3):
                          pose_t[i] = pose_t[i] + params['offset'][i]
                          pose_t[i] = pose_t[i] * params['scale'][i]
                          
                          # 高度补偿，越远重力下垂越严重补偿下垂高度(height compensation, the farther the gravity droop is more severe, compensating for droop height)
                       pose_t[2] += (math.sqrt(pose_t[1] ** 2 + pose_t[0] ** 2) - 0.15) / 0.20 * 0.025
                       print(pose_t)
                       self.target = target
                       if self.moving_step == 0:
                            self.moving_step = 1
                            self.status = 1
                            threading.Thread(target=self.action_starting, args=(pose_t, (0, 0, target[3]))).start()
                   break

        # 计算帧率及发布结果图像(calculate the frame rate and publish the resulting image)
        #self.fps.update()
        #result_image = self.fps.show_fps(result_image)
        cv2.imshow('color_sorting', cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)



if __name__ == "__main__":
    node = ColorSorttingNode("color_sortting", log_level=rospy.DEBUG)
    rospy.spin()
