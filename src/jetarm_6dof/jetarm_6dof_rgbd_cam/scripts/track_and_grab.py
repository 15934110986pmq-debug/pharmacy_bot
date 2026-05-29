#!/usr/bin/env python3
# coding: utf8
# 机械前超前看识别追踪空中指定颜色物品(mechanical forward-looking recognition and tracking of specified color objects in the air)
# 通过深度相机识别计算物品的空间位置(recognize and calculate the spatial position of objects using a depth camera)
# 完成抓取并放到指定位置(complete grasping and place it to the specified position)

import os
import cv2
import rospy
import numpy as np
import threading
import math
import time
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker
from hiwonder_interfaces.msg import MultiRawIdPosDur
from hiwonder_interfaces.srv import GetRobotPose
from vision_utils import fps, colors, get_area_max_contour
from vision_utils import xyz_quat_to_mat, xyz_euler_to_mat, pixels_to_world, box_center, mat_to_xyz_euler, distance, extristric_plane_shift
from jetarm_sdk import pid, bus_servo_control, common
import message_filters
from jetarm_kinematics import kinematics_control
from sensor_msgs.msg import Image as RosImage
from  geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool
import queue

CONFIG_NAME="/config"

def depth_pixel_to_camera(pixel_coords, depth, intrinsics):
    fx, fy, cx, cy = intrinsics
    px, py = pixel_coords
    x = (px - cx) * depth / fx
    y = (py - cy) * depth / fy
    z = depth
    return np.array([x, y, z])


class ColorTracker:
    def __init__(self, target_color):
        self.target_color = target_color
        self.pid_yaw = pid.PID(20.5, 1.0, 1.2)
        self.pid_pitch = pid.PID(20.5, 1.0, 1.2)
        self.yaw = 500
        self.pitch = 150
    
    def proc (self, source_image, result_image, color_ranges):
        h, w = source_image.shape[:2]
        color = color_ranges[self.target_color]

        img = cv2.resize(source_image, (int(w/2), int(h/2)))
        img_blur = cv2.GaussianBlur(img, (3, 3), 3) # 高斯模糊(Gaussian blur)
        img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB) # 转换到 LAB 空间(convert to the LAB space)
        mask = cv2.inRange(img_lab, tuple(color['min']), tuple(color['max'])) # 二值化(binarization)

        # 平滑边缘，去除小块，合并靠近的块(smooth edges, remove small blocks, and merge adjacent blocks)
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

        # 找出最大轮廓(find out the largest contour)
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        #max_contour_area = get_area_max_contour(contours, 10)
        min_c = None
        for c in contours:
            if math.fabs(cv2.contourArea(c)) < 500:
                continue
            (center_x, center_y), radius = cv2.minEnclosingCircle(c) # 最小外接圆(the minimum circumcircle)
            if min_c is None:
                min_c = (c, center_x)
            elif center_x < min_c[1]:
                if center_x < min_c[1]:
                    min_c = (c, center_x)

        # 如果有符合要求的轮廓(if there are contours meet requirement)
        if min_c is not None:
            (center_x, center_y), radius = cv2.minEnclosingCircle(min_c[0]) # 最小外接圆(the minimum circumcircle)

            # 圈出识别的的要追踪的色块(outline the identified color blocks to be tracked)
            circle_color = colors.rgb[self.target_color] if self.target_color in colors.rgb else (0x55, 0x55, 0x55)
            cv2.circle(result_image, (int(center_x * 2), int(center_y * 2)), int(radius * 2), circle_color, 2)

            center_x = center_x * 2
            center_x_1 = center_x / w
            if abs(center_x_1 - 0.5) > 0.02: # 相差范围小于一定值就不用再动了(if the difference range is smaller than a certain value, there's no need to move anymore)
                self.pid_yaw.SetPoint = 0.5 # 我们的目标是要让色块在画面的中心, 就是整个画面的像素宽度的 1/2 位置(our goal is to make the color block appear in the center of the frame, which is at half the pixel width of the entire frame)
                self.pid_yaw.update(center_x_1)
                self.yaw = min(max(self.yaw + self.pid_yaw.output, 0), 1000)
            else:
                self.pid_yaw.clear() # 如果已经到达中心了就复位一下 pid 控制器(if it has already reached the center, reset the PID controller)

            center_y = center_y * 2
            center_y_1 = center_y / h
            if abs(center_y_1 - 0.5) > 0.02:
                self.pid_pitch.SetPoint = 0.5
                self.pid_pitch.update(center_y_1)
                self.pitch = min(max(self.pitch + self.pid_pitch.output, 100), 720)
            else:
                self.pid_pitch.clear()
            # rospy.loginfo("x:{:.2f}\ty:{:.2f}".format(self.x , self.y))
            return (result_image, (self.pitch, self.yaw), (center_x, center_y), radius * 2)
        else:
            return (result_image, None, None, 0)


class TrackAndGrapNode:
    def __init__(self):
        rospy.init_node("track_and_grap", anonymous=True, log_level=rospy.INFO)
        self.fps = fps.FPS()
        self.moving = False
        self.last_pitch_yaw = (0, 0)
        self.first_time = time.time()

        self.enable_disp = rospy.get_param('~enable_disp', True)
        self.color_ranges = rospy.get_param('/config/lab', {})
        self.last_position = (0, 0, 0)
        self.stamp = time.time()
        self.servos_pub = rospy.Publisher('/controllers/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        rospy.sleep(3)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 200)))
        rospy.sleep(2)

        config = rospy.get_param(CONFIG_NAME)
        self.hand2cam_tf_matrix = config['hand2cam_tf_matrix']

        self.target_color = rospy.get_param('~target_color', 'red')
        rospy.loginfo("正在设置将要追踪的目标认识为" + self.target_color)
        self.tracker = ColorTracker(self.target_color)
        rospy.wait_for_service('/rgbd_cam/set_ldp')
        
        self.target_marker = Marker()
        self.pose = PoseStamped()
        self.marker_pub = rospy.Publisher('target_object_sphere', Marker, queue_size=1) 
        self.pose_pub = rospy.Publisher('/target_pose_to_tf/pose', PoseStamped, queue_size=1)

        rgb_sub = message_filters.Subscriber('/rgbd_cam/color/image_raw', RosImage, queue_size=1)
        depth_sub = message_filters.Subscriber('/rgbd_cam/depth/image_raw', RosImage, queue_size=1)
        info_sub = message_filters.Subscriber('/rgbd_cam/depth/camera_info', CameraInfo, queue_size=1)

        rospy.ServiceProxy('/rgbd_cam/set_ldp', SetBool)(False)
    
        # 同步时间戳, 时间允许有误差在0.03s(synchronize timestamps, allowing for a time error of up to 0.03 seconds)
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, 0.03)
        sync.registerCallback(self.multi_callback) #执行反馈函数(execute feedback function)
        self.queue = queue.Queue(maxsize=1)
        self.endpoint = None
        self.ttt = time.time() + 10
        common.loginfo("TrackAndGrapNode initailized")

    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if self.queue.empty():
            self.queue.put_nowait((ros_rgb_image, ros_depth_image, depth_camera_info))

    def get_endpoint(self):
        endpoint = rospy.ServiceProxy('/kinematics/get_current_pose', GetRobotPose)().pose
        self.endpoint = xyz_quat_to_mat([endpoint.position.x, endpoint.position.y, endpoint.position.z],
                                        [endpoint.orientation.w, endpoint.orientation.x, endpoint.orientation.y, endpoint.orientation.z])
        return self.endpoint

    def pick(self, position):
        ret = kinematics_control.set_pose_target(position, 30)
        if len(ret[1]) > 0:
            bus_servo_control.set_servos(self.servos_pub, 1000, ((1, ret[1][0]), ))
            rospy.sleep(1)
            bus_servo_control.set_servos(self.servos_pub, 1500, ((1, ret[1][0]),(2, ret[1][1]), (3, ret[1][2]),(4, ret[1][3]), (5, ret[1][4])))
            rospy.sleep(1.5)
        bus_servo_control.set_servos(self.servos_pub, 500, ((10, 700),))
        rospy.sleep(1)
        position[2] += 0.03
        ret = kinematics_control.set_pose_target(position, 30)
        if len(ret[1]) > 0:

            bus_servo_control.set_servos(self.servos_pub, 1000, ((1, ret[1][0]),(2, ret[1][1]), (3, ret[1][2]),(4, ret[1][3]), (5, ret[1][4])))
            rospy.sleep(1)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 700)))
        rospy.sleep(1)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 125), (2, 635), (3, 120), (4, 200), (5, 500)))
        rospy.sleep(1)
        bus_servo_control.set_servos(self.servos_pub, 500, ((1, 125), (2, 550), (3, 120), (4, 150), (5, 500)))
        rospy.sleep(0.5)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 125), (2, 550), (3, 120), (4, 170), (5, 500), (10, 200)))
        rospy.sleep(1)
        #bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 125), (2, 720), (3, 120), (4, 170), (5, 500), (10, 200)))
        #rospy.sleep(1)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 720), (3, 100), (4, 150), (5, 500), (10, 200)))
        rospy.sleep(2)
        self.tracker.yaw = 500
        self.tracker.pitch = 150
        self.tracker.pid_yaw.clear()
        self.tracker.pid_pitch.clear()
        self.stamp = time.time()
        self.first_time = time.time() + 2
        self.moving = False

    def image_proc(self):
        ros_rgb_image, ros_depth_image, depth_camera_info = self.queue.get(block=True)
        try:
            rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)[40:440,]
            depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
            result_image = np.copy(rgb_image)

            h, w = depth_image.shape[:2]
            depth = np.copy(depth_image).reshape((-1, ))
            depth[depth<=0] = 55555
            
            sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64)

            sim_depth_image = sim_depth_image / 2000.0 * 255.0
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

            depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)

            if self.tracker is not None and self.moving == False and time.time() > self.ttt:
                result_image, p_y, center, r = self.tracker.proc(rgb_image, result_image, self.color_ranges)
                if p_y is not None:
                    bus_servo_control.set_servos(self.servos_pub, 20, ((1, p_y[1]), (4, p_y[0])))
                    # print(p_y)
                    center_x, center_y = center
                    center_x += 5
                    center_y += r - 20
                    if center_x > 639:
                        center_x = 639
                    if center_y > 399:
                        center_y = 399
                    #print("center_x:", center_x)
                    if abs(self.last_pitch_yaw[0] - p_y[0]) < 5 and abs(self.last_pitch_yaw[1] - p_y[1]) < 5:
                        if time.time() - self.stamp > 2:
                            self.stamp = time.time()
                            dist = depth_image[int(center_y),int(center_x)]/1000.0
                            dist += 0.015 # 物体半径补偿(object radius compensation)
                            K = depth_camera_info.K
                            self.get_endpoint()
                            position = depth_pixel_to_camera((center_x, center_y), dist, (K[0], K[4], K[2], K[5]))
                            position[0] -= 0.01  # rgb相机和深度相机tf有1cm偏移(the RGB camera and the depth camera have a 1cm offset in their transforms)
                            pose_end = np.matmul(self.hand2cam_tf_matrix, xyz_euler_to_mat(position, (0, 0, 0)))  # 转换的末端相对坐标(transform to end effector relative coordinates)
                            world_pose = np.matmul(self.endpoint, pose_end)  # 转换到机械臂世界坐标(transform to the robotic arm's world coordinates)
                            pose_t, pose_R = mat_to_xyz_euler(world_pose)
                            offset_x = 0.03
                            offset_y = 0.00
                            offset_z = 0.015
                            factor_y = 1.05
                            factor_z = 0.035
                            pose_t[0] += offset_x
                            pose_t[1] += offset_y
                            pose_t[2] += offset_z
                            # 左右(Y轴), 左右幅度不足，机械臂离中心线越远偏离目标远越远的情况增加factor_y(left and right (Y-axis), insufficient left and right amplitudes, when the robotic arm is further from the centerline, the deviation from the target increases as well, increasing factor_y)
                            pose_t[1] = pose_t[1] * factor_y
                            # 高度补偿(Z轴)，由于重力作用机械臂伸出越长下垂越多需要做些补偿(height compensation (Z-axis), due to the effect of gravity, the longer the robotic arm extends, the more it droops, so some compensation needs to be made)
                            # 简单的线性补偿, 当末端距离机械臂中心距离小于0.2时不补偿，超过0.2时距离越大补偿越多，调节factor变补偿效果(simple linear compensation means that when the distance between the end effector and the center of the robotic arm is less than 0.2, no compensation is applied. As the distance exceeds 0.2, the compensation increases with greater distance. Adjusting the factor modifies the compensation effect)
                            pose_t[2] += (math.sqrt(pose_t[1] ** 2 + pose_t[0] ** 2) - 0.2) / 0.25 * factor_z
                            # print(pose_t)
                            self.stamp = time.time()
                            self.moving = True
                            threading.Thread(target=self.pick, args=(pose_t,)).start()
                            # pose_t[1] -= 0.02
                            #self.pose.header.stamp = rospy.get_rostime()
                            #self.pose.header.frame_id = 'target_object'
                            #self.pose.pose.position.x = pose_t[0]
                            #self.pose.pose.position.y = pose_t[1]
                            #self.pose.pose.position.z = pose_t[2]
                            #self.pose.pose.orientation.x = 0
                            #self.pose.pose.orientation.y = 0
                            #self.pose.pose.orientation.z = 0
                            #self.pose.pose.orientation.w = 1.0
                            #self.pose_pub.publish(self.pose)

                            #scale = 0.03
                            #self.target_marker.header = self.pose.header
                            #self.target_marker.id = 42
                            #self.target_marker.type = 9 # View-Oriented Text
                            #self.target_marker.action = 0
                            #self.target_marker.pose.position.x = pose_t[0] * scale - 0.01
                            #self.target_marker.pose.position.y = pose_t[1] * scale
                            #self.target_marker.pose.position.z = pose_t[2] * scale + 0.03
                            #self.target_marker.pose.orientation.x = 0
                            #self.target_marker.pose.orientation.y = 0
                            #self.target_marker.pose.orientation.z = 0
                            #self.target_marker.pose.orientation.w = 1.0
                            #self.target_marker.scale.x = scale
                            #self.target_marker.scale.y = scale
                            #self.target_marker.scale.z = scale
                            #self.target_marker.color.r = 1.0
                            #self.target_marker.color.g = 1.0
                            #self.target_marker.color.b = 1.0
                            #self.target_marker.color.a = 1.0
                            #self.target_marker.lifetime = rospy.Duration(1)
                            #self.target_marker.text = 'X:{:.3f} Y:{:.3f} Z:{:.3f}'.format(pose_t[0], pose_t[1], pose_t[2])
                            #self.marker_pub.publish(self.target_marker)

                            #self.target_marker.id = 43
                            #self.target_marker.pose.position.z = pose_t[2] * scale - 0.005
                            #self.target_marker.type = 2
                            #self.target_marker.color.g = 0
                            #self.target_marker.color.b = 1.0
                            #self.target_marker.text = ""
                            #self.marker_pub.publish(self.target_marker)
     
                            
                    else:
                        self.stamp = time.time()
                    dist = depth_image[int(center_y),int(center_x)]
                    if dist < 100:
                        txt = "TOO CLOSE !!!"
                    else:
                        txt = "Dist: {}mm".format(dist)
                    cv2.circle(result_image, (int(center_x), int(center_y)), 5, (255, 255, 255), -1)
                    cv2.circle(depth_color_map, (int(center_x), int(center_y)), 5, (255, 255, 255), -1)
                    cv2.putText(depth_color_map, txt, (10, 400 - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 0), 10, cv2.LINE_AA)
                    cv2.putText(depth_color_map, txt, (10, 400 - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (255, 255, 255), 2, cv2.LINE_AA)
                    self.last_pitch_yaw = p_y
                else:
                    self.stamp = time.time()

                    #cv2.circle(depth_color_map, (x, y), 12, (0, 0, 0), -1)
                    #cv2.circle(depth_color_map, (x, y), 10, (255, 255, 255), -1)
            if self.enable_disp:
                self.fps.update()
                bgr_image = self.fps.show_fps(result_image)
                result_image = np.concatenate([cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR), depth_color_map, ], axis=1)
                cv2.imshow("depth", result_image)
                key = cv2.waitKey(1)
                if key != -1:
                    rospy.signal_shutdown('shutdown1')

        except Exception as e:
            rospy.logerr('callback error:', str(e))

if __name__ == "__main__":
    print('''
    *********************************************
    *                                           *
    * 此程序需要订阅深度相机节点,开启前请确保已(before starting this program, make sure to subscribe to the depth camera node) *
    * 开启摄像头节点, 通过rostopic list可查看是看(the camera node has been started, you can verify this by using "rostopic list") *
    * 否有rgbd_cam相关节点,成功运行可看到终端(is there a rgbd_cam related node? You should see the terminal output upon successful execution)   *
    * running ...                               *
    *                                           * 
    *********************************************
    ''')

    try:
        node = TrackAndGrapNode()
        while not rospy.is_shutdown():
            node.image_proc()
    except KeyboardInterrupt:
        rospy.loginfo("shutdown2")

