#!/usr/bin/python3
#coding=utf8

# 通过深度相机追踪物体，控制机械臂上下左右运动，夹爪旋转(tracking objects using a depth camera, controlling the up, down, left, and right movements of the robotic arm, and rotating the gripper)

import cv2
import rospy
import numpy as np
import math
import message_filters
import time
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import SetBool
from vision_utils import fps
from hiwonder_interfaces.srv import GetRobotPose
import threading
import queue
from hiwonder_interfaces.msg import MultiRawIdPosDur
from vision_utils import xyz_quat_to_mat, xyz_euler_to_mat, mat_to_xyz_euler
from jetarm_sdk import pid, bus_servo_control, sdk_client, tone
from jetarm_kinematics import kinematics_control
from jetarm_kinematics.inverse_kinematics import get_ik
from jetarm_kinematics import kinematics_control
import jetarm_kinematics.transform as transform

CONFIG_NAME="/config"

def depth_pixel_to_camera(pixel_coords, depth, intrinsics):
    fx, fy, cx, cy = intrinsics
    px, py = pixel_coords
    x = (px - cx) * depth / fx
    y = (py - cy) * depth / fy
    z = depth
    return np.array([x, y, z])


class RgbDepthImageNode:
    def __init__(self):
        self.pid_yaw = pid.PID(20.5, 0, 1.2)
        self.pid_z = pid.PID(0.025, 0, 0.0000)
        self.pid_dist = pid.PID(0.00008, 0, 0.0)
        self.yaw = 500
        self.z = 0.25
        self.x = 0.15
        self.gripper = 500

        rospy.init_node('remove_too_high_node', anonymous=True)
        config = rospy.get_param(CONFIG_NAME)
        self.endpoint = None
        self.hand2cam_tf_matrix = config['hand2cam_tf_matrix']
        self.fps = fps.FPS()
        self.last_shape = "none"
        self.moving = True
        self.stamp = time.time()
        self.count = 0
        self.last_position = (0, 0, 0)
        self.sdk = sdk_client.JetArmSDKClient()

        self.servos_pub = rospy.Publisher('/controllers/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        rospy.sleep(3)
        self.current_servo_positions = [500, 500, 500, 500, 500]
        p = self.set_pose_target((self.x, 0, self.z), 0)
        if len(p[1]) > 0:
            p[1][0] = self.yaw
            p[1][4] = int(self.gripper)
            self.current_servo_positions = p[1]
            bus_servo_control.set_servos(self.servos_pub, 1000, ((1, p[1][0]), (2, p[1][1]), (3, p[1][2]), (4, p[1][3]), (5, p[1][4])))
        rospy.sleep(1)
        
        rospy.wait_for_service('/rgbd_cam/set_ldp')
        rospy.ServiceProxy('/rgbd_cam/set_ldp', SetBool)(False)
        threading.Thread(target=self.get_endpoint, args=()).start()
        
        rgb_sub = message_filters.Subscriber('/rgbd_cam/color/image_raw', RosImage, queue_size=1)
        depth_sub = message_filters.Subscriber('/rgbd_cam/depth/image_raw', RosImage, queue_size=1)
        info_sub = message_filters.Subscriber('/rgbd_cam/depth/camera_info', CameraInfo, queue_size=1)
    
        # 同步时间戳, 时间允许有误差在0.03s(synchronize timestamps, allowing for a time error of up to 0.03 seconds)
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, 0.03)
        sync.registerCallback(self.multi_callback) #执行反馈函数(perform feedback function)
        self.queue = queue.Queue(maxsize=1)


    def set_pose_target(self, position, pitch):
        # 逆运动学解，获取最优解(所有电机转动最小)(inverse kinematics solution, obtain optimal solution (minimize rotation of all motors))
        # t1 = rospy.get_time()
        all_solutions = get_ik(list(position), pitch, [-180, 180], 1)
        if all_solutions != [] and self.current_servo_positions != []:
            rpy = []
            min_d = 1000*5
            optimal_solution = []
            for s in all_solutions:
                pulse_solutions = transform.angle2pulse(s[0])
                try:
                    for i in pulse_solutions:
                        d = np.array(i) - self.current_servo_positions
                        d_abs = np.maximum(d, -d)
                        min_sum = np.sum(d_abs)
                        if min_sum < min_d:
                            min_d = min_sum
                            for k in range(len(i)):
                                if i[k] < 0:
                                    i[k] = 0
                                elif i[k] > 1000:
                                    i[k] = 1000
                            rpy = s[1]
                            optimal_solution = i
                except BaseException as e:
                    print('choose solution error', e)
                    #print(pulse_solutions, current_servo_positions)
                # print(rospy.get_time() - t2)
            return [True, optimal_solution, self.current_servo_positions, rpy, min_d]
        else:
            return [True, [], [], [], 0]


    def get_endpoint(self):
        while not rospy.is_shutdown():
            endpoint = rospy.ServiceProxy('/kinematics/get_current_pose', GetRobotPose)().pose
            self.endpoint = xyz_quat_to_mat([endpoint.position.x, endpoint.position.y, endpoint.position.z],
                                        [endpoint.orientation.w, endpoint.orientation.x, endpoint.orientation.y, endpoint.orientation.z])
            rospy.sleep(0.5)

    def goto(self, position, angle, g_angle,  gripper, duration):
        ret = kinematics_control.set_pose_target(position, angle)
        if len(ret[1]) > 0:
            bus_servo_control.set_servos(self.servos_pub, duration, ((1, ret[1][0]),(2, ret[1][1]), (3, ret[1][2]),(4, ret[1][3]), (5, g_angle), (10, gripper)))
            rospy.sleep(duration / 1000.0)
        else:
            print("ERROR not found")

    def pick(self):
        p = self.set_pose_target((self.x + 0.20, 0, self.z + 0.02), 0)
        if len(p[1]) > 0:
            p[1][0] = self.yaw
            p[1][4] = int(self.gripper)
            self.current_servo_positions = p[1]
            bus_servo_control.set_servos(self.servos_pub, 2000, ((1, p[1][0]), (2, p[1][1]), (3, p[1][2]), (4, p[1][3]), (5, p[1][4])))
        else:
            print("ERR")
        rospy.sleep(2.2)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((10, 600),))
        rospy.sleep(2)
        self.x = 0.15
        self.z = 0.20
        self.yaw = 500
        self.gripper = 500
        p = self.set_pose_target((self.x, 0, self.z), 0)
        if len(p[1]) > 0:
            p[1][0] = self.yaw
            p[1][4] = int(self.gripper)
            self.current_servo_positions = p[1]
            bus_servo_control.set_servos(self.servos_pub, 2000, ((1, p[1][0]), (2, p[1][1]), (3, p[1][2]), (4, p[1][3]), (5, p[1][4])))
        rospy.sleep(2)
        self.goto((0.15, 0.25, 0.19),  0, 500, 600, 1400)
        self.goto((0.15, 0.25, 0.1),  0, 500, 550, 1200)
        self.goto((0.15, 0.25, 0.1),  0, 500, 200, 2500)
        self.goto((0.15, 0.25, 0.19),  0, 500, 200, 800)
        bus_servo_control.set_servos(self.servos_pub, 2000, ((1, p[1][0]), (2, p[1][1]), (3, p[1][2]), (4, p[1][3]), (5, p[1][4])))


    def multi_callback(self, ros_rgb_image, ros_depth_image, camera_info):
        if self.queue.empty():
            self.queue.put_nowait((ros_rgb_image, ros_depth_image, camera_info))

    def image_proc(self):
        ros_rgb_image, ros_depth_image, camera_info = self.queue.get(block=True)
        try:
            rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
            depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
            K = camera_info.K


            ih, iw = depth_image.shape[:2]

            
            depth = depth_image.copy()
            depth[380:400, :] = np.array([[55555,]*640]*20)
            depth = depth.reshape((-1, )).copy()
            depth[depth<=100] = 55555
            min_index = np.argmin(depth)
            min_y = min_index // iw
            min_x = min_index - min_y * iw

            min_dist = depth_image[min_y, min_x]
            sim_depth_image = np.clip(depth_image, 0, 400).astype(np.float64) / 400 * 255
            depth_image = np.where(depth_image > min_dist + 17, 0, depth_image)
            sim_depth_image_sort = np.clip(depth_image, 0, 400).astype(np.float64) / 400 * 255
            depth_gray = sim_depth_image_sort.astype(np.uint8)
            depth_gray = cv2.GaussianBlur(depth_gray, (5, 5), 0)
            _, depth_bit = cv2.threshold(depth_gray, 1, 255, cv2.THRESH_BINARY)
            depth_bit = cv2.erode(depth_bit, np.ones((5, 5), np.uint8))
            depth_bit = cv2.dilate(depth_bit, np.ones((3, 3), np.uint8))
            depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)

            contours, hierarchy = cv2.findContours(depth_bit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            shape = 'none'
            txt = ""
            z = 0
            largest = None
            largest_area = 0
            for obj in contours:
                area = cv2.contourArea(obj)
                if area < 500 or self.moving is True:
                    continue
                if area > largest_area:
                    largest = obj

            if largest is not None and not self.moving:
                obj = largest
                #cv2.drawContours(depth_color_map, obj, -1, (255, 255, 0), 4) # 绘制轮廓线(draw contour line)
                (cx, cy), radius = cv2.minEnclosingCircle(obj)
                cv2.circle(depth_color_map, (int(cx), int(cy)), int(radius), (0, 0, 255), 2)
                #position = depth_pixel_to_camera((cx, cy), depth_image[int(cy), int(cx)] / 1000.0, (K[0], K[4], K[2], K[5]))
                #position[2] = position[2] + 0.03
                #pose_end = np.matmul(self.hand2cam_tf_matrix, xyz_euler_to_mat(position, (0, 0, 0)))  # 转换的末端相对坐标(the relative coordinates of the transformed end effector)
                #world_pose = np.matmul(self.endpoint, pose_end)  # 转换到机械臂世界坐标(transfer to the robotic arm's world coordinates)
                #pose_t, pose_R = mat_to_xyz_euler(world_pose)
                #pose_t[1] += 0.01
                #if pose_t[2] > z:
                #    largest = obj, pose_t
                min_x = cx
                min_y = cy
                #min_dist = depth_image[int(cy), int(cx)]
                txt = 'Distance: {}mm'.format(min_dist)
            
            if min_dist > 200 and min_dist < 380 and largest is not None and not self.moving:
                center_x = min_x / 640
                if abs(center_x - 0.5) > 0.02: # 相差范围小于一定值就不用再动了(if the difference range is smaller than a certain value, there is no need to move anymore)
                    self.pid_yaw.SetPoint = 0.5 # 我们的目标是要让色块在画面的中心, 就是整个画面的像素宽度的 1/2 位置(our goal is to position the colored block at the center of the frame, which is at a position equal to half the pixel width of the entire frame)
                    self.pid_yaw.update(center_x)
                    self.yaw = min(max(self.yaw + self.pid_yaw.output, 0), 1000)
                else:
                    self.pid_yaw.clear() # 如果已经到达中心了就复位一下 pid 控制器(if it has already reached the center, reset the PID controller)

                center_y = min_y / 400
                if abs(center_y - 0.5) > 0.02:
                    self.pid_z.SetPoint = 0.5
                    self.pid_z.update(center_y)
                    self.z = self.z + self.pid_z.output
                else:
                    self.pid_z.clear()

                dist_err = min_dist - 260
                if abs(dist_err) > 10:
                    self.pid_dist.SetPoint = 0
                    self.pid_dist.update(dist_err)
                    self.x = self.x - self.pid_dist.output
                else:
                    self.pid_dist.clear()

                rect = cv2.minAreaRect(largest)  # 获取最小外接矩形(get the minimum bounding rectangle)
                angle = rect[2]
                if angle > 45:
                    angle = angle - 90
                self.gripper = self.gripper * 0.95 + (500 + angle * 1000 / 260) * 0.05
                p = self.set_pose_target((self.x, 0, self.z), 0)
                if len(p[1]) > 0:
                    p[1][0] = self.yaw
                    p[1][4] = int(self.gripper)
                    self.current_servo_positions = p[1]
                    bus_servo_control.set_servos(self.servos_pub, 30, ((1, p[1][0]), (2, p[1][1]), (3, p[1][2]), (4, p[1][3]), (5, p[1][4])))
            else:
                self.pid_dist.clear()
                self.pid_z.clear()
                self.pid_yaw.clear()

            #if largest is not None:
            #    obj, pose_t = largest
            #    dist = math.sqrt((self.last_position[0] - pose_t[0]) ** 2 + (self.last_position[1] - pose_t[1])** 2 + (self.last_position[2] - pose_t[2])**2)
            bgr_image = cv2.cvtColor(rgb_image[40:440, ], cv2.COLOR_RGB2BGR)
            if not self.moving:

                self.last_shape = shape
                cv2.circle(depth_color_map, (int(min_x), int(min_y)), 8, (32, 32, 32), -1)
                cv2.circle(depth_color_map, (int(min_x), int(min_y)), 6, (255, 255, 255), -1)
                cv2.putText(depth_color_map, txt, (11, ih-20), cv2.FONT_HERSHEY_PLAIN, 2.0, (32, 32, 32), 6, cv2.LINE_AA)
                cv2.putText(depth_color_map, txt, (10, ih-20), cv2.FONT_HERSHEY_PLAIN, 2.0, (240, 240, 240), 2, cv2.LINE_AA)

                #cv2.circle(bgr_image, (int(min_x), int(min_y)), 8, (32, 32, 32), -1)
                #cv2.circle(bgr_image, (int(min_x), int(min_y)), 6, (255, 255, 255), -1)
                cv2.putText(bgr_image, txt, (11, ih - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (32, 32, 32), 6, cv2.LINE_AA)
                cv2.putText(bgr_image, txt, (10, ih - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (240, 240, 240), 2, cv2.LINE_AA)

                self.fps.update()
                #bgr_image = self.fps.show_fps(bgr_image)
            result_image = np.concatenate([bgr_image, depth_color_map], axis=1)
            cv2.imshow("depth", result_image)
            # cv2.imshow("depth_gray", depth_bit)
            key = cv2.waitKey(1)
            if key != -1:
                if key == 97: #A
                    self.moving = True
                if key == 115: # S
                    self.moving = False
                if key == 100:
                    self.x = 0.15
                    self.z = 0.20
                    self.yaw = 500
                    self.gripper = 500
                    p = self.set_pose_target((self.x, 0, self.z), 0)
                    p[1][0] = self.yaw
                    p[1][4] = int(self.gripper)
                    self.current_servo_positions = p[1]
                    bus_servo_control.set_servos(self.servos_pub, 1000, ((1, p[1][0]), (2, p[1][1]), (3, p[1][2]), (4, p[1][3]), (5, p[1][4])))
                print(key)

        except Exception as e:
            rospy.logerr('callback error:', str(e))

if __name__ == "__main__":
    print('''
    *********************************************
    *                                           *
    * 此程序需要订阅深度摄像头节点,开启前请确保(before starting this program, make sure to subscribe to the depth camera node) *
    * 已开启摄像头节点, 通过rostopic list可查看(the camera node has been started, you can verify this by using "rostopic list") *
    * 是否有usb_cam相关节点,成功运行可看到终端(is there a USB_cam related node? You should see the terminal output upon successful execution)  *
    * running ...                               *
    *                                           * 
    *********************************************
    ''')

    try:
        node = RgbDepthImageNode()
        while not rospy.is_shutdown():
            node.image_proc()
    except KeyboardInterrupt:
        rospy.loginfo("shutdown2")

