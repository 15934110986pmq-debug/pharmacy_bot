#!/usr/bin/python3
#coding=utf8

# 等高保持(maintain equal height)
# 找出识别区域高度最高的物体(find the tallest object in the recognition area)
# 如果物体高度超过阈值(if the object height exceeds the threshold)
# 则将最高的物体移除(remove the tallest object)
# 程序测试使用的是 30x30x30mm的木块和 40x40x40mm的木块(the program is tested using a 30x30x30mm wooden block and a 40x40x40mm wooden block)

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
        rospy.init_node('remove_too_high_node', anonymous=True)
        config = rospy.get_param(CONFIG_NAME)
        self.endpoint = None
        self.hand2cam_tf_matrix = config['hand2cam_tf_matrix']
        self.fps = fps.FPS()
        self.last_shape = "none"
        self.moving = False
        self.stamp = time.time()
        self.count = 0
        self.last_position = (0, 0, 0)
        self.sdk = sdk_client.JetArmSDKClient()

        self.servos_pub = rospy.Publisher('/controllers/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        rospy.sleep(3)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 540), (3, 220), (4, 50), (5, 500), (10, 200)))
        rospy.sleep(2)
        
        rospy.wait_for_service('/rgbd_cam/set_ldp')
        rospy.ServiceProxy('/rgbd_cam/set_ldp', SetBool)(False)
        threading.Thread(target=self.get_endpoint, args=()).start()
        
        rgb_sub = message_filters.Subscriber('/rgbd_cam/color/image_raw', RosImage, queue_size=1)
        depth_sub = message_filters.Subscriber('/rgbd_cam/depth/image_raw', RosImage, queue_size=1)
        info_sub = message_filters.Subscriber('/rgbd_cam/depth/camera_info', CameraInfo, queue_size=1)
    
        # 同步时间戳, 时间允许有误差在0.03s(synchronize timestamps, allowing for a time error of up to 0.03 seconds)
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, 0.03)
        sync.registerCallback(self.multi_callback) #执行反馈函数(execute feedback function)
        self.queue = queue.Queue(maxsize=1)

    def get_endpoint(self):
        while not rospy.is_shutdown():
            endpoint = rospy.ServiceProxy('/kinematics/get_current_pose', GetRobotPose)().pose
            self.endpoint = xyz_quat_to_mat([endpoint.position.x, endpoint.position.y, endpoint.position.z],
                                        [endpoint.orientation.w, endpoint.orientation.x, endpoint.orientation.y, endpoint.orientation.z])
            rospy.sleep(0.5)

    def pick(self, position, angle):
        angle =  angle % 90
        angle = angle - 90 if angle > 40 else (angle + 90 if angle < -45 else angle)
        position[2] += 0.05
        ret = kinematics_control.set_pose_target(position, 80)
        if len(ret[1]) > 0:
            bus_servo_control.set_servos(self.servos_pub, 1500, ((1, ret[1][0]),(2, ret[1][1]), (3, ret[1][2]),(4, ret[1][3]), (5, ret[1][4])))
            rospy.sleep(1.5)
        angle = 500 + int(1000 * (angle + ret[3][-1]) / 240)
        bus_servo_control.set_servos(self.servos_pub, 500, ((5, angle),))
        rospy.sleep(0.5)
        position[2] -= 0.05
        ret = kinematics_control.set_pose_target(position, 80)
        if len(ret[1]) > 0:
            bus_servo_control.set_servos(self.servos_pub, 1500, ((1, ret[1][0]),(2, ret[1][1]), (3, ret[1][2]),(4, ret[1][3]), (5, angle)))
            rospy.sleep(1.5)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((10, 700),))
        rospy.sleep(1)
        position[2] += 0.05
        ret = kinematics_control.set_pose_target(position, 80)
        if len(ret[1]) > 0:
            bus_servo_control.set_servos(self.servos_pub, 1500, ((1, ret[1][0]),(2, ret[1][1]), (3, ret[1][2]),(4, ret[1][3]), (5, angle)))
            rospy.sleep(1.5)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 740), (3, 100), (4, 260), (5, 500)))
        rospy.sleep(1)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 150), (2, 635), (3, 100), (4, 260), (5, 500)))
        rospy.sleep(1)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 150), (2, 600), (3, 125), (4, 175), (5, 500)))
        rospy.sleep(1)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 150), (2, 600), (3, 125), (4, 175), (5, 500), (10, 200)))
        rospy.sleep(1)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 540), (3, 220), (4, 50), (5, 500), (10, 200)))
        rospy.sleep(2)
        self.stamp = time.time()
        self.moving = False


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
            sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64) / 2000 * 255
            depth_image = np.where(depth_image > min_dist + 10, 0, depth_image)
            sim_depth_image_sort = np.clip(depth_image, 0, 2000).astype(np.float64) / 2000 * 255
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
            for obj in contours:
                area = cv2.contourArea(obj)
                if area < 500 or self.moving is True:
                    continue
                #cv2.drawContours(depth_color_map, obj, -1, (255, 255, 0), 4) # 绘制轮廓线(draw contour line)
                (cx, cy), radius = cv2.minEnclosingCircle(obj)
                cv2.circle(depth_color_map, (int(cx), int(cy)), int(radius), (0, 0, 255), 2)

                position = depth_pixel_to_camera((cx, cy), depth_image[int(cy), int(cx)] / 1000.0, (K[0], K[4], K[2], K[5]))
                position[2] = position[2] + 0.03
                pose_end = np.matmul(self.hand2cam_tf_matrix, xyz_euler_to_mat(position, (0, 0, 0)))  # 转换的末端相对坐标(relative coordinates of the converted end)
                world_pose = np.matmul(self.endpoint, pose_end)  # 转换到机械臂世界坐标(convert to the robotic arm's world coordinates)
                pose_t, pose_R = mat_to_xyz_euler(world_pose)
                pose_t[1] += 0.01
                if pose_t[2] > z:
                    largest = obj, pose_t
                    min_x = cx
                    min_y = cy
                    txt = 'Dist: {}mm'.format(depth_image[int(cy), int(cx)])

            if largest is not None:
                obj, pose_t = largest
                dist = math.sqrt((self.last_position[0] - pose_t[0]) ** 2 + (self.last_position[1] - pose_t[1])** 2 + (self.last_position[2] - pose_t[2])**2)
                print(dist)
                self.last_position = pose_t
                if dist < 0.002 and pose_t[2] > 0.009:
                    if time.time() - self.stamp > 0.5:
                        self.stamp = time.time()
                        rect = cv2.minAreaRect(obj)
                        self.moving = True
                        threading.Thread(target=self.pick, args=(pose_t, rect[2])).start()
                        txt = 'X:{:.3f} Y:{:.3f} Z:{:.3f}'.format(pose_t[0], pose_t[1], pose_t[2])
                else:
                    self.stamp = time.time()
            self.last_shape = shape
            cv2.circle(depth_color_map, (int(min_x), int(min_y)), 8, (32, 32, 32), -1)
            cv2.circle(depth_color_map, (int(min_x), int(min_y)), 6, (255, 255, 255), -1)
            cv2.putText(depth_color_map, txt, (11, ih-20), cv2.FONT_HERSHEY_PLAIN, 2.0, (32, 32, 32), 6, cv2.LINE_AA)
            cv2.putText(depth_color_map, txt, (10, ih-20), cv2.FONT_HERSHEY_PLAIN, 2.0, (240, 240, 240), 2, cv2.LINE_AA)

            bgr_image = cv2.cvtColor(rgb_image[40:440, ], cv2.COLOR_RGB2BGR)
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
                rospy.signal_shutdown('shutdown1')

        except Exception as e:
            rospy.logerr('callback error:', str(e))

if __name__ == "__main__":
    print('''
    *********************************************
    *                                           *
    * 此程序需要订阅深度摄像头节点,开启前请确保(The depth camera node needs to be subscribed for this program, please ensure that before starting) *
    * 已开启摄像头节点, 通过rostopic list可查看(The camera node has been started, please check via rostopic list) *
    * 是否有usb_cam相关节点,成功运行可看到终端(Whether there are usb_cam related nodes, you can see the terminal if it runs successfully)  *
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


