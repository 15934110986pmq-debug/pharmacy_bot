#!/usr/bin/python3
#coding=utf8

# 通过深度图识别物体的外形进行分类(classify objects based on their shapes recognized through the depth map)
# 机械臂向下识别(the robotic arm recognizes downwards)
# 可以识别长方体，球，圆柱体(it can recognize cuboids, spheres, and cylinders)


import cv2
import rospy
import numpy as np
import message_filters
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import SetBool
from vision_utils import fps
import threading
import queue
from hiwonder_interfaces.msg import MultiRawIdPosDur
from vision_utils import xyz_quat_to_mat, xyz_euler_to_mat, mat_to_xyz_euler
from jetarm_sdk import pid, bus_servo_control, sdk_client, tone
from jetarm_sdk.servo_controller import controller as actiongroup
from jetarm_kinematics import kinematics_control

CONFIG_NAME = "/config"

def depth_pixel_to_camera(pixel_coords, depth, intrinsics):
    fx, fy, cx, cy = intrinsics
    px, py = pixel_coords
    x = (px - cx) * depth / fx
    y = (py - cy) * depth / fy
    z = depth
    return np.array([x, y, z])

class RgbDepthImageNode:
    def __init__(self):
        rospy.init_node('shape_recognition', anonymous=True)
        self.fps = fps.FPS()
        self.last_shape = "none"
        self.moving = False
        self.count = 0
        self.endpoint = None
        self.sdk = sdk_client.JetArmSDKClient()
        config = rospy.get_param(CONFIG_NAME)
        self.hand2cam_tf_matrix = config['hand2cam_tf_matrix']

        self.servos_pub = rospy.Publisher('/controllers/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        rospy.sleep(3)
        rospy.wait_for_service('/kinematics/set_joint_value_target')
        self.goto_default()
        rospy.sleep(2)
        rospy.wait_for_service('/rgbd_cam/set_ldp')
        rospy.ServiceProxy('/rgbd_cam/set_ldp', SetBool)(False)
        
        rgb_sub = message_filters.Subscriber('/rgbd_cam/color/image_raw', RosImage, queue_size=1)
        depth_sub = message_filters.Subscriber('/rgbd_cam/depth/image_raw', RosImage, queue_size=1)
        info_sub = message_filters.Subscriber('/rgbd_cam/depth/camera_info', CameraInfo, queue_size=1)
    
        # 同步时间戳, 时间允许有误差在0.03s(synchronize timestamps, allowing for a time error of up to 0.03 seconds)
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, 0.03)
        sync.registerCallback(self.multi_callback) #执行反馈函数(execute feedback function)
        self.queue = queue.Queue(maxsize=1)

    def goto_default(self):
        endpoint = kinematics_control.set_joint_value_target([500, 540, 220, 50, 500])
        pose_t = endpoint.pose.position
        pose_r = endpoint.pose.orientation
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 540), (3, 220), (4, 50), (5, 500), (10, 200)))
        self.endpoint = xyz_quat_to_mat([pose_t.x, pose_t.y, pose_t.z], [pose_r.w, pose_r.x, pose_r.y, pose_r.z])


    def move(self, shape, pose_t, angle):
        self.sdk.set_buzzer(int(tone.G4), 200, 100, 1) 
        rospy.sleep(0.5)
        pose_t[2] += 0.02
        ret1 = kinematics_control.set_pose_target(pose_t, 85)
        if len(ret1[1]) > 0:
            bus_servo_control.set_servos(self.servos_pub, 1500, ((1, ret1[1][0]), (2, ret1[1][1]), (3, ret1[1][2]), (4, ret1[1][3]),(5, ret1[1][4])))
            rospy.sleep(1.5)
        pose_t[2] -= 0.05
        ret2 = kinematics_control.set_pose_target(pose_t, 85)
        if angle != 0:
            angle = angle % 180
            angle = angle - 180 if angle > 90 else (angle + 180 if angle < -90 else angle)
            angle = 500 + int(1000 * (angle + ret2[3][-1]) / 240)
        else:
            angle = 500
        if len(ret2[1]) > 0:
            bus_servo_control.set_servos(self.servos_pub, 500, ((5, angle),))
            rospy.sleep(0.5)
            bus_servo_control.set_servos(self.servos_pub, 1000, ((1, ret2[1][0]), (2, ret2[1][1]), (3, ret2[1][2]), (4, ret2[1][3]),(5, angle)))
            rospy.sleep(1)
            bus_servo_control.set_servos(self.servos_pub, 600, ((10, 600),))
            rospy.sleep(0.6)
        if len(ret1[1]) > 0:
            bus_servo_control.set_servos(self.servos_pub, 1000, ((1, ret1[1][0]), (2, ret1[1][1]), (3, ret1[1][2]), (4, ret1[1][3]),(5, angle)))
            rospy.sleep(1)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 540), (3, 220), (4, 50), (5, 500), (10, 600)))
        rospy.sleep(1)
        print("shape: ", shape)
        if shape == "sphere":
            actiongroup.runAction("target_1")
        if shape == "cylinder":
            actiongroup.runAction("target_2")
        if shape == "cuboid":
            actiongroup.runAction("target_3")
        self.goto_default()
        rospy.sleep(2)
        self.moving = False

    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if self.queue.empty():
            self.queue.put_nowait((ros_rgb_image, ros_depth_image, depth_camera_info))


    def image_proc(self):
        ros_rgb_image, ros_depth_image, depth_camera_info = self.queue.get(block=True)
        try:
            rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
            depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)

            ih, iw = depth_image.shape[:2]

            depth_image = depth_image.copy()
            # 屏蔽掉一些区域，降低识别条件，使识别跟可靠(mask off certain areas to lower recognition conditions and increase reliability)
            depth_image[:, :100] = np.array([[1000,]*100]* 400)
            depth_image[:, 540:] = np.array([[1000,]*100]* 400)
            depth_image[350:400, :] = np.array([[1000,]*640]* 50)
            depth_image[0:50, :] = np.array([[1000,]*640]* 50)
            depth = np.copy(depth_image).reshape((-1, ))
            depth[depth<=0] = 55555 # 距离为0可能是进入死区，或者颜色问题识别不到，将距离赋一个大值(a distance of 0 could indicate an area of no return or a color recognition issue; assign a large value to distance)
            min_index = np.argmin(depth) # 距离最小的像素(the pixel with the minimum distance)
            min_y = min_index // iw
            min_x = min_index - min_y * iw

            min_dist = depth_image[min_y, min_x] # 获取最小距离值(get the minimum distance measurement)
            sim_depth_image = np.clip(depth_image, 0, 300).astype(np.float64) / 300 * 255
            depth_image = np.where(depth_image > 220, 0, depth_image)
            depth_image = np.where(depth_image > min_dist + 15, 0, depth_image) # 将深度值大于最小距离15mm的像素置0(set the pixels with depth values greater than the minimum distance of 15mm to 0)
            sim_depth_image_sort = np.clip(depth_image, 0, 2000).astype(np.float64) / 2000 * 255 # 
            depth_gray = sim_depth_image_sort.astype(np.uint8)
            depth_gray = cv2.GaussianBlur(depth_gray, (3, 3), 0)
            _, depth_bit = cv2.threshold(depth_gray, 1, 255, cv2.THRESH_BINARY)
            #depth_bit = cv2.erode(depth_bit, np.ones((3, 3), np.uint8))
            #depth_bit = cv2.dilate(depth_bit, np.ones((3, 3), np.uint8))
            depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)

            contours, hierarchy = cv2.findContours(depth_bit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            shape = 'none'
            contour = None
            for obj in contours:
                if min_dist > 225:
                    break
                area = cv2.contourArea(obj)
                if area < 2000 or self.moving is True:
                    continue
                cv2.drawContours(depth_color_map, obj, -1, (255, 255, 0), 4) # 绘制轮廓线(draw contour line)
                perimeter = cv2.arcLength(obj, True)  # 计算轮廓周长(calculate contour perimeter)
                approx = cv2.approxPolyDP(obj, 0.035 * perimeter, True) # 获取轮廓角点坐标(get contour angular point coordinates)
                cv2.drawContours(depth_color_map, approx, -1, (255, 0, 0), 4) # 绘制轮廓线(draw contour line)
                CornerNum = len(approx)

                x, y, w, h = cv2.boundingRect(approx)
                contour_depth = depth_image[y + int(h / 2) - 2: y + int(h / 2) + 2, x: x + w] # 获取轮廓中心一整行的深度数据(retrieve depth data for the entire row at the center of the contour)
                # 我们根据X轴像素的标准差来判断是曲面类物体(圆柱体/球体)还是多面体(立方体、长方体)(we determine whether it's a curved object (cylinder/sphere) or a polyhedron (cube, cuboid) based on the standard deviation of the X-axis pixels)
                # 曲面的标准差较大(the standard deviation for curved surfaces is larger)
                # 因摄像头与桌面有一定角度， 所以Y轴的标准差在平面上也会比较大,不好判断. X轴不存在这个问题(due to the angle between the camera and the table, the standard deviation on the Y-axis will also be relatively large on the plane, making it difficult to determine. This issue does not exist on the X-axis)
                depth  =  np.where(contour_depth == 0, np.nan, contour_depth) # 计算这行数据中有效数据的标准差(calculate the standard deviation of the valid data in this row)
                depth_std = np.nanstd(depth)
                # print(depth_std, CornerNum)
                if depth_std > 1: 
                    if CornerNum > 4:
                        objType = "sphere_1" # 球体(sphere)
                    elif CornerNum == 4:
                        objType = "cylinder_1" # 圆柱体(cylinder)
                    else:
                        objType = "none"
                else:
                    if CornerNum == 4:
                        if min_y > (y + int(h / 10 * 7)):
                            objType="cuboid_1" # 立方体/长方体(cube/cuboid)
                        else:
                            objType="cylinder_2" # 圆柱体(cylinder)
                    else:
                        if  4 < CornerNum < 10:
                            objType="cylinder_2" # 圆柱体(cylinder)
                        else:
                            objType = "none"
                shape=objType
                contour = obj
                if 'cubo' in objType:
                    # 判断是正方体还是长方体(determine whether it is a cube or a cuboid)
                    # 这里取巧，我们的木块正方体的高度都不超过4.0直接判断一下(here we can take a shortcut: our wooden blocks are all cubes with a height not exceeding 4.0. We can directly make a judgment based on this criterion)
                    # 不取巧的方法就是通过最小外接矩形的四个角点及中心点像素坐标(the non-shortcut method involves using the pixel coordinates of the four corner points and the center point of the minimum bounding rectangle)
                    # 调用 depth_pixel_to_camera获取各点在空间中的位置(call depth_pixel_to_camera to get the positions of each point in space)
                    # 然后计算其真实变成及中心点高度(then calculate their actual height and center point height)
                    # 再通过真实值判断其形状(then determine its shape based on the actual values)
                    (cx, cy), r = cv2.minEnclosingCircle(obj)
                    K = depth_camera_info.K
                    position = depth_pixel_to_camera((cx, cy), min_dist / 1000, (K[0], K[4], K[2], K[5]))
                    position[0] -= 0.01
                    pose_end = np.matmul(self.hand2cam_tf_matrix, xyz_euler_to_mat(position, (0, 0, 0)))
                    world_pose = np.matmul(self.endpoint, pose_end)
                    pose_t, pose_r = mat_to_xyz_euler(world_pose)
                    if pose_t[2] > 0.04:  
                        objType = "box_1"  # 长方体(rectangle)
                    else:
                        objType = "cube_1" # 正方体(square)

                cv2.rectangle(depth_color_map, (x, y), (x + w, y + h), (255, 255, 255), 2)
                cv2.putText(depth_color_map, objType[:-2], (x + w // 2, y + (h //2) - 10), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0, 0, 0), 2, cv2.LINE_AA)
                cv2.putText(depth_color_map, objType[:-2], (x + w // 2, y + (h //2) - 10), cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255), 1)
                break
            if self.last_shape == shape and shape != 'none':
                #print(self.count)
                self.count += 1
            else:
                self.count = 0
            if contour is not None :
                (cx, cy), r = cv2.minEnclosingCircle(obj)
                K = depth_camera_info.K
                position = depth_pixel_to_camera((cx, cy), min_dist / 1000, (K[0], K[4], K[2], K[5]))
                position[0] -= 0.01
                pose_end = np.matmul(self.hand2cam_tf_matrix, xyz_euler_to_mat(position, (0, 0, 0)))
                world_pose = np.matmul(self.endpoint, pose_end)
                pose_t, pose_r = mat_to_xyz_euler(world_pose)
                min_x, min_y = cx, cy
               # print(pose_t)
                angle = 0
                offset_x = 0.011
                offset_y = 0.0
                offset_z = 0.00
                pose_t[0] += offset_x
                pose_t[1] += offset_y
                pose_t[2] += offset_z
                if pose_t[1] > 0:
                    pose_t[1] = pose_t[1] * 1.4
                if shape == "cylinder_1" or shape == "cuboid_1":
                    center, (width, height), angle = cv2.minAreaRect(contour)
                    if angle < -45:
                        angle += 90
                    if width > height and width / height > 1.5:
                        print("wh: ", width, height)
                        angle = angle + 90
                    cv2.drawContours(depth_color_map, [np.int0(cv2.boxPoints((center, (width,height), angle)))], -1, (0, 0, 255), 2, cv2.LINE_AA)
                if self.count > 30:
                    self.count = 0
                    self.moving = True
                    threading.Thread(target=self.move, args=(shape[:-2], pose_t, angle)).start()
            self.last_shape = shape

            if not self.moving : 
                txt = 'Dist: {}mm'.format(min_dist)
                cv2.circle(depth_color_map, (int(min_x), int(min_y)), 8, (32, 32, 32), -1)
                cv2.circle(depth_color_map, (int(min_x), int(min_y)), 6, (255, 255, 255), -1)
                cv2.putText(depth_color_map, txt, (11, ih-20), cv2.FONT_HERSHEY_PLAIN, 2.0, (32, 32, 32), 6, cv2.LINE_AA)
                cv2.putText(depth_color_map, txt, (10, ih-20), cv2.FONT_HERSHEY_PLAIN, 2.0, (240, 240, 240), 2, cv2.LINE_AA)

                bgr_image = cv2.cvtColor(rgb_image[40:440, ], cv2.COLOR_RGB2BGR)
                cv2.circle(bgr_image, (int(min_x), int(min_y)), 8, (32, 32, 32), -1)
                cv2.circle(bgr_image, (int(min_x), int(min_y)), 6, (255, 255, 255), -1)
                cv2.putText(bgr_image, txt, (11, ih - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (32, 32, 32), 6, cv2.LINE_AA)
                cv2.putText(bgr_image, txt, (10, ih - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (240, 240, 240), 2, cv2.LINE_AA)
            else:
                pass

            bgr_image = cv2.cvtColor(rgb_image[40:440, ], cv2.COLOR_RGB2BGR)

            self.fps.update()
            #bgr_image = self.fps.show_fps(bgr_image)
            result_image = np.concatenate([depth_color_map, bgr_image], axis=1)
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
    * 此程序需要订阅深度摄像头节点,开启前请确保(before starting this program, make sure to subscribe to the depth camera node) *
    * 已开启摄像头节点, 通过rostopic list可查看(the camera node has been started, you can verify this by using "rostopic list") *
    * 是否有rgbd_cam 相关节点,成功运行可看到终端(is there a rgbd_cam related node? You should see the terminal output upon successful execution)  *
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


