#!/usr/bin/env python3
# coding: utf8

import threading
import rospy
import numpy as np
from math import radians
from utils import unregister
from sensor_msgs.msg import Image as RosImage, CameraInfo
from utils import show_faces, mp_face_location
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from hiwonder_interfaces.srv import SetInt8, SetInt8Request, SetInt8Response
from hiwonder_interfaces.msg import MultiRawIdPosDur
from jetarm_sdk import bus_servo_control
from vision_utils import fps
import color_tracker
import face_tracker
import os
import heart
from jetarm_kinematics.inverse_kinematics import get_ik
from jetarm_kinematics import kinematics_control
import jetarm_kinematics.transform as transform


#os.chdir("/usr/local/lib/python3.6/dist-packages/mediapipe")


class ObjectTrackingNode:
    def __init__(self, node_name, log_level=rospy.INFO):
        rospy.init_node(node_name, anonymous=True, log_level=log_level)

        self.current_pose = None
        self.tracker = None
        self.enable_color_tracking = False
        self.enable_face_tracking = False
        self.fps = fps.FPS()
        self.lock = threading.RLock()
        self.thread = None
        self.color_ranges = rospy.get_param('/config/lab', {})

        # services and topics
        self.image_sub = None
        self.servos_pub = rospy.Publisher('/controllers/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        self.result_image_pub = rospy.Publisher('~image_result', RosImage, queue_size=10)

        self.enter_srv = rospy.Service('~enter', Trigger, self.enter_srv_callback)
        self.exit_srv = rospy.Service('~exit', Trigger, self.exit_srv_callback)
        self.enable_detect_srv = rospy.Service('~enable_color_tracking', SetBool, self.enable_color_srv_callback)
        self.enable_transport_srv = rospy.Service('~enable_face_tracking', SetBool, self.enable_face_srv_callback)
        self.set_target_color_srv = rospy.Service('~set_target_color', SetInt8, self.set_target_color_srv_callback)
        self.heart = heart.Heart('~heartbeat', 5, lambda e: self.exit_srv_callback(e))

    def to_face_tracking_base(self):
        with self.lock:
            bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 700), (3, 85), (4, 350), (5, 500), (10, 200)))
        rospy.sleep(2)
        with self.lock:
            if self.enable_color_tracking:
                self.enable_color_tracking = False
                self.tracker = None
            self.tracker = face_tracker.FaceTracker()
            self.enable_face_tracking = True
            self.thread = None
        rospy.loginfo("已开启人脸追踪")

    def to_color_tracking_base(self):
        with self.lock:
            p = kinematics_control.set_pose_target((0.15, 0, 0.28), 0)
            if len(p[1]) > 0:
                self.current_servo_positions = p[1]
                bus_servo_control.set_servos(self.servos_pub, 1000, ((1, p[1][0]), (2, p[1][1]), (3, p[1][2]), (4, p[1][3]), (5, p[1][4])))
            #bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 700), (3, 85), (4, 350), (5, 500), (10, 200)))
        rospy.sleep(2)
        with self.lock:
            if self.enable_face_tracking:
                self.enable_face_tracking = False
                self.tracker = None
            self.enable_color_tracking = True
            self.thread = None
        rospy.loginfo("已开启颜色追踪")

    def enable_color_srv_callback(self, req: SetBoolRequest):
        with self.lock:
            rospy.loginfo("Enable Color Tacking: " + str(req.data))
            if req.data:
                if self.thread is None:
                    rospy.loginfo("正在开启颜色追踪...")
                    self.thread = threading.Thread(target=self.to_color_tracking_base)
                    self.thread.start()
                    return SetBoolResponse(success=True)
                else:
                    msg = "Enable Color Tracking, 有其他操作正在进行, 请稍后重试"
                    rospy.logerr(msg)
                    return SetBoolResponse(success=False, message=msg)
            else:
                rospy.loginfo("正在关闭颜色追踪...")
                if self.enable_color_tracking:
                    self.enable_color_tracking = False
                    self.tracker = None
                return SetBoolResponse(success=True)

    def enable_face_srv_callback(self, req: SetBoolRequest):
        with self.lock:
            if self.thread is None:
                if req.data:
                    rospy.loginfo("正在开启人脸追踪...")
                    self.thread = threading.Thread(target=self.to_face_tracking_base)
                    self.thread.start()
                else:
                    rospy.loginfo("正在关闭人脸追踪...")
                    if self.enable_face_tracking:
                        self.enable_face_tracking = False
                        self.tracker = None
                return SetBoolResponse(success=True)
            else:
                msg = "Enable Face Tracking, 有其他操作正在进行, 请稍后重试"
                rospy.logerr(msg)
                return SetBoolResponse(success=False, message=msg)

    def set_target_color_srv_callback(self, req: SetInt8Request):
        with self.lock:
            if req.data != 0:
                colors = ["空", "red", "green", "blue"]
                rospy.loginfo("正在设置将要追踪的目标认识为" + colors[req.data])
                self.tracker = color_tracker.ColorTracker(colors[req.data])
                if self.enable_face_tracking:
                    self.enable_face_tracking = False
                    self.tracker = None
                rospy.loginfo("目标颜色设置成功")
            else:
                self.tracker = None
            return SetInt8Response(success=True)

    def enter_srv_callback(self, _: TriggerRequest):
        # 获取和发布图像的topic(get and publish the topic of image)
        rospy.loginfo("加载玩法")
        with self.lock:
            self.source_image_topic = rospy.get_param('~source_image_topic', '/camera/image_raw')
            unregister(self.image_sub)
            self.enable_face_tracking = False
            self.enable_color_tracking = False
            self.image_sub = rospy.Subscriber(self.source_image_topic, RosImage, self.image_callback, queue_size=1)
        return TriggerResponse(success=True)

    def exit_srv_callback(self, _: TriggerRequest):
        rospy.loginfo("卸载玩法")
        with self.lock:
            unregister(self.image_sub)
            self.heart.reset()
            self.enable_face_tracking = False
            self.enable_color_tracking = False
            self.tracker = None
        return TriggerResponse(success=True)

    def image_callback(self, ros_image: RosImage):
        # rospy.logdebug('Received an image! ')
        # 将ros格式图像转换为opencv格式(convert the ros format image to opencv format)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        result_image = np.copy(rgb_image)

        with self.lock:
            if self.thread is None and self.tracker is not None:
                if self.tracker.tracker_type == 'color' and self.enable_color_tracking:
                    color_ranges = rospy.get_param('/config/lab', self.color_ranges)
                    result_image, p_y = self.tracker.proc(rgb_image, result_image, color_ranges)
                    if p_y is not None:
                        p = self.set_pose_target((0.15, 0, p_y[0]), 0)
                        if len(p[1]) > 0:
                            p[1][0] = p_y[1]
                            self.current_servo_positions = p[1]
                            bus_servo_control.set_servos(self.servos_pub, 20, ((1, p[1][0]), (2, p[1][1]), (3, p[1][2]), (4, p[1][3]), (5, p[1][4])))
                    """
                    if new_pose is not None:
                        pose = [self.current_pose[0][0] + new_pose[0], self.current_pose[0][1] + new_pose[1], self.current_pose[0][2]]
                        try:
                            solve = self.kine.ikine_euler(pose, self.current_pose[1])
                            self.joint.set_joints(solve[1], 0.03)
                        except Exception as e:
                            rospy.logerr(str(e))
                    """
                elif self.tracker.tracker_type == 'face' and self.enable_face_tracking:
                    result_image, p_y = self.tracker.proc(rgb_image, result_image)
                    if p_y is not None:
                        bus_servo_control.set_servos(self.servos_pub, 30, ((1, p_y[1]), (4, p_y[0])))

        # 计算帧率及发布结果图像(calculate the frame rate and publish the resulting image)
        # self.fps.update()
        # result_image = self.fps.show_fps(result_image)
        ros_image.data = result_image.tobytes()
        self.result_image_pub.publish(ros_image)

    def set_pose_target(self, position, pitch):
        # 逆运动学解，获取最优解(所有电机转动最小)(inverse kinematics solution, get the optimal solution (minimizing the rotation of all motors))
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
                # print(optimal_solution)
            return [True, optimal_solution, self.current_servo_positions, rpy, min_d]
        else:
            return [True, [], [], [], 0]



if __name__ == "__main__":
    node = ObjectTrackingNode("object_tracking", log_level=rospy.INFO)
    rospy.spin()
