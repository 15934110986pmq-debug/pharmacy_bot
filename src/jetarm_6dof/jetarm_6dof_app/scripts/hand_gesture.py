#!/usr/bin/env python3
# coding: utf8

import os
import sys
import math
import queue
import enum
import rospy
import actionlib
import cv2
import time
import heart
import numpy as np
import threading
import mediapipe as mp
import actions
from utils import unregister
from jetarm_kinematics import kinematics_control
from jetarm_sdk import bus_servo_control, sdk_client, tone
from jetarm_sdk.servo_controller import actiongroup
from vision_utils import fps, distance, vector_2d_angle, get_area_max_contour,  point_remapped
from sensor_msgs.msg import Image as RosImage
from hiwonder_interfaces.msg import Grasp, MoveAction, MoveGoal, MultiRawIdPosDur
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse


POSITIONS_PATH='/positions'


def get_hand_landmarks(img_size, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标(convert landmarks from mediapipe's normalized output to pixel coordinates)
    :param img: 像素坐标对应的图片(the image corresponding to the pixel coordinates)
    :param landmarks: 归一化的关键点(normalized key points)
    :return:
    """
    w, h = img_size
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)


def hand_angle(landmarks):
    """
    计算各个手指的弯曲角度(calculate the bending angle of each finger)
    :param landmarks: 手部关键点(hand key points)
    :return: 各个手指的角度(each finger angle)
    """
    angle_list = []
    # thumb 大拇指
    angle_ = vector_2d_angle(landmarks[3] - landmarks[4], landmarks[0] - landmarks[2])
    angle_list.append(angle_)
    # index 食指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[6], landmarks[7] - landmarks[8])
    angle_list.append(angle_)
    # middle 中指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[10], landmarks[11] - landmarks[12])
    angle_list.append(angle_)
    # ring 无名指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[14], landmarks[15] - landmarks[16])
    angle_list.append(angle_)
    # pink 小拇指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[18], landmarks[19] - landmarks[20])
    angle_list.append(angle_)
    angle_list = [abs(a) for a in angle_list]
    return angle_list


def h_gesture(angle_list):
    """
    通过二维特征确定手指所摆出的手势(determine the gesture formed by the fingers through two-dimensional features)
    :param angle_list: 各个手指弯曲的角度(the bending angle of each finger)
    :return : 手势名称字符串(gesture name string)
    """
    thr_angle, thr_angle_thumb, thr_angle_s = 65.0, 53.0, 49.0
    if (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "fist"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "gun"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "hand_heart"
    elif (angle_list[0] > 5) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "one"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "two"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] > thr_angle):
        gesture_str = "three"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "OK"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "four"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "five"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
        gesture_str = "six"
    else:
        gesture_str = "none"
    return gesture_str


class State(enum.Enum):
    NULL = 0
    TRACKING = 1
    RUNNING = 2


def draw_points(img, points, tickness=4, color=(255, 0, 0)):
    points = np.array(points).astype(dtype=np.int32)
    if len(points) > 2:
        for i, p in enumerate(points):
            if i + 1 >= len(points):
                break
            cv2.line(img, p, points[i + 1], color, tickness)


class HandGestureNode:
    def __init__(self, node_name, log_level=rospy.DEBUG):
        rospy.init_node(node_name, anonymous=False, log_level=log_level)
        self.drawing = mp.solutions.drawing_utils
        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            #model_complexity=0,
            min_tracking_confidence=0.1,
            min_detection_confidence=0.6
        )

        self.fps = fps.FPS()  # fps计算器(fps calculator)
        self.entered = False
        self.enable = False
        self.state = State.NULL
        self.points = [[0, 0], ]
        self.one_count = 0
        self.count = 0
        self.direction = ""
        self.last_gesture = "none"
        self.lock = threading.RLock()
        self.no_finger_timestamp = time.time()
        self.sdk = sdk_client.JetArmSDKClient()

        self.servos_pub = rospy.Publisher('/controllers/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        self.action_client = actionlib.SimpleActionClient('/grasp', MoveAction)

        # services and topics
        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = None
        self.result_image_pub = rospy.Publisher("~image_result", RosImage, queue_size=2)
        self.enter_srv = rospy.Service("~enter", Trigger, self.enter_srv_callback)
        self.exit_srv = rospy.Service("~exit", Trigger, self.exit_srv_callback)
        self.enable_stack_up_srv = rospy.Service("~enable_drawing", SetBool, self.enable_srv_callback)
        self.enable_stack_up_srv = rospy.Service("~enable_trace", SetBool, self.enable_srv_callback)
        self.heart = heart.Heart("~heartbeat", 5, lambda e: self.exit_srv_callback(e))

        rospy.loginfo("hand gesture node created")

    def enter_srv_callback(self, _: TriggerRequest):
        # 获取和发布图像的topic(get and publish the topic of image)
        if self.entered:
            return TriggerResponse(success=True)
        self.entered = True
        rospy.loginfo("加载玩法")
        unregister(self.image_sub)
        source_image_topic = rospy.get_param("~source_image_topic", "/rgbd_cam/color/image_raw")
        threading.Thread(target=self.goto_default).start()
        self.image_sub = rospy.Subscriber(source_image_topic, RosImage, self.image_callback, queue_size=1)
        return TriggerResponse(success=True)

    def exit_srv_callback(self, _: TriggerRequest):
        unregister(self.image_sub)
        self.entered = False
        self.heart.reset()
        return TriggerResponse(success=True)

    def goto_default(self):
        with self.lock:
            bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 700), (3, 85), (4, 350), (5, 500), (10, 200)))

    def enable_srv_callback(self, req: SetBoolRequest):
        with self.lock:
            if req.data:
                self.enable = True
                rospy.loginfo("开始模仿")
            else:
                self.enable = False
                rospy.loginfo("关闭模仿")
        return SetBoolResponse(success=True)

    def done_callback(self, state, result): #动作执行完毕回调(callback function when the action is completed)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 700), (3, 85), (4, 350), (5, 500), (10, 200)))
        rospy.sleep(1)
        self.count = 0
        self.last_gesture = "none"
        self.state = State.NULL
    
    def active_callback(self): # 运动开始回调(callback function when the motion starts)
        self.start_move = True
        rospy.loginfo("start move")
    
    def feedback_callback(self, msg): # 动作执行进度回调(progress callback during action execution)
        rospy.loginfo("finish action: {:.2%}".format(msg.percent))
        self.finish_percent = msg.percent;

    def do_re_mi(self):
        self.sdk.set_buzzer(round(tone.C5), 150, 50, 1)
        rospy.sleep(0.15)
        self.sdk.set_buzzer(round(tone.D5), 150, 50, 1)
        rospy.sleep(0.15)
        self.sdk.set_buzzer(round(tone.E5), 150, 50, 1)
        rospy.sleep(0.15)


    def do_act(self, gesture):
        print(gesture)
        if gesture == 'one' or gesture == 'two' or gesture == 'three':
            # rospy.sleep(2)
            bus_servo_control.set_servos(self.servos_pub, 1000, ((10, 650),))
            rospy.sleep(1.5)
            goal = MoveGoal()
            goal.grasp.mode = 'place'
            params = rospy.get_param(os.path.join(POSITIONS_PATH, 'tag_sortting'))
            target_name = 'target_1'
            if gesture == 'two':
                target_name = 'target_2'
            if gesture == 'three':
                target_name = 'target_3'
            target = params[target_name]
            goal.grasp.position.x = target[0]
            goal.grasp.position.y = target[1]
            goal.grasp.position.z = target[2]
            goal.grasp.pitch = 75
            goal.grasp.align_angle = -90 #yaw #- 20/1000* 240
            goal.grasp.grasp_approach.z = 0.04 # 放置时靠近的方向和距离(direction and distance of proximity during placement)
            goal.grasp.grasp_retreat.z = 0.04 # 放置后后撤的方向和距离(direction and distance of retreat after placement)
            goal.grasp.pre_grasp_posture = 650
            goal.grasp.grasp_posture = 400  # 夹取前后夹持器的开合角度(the opening and closing angle of the grippers before and after gripping)
            self.action_client.send_goal(goal, self.done_callback, self.active_callback, self.feedback_callback)
        else:
            actiongroup.runAction(gesture)
            rospy.sleep(1)
            self.count = 0
            self.last_gesture = "none"
            self.state = State.NULL


    def image_callback(self, ros_image: RosImage):
        #rospy.loginfo('Received an image! ')
        if self.image_queue.empty():
            self.image_queue.put_nowait(ros_image)

    def image_proc(self):
        # 将ros格式图像转换为opencv格式(convert the ros format image to opencv format)
        ros_image = self.image_queue.get(block=True)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面(original RGB image)
        rgb_image = cv2.flip(rgb_image, 1) # 镜像画面(mirrored image)
        result_image = np.copy(rgb_image) # 拷贝一份用作结果显示，以防处理过程中修改了图像(make a copy for result display to prevent modification of the image during processing)

        if time.time() - self.no_finger_timestamp > 2:
            self.direction = ""
        else:
            if self.direction != "":
                cv2.putText(result_image, self.direction, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        try:
            results = self.hand_detector.process(rgb_image) # 手部、关键点识别(hand and key points recognition)
            if results.multi_hand_landmarks:
                gesture = "none"
                index_finger_tip = [0, 0]
                for hand_landmarks in results.multi_hand_landmarks: 
                    self.drawing.draw_landmarks(
                        result_image,
                        hand_landmarks,
                        mp.solutions.hands.HAND_CONNECTIONS)
                    h, w = rgb_image.shape[:2]
                    landmarks = get_hand_landmarks((w, h), hand_landmarks.landmark)
                    angle_list = hand_angle(landmarks)
                    gesture = h_gesture(angle_list) # 根据关键点位置判断手势(judge gesture based on key points position)
                    index_finger_tip = landmarks[8].tolist()

                cv2.putText(result_image, gesture.upper(), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 5)
                cv2.putText(result_image, gesture.upper(), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 0), 2)
                draw_points(result_image, self.points[1:])
                if self.state != State.RUNNING and self.enable:
                    if gesture == self.last_gesture and gesture != "none":
                        self.count += 1
                    else:
                        self.count = 0
                    if self.count > 20:
                        self.state = State.RUNNING
                        threading.Thread(target=self.do_re_mi).start()
                        threading.Thread(target=self.do_act, args=(gesture, )).start()
                else:
                    self.count = 0
                self.last_gesture = gesture

            else:
                if self.state != State.NULL:
                    if time.time() - self.no_finger_timestamp > 2:
                        self.one_count = 0
                        self.points = [[0, 0],]
                        self.state = State.NULL
        except Exception as e:
            rospy.logerr(str(e))

        self.fps.update()
        result_image = self.fps.show_fps(result_image)
        ros_image.data = result_image.tobytes()
        self.result_image_pub.publish(ros_image)
        #result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        #cv2.imshow('image', result_image)
        #cv2.waitKey(1)


if __name__ == "__main__":
    node = HandGestureNode("finger_trace", log_level=rospy.INFO)
    while not rospy.is_shutdown():
        try:
            node.image_proc()
        except Exception as e:
            rospy.logerr(str(e))

