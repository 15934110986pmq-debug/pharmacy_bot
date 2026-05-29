#!/usr/bin/env python3
# coding: utf8

import os
import cv2
import rospy
import numpy as np
import threading
import queue
import time
import math
from utils import unregister
from sensor_msgs.msg import Image as RosImage
import mediapipe as mp
from vision_utils import fps, distance, vector_2d_angle, get_area_max_contour
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
import enum
import heart
import actions
from vision_utils import point_remapped
from jetarm_sdk import bus_servo_control, sdk_client, tone
from jetarm_kinematics import kinematics_control
from hiwonder_interfaces.msg import  MultiRawIdPosDur

os.chdir("/usr/local/lib/python3.6/dist-packages/mediapipe")


class State(enum.Enum):
    NULL = 0
    TRACKING = 1
    RUNNING = 2


def get_hand_landmarks(img, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标(convert landmarks from the normalized output of mediapipe to pixel coordinates)
    :param img: 像素坐标对应的图片(pixel coordinates corresponding image)
    :param landmarks: 归一化的关键点(normalized key points)
    :return:
    """
    h, w, _ = img.shape
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)


def hand_angle(landmarks):
    """
    计算各个手指的弯曲角度(calculate the blending angle of each finger)
    :param landmarks: 手部关键点(hand key point)
    :return: 各个手指的角度(the angle of each finger)
    """
    angle_list = []
    # thumb 大拇指
    angle_ = vector_2d_angle(landmarks[3] - landmarks[4], landmarks[0] - landmarks[2])
    angle_list.append(angle_)
    # index 食指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[6], landmarks[7] - landmarks[8])
    angle_list.append(angle_)
    # middle 中指
    angle_ = vector_2d_angle(
        landmarks[0] - landmarks[10], landmarks[11] - landmarks[12]
    )
    angle_list.append(angle_)
    # ring 无名指
    angle_ = vector_2d_angle(
        landmarks[0] - landmarks[14], landmarks[15] - landmarks[16]
    )
    angle_list.append(angle_)
    # pink 小拇指
    angle_ = vector_2d_angle(
        landmarks[0] - landmarks[18], landmarks[19] - landmarks[20]
    )
    angle_list.append(angle_)
    angle_list = [abs(a) for a in angle_list]
    return angle_list


def h_gesture(angle_list):
    """
    通过二维特征确定手指所摆出的手势(determine the gesture formed by the fingers through two-dimensional features)
    :param angle_list: 各个手指弯曲的角度(the blending angle of each finger)
    :return : 手势名称字符串(gesture name string)
    """
    thr_angle, thr_angle_thumb, thr_angle_s = (65.0, 53.0, 49.0)
    if (
        (angle_list[0] < thr_angle_s)
        and (angle_list[1] < thr_angle_s)
        and (angle_list[2] < thr_angle_s)
        and (angle_list[3] < thr_angle_s)
        and (angle_list[4] < thr_angle_s)
    ):
        gesture_str = "five"
    elif (
        (angle_list[0] > 5)
        and (angle_list[1] < thr_angle_s)
        and (angle_list[2] > thr_angle)
        and (angle_list[3] > thr_angle)
        and (angle_list[4] > thr_angle)
    ):
        gesture_str = "one"
    else:
        gesture_str = "none"
    return gesture_str


def draw_points(img, points, tickness=4, color=(255, 0, 0)):
    """
    将记录的点连线画在画面上(draw lines connecting the recorded points on the screen)
    """
    points = np.array(points).astype(dtype=np.int)
    if len(points) > 2:
        for i, p in enumerate(points):
            if i + 1 >= len(points):
                break
            cv2.line(img, p, points[i + 1], color, tickness)


def get_track_img(points):
    """
    用记录的点生成一张黑底白线的轨迹图(generate a trajectory image with a black background and white lines using the recorded points)
    """
    points = np.array(points).astype(dtype=np.int)
    x_min, y_min = np.min(points, axis=0).tolist()
    x_max, y_max = np.max(points, axis=0).tolist()
    track_img = np.full(
        [y_max - y_min + 100, x_max - x_min + 100, 1], 0, dtype=np.uint8
    )
    points = points - [x_min, y_min]
    points = points + [50, 50]
    draw_points(track_img, points, 1, (255, 255, 255))
    return track_img


class FingerTraceNode:
    def __init__(self, node_name, log_level=rospy.INFO):
        rospy.init_node(node_name, anonymous=False, log_level=log_level)

        self.fps = fps.FPS()
        self.entered = False
        self.enable = False
        self.points = []
        self.start_count = 0
        self.sdk = sdk_client.JetArmSDKClient()
        self.current_point = None
        self.lock = threading.RLock()
        self.drawing = mp.solutions.drawing_utils
        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            # model_complexity=0,
            min_tracking_confidence=0.05,
            min_detection_confidence=0.6,
        )

        self.image_queue = queue.Queue(maxsize=2)
        self.state = State.NULL
        self.no_finger_timestamp = time.time()
        self.servos_pub = rospy.Publisher("/controllers/multi_id_pos_dur", MultiRawIdPosDur, queue_size=1)

        # services and topics
        self.image_sub = None
        self.result_image_pub = rospy.Publisher("~image_result", RosImage, queue_size=2)
        self.enter_srv = rospy.Service("~enter", Trigger, self.enter_srv_callback)
        self.exit_srv = rospy.Service("~exit", Trigger, self.exit_srv_callback)
        self.enable_stack_up_srv = rospy.Service("~enable_drawing", SetBool, self.enable_trace_srv_callback)
        self.enable_stack_up_srv = rospy.Service("~enable_trace", SetBool, self.enable_trace_srv_callback)
        self.heart = heart.Heart("~heartbeat", 5, lambda e: self.exit_srv_callback(e))

    def to_finger_trace_base(self):
        with self.lock:
            bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 700), (3, 85), (4, 350), (5, 500), (10, 200)))

    def do_re_mi(self):
        self.sdk.set_buzzer(round(tone.C5), 150, 50, 1)
        rospy.sleep(0.15)
        self.sdk.set_buzzer(round(tone.D5), 150, 50, 1)
        rospy.sleep(0.15)
        self.sdk.set_buzzer(round(tone.E5), 150, 50, 1)
        rospy.sleep(0.15)

    def di_di(self):
        self.sdk.set_buzzer(round(tone.E5), 150, 50, 1)
        rospy.sleep(0.15)
        self.sdk.set_buzzer(round(tone.E5), 150, 50, 1)
        rospy.sleep(0.15)


    def enter_srv_callback(self, _: TriggerRequest):
        # 获取和发布图像的topic(obtain and publish the topic of image)
        if self.entered:
            return TriggerResponse(success=True)
        self.entered = True
        rospy.loginfo("加载玩法")
        source_image_topic = rospy.get_param("~source_image_topic", "/rgbd_cam/color/image_raw")
        unregister(self.image_sub)
        threading.Thread(target=self.to_finger_trace_base).start()
        self.image_sub = rospy.Subscriber(source_image_topic, RosImage, self.image_callback, queue_size=1)
        return TriggerResponse(success=True)

    def exit_srv_callback(self, _: TriggerRequest):
        unregister(self.image_sub)
        self.heart.reset()
        return TriggerResponse(success=True)

    def enable_trace_srv_callback(self, req: SetBoolRequest):
        with self.lock:
            if req.data:
                self.enable = True
                rospy.loginfo("开始模仿")
            else:
                self.enable = False
                rospy.loginfo("关闭模仿")
        return SetBoolResponse(success=True)

    def image_callback(self, ros_image: RosImage):
        if self.image_queue.empty():
            self.image_queue.put_nowait(ros_image)

    def arm_draw_points(self, points):
        bus_servo_control.set_servos(self.servos_pub, 300, ((10, 700), ))
        step = 0
        last_p = None
        for p in points:
            y, z = point_remapped(p, (640, 480), (0.333, 0.25))
            x = 0.2
            #print(x, y, z)
            ret = kinematics_control.set_pose_target((x, y - 0.27 / 2, 0.3 - z), 0)
            self.current_point = p
            if len(ret[1]) > 0:
                if last_p is None:  # 第一个点(the first point)
                    bus_servo_control.set_servos(self.servos_pub, 1500, ((1, ret[1][0]), (2, ret[1][1]), (3, ret[1][2]), (4, ret[1][3]), (5, ret[1][4])))
                    rospy.sleep(1.5)
                else:
                    dist = math.sqrt((last_p[0] - x) ** 2 + (last_p[1] - y) ** 2 + (last_p[2] - z) ** 2)
                    t = dist / 0.1  # 0.1米每秒(0.1 meters per second)
                    bus_servo_control.set_servos(self.servos_pub, int(t * 1000), ((1, ret[1][0]), (2, ret[1][1]), (3, ret[1][2]), (4, ret[1][3]), (5, ret[1][4])))
                    rospy.sleep(t)
                last_p = (x, y, z)
                step += 1
            else:
                print("CAN NOT ATTA")
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 700), (3, 85), (4, 350), (5, 500), (10, 200)))
        self.sdk.set_buzzer(round(tone.G5), 150, 50, 1)
        self.current_point = None
        self.state = State.NULL

    def image_proc(self):
        # rospy.logdebug('Received an image! ')
        # 将ros格式图像转换为opencv格式(convert the ros format image to opencv format)
        ros_image = self.image_queue.get(block=True)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        rgb_image = cv2.flip(rgb_image, 1)
        result_image = np.copy(rgb_image)

        try:
            results = self.hand_detector.process(rgb_image)
            if results is not None and results.multi_hand_landmarks:
                gesture = "none"
                index_finger_tip = [0, 0]
                self.no_finger_timestamp = time.time()  # 记下当期时间，以便超时处理(note the current time for timeout handling)
                for hand_landmarks in results.multi_hand_landmarks:
                    self.drawing.draw_landmarks(
                        result_image,
                        hand_landmarks,
                        mp.solutions.hands.HAND_CONNECTIONS,
                    )

                    landmarks = get_hand_landmarks(rgb_image, hand_landmarks.landmark)
                    angle_list = hand_angle(landmarks)
                    gesture = h_gesture(angle_list)
                    index_finger_tip = landmarks[8].tolist()

                if self.state == State.NULL and self.enable:
                    self.points = []
                    if gesture == "one":  # 检测到单独伸出食指，其他手指握拳(detect the index finger extended while the other fingers are clenched into a fist)
                        self.start_count += 1
                    else:
                        self.start_count = 0
                    if self.start_count > 20:
                        self.state = State.TRACKING
                        threading.Thread(target=self.do_re_mi).start()
                        self.points = []

                elif self.state == State.TRACKING:
                    if gesture == "five":  # 伸开五指结束画图(finish drawing when all five fingers are spread out)
                        self.state = State.RUNNING
                        threading.Thread(target=self.di_di).start()
                        threading.Thread(target=self.arm_draw_points, args=(self.points,)).start()
                        """
                        # 生成黑白轨迹图(generate a black and white trajectory image)
                        track_img = get_track_img(self.points)
                        contours = cv2.findContours(track_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                        contour, area = get_area_max_contour(contours, 300)

                        # 按轨迹图识别所画图形(identify the drawn shape based on the trajectory image)
                        track_img = cv2.fillPoly(track_img, [contour, ], (255, 255, 255), )
                        for _ in range(3):
                            track_img = cv2.erode(track_img,cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),)
                            track_img = cv2.dilate(track_img,cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),)
                        contours = cv2.findContours(track_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                        contour, area = get_area_max_contour(contours, 300)
                        h, w = track_img.shape[:2]
                        track_img = np.full([h, w, 3], 0, dtype=np.uint8)
                        track_img = cv2.drawContours(track_img,[contour,],-1,(0, 255, 0),2,)
                        approx = cv2.approxPolyDP(contour, 0.026 * cv2.arcLength(contour, True), True)
                        track_img = cv2.drawContours(track_img,[approx,],-1,(0, 0, 255),2,)
                        print(len(approx))
                        cv2.imshow("track", track_img)
                        cv2.waitKey(1)
                        """

                    else:
                        if len(self.points) > 0:
                            if distance(self.points[-1], index_finger_tip) > 5:
                                self.points.append(index_finger_tip)
                        else:
                            self.points.append(index_finger_tip)
                    draw_points(result_image, self.points)
                else:
                    pass
            else:
                if (self.state == State.TRACKING and time.time() - self.no_finger_timestamp > 2):
                    self.state = State.NULL
            if len(self.points) > 0:
                draw_points(result_image, self.points)
                if self.current_point is not None:
                    x, y = self.current_point
                    cv2.circle(result_image, (int(x), int(y)), 8, (0, 255, 0), -1)

        except Exception as e:
            rospy.logerr(str(e))

        # 计算帧率及发布结果图像(calculate frame rate and publish result image)
        self.fps.update()
        result_image = self.fps.show_fps(result_image)
        ros_image.data = result_image.tostring()
        self.result_image_pub.publish(ros_image)


if __name__ == "__main__":
    node = FingerTraceNode("finger_trace", log_level=rospy.INFO)
    while not rospy.is_shutdown():
        try:
            node.image_proc()
        except Exception as e:
            rospy.logerr(str(e))
