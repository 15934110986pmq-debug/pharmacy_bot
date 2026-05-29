#!/usr/bin/env python3
# coding: utf8

import os
import math
import rospy
import numpy as np
import threading
import cv2
from utils import unregister
from vision_utils import fps, xyz_quat_to_mat, xyz_euler_to_mat, xyz_rot_to_mat, mat_to_xyz_euler, draw_tags, distance
from sensor_msgs.msg import Image as RosImage, CameraInfo
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from hiwonder_interfaces.srv import GetRobotPose
from hiwonder_interfaces.msg import Grasp, MoveAction, MoveGoal, MultiRawIdPosDur
import actions
from dt_apriltags import Detector
from jetarm_sdk import bus_servo_control
import actionlib
import heart
from actionlib_msgs.msg import GoalID


POSITIONS_PATH = '/positions'

class TagStackup():
    def __init__(self, node_name, log_level=rospy.INFO):
        rospy.init_node(node_name, anonymous=True, log_level=log_level)
        self.tag_size = 0.025
        self.endpoint = None
        self.hand2cam_tf_matrix = rospy.get_param('/config/hand2cam_tf_matrix')
        self.tag_size = rospy.get_param("/config/tag_size", 0.025)

        self.stop_thread = False
        self.enable_stackup = False
        self.last_position = None
        self.stackup_step = 99
        self.status = 1
        self.target_position = None
        self.target_count = 0
        self.finish_percent = 0
        self.pick_pitch = 80
        self.entered = False
        self.err_msg = None
        self.thread = None

        self.K = None
        self.D = None

        self.lock = threading.RLock()
        self.fps = fps.FPS()  # 帧率统计器(frame rate counter)

        self.at_detector = Detector(searchpath=['apriltags'], 
                                    families='tag36h11',
                                    nthreads=4,
                                    quad_decimate=1.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)

        # services and topics
        self.image_sub = None
        self.camera_info_sub = None
        self.joints_pub = rospy.Publisher('/controllers/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        self.action_client = actionlib.SimpleActionClient('/grasp', MoveAction)
        self.result_image_pub = rospy.Publisher('~image_result', RosImage, queue_size=10)
        self.cancel_pub = rospy.Publisher('/grasp/cancel', GoalID, queue_size=10)
        self.goal_id = GoalID()

        self.enter_srv = rospy.Service('~enter', Trigger, self.enter_srv_callback)
        self.exit_srv =  rospy.Service('~exit', Trigger, self.exit_srv_callback)
        self.enable_stack_up_srv = rospy.Service('~enable_stackup', SetBool, self.enable_stackup_srv_callback)
        self.heart = heart.Heart('~heartbeat', 5, lambda e: self.exit_srv_callback(e))

    def get_endpoint(self):
        endpoint = rospy.ServiceProxy('/kinematics/get_current_pose', GetRobotPose)().pose
        self.endpoint = xyz_quat_to_mat([endpoint.position.x, endpoint.position.y,  endpoint.position.z],
                                        [endpoint.orientation.w, endpoint.orientation.x, endpoint.orientation.y, endpoint.orientation.z])

    def enter_srv_callback(self, _: TriggerRequest):
        # 获取和发布图像的topic(get and publish the topic of image)
        rospy.loginfo("加载玩法")
        with self.lock:
            if self.entered:
                return TriggerResponse(success=True)
            self.entered = True
            source_image_topic = rospy.get_param('~source_image_topic', '/camera/image_raw')
            camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/camera_info')
            self.enable_stackup = False
            unregister(self.image_sub)
            unregister(self.camera_info_sub)
            self.image_sub = rospy.Subscriber(source_image_topic, RosImage, self.image_callback, queue_size=1)
            self.camera_info_sub = rospy.Subscriber(camera_info_topic, CameraInfo, self.camera_info_callback)
            # self.hand2cam_tf_matrix = rospy.get_param('/config/hand2cam_tf_matrix')
            # self.get_endpoint()
            # self.stackup_step = 99
            # threading.Thread(target=self.start_set_target).start()
            return TriggerResponse(success=True)

    def camera_info_callback(self, msg):
        with self.lock:
            K = np.matrix(msg.K).reshape(1, -1, 3)
            D = np.array(msg.D)
            new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (640, 480), 0, (640, 480))
            self.K, self.D = new_K, np.zeros((5, 1))


    def exit_srv_callback(self, _: TriggerRequest):
        rospy.loginfo("卸载玩法")
        with self.lock:
            self.entered = False
            self.enable_stackup = False
            self.cancel_pub.publish(self.goal_id)
            unregister(self.image_sub)
            unregister(self.camera_info_sub)
            self.heart.reset()
            return TriggerResponse(success=True)

    def enable_stackup_srv_callback(self, req: SetBoolRequest):
        with self.lock:
            if req.data:
                self.enable_stackup = True
                self.target_count = 0
                self.last_position = None
                rospy.loginfo("开始搬运")
                self.hand2cam_tf_matrix = rospy.get_param('/config/hand2cam_tf_matrix')
                self.get_endpoint()
                self.stackup_step = 99
                threading.Thread(target=self.start_set_target).start()

            else:
                self.enable_stackup = False
                self.cancel_pub.publish(self.goal_id)
                rospy.loginfo("Sent cancel request")
                rospy.loginfo("关闭搬运")
        return SetBoolResponse(success=True)

    def start_set_target(self):
        actions.goto_left(self.joints_pub)
        self.get_endpoint()
        rospy.sleep(0.5)
        self.stackup_step = 10

    def set_target(self):
        actions.goto_home(self.joints_pub)
        self.get_endpoint()
        self.stackup_step = 0

    def done_callback(self, state, result):  # 动作执行完毕回调(callback function when the action is completed)
        rospy.loginfo("state:%f", state)
        if not result.result.complete:  # 如果在移动中被取消，需要回到初始位置(if canceled while in motion, return to the initial position)
            bus_servo_control.set_servos(self.joints_pub, 800, ( (2, 560), (3, 130), (4, 115), (5, 500), (10, 200)))
            rospy.sleep(1)
            bus_servo_control.set_servos(self.joints_pub, 800, ((1, 500), ))
            rospy.sleep(0.8)
            bus_servo_control.set_servos(self.joints_pub, 500, ((10, 200), ))
            rospy.sleep(0.5)
        elif self.stackup_step != 1:
            bus_servo_control.set_servos(self.joints_pub, 500, ((10, 200), ))

        if self.finish_percent == 1:  # 如果完整的完成移动(if the motion is completed in full)
            if self.stackup_step == 1:  # 如果完成了夹取(if the grasping is completed)
                self.stackup_step = 2
                self.status = 2
          
            elif self.stackup_step == 2:  # 如果完成了放置(if the placement is completed)
                actions.goto_left(self.joints_pub)
                rospy.sleep(0.5)
                self.get_endpoint()
                self.last_position = None
                self.target_position = None
                self.stackup_step = 10
                self.stop_thread = True
        else:  # 如果被取消或者无法到达指定位置(if canceled or unable to reach the specified position)
            actions.goto_home(self.joints_pub)
            self.status = 0
            self.stackup_step = 0
            self.last_position = None
            self.stop_thread = True

    def active_callback(self):  # 运动开始回调(callback function when the motion starts)
        self.start_move = True
        rospy.loginfo("start move")

    def feedback_callback(self, msg):  # 动作执行进度回调(progress callback during action execution)
        rospy.loginfo("finish action: {:.2%}".format(msg.percent))
        self.finish_percent = msg.percent

    def place(self):
        goal = MoveGoal()
        goal.grasp.mode = 'place'
        rospy.sleep(1)
        target_position = rospy.get_param(os.path.join(POSITIONS_PATH, 'tag_stackup/target_1'))
        goal.grasp.position.x = target_position[0]
        goal.grasp.position.y = target_position[1]
        goal.grasp.position.z = self.target_position[0][2] + target_position[2]
        goal.grasp.pitch = self.pick_pitch
        goal.grasp.align_angle = -90  # yaw #- 20/1000* 240
        goal.grasp.grasp_approach.z = 0.01  # 放置时靠近的方向和距离(direction and distance of approach during placement)
        goal.grasp.grasp_retreat.z = 0.04  # 放置后后撤的方向和距离(direction and distance of retreat after placement)
        goal.grasp.grasp_posture = 400  # 夹取前后夹持器的开合角度(the opening and closing angle of the grippers before and after grasping)
        goal.grasp.pre_grasp_posture = 600
        self.action_client.send_goal(goal, self.done_callback, self.active_callback, self.feedback_callback)

    def start_stackup(self, pose_t, pose_R):
        rospy.loginfo("开始搬运堆叠...")
        print(pose_t, pose_R)
        self.stackup_step = 1
        goal = MoveGoal()
        goal.grasp.mode = 'pick'
        # 物体坐标(object coordinates)
        goal.grasp.position.x = pose_t[0]
        goal.grasp.position.y = pose_t[1]
        goal.grasp.position.z = pose_t[2]
        # 夹取时的姿态角(position angle during grasping)
        goal.grasp.pitch = self.pick_pitch
        goal.grasp.align_angle = -pose_R[2]
        # 夹取时靠近的方向和距离(direction and distance of approach during grasping)
        goal.grasp.grasp_approach.z = 0.03
        # 夹取后后撤方向和距离(direction and distance of retreat after grasping)
        goal.grasp.grasp_retreat.z = 0.05
        # 夹取前后夹持器的开合(the opening and closing of the grippers before and after grasping)
        goal.grasp.grasp_posture = 600
        goal.grasp.pre_grasp_posture = 350
        self.action_client.send_goal(goal, self.done_callback, self.active_callback, self.feedback_callback) # 发送夹取请求 (send grasping requirement)

    def action_starting(self, pose_world_T, pose_world_euler):
        while not self.stop_thread: 
            if self.status == 1:
                self.status = 0
                self.start_stackup(pose_world_T, pose_world_euler)
            elif self.status == 2:
                self.status = 0
                self.place()
            else:
                rospy.sleep(0.02)
                # pass
    def image_callback(self, ros_image: RosImage):
        # rospy.logdebug('Received an image! ')
        # 将ros格式图像转换为opencv格式(convert the ros format image to opencv format)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        result_image = np.copy(rgb_image)

        if (self.stackup_step == 0 or self.stackup_step == 10) and self.K is not None and self.D is not None and self.hand2cam_tf_matrix is not None and self.endpoint is not None:
            tags = self.at_detector.detect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY), True, (self.K[0,0], self.K[1,1], self.K[0,2], self.K[1,2]), self.tag_size)
            if len(tags) > 0:
                # tags = sorted(tags, key=lambda tag: tag.tag_id) # 出来就是升序排列的不需要手动进行排列(if the result is already sorted in ascending order, manual sorting is not necessary)
                draw_tags(result_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))
                pose_end = np.matmul(self.hand2cam_tf_matrix, xyz_rot_to_mat(tags[0].pose_t, tags[0].pose_R))  # 转换的末端相对坐标(relative coordinates of the converted end)
                pose_world = np.matmul(self.endpoint, pose_end)  # 转换到机械臂世界坐标(convert to the robotic arm world coordinates)
                pose_world_T, pose_world_euler = mat_to_xyz_euler(pose_world, degrees=True)
                cv2.putText(result_image, "{:.3f} {:.3f}".format(pose_world_T[0], pose_world_T[1]), (int(tags[0].center[0]-50), int(tags[0].center[1]+22)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                r = pose_world_euler[2] % 90  # 将旋转角限制到 ±45°(limit the rotation angle to ±45°)
                r = r - 90 if r > 45 else (r + 90 if r < -45 else r)
                pose_world_euler[-1] = r
                # print("Smallest TAG ID %d: "%tags[0].tag_id, pose_world_T, pose_world_euler)
                if  self.last_position is None or distance(self.last_position[0], pose_world_T) > 0.005:  # 前后距离大于 5mm 就是结果不可信(if the distance forward or backward is greater than 5mm, the result is considered unreliable)
                    self.target_count = 0
                else:
                    self.target_count += 1
                self.last_position = pose_world_T, pose_world_euler
                if self.target_count > 30:
                    self.target_count = 0
                    if self.stackup_step == 0 and self.enable_stackup and tags[0].tag_id != 100:
                        params = rospy.get_param(os.path.join(POSITIONS_PATH, 'tag_stackup'))
                        pose_world_T[2] = 0.015
                        for i in range(3):
                            pose_world_T[i] = pose_world_T[i] + params['offset'][i]
                            pose_world_T[i] = pose_world_T[i] * params['scale'][i]
                        pose_world_T[2] += (math.sqrt(pose_world_T[1] ** 2 + pose_world_T[0] ** 2) - 0.21) / 0.20 * 0.030
                        self.status = 1
                        self.stop_thread = False
                        threading.Thread(target=self.action_starting, args=(pose_world_T, pose_world_euler)).start()
                    elif self.stackup_step == 10:
                        self.target_position = pose_world_T, pose_world_euler
                        if pose_world_T[-1] > 0.08:
                            self.err_msg = "Too high, please remove some blocks first!!!"
                        else:
                            self.err_msg = None
                        
                            threading.Thread(target=self.set_target).start()
            else:
                self.target_count = 0
        if self.err_msg is not None:
            rospy.logerr(self.err_msg)
            err_msg = self.err_msg.split(';')
            for i, m in enumerate(err_msg):
                cv2.putText(result_image, m, (10, 150 + (i * 30)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 7)
                cv2.putText(result_image, m, (10, 150 + (i * 30)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        # 计算帧率及发布结果图像(calculate the frame rate and publish the resulting image)
        # self.fps.update()
        # result_image = self.fps.show_fps(result_image)
        ros_image.data = result_image.tobytes()
        self.result_image_pub.publish(ros_image)


if __name__ == "__main__":
    node = TagStackup("tag_stackup", log_level=rospy.INFO)
    rospy.spin()
