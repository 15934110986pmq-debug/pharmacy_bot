#!/usr/bin/env python3
import os
import cv2
import rospy
import queue
import threading

import numpy as np
from vision_utils import fps, xyz_quat_to_mat, xyz_euler_to_mat, pixels_to_world, box_center, mat_to_xyz_euler, distance, extristric_plane_shift
from sensor_msgs.msg import Image as RosImage, CameraInfo
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

from sensor_msgs.msg import Image
from vision_utils import fps, draw_tags
from hiwonder_interfaces.srv import GetRobotPose
from hiwonder_interfaces.msg import MoveAction, MoveGoal, MultiRawIdPosDur
from dt_apriltags import Detector
from jetarm_sdk import bus_servo_control, pid
import actions
import actionlib
import stepper as Stepper
CONFIG_NAME = '/config'
POSITIONS_PATH = '/positions'

TARGET_POSITION_STEPPER = {
    "target_1": [0.25, 0, -0.023],
    "target_2": [0.25, 0, -0.014],
    "target_3": [0.25, 0.01, -0.014],
}

TARGET_POSITION = {
    'target_1': (2600),
    'target_2': (4000),
    'target_3': (5200),
}

class TagTrackingNode:
    def __init__(self):
        rospy.init_node('tag_tracking_node')

        self.yaw = 500
        self.pitch = 350

        self.tracker = None
        self.enable_select = False

        self.at_detector = Detector(searchpath=['apriltags'], 
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)

        self.fps = fps.FPS() # 帧率统计器(frame rate counter)
        self.servos_pub = rospy.Publisher('/controllers/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        rospy.sleep(3)
   
        self.lock = threading.RLock()
        self.K = None
        self.D = None
        self.waste_class_name = None
        config = rospy.get_param(CONFIG_NAME)
        self.hand2cam_tf_matrix = config['hand2cam_tf_matrix']
        self.extristric = None
        self.roi = None
        self.moving_step = 0
        self.status = 1
        self.tag_size = rospy.get_param("/config/tag_size", 0.025)

        self.pick_pitch = 80
        

        self.target = None
        self.count = 0
        self.last_card = None
        self.endpoint = None
        self.image_queue = queue.Queue(maxsize=2)
        self.fps = fps.FPS()

        rospy.sleep(1)
        actions.go_home(self.servos_pub)

        stepper = Stepper.Stepper(1)
        stepper.go_home(True)
        stepper.set_mode(stepper.EN)
        stepper.set_div(stepper.DIV_1_4)
        self.target = None
        self.target_labels = {
            "target_1": False,
            "target_2": False,
            "target_3": False,
        }

        self.servos_pub = rospy.Publisher("/controllers/multi_id_pos_dur", MultiRawIdPosDur, queue_size=1)
        camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/camera_info')
        self.camera_info_sub = rospy.Subscriber(camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=1) #订阅相机内参(subscribe camera intrinsic parameter)
        
        
         # 识别区域的四个角的世界坐标(the world coordinates of four corners in recognition region)
        white_area_cam = config['white_area_pose_cam']
        white_area_center = config['white_area_pose_world']
        self.white_area_center = white_area_center
        self.white_area_cam = white_area_cam
        white_area_height = config['white_area_world_size']['height']
        white_area_width = config['white_area_world_size']['width']
        white_area_lt = np.matmul(white_area_center, xyz_euler_to_mat((white_area_height / 2, white_area_width / 2, 0.0), (0, 0, 0)))
        white_area_lb = np.matmul(white_area_center, xyz_euler_to_mat((-white_area_height / 2 - 0.01, white_area_width / 2, 0.0), (0, 0, 0)))
        white_area_rb = np.matmul(white_area_center, xyz_euler_to_mat((-white_area_height / 2 - 0.01, -white_area_width / 2, 0.0), (0, 0, 0)))
        white_area_rt = np.matmul(white_area_center, xyz_euler_to_mat((white_area_height / 2, -white_area_width / 2, 0.0), (0, 0, 0)))
        self.get_endpoint()
        corners_cam =  np.matmul(np.linalg.inv(np.matmul(self.endpoint, config['hand2cam_tf_matrix'])), [white_area_lt, white_area_lb, white_area_rb, white_area_rt, white_area_center])
        corners_cam = np.matmul(np.linalg.inv(white_area_cam), corners_cam)
        corners_cam = corners_cam[:, :3, 3:].reshape((-1, 3))
        tvec, rmat = config['extristric']


        while self.K is None or self.D is None: # 等待获取相机内参(wait for obtaining camera intrinsic parameters)
            rospy.sleep(0.5)

        self.hand2cam_tf_matrix = config['hand2cam_tf_matrix']
        center_imgpts, jac = cv2.projectPoints(corners_cam[-1:], np.array(rmat), np.array(tvec), self.K, self.D)
        self.center_imgpts = np.int32(center_imgpts).reshape(2)
        tvec, rmat = extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.04)
        self.extristric = tvec, rmat
        imgpts, jac = cv2.projectPoints(corners_cam[:-1], np.array(rmat), np.array(tvec), self.K, self.D)
        self.imgpts = np.int32(imgpts).reshape(-1, 2)

        # 计算ROI区域(calculate RIO region)
        x_min = min(self.imgpts, key=lambda p: p[0])[0]  # x轴最小值(the minimum value of X-axis)
        x_max = max(self.imgpts, key=lambda p: p[0])[0]  # x轴最大值(the maximum value of X-axis)
        y_min = min(self.imgpts, key=lambda p: p[1])[1]  # y轴最小值(the minimum value of Y-axis)
        y_max = max(self.imgpts, key=lambda p: p[1])[1]  # y轴最大值(the maximum value of Y-axis)
        roi = np.maximum(np.array([y_min, y_max, x_min, x_max]), 0)
        self.roi = roi
        
        # 订阅相机图像话题(subscribe camera image topic)
        source_image_topic = rospy.get_param('~source_image_topic', '/camera/image_raw')
        rospy.loginfo("订阅原图像节点 " + source_image_topic)
        self.action_client = actionlib.SimpleActionClient('/grasp', MoveAction)
        self.image_sub = rospy.Subscriber(source_image_topic, RosImage, self.image_callback, queue_size=1)
        rospy.loginfo("启动完成\r\n\r\n")

    def camera_info_callback(self, msg): # 相机内参回调(callback camera intrinsic parameters)
        with self.lock:
            K = np.matrix(msg.K).reshape(1, -1, 3)
            D = np.array(msg.D)
            new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (640, 480), 0, (640, 480))
            self.K, self.D = np.matrix(new_K), np.zeros((5, 1))

    def get_endpoint(self):
        endpoint = rospy.ServiceProxy('/kinematics/get_current_pose', GetRobotPose)().pose
        self.endpoint = xyz_quat_to_mat([endpoint.position.x, endpoint.position.y, endpoint.position.z],
                                        [endpoint.orientation.w, endpoint.orientation.x, endpoint.orientation.y, endpoint.orientation.z])

    def done_callback(self, state, result):  # 动作执行完毕回调(callback for action execution completion)
        rospy.loginfo("state:%f", state)
        if not result.result.complete:  # 如果在移动中被取消，需要回到初始位置(If the action is cancelled during movement, return to the initial position)
            bus_servo_control.set_servos(self.servos_pub, 500, ((1, 500), ))
            rospy.sleep(1)
            actions.go_home(self.servos_pub)
            bus_servo_control.set_servos(self.servos_pub, 500, ((10, 200), ))
            rospy.sleep(0.5)
        elif self.moving_step != 1:
            bus_servo_control.set_servos(self.servos_pub, 500, ((10, 200), ))

        if self.finish_percent == 1:  # 如果完整的完成移动(if the movement is completed in full)
            if self.moving_step == 1:  # 如果完成了夹取(if the gripper has completed the grasping)
                actions.go_back(self.servos_pub)

                self.moving_step = 2
                goal = MoveGoal()
                goal.grasp.mode = 'place'
                rospy.sleep(1)
               
                stepper_position = TARGET_POSITION['target_' + str(self.target)]
                stepper = Stepper.Stepper(1)
                stepper.set_mode(stepper.EN) # 设置滑轨使能(enable the slider track)
                stepper.goto(stepper_position) # 驱动滑轨移动到放置位置(drive the slider track to move to the placing position)
                stepper_time = stepper_position/1000 # 计算需要的时间(calculate the required time)
                rospy.sleep(stepper_time)

                target_position = TARGET_POSITION_STEPPER['target_'+ str(self.target)]# 读取放置坐标(read placing coordinate)
                goal.grasp.position.x = target_position[0]
                goal.grasp.position.y = target_position[1]
                goal.grasp.position.z = target_position[2]
                goal.grasp.pitch = self.pick_pitch
                goal.grasp.grasp_approach.z = 0.06  # 放置时靠近的方向和距离(the direction and distance for approaching during placing)
                goal.grasp.grasp_retreat.z = 0.06  # 放置后后撤的方向和距离(the direction and distance of retreat during placing)
                goal.grasp.grasp_posture = 370  # 夹取前后夹持器的开合角度(the opening and closing of the gripper before and after the grasping)
                goal.grasp.pre_grasp_posture = 550
                self.action_client.send_goal(goal, self.done_callback, self.active_callback, self.feedback_callback)

            elif self.moving_step == 2:  # 如果完成了放置(if the placement is completed)

                #actions.go_home(self.servos_pub)
                rospy.sleep(1.5)

                stepper = Stepper.Stepper(1)
                stepper_position = TARGET_POSITION['target_' + str(self.target)]
                stepper.goto(-stepper_position) # 滑轨回到初始位置(the slider track returns to the initial position)
                stepper_time = stepper_position/1000
                rospy.sleep(stepper_time)
                stepper.set_mode(stepper.EN) # 解除滑轨锁定(release slider track lock)
                print("FINISHED")

                self.get_endpoint()
                self.last_position = None
                self.target = None
                self.count = 0
                self.moving_step = 0
        else:  # 如果被取消或者无法到达指定位置(if the action is cancelled, or it is unable to reach the specified position)
            actions.go_home(self.servos_pub)
            self.moving_step = 0
            self.last_position = None

    def active_callback(self):  # 运动开始回调(callback for starting the movement)
        self.start_move = True
        rospy.loginfo("start move")

    def feedback_callback(self, msg):  # 动作执行进度回调(callback action execution progress)
        rospy.loginfo("finish action: {:.2%}".format(msg.percent))
        self.finish_percent = msg.percent

    def start_moving(self, pose_t, pose_R):
        rospy.loginfo("开始搬运堆叠...")
        #print(pose_t, pose_R)
        self.moving_step = 1
        goal = MoveGoal()
        goal.grasp.mode = 'pick'
        # 物体坐标(object coordinate)
        goal.grasp.position.x =  pose_t[0]
        goal.grasp.position.y =  pose_t[1]
        goal.grasp.position.z = -0.033
        # 夹取时的姿态角(the posture angle during grasping)
        goal.grasp.pitch = self.pick_pitch
        goal.grasp.align_angle = 0  # 总是朝前(always moving forward)
        # 夹取时靠近的方向和距离(the direction and distance for approaching during grasping)
        goal.grasp.grasp_approach.z = 0.03
        # 夹取后后撤方向和距离(the direction and distance of retreat during grasping)
        goal.grasp.grasp_retreat.z = 0.05
        # 夹取前后夹持器的开合(the opening and closing of the gripper before and after the grasping)
        goal.grasp.grasp_posture = 580
        goal.grasp.pre_grasp_posture = 220

        self.action_client.send_goal(goal, self.done_callback, self.active_callback, self.feedback_callback)  # 发送夹取请求(send grasping request)
        rospy.sleep(3)
 
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
        #rospy.logdebug('Received an image! ')
        # 将画面转为 opencv 格式(convert the image to opencv format)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        result_image = np.copy(rgb_image)
        try:

            if self.moving_step == 0 and self.roi is not None and self.K is not None and self.D is not None:
                roi_area_mask = np.zeros(shape=(ros_image.height, ros_image.width, 1), dtype=np.uint8)
                roi_area_mask = cv2.drawContours(roi_area_mask, [self.imgpts], -1, 255, cv2.FILLED)
                rgb_image = cv2.bitwise_and(rgb_image, rgb_image, mask=roi_area_mask)  # 和原图做遮罩，保留需要识别的区域(create a mask based on the original image to retain the region to be recognized)
                tags = self.at_detector.detect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY), True, (self.K[0,0], self.K[1,1], self.K[0,2], self.K[1,2]), self.tag_size)
                tags = sorted(tags, key=lambda tag: tag.tag_id) # 貌似出来就是升序排列的不需要手动进行排列(It seems to be sorted in ascending order automatically and does not need to be manually sorted)
                draw_tags(result_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))
                if len(tags) > 0:  
                    projection_matrix = np.row_stack((np.column_stack((self.extristric[1], self.extristric[0])), np.array([[0, 0, 0, 1]])))
                    world_pose = pixels_to_world([tags[0].center, ], self.K, projection_matrix)[0]  # 像素坐标相对于识别区域中心的相对坐标(the relative coordinate of the pixel coordinate with respect to the center of the recognition region)
                    world_pose[1] = -world_pose[1]
                    world_pose[2] = 0.04
                    world_pose = np.matmul(self.white_area_center, xyz_euler_to_mat(world_pose, (0, 0, 0)))  # 转换到相机相对坐标(convert to the relative coordinate of the camera)
                    world_pose[2] = 0.04
                    
                    pose_t, pose_R = mat_to_xyz_euler(world_pose)
                    params = rospy.get_param(os.path.join(POSITIONS_PATH, 'tag_sortting'))
                            
                    for i in range(3):
                        pose_t[i] = pose_t[i] + params['offset'][i]
                        pose_t[i] = pose_t[i] * params['scale'][i]
                       # print(pose_t)

                    self.target = tags[0].tag_id
                    #print("tags[0].tag_id=",tags[0].tag_id)
                    if self.target in [1, 2, 3]:
                        self.count += 1
                        if self.count > 40:
                            self.moving_step = 1
                            self.status = 1
                            threading.Thread(target=self.action_starting, args=(pose_t, pose_R)).start()


                  
        except Exception as e:
            rospy.logerr(str(e))
            
        # 计算帧率及发布结果图像(calculate frame rate and publish result image)
        self.fps.update()
        self.fps.show_fps(result_image)
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow("tag_tracking", result_image)
        key = cv2.waitKey(1)
                
        

       

if __name__ == '__main__':
    try:
        tag_tracking = TagTrackingNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))


