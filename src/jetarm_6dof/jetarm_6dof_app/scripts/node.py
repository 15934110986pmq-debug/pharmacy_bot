import rospy
import math
import threading
from sensor_msgs.msg import Image as RosImage
from jetarm_kinematics import kinematics_control
from jetarm_sdk import controller_client, sdk_client
from vision_utils import fps, point_remapped
import heart
import cv2
import numpy as np
import queue
from math import radians

class AppNode:
    def __init__(self, node_name, log_level=rospy.INFO):
        rospy.init_node(node_name, anonymous=True, log_level=log_level)

        self.robot_dof = rospy.get_param('~robot_dof', 7)
        self.K = None
        self.D = None

        self.joint = controller_client.JointControllerClient()
        self.sdk = sdk_client.JetArmSDKClient()
        self.tag_detector = TagDetector(tag_size=0.0335)
        self.tag_image_queue = queue.Queue(maxsize=2)
        self.camera_pose = None

        self.target_position = None
        self.target_position_count = 0

        self.lock = threading.RLock()
        self.fps = fps.FPS()    # 帧率统计器(frame rate counter)
        self.thread = None

        self.result_image_pub = rospy.Publisher('~image_result', RosImage, queue_size=10)
        self.heart = heart.Heart('~heartbeat', 5, lambda e: self.exit_srv_callback(e))

    def camera_info_callback(self, msg):
        with self.lock:
            print("FJKLSJDKLJFKJFJJJJJJJJJJJJJJJJJJJJ")
            self.K = np.matrix(msg.K).reshape(1, -1, 3)
            self.D = np.array(msg.D)
    

    def exit_srv_callback(self, _):
        self.target_position_checked = False
        self.target_position = None
        self.target_position_count = 0

    
