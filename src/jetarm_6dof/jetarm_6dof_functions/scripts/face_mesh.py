#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from vision_utils import fps
from hiwonder_interfaces.msg import MultiRawIdPosDur
from jetarm_sdk import bus_servo_control

mp_drawing = mp.solutions.drawing_utils
mp_face_mesh = mp.solutions.face_mesh
drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)

class FaceDetectNode:
    def __init__(self):
        rospy.init_node("face_mesh_node")
        self.face_mesh = mp.solutions.face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=1,
            min_detection_confidence=0.5,
        )
        self.drawing = mp.solutions.drawing_utils
        self.servos_pub = rospy.Publisher('/controllers/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        rospy.sleep(3)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 700), (3, 85), (4, 350), (5, 500), (10, 200)))
        rospy.sleep(2)

        self.fps = fps.FPS()
        source_image_topic = rospy.get_param('~source_image_topic', '/camera/image_raw')
        self.image_sub = rospy.Subscriber(source_image_topic, Image, self.image_callback, queue_size=1)

    def image_callback(self, ros_image):
        # self.get_logger().debug('Received an image! ')
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面(original RGB image)
        black_image = np.zeros((ros_image.height, ros_image.width, 3), dtype=np.uint8)
        resize_image = cv2.resize(rgb_image, (int(ros_image.width / 2), int(ros_image.height / 2)), cv2.INTER_NEAREST) # 缩放图片(resize the image)
        try:
            results = self.face_mesh.process(resize_image) # 调用人脸检测(call human face detection)
            if results.multi_face_landmarks is not None:
                for face_landmarks in results.multi_face_landmarks:
                    mp_drawing.draw_landmarks(
                            image=black_image,
                            landmark_list=face_landmarks,
                            connections = mp_face_mesh.FACEMESH_CONTOURS,
                            landmark_drawing_spec=drawing_spec,
                            connection_drawing_spec=drawing_spec)
        except Exception as e:
            rospy.logerr(str(e))

        result_image = np.concatenate([rgb_image, black_image], axis=1)
        self.fps.update()
        result_image = self.fps.show_fps(result_image)
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('image', result_image)
        cv2.waitKey(1)


if __name__ == "__main__":
    try:
        face_detection_node = FaceDetectNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

