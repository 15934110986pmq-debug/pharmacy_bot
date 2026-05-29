import cv2
import numpy as np
from apriltag import apriltag
from vision_utils import point_remapped
from scipy.spatial.transform import Rotation
from spatialmath import SE3


class Detector:
    def __init__(self, family="tag36h11", tag_size=0.025):
        self.detector = apriltag(family)
        self.objp = np.array([[-tag_size/2, -tag_size/2,  0],
                              [ tag_size/2,  tag_size/2,  0],
                              [-tag_size/2,  tag_size/2,  0],
                              [ tag_size/2,  -tag_size/2,  0],
                              [ 0,                0,                0]], dtype=np.float64)
        self.proc_size = (0, 0)
        self.ori_size = (0, 0)
        self.detections = ()
        self.K = None
        self.D = None

    def set_tag_size(self, tag_size):
        self.objp = np.array([[-tag_size/2, -tag_size/2,  0],
                              [ tag_size/2, -tag_size/2,  0],
                              [-tag_size/2,  tag_size/2,  0],
                              [ tag_size/2,  tag_size/2,  0],
                              [ 0,           0,           0]], dtype=np.float64)
    
    def get_pose(self, tag):
        center = point_remapped(tag['center'], self.proc_size, self.ori_size)
        lb, rb, rt, lt =[point_remapped(p, self.proc_size, self.ori_size) for p in tag['lb-rb-rt-lt']]
        corners = np.array([lb, rb, lt, rt, center]).reshape(5, -1)
        ret, rvecs, tvecs = cv2.solvePnP(self.objp, corners, self.K, self.D)
        euler = Rotation.from_rotvec(rvecs.reshape((3, ))).as_euler('xyz', degrees=True)
        return tvecs.reshape((3, )).tolist(), euler.tolist()


    def get_world_pose(self, tag, camera_pose):
        center = point_remapped(tag['center'], self.proc_size, self.ori_size)
        lb, rb, rt, lt =[point_remapped(p, self.proc_size, self.ori_size) for p in tag['lb-rb-rt-lt']]
        corners = np.array([lb, rb, lt, rt, center]).reshape(5, -1)
        ret, rvecs, tvecs = cv2.solvePnP(self.objp, corners, self.K, self.D)
        euler = Rotation.from_rotvec(rvecs.reshape((3, ))).as_euler('xyz')
        camera_pose = SE3(camera_pose[0]) * SE3.RPY(camera_pose[1])
        world_pose = camera_pose * (SE3(tvecs.reshape((3,))) * SE3.RPY(euler))
        euler =  Rotation.from_matrix(world_pose.R).as_euler('xyz', degrees=True).tolist()
        return world_pose.t.tolist(), euler

    def get_angle(self, tag):
        lb, rb, rt, lt =[point_remapped(p, self.proc_size, self.ori_size) for p in tag['lb-rb-rt-lt']]
        top, sec = sorted([lb, rb, rt, lt], key=lambda a: a[1])[:2]
        angle = np.degrees(np.arctan2(top[1] - sec[1], top[0] - sec[0]))
        angle = angle + 180 if angle < -45 else angle
        return angle

    def detect(self, img, K, D, scale=1):
        ori_height, ori_width = img.shape[:2]
        proc_height, proc_width = int(ori_height * scale), int(ori_width * scale)
        self.ori_size = (ori_width, ori_height)
        self.proc_size = (proc_width, proc_height)
        self.K = K
        self.D = D
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY) # 将 RGB 图片转为灰度图(convert an RGB image to a grayscale image)
        if ori_height != proc_height or ori_width != proc_width:
            gray = cv2.resize(gray, self.proc_size) # 缩放尺寸减少计算量(scale down the size to reduce computational complexity)
        try:
            detections = self.detector.detect(gray)
        except Exception as e: # 没有找到二维码(QR code has not been found)
            detections = ()
        self.detections = sorted(detections, key=lambda t: t['id'] )
        return detections
    
    def draw(self, image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0)):
        image_size = (image.shape[1], image.shape[0])
        for tag in tags:
            corners =[point_remapped(p, self.proc_size, image_size) for p in tag['lb-rb-rt-lt']]
            center = point_remapped(tag['center'], self.proc_size, image_size)
            # 画出四角(draw four corners)
            if corners_color is not None:
                for p in corners:
                    cv2.circle(image, (int(p[0]), int(p[1])), 5, corners_color, -1)
            if center_color is not None:
                cv2.circle(image, (int(center[0]), int(center[1])), 8, center_color, -1)
            cv2.putText(image, "ID:%d"%tag['id'], (int(center[0]-25), int(center[1]-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        return image

    def id_filter(self, id_want):
        return list(filter(lambda t: t['id']==id_want, self.detections))
        

