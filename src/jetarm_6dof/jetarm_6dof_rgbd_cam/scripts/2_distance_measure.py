#!/usr/bin/python3
#coding=utf8

# 实现距离测量(implement distance measurement)
# 默认情况下回显示距离相机最近的点的距离(by default, display the distance of the point closest to the camera)
# 可以鼠标点击画面任意一点测量对应点的距离(you can measure the distance of any point on the screen by clicking it with the mouse)
# 可以鼠标恢复最近点测量(you can use the mouse to revert to measuring the distance of the closest point)


import cv2
import rospy
import numpy as np
import message_filters
from sensor_msgs.msg import Image as RosImage
from std_srvs.srv import SetBool
from vision_utils import fps
import queue
from hiwonder_interfaces.msg import MultiRawIdPosDur
from jetarm_sdk import pid, bus_servo_control


class RgbDepthImageNode:
    def __init__(self):
        rospy.init_node('g1et_rgb_and_depth_image', anonymous=True)
        self.fps = fps.FPS()

        self.servos_pub = rospy.Publisher('/controllers/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        rospy.sleep(3)
        bus_servo_control.set_servos(self.servos_pub, 1000, ((1, 500), (2, 700), (3, 85), (4, 350), (5, 500), (10, 200)))
        rospy.sleep(2)
        
        rospy.wait_for_service('/rgbd_cam/set_ldp')
        rospy.ServiceProxy('/rgbd_cam/set_ldp', SetBool)(False)
        
        rgb_sub = message_filters.Subscriber('/rgbd_cam/color/image_raw', RosImage, queue_size=1)
        depth_sub = message_filters.Subscriber('/rgbd_cam/depth/image_raw', RosImage, queue_size=1)
    
        # 同步时间戳, 时间允许有误差在0.03s(synchronize timestamps, allowing for a time error of up to 0.03 seconds)
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 2, 0.03)
        sync.registerCallback(self.multi_callback) #执行反馈函数(execute feedback function)
        self.queue = queue.Queue(maxsize=1)
        self.target_point = None
        self.last_event = 0
        cv2.namedWindow("depth")
        cv2.setMouseCallback('depth', self.click_callback)

    def click_callback(self, event, x, y, flags, params):
        if event == cv2.EVENT_RBUTTONDOWN or event == cv2.EVENT_MBUTTONDOWN or event == cv2.EVENT_LBUTTONDBLCLK:
            self.target_point = None
        if event == cv2.EVENT_LBUTTONDOWN and self.last_event != cv2.EVENT_LBUTTONDBLCLK:
            if x >= 640:
                self.target_point = (x - 640, y)
            else:
                self.target_point = (x, y)
        self.last_event = event


    def multi_callback(self, ros_rgb_image, ros_depth_image):
        if self.queue.empty():
            self.queue.put_nowait((ros_rgb_image, ros_depth_image))

    def image_proc(self):
        ros_rgb_image, ros_depth_image = self.queue.get(block=True)
        try:
            rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
            depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)

            h, w = depth_image.shape[:2]

            depth = np.copy(depth_image).reshape((-1, ))
            depth[depth<=0] = 55555
            min_index = np.argmin(depth)
            min_y = min_index // w
            min_x = min_index - min_y * w
            if self.target_point is not None:
                min_x, min_y = self.target_point

            sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64) / 2000 * 255
            depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)

            txt = 'Dist: {}mm'.format(depth_image[min_y, min_x])
            cv2.circle(depth_color_map, (int(min_x), int(min_y)), 8, (32, 32, 32), -1)
            cv2.circle(depth_color_map, (int(min_x), int(min_y)), 6, (255, 255, 255), -1)
            cv2.putText(depth_color_map, txt, (11, 200), cv2.FONT_HERSHEY_PLAIN, 2.0, (32, 32, 32), 6, cv2.LINE_AA)
            cv2.putText(depth_color_map, txt, (10, 200), cv2.FONT_HERSHEY_PLAIN, 2.0, (240, 240, 240), 2, cv2.LINE_AA)

            bgr_image = cv2.cvtColor(rgb_image[40:440, ], cv2.COLOR_RGB2BGR)
            cv2.circle(bgr_image, (int(min_x), int(min_y)), 8, (32, 32, 32), -1)
            cv2.circle(bgr_image, (int(min_x), int(min_y)), 6, (255, 255, 255), -1)
            cv2.putText(bgr_image, txt, (11, h - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (32, 32, 32), 6, cv2.LINE_AA)
            cv2.putText(bgr_image, txt, (10, h - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (240, 240, 240), 2, cv2.LINE_AA)

            self.fps.update()
            #bgr_image = self.fps.show_fps(bgr_image)
            result_image = np.concatenate([bgr_image, depth_color_map], axis=1)
            cv2.imshow("depth", result_image)
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
