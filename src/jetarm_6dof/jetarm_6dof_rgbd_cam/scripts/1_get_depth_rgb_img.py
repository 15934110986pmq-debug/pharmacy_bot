#!/usr/bin/python3
#coding=utf8

# 将深度图转换为伪彩色图像，(convert the depth map to a pseudo-color image)
# 伪彩色图即距离不同回显示不同的颜色(a pseudo-color image means displaying different colors for different distances)

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
        rgb_sub = message_filters.Subscriber('/rgbd_cam/color/image_raw', RosImage, queue_size=1)
        depth_sub = message_filters.Subscriber('/rgbd_cam/depth/image_raw', RosImage, queue_size=1)
        rospy.ServiceProxy('/rgbd_cam/set_ldp', SetBool)(False)
    
        # 同步时间戳, 时间允许有误差在0.03s(synchronize timestamps, allowing for a time error of up to 0.03 seconds)
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 2, 0.02)
        sync.registerCallback(self.multi_callback) #执行反馈函数(execute feedback function)
        self.queue = queue.Queue(maxsize=1)

    def multi_callback(self, ros_rgb_image, ros_depth_image):
        if self.queue.empty():
            self.queue.put_nowait((ros_rgb_image, ros_depth_image))

    def image_proc(self):
        ros_rgb_image, ros_depth_image = self.queue.get(block=True)
        try:
            rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
            rgb_image = rgb_image[40:440,]
            depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)

            h, w = depth_image.shape[:2]

            depth = np.copy(depth_image).reshape((-1, ))
            depth[depth<=0] = 0
            sim_depth_image = np.clip(depth_image, 0, 4000).astype(np.float64)
            sim_depth_image = sim_depth_image / 2000.0 * 255.0
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)
            result_image = np.concatenate([cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR), depth_color_map], axis=1)
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

