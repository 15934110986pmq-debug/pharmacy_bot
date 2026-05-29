#!/usr/bin/env python3

"""
此程序提供了阈值调节,保存等功能的服务(this program provides services such as threshold adjustment and saving functionality)
"""
import os
import sys
import cv2
import rospy
import yaml
import numpy as np
from threading import RLock
from sensor_msgs.msg import Image
from std_srvs.srv import Empty, Trigger, SetBool, TriggerResponse, SetBoolResponse
from lab_config.srv import StashRange, GetRange, ChangeRange, GetAllColorName
from lab_config.srv import StashRangeResponse, GetRangeResponse, ChangeRangeResponse, GetAllColorNameResponse


class LabConfigManagerNode:
    def __init__(self, node_name):
        rospy.init_node(node_name, log_level=rospy.INFO)
        self.node_name = node_name

        # 读取需要的参数(read the required parameters)
        self.config_file_path = '/home/ubuntu/jetarm/src/hiwonder_imgproc/color_detection/config/config.yaml'
        # rospy.get_param('~config_file_path', os.path.join(sys.path[0], "../config/lab_config.yaml"))
        self.color_ranges = rospy.get_param('~color_range_list', {})
        self.kernel_erode = rospy.get_param('~kernel_erode', 3)
        self.kernel_dilate = rospy.get_param('~kernel_dilate', 3)
        self.current_range = {'min': [0, 0, 0], 'max': [100, 100, 100]}
        if 'red' in self.color_ranges:
            self.current_range = self.color_ranges['red']

        # 画面相关的topic(visual-related topics)
        self.image_sub = None
        self.result_image_pub = rospy.Publisher('~image_result', Image, queue_size=1)
        # self.source_image_pub = rospy.Publisher(self.node_name + '/image_source', Image, queue_size=1)

        # 进入, 退出, 启动, 停止 服务(enter, exit, start, stop service)
        self.enter_srv = rospy.Service(self.node_name + '/enter', Trigger, self.enter_srv_callback)
        self.exit_srv = rospy.Service(self.node_name + '/exit', Trigger, self.exit_srv_callback)
        self.running_srv = rospy.Service(self.node_name + '/set_running', SetBool, self.set_running_srv_callback)

        # 修改阈值, 保持阈值等服务(modify threshold, maintain threshold and other related services)
        self.save_to_disk_srv = rospy.Service(self.node_name + '/save_to_disk', Trigger, self.save_to_disk_srv_callback)
        self.get_color_range_srv = rospy.Service(self.node_name + '/get_range', GetRange, self.get_range_srv_callback)
        self.change_range_srv = rospy.Service(self.node_name + '/change_range', ChangeRange, self.change_range_srv_callback)
        self.stash_range_srv = rospy.Service(self.node_name + '/stash_range', StashRange, self.stash_range_srv_callback)
        self.get_all_color_name_srv = rospy.Service(self.node_name + '/get_all_color_name', GetAllColorName,
                                           self.get_all_color_name_srv_callback)

        # 心跳, 保证APP非正常退出时功能能自动停止运行, 避免资源占用(heartbeat: ensure that the application automatically stops running when it exits abnormally to prevent resource consumption)
        self.heartbeat_timer = None
        self.heartbeat_srv = rospy.Service('lab_config_manager/heartbeat', SetBool, self.heartbeat_srv_callback)

    def image_callback(self, ros_image):
        """
        相机画面回调(camera screen callback)
        :params ros_image: 画面数据(frame data)
        """
        range_ = self.current_range
        # 将ros格式图像转换为opencv格式(convert the ros format image to opencv format)
        image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        ow, oh = ros_image.width, ros_image.height

        # 对图像进行处理做二值化等操作(perform image processing tasks such as binarization and other operations)
        image_resize = cv2.resize(image, (int(ow/2), int(oh/2)), interpolation=cv2.INTER_NEAREST)
        frame_result = cv2.cvtColor(image_resize, cv2.COLOR_RGB2LAB)
        frame_result = cv2.GaussianBlur(frame_result, (3, 3), 3)
        mask = cv2.inRange(frame_result, tuple(range_['min']), tuple(range_['max']))  # 对原图像和掩模进行位运算(perform bitwise operations on the original image and the mask)
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (self.kernel_erode, self.kernel_erode)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (self.kernel_dilate, self.kernel_dilate)))

        # 将处理后的二值化图像发布(publish the processed binarized image)
        rgb_image = cv2.resize(dilated, (ow, oh))
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_GRAY2RGB).tobytes()
        ros_image.data = rgb_image
        # ros_image.height = int(oh/2)
        # ros_image.width = int(ow/2)
        # ros_image.step = ros_image.width * 3
        self.result_image_pub.publish(ros_image)

        # 发布缩放后的原图(publish the scaled original image)
        # rgb_image = image_resize.tostring()
        # ros_image.data = rgb_image
        # self.source_image_pub.publish(ros_image)


    def enter_srv_callback(self, _):
        """
        APP 进入功能(APP enter function)
        注册对相机的订阅(subscribe to camera feeds)
        """
        rospy.loginfo('enter')
        try:
            self.image_sub.unregister() # 总时尝试注销一下对相机的订阅, 避免重复订阅造成的资源占用和异常(periodically attempt to unsubscribe from camera feeds to avoid resource consumption and anomalies caused by redundant subscriptions)
        except Exception as e:
            rospy.logerr(str(e))
        source_image_topic = rospy.get_param('~source_image_topic', '/rgbd_cam/color/image_rect_color')
        self.image_sub = rospy.Subscriber(source_image_topic, Image, self.image_callback, queue_size=1)
        return [True, '']
    

    def exit_srv_callback(self, _):
        """
        APP退出功能(APP exit function)
        会注销掉相机的订阅, 并停止心跳定时器(it will unsubscribe from the camera feed and stop the heartbeat timer)
        """
        rospy.loginfo('exit')
        try:
            self.image_sub.unregister() # 注销订阅(unsubscribe from subscription)
        except Exception as e:
            rospy.logerr(str(e))
        try:
            self.heartbeat_timer.shutdown() # 注销订阅(unsubscribe from subscription)
        except Exception as e:
            rospy.logerr(str(e))
        return [True, '']
    
    
    def set_running_srv_callback(_):
        """
        本来时用来控制运行或者暂停运行的, 但是这里废弃了(originally intended to control running or pausing operations, but it has been deprecated here)
        为了保持兼容性留着这个空函数(to maintain compatibility, this empty function is retained)
        """
        rospy.loginfo("set running called")
        return [True, 'set_running']
    
    
    def save_to_disk_srv_callback(self, _):
        """
        保存当前的阈值列表到硬盘(sd卡)中(save the current list of thresholds to the hard disk (SD card))
        """
        rospy.loginfo("saving thresholds to dist")
        # 将阈值列表放到对应格式的字典中(put the list of thresholds into a dictionary corresponding to the format)
        #cf = {"color_range_list": self.color_ranges} 
        #rospy.loginfo(cf)
        # 字典转为yaml格式字符串写入文件中(convert the dictionary to a YAML format string and write it into a file)
        config = rospy.get_param('/config')
        with open(self.config_file_path, 'w') as f:
            yaml.dump(config, f)
        return TriggerResponse(success=True)
    
    
    def get_range_srv_callback(self, msg):
        """
        获取指定颜色的阈值(get specified color threshold)
        """
        rospy.loginfo(msg)
        rsp = GetRangeResponse()
        ranges = rospy.get_param('/config/lab', self.color_ranges) # 从参数服务器获取所有颜色阈值的了列表(get the list of all color thresholds from parameter server)
        if msg.color_name in ranges: # 在阈值列表中有要获取的颜色(there is a color to be retrieved in the threshold list)
            rsp.success = True
            # 将阈值填入返回结果中(insert the thresholds into the returned result)
            rsp.min = ranges[msg.color_name]['min']
            rsp.max = ranges[msg.color_name]['max']
        else:
            rsp.success = False
            color_ranges = ranges
        return rsp
    
    
    def change_range_srv_callback(self, msg):
        """
        修改当前的颜色阈值(modify current color threshold)
        就是控制结果画面改变(change the control result frame)
        :param msg: msg.min 阈值下限， msg.max 阈值上限(msg.min lower limit: msg.min, upper limit: msg.max)
        """
        rospy.loginfo(msg)
        self.current_range = dict(min=list(msg.min), max=list(msg.max))
        return ChangeRangeResponse(success=True)
    
    
    def stash_range_srv_callback(self, msg):
        """
        暂存当前阈值(temporarily store the current threshold)
        修改指定颜色的阈值为当前阈值(就是结果画面对应的阈值)(modify the threshold of the specified color to the current threshold (i.e., the threshold corresponding to the resulting image))
        :param msg: msg.color_name 要修改的颜色名称(color name to be modified)
        """
        rospy.loginfo(msg)
        ranges = rospy.get_param('/config/lab', self.color_ranges) # 获取当前的颜色阈值列表(get the current list of color threshold)
        ranges[msg.color_name] = self.current_range  # 修改指定颜色的阈值(modify specified color threshold)
        rospy.set_param('/config/lab', ranges) # 存回参数服务器(store parameter server)
        self.color_ranges = ranges
        return StashRangeResponse(success=True)
    
    
    def get_all_color_name_srv_callback(self, msg):
        """
        获取保存的全部颜色的名称(retrieve the names of all saved colors)
        """
        rospy.loginfo(msg)
        ranges = rospy.get_param('/config/lab', self.color_ranges) # 从参数服务器获取所有颜色阈值(get all color threshold from parameter server)
        color_names = list(ranges.keys()) # 取键名, 就是颜色名称(retrieve the key names, which are the color names)
        return GetAllColorNameResponse(color_names=color_names)
    
    def heartbeat_timeout_callback(self, _):
        """
        心跳超时回调(heartbeat timeout callback)
        """
        rospy.loginfo("heartbeat timeout. exiting...")
        self.exit_srv_callback(None)  # 停止功能的运行(stop function running)
    
    def heartbeat_srv_callback(self, msg):
        """
        心跳服务回调(heartbeat service callback)
        :param msg: msg.data 控制起跳或停跳, 为True时起跳(start or stop control; True for starting)
        """
        try:
            # 无论起跳还是停跳都将上次的定时器停止(stop the previous timer regardless of starting or stopping)
            self.heartbeat_timer.shutdown()
        except Exception as e:
            rospy.logerr(str(e))
        rospy.logdebug("Heartbeat, " + str(msg))
        if msg.data:
            # 起跳，建立新的定时器(start by establishing a new timer)
            self.heartbeat_timer = rospy.Timer(rospy.Duration(5), self.heartbeat_timeout_callback, oneshot=True)
        return SetBoolResponse(success=True)
    

if __name__ == '__main__':
    try:
        lab_conf_manager_node = LabConfigManagerNode('lab_config_manager')
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
        sys.exit(0)
