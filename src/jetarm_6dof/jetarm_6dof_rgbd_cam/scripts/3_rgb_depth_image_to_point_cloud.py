#!/usr/bin/python3
# coding=utf8
# Date:2022/03/03
# Author:Aiden-Wei

# 从深度相机中订阅深度图以及彩色图(subscribe to both the depth map and the color map from the depth camera)
# 合成rgbd, 再转成点云，对点云裁剪(combine RGB and depth data to form an RGBD image, then convert it to a point cloud, and finally, perform clipping on the point cloud)

import cv2
import rospy
import numpy as np
import message_filters
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from vision_utils import fps
import queue
from hiwonder_interfaces.msg import MultiRawIdPosDur
from jetarm_sdk import pid, bus_servo_control
import open3d as o3d

haved_add = False
get_point = False
target_cloud = o3d.geometry.PointCloud() # 要显示的点云(the point cloud to be displayed)

# 启用cuda来加速部分open3d程序(enable CUDA to accelerate certain Open3D functions)
device = o3d.core.Device('CUDA:0')

# 相机内外参，用来将rgbd转为point cloud(camera intrinsic and extrinsic parameters are used to convert RGBD data to a point cloud)
intrinsic = o3d.core.Tensor([[477, 0,   316], 
                             [0,   477, 189], 
                             [0,   0,   1]])

extrinsic = o3d.core.Tensor([[1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.17], 
                             [0.0, 0.0, 1.0, 0.433], 
                             [0.0, 0.0, 0.0, 1.0]])
###################################
# 裁剪roi(crop roi)
# x, y, z
'''
roi = np.array([
    [-0.1, -0.25, 0],
    [-0.1, -0.05, 0],
    [0.1,  -0.05, 0],
    [0.1,  -0.25, 0]], 
    dtype = np.float64)
'''
roi = np.array([
    [-0.05, -0.20, 0],
    [-0.05, -0.05, 0],
    [0.1,  -0.05, 0],
    [0.1,  -0.20, 0]], 
    dtype = np.float64)

vol = o3d.visualization.SelectionPolygonVolume()
# 裁剪z轴，范围(crop z-axis, range)
vol.orthogonal_axis = 'Z'
vol.axis_max = 0.26
vol.axis_min = -0.26
vol.bounding_polygon = o3d.utility.Vector3dVector(roi)
intrinsic = o3d.camera.PinholeCameraIntrinsic(640, 400, 477, 477, 316, 189)#356, 260
FPS = None
def multi_callback(ros_rgb_image, ros_depth_image):
    global get_point, t0
    
    try:
        # ros格式转为numpy(convert ros format to numpy)
        image_rgb = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
        depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
        # 统一分辨率为640x400(standardize the resolution to 640x400)
        image_rgb = image_rgb[40:440, ]
      
        o3d_image_rgb = o3d.geometry.Image(image_rgb)
        o3d_image_depth = o3d.geometry.Image(np.ascontiguousarray(depth_image))        
        
        # rgbd --> point_cloud
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_image_rgb, o3d_image_depth, convert_rgb_to_intensity=False)
        # cpu占用大 (high CPU usage)
        pc = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)#, extrinsic=extrinsic)
        
        '''
        # numpy格式转为open3d格式(convert numpy format to open3d format)
        o3d_image_rgb = o3d.t.geometry.Image(o3d.core.Tensor(image_rgb, dtype=o3d.core.Dtype.UInt8, device=device))
        o3d_image_depth = o3d.t.geometry.Image(o3d.core.Tensor(np.ascontiguousarray(depth_image), dtype=o3d.core.Dtype.Float32, device=device))       
        
        # rgb depth ---> rgbd
        rgbd = o3d.t.geometry.RGBDImage(o3d_image_rgb, o3d_image_depth)
        
        # rgbd ---> point cloud
        point_cloud = o3d.t.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)#, extrinsic)
        
        # 取出gpu的数据(retrieve data from the GPU)
        pc = point_cloud.to_legacy()
        '''
        # 裁剪(crop)
        roi_pc = pc#vol.crop_point_cloud(pc)
        
        target_cloud.points = roi_pc.points
        target_cloud.colors = roi_pc.colors
        # 转180度方便查看(rotate 180 degrees for easier viewing)
        target_cloud.transform(np.asarray([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]))

        get_point = True
        fps = int(1.0/(rospy.get_time() - t0))
        print('\r', 'FPS: ' + str(fps), end='')
    except BaseException as e:
        print('callback error:', e)
    t0 = rospy.get_time()

if __name__ == "__main__":
    print('''
    *********************************************
    *                                           *
    * 此程序需要订阅深度摄像头节点,开启前请确保 *
    * 已开启摄像头节点，通过rostopic list可查看 *
    * 是否有usb_cam相关节点,成功运行可看到终端  *
    * running ...                               *
    *                                           * 
    *********************************************
    ''')
    rospy.init_node('point_cloud', anonymous=True)
    servos_pub = rospy.Publisher('/controllers/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(2)
    bus_servo_control.set_servos(servos_pub, 1000, ((1, 500), (2, 560), (3, 160), (4, 80), (5, 500), (10, 200)))
    rospy.sleep(3)
        
    rospy.wait_for_service('/rgbd_cam/set_ldp')
    rospy.ServiceProxy('/rgbd_cam/set_ldp', SetBool)(False)
    rospy.sleep(1)
    
    t0 = rospy.get_time()
    
    # 创建可视化窗口(create a visualization window)
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='point cloud', width=640, height=480, visible=1)

    rgb_sub = message_filters.Subscriber('/rgbd_cam/color/image_raw', Image)
    depth_sub = message_filters.Subscriber('/rgbd_cam/depth/image_raw', Image)
    
    # 同步时间戳, 时间允许有误差在0.02s(synchronize timestamps, allowing for a time error of up to 0.02 seconds)
    sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 2, 0.015)
    sync.registerCallback(multi_callback) #执行反馈函数(execute feedback function)
    
    print('running ...')
    while not rospy.is_shutdown():
        if not haved_add and get_point:
            vis.add_geometry(target_cloud)
            haved_add = True
        if haved_add:
            if get_point:
                get_point = False
                # 刷新(refresh)
                vis.update_geometry(target_cloud)
                vis.poll_events()
                vis.update_renderer()
            else:
                rospy.sleep(0.001)
        else:
            rospy.sleep(0.01)

