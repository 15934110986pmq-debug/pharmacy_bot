#!/usr/bin/python3
# coding=utf8
# Date:2022/03/03
# Author:Aiden-Wei

# 从深度相机中订阅深度图以及彩色图(subscribe to both the depth map and the color map from the depth camera)
# 合成rgbd, 再转成点云，对点云裁剪(combine rgbd, convert it to point cloud, perform clipping on the point cloud)
# 去除最大平面，去除外围边缘点(remove the largest plane and eliminate points on the outer edges)

import rospy
import numpy as np
import open3d as o3d
import message_filters
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from hiwonder_interfaces.msg import MultiRawIdPosDur
from jetarm_sdk import pid, bus_servo_control

haved_add = False
haved_add_ = False
get_point = False
target_cloud = o3d.geometry.PointCloud() # 要显示的点云(the point cloud to be displayed)
process_cloud = o3d.geometry.PointCloud() # 要显示的点云(the point cloud to be displayed)

# 启用cuda来加速部分open3d程序(enable CUDA to accelerate certain Open3D functions)
device = o3d.core.Device('CUDA:0')

# 相机内外参，用来将rgbd转为point cloud(camera intrinsic and extrinsic parameters are used to convert RGBD data to a point cloud)
intrinsic = o3d.core.Tensor([[477, 0,   316], 
                             [0,   477, 189], 
                             [0,   0,   1]])

extrinsic = o3d.core.Tensor([[1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.17], 
                             [0.0, 0.0, 1.0, 0.413], 
                             [0.0, 0.0, 0.0, 1.0]])
###################################
# 裁剪roi(crop roi)
# x, y, z
roi = np.array([
    [-0.1, -0.25, 0],
    [-0.1, -0.05, 0],
    [0.1,  -0.05, 0],
    [0.1,  -0.25, 0]], 
    dtype = np.float64)

vol = o3d.visualization.SelectionPolygonVolume()
# 裁剪z轴，范围(crop z-axis, range)
vol.orthogonal_axis = 'Z'
vol.axis_max = 0.26
vol.axis_min = -0.26
vol.bounding_polygon = o3d.utility.Vector3dVector(roi)
def multi_callback(ros_rgb_image, ros_depth_image):
    global get_point
    
    try:
        # ros格式转为numpy(convert ros format to numpy)
        image_rgb = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
        depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
        # 统一分辨率为640x400(standardize the resolution to 640x400)
        image_rgb = image_rgb[40:440, ]
        
        # numpy格式转为open3d格式(convert numpy format to open3d format)
        o3d_image_rgb = o3d.t.geometry.Image(o3d.core.Tensor(image_rgb, dtype=o3d.core.Dtype.UInt8, device=device))
        o3d_image_depth = o3d.t.geometry.Image(o3d.core.Tensor(np.ascontiguousarray(depth_image), dtype=o3d.core.Dtype.Float32, device=device))       
        
        # rgb depth ---> rgbd
        rgbd = o3d.t.geometry.RGBDImage(o3d_image_rgb, o3d_image_depth)
        
        # rgbd ---> point cloud
        point_cloud = o3d.t.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic, extrinsic)
        
        # 取出gpu的数据(retrieve data from the GPU)
        pc = point_cloud.to_legacy()

        # 裁剪(crop)
        roi_pc = vol.crop_point_cloud(pc)
        
        if len(roi_pc.points) > 0:
            # 去除最大平面，即地面, 距离阈4mm，邻点数，迭代次数(remove the largest plane, i.e., the ground, with a distance threshold of 4mm, and specified numbers of neighbors and iterations)
            plane_model, inliers = roi_pc.segment_plane(distance_threshold=0.004,
                     ransac_n=5,
                     num_iterations=50)
            
            # 保留内点(retain the inliers)
            inlier_cloud = roi_pc.select_by_index(inliers, invert=True)
            target_cloud.points = inlier_cloud.points
            target_cloud.colors = inlier_cloud.colors
        else:
            target_cloud.points = roi_pc.points
            target_cloud.colors = roi_pc.colors
        # 转180度方便查看(rotate 180 degrees for easier viewing)
        target_cloud.transform(np.asarray([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]))

        get_point = True 
    except BaseException as e:
        print('callback error:', e)

def remove_canny(kd, cloud_numpy, color_numpy, rgb_d_threshold=10, point_num_threshold=3, nb_neighbors=5):

    remain = []
    points = len(cloud_numpy) 
    if points != 0:
        try:
            for j in range(points):
                idx = kd.search_knn_vector_3d(cloud_numpy[j], nb_neighbors)[1]
                if len(idx) != 0:
                    center = color_numpy[idx[0], :]
                    count = 0
                    near_point = idx[1:]
                    for i in range(len(near_point)):
                        rgb_d = (center - color_numpy[near_point[i], :])*255
                        rgb_abs = np.maximum(rgb_d, -rgb_d)
                        max_ = np.max(rgb_abs)
                        if max_ < rgb_d_threshold:
                            count += 1
                        if (i > 1 and count < 1) or count > point_num_threshold:
                            break
                    
                    if count <= point_num_threshold:
                        remain.append(idx[0])
        except BaseException as e:
            print('remove canny error:', e)

    color_numpy_copy = None
    cloud_numpy_copy = None
    if remain != []:
        color_numpy_copy = (np.delete(color_numpy, remain, axis=0)).reshape(-1, 3)
        cloud_numpy_copy = (np.delete(cloud_numpy, remain, axis=0)).reshape(-1, 3)
    
    return cloud_numpy_copy, color_numpy_copy

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
    rospy.sleep(1)
    rospy.wait_for_service('/rgbd_cam/set_ldp')
    rospy.ServiceProxy('/rgbd_cam/set_ldp', SetBool)(False)
   
    # 创建可视化窗口(create a visualization window)
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='point cloud', width=640, height=480, visible=1)

    vis1 = o3d.visualization.Visualizer()
    vis1.create_window(window_name='process cloud', width=640, height=480, visible=1)
    
    rgb_sub = message_filters.Subscriber('/rgbd_cam/color/image_raw', Image)
    depth_sub = message_filters.Subscriber('/rgbd_cam/depth/image_raw', Image)
    
    # 同步时间戳, 时间允许有误差在0.02s(synchronize timestamps, allowing for a time error of up to 0.02 seconds)
    sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 2, 0.015)
    sync.registerCallback(multi_callback) #执行反馈函数(execute feedback function)
    
    print('running ...')

    t0 = rospy.get_time()
    while not rospy.is_shutdown():
        if not haved_add and get_point:
            vis.add_geometry(target_cloud)
            haved_add = True
        if haved_add:
            if get_point:
                get_point = False
                # 刷新(refresh)
                if len(target_cloud.points) != 0:
                    sample_cloud = target_cloud.voxel_down_sample(voxel_size=0.003)
                    if len(sample_cloud.points) != 0:
                        kd_tree = o3d.geometry.KDTreeFlann(sample_cloud)
                        cloud_numpy = np.asarray(sample_cloud.points)
                        color_numpy = np.asarray(sample_cloud.colors)
                        numpy_cloud, numpy_color = remove_canny(kd_tree, cloud_numpy, color_numpy)
                        process_cloud.points = o3d.utility.Vector3dVector(numpy_cloud)
                        process_cloud.colors = o3d.utility.Vector3dVector(numpy_color)
                        if not haved_add_:
                            vis1.add_geometry(process_cloud)
                            haved_add_ = True
                        if haved_add_:
                            vis1.update_geometry(process_cloud)
                            vis1.poll_events()
                            vis1.update_renderer()
                    else:
                        rospy.sleep(0.01)
                else:
                    rospy.sleep(0.01)
                vis.update_geometry(target_cloud)
                vis.poll_events()
                vis.update_renderer()
                fps = round(1.0/(rospy.get_time() - t0), 2)
                print('\r', 'FPS: '+ str(fps), end='')
            else:
                rospy.sleep(0.001)
        else:
            rospy.sleep(0.01)
        t0 = rospy.get_time()

