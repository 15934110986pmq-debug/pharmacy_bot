#!/usr/bin/python3
# coding=utf8
# @data:2023/01/31
# @author:aiden
# 夹取转动策略(gripping rotation strategy)
import cv2
import math
import numpy as np

def calculate_e_distance(point1, point2):
    # 计算两个点间的欧式距离(calculate the Euclidean distance between two points)
    e_distance = round(math.sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2)), 5)                
        
    return e_distance

def cal_intersect_area(image, rect1, rect2):
    '''
    计算两个矩形的碰撞面积(calculate two rectangle collision area)
    :param rect1:
    :param rect2:
    :return:
    '''
    contour = cv2.rotatedRectangleIntersection(rect1, rect2)[1]

    if contour is not None:
        '''
        img = np.zeros((640, 640), np.uint8)
        '''
        box = []
        for i in contour:
            box.append(list(map(int, i[0])))
        rect_box = [np.array(box)]
         
        # cv2.drawContours(image, rect_box, -1, (255, 0, 0), 2, cv2.LINE_AA) 
        area = cv2.contourArea(contour)
        return area
    else:
        return None

def collision_detect(image, target, points, rect1, rect2, collision_radius):
    '''
    计算两个矩形和其他矩形是否有碰撞(calculate whether there is a collision between two rectangles and other rectangles)
    :param target: 当前物体颜色(current object color)
    :param points: 所有物体信息，包含坐标和颜色(all color information, including coordinates and color)
    :param rect1: 在当前物件中心创建的用来检测碰撞的矩形1(rectangle 1 created around the current object center for collision detection)
    :param rect2: 在当前物件中心创建的用来检测碰撞的矩形2, 和矩形1互相垂直(rectangle 2 created around the current object center for collision detection, perpendicular to rectangle 1)
    :return:
    '''
    only_one = True
    collision_rect_list = []
    for i in points:
        if i != target: # 除当前物体的外的其他物体(other objects except the current object)
            only_one = False
            point1 = [points[target][0].x, points[target][0].y]
            point2 = [points[i][0].x, points[i][0].y]
            if calculate_e_distance(point1, point2) < collision_radius:  # 计算当前物体和其他物体间的距离(calculate the distance between the current object and other objects)
                # 构建被碰撞的物体(construct the collided object)
                rect3 = ((points[i][0].x, points[i][0].y),
                         (points[i][1].width, points[i][1].height),
                         points[i][2])
                # box = np.int0(cv2.boxPoints(rect3))
                # cv2.drawContours(image, [box], -1, (255, 255, 255), 1, cv2.LINE_AA)
                # 计算当前物体和碰撞物体的面积(calculate the area of the current object and the collided object)
                area1 = cal_intersect_area(image, rect1, rect3) 
                area2 = cal_intersect_area(image, rect2, rect3)
                # print('area', area1, area2, rect3)
                if area1 is not None:
                    if rect1 not in collision_rect_list:
                        collision_rect_list.append(rect1)
                if area2 is not None:
                    if rect2 not in collision_rect_list:
                        collision_rect_list.append(rect2)
    if only_one:
        return None
    else:
        if collision_rect_list == []:
            return None
        else:
            return collision_rect_list

def rotate(ps, m):
    pts = np.float32(ps).reshape([-1, 2])  # 要映射的点(points to be mapped)
    pts = np.hstack([pts, np.ones([len(pts), 1])]).T
    target_point = np.dot(m, pts).astype(np.int)
    target_point = np.where(target_point > 0, target_point, 0)
    target_point = [[target_point[0][x], target_point[1][x]] for x in range(len(target_point[0]))]
    
    return target_point

def rotate_point(center_point, corners, angle):
    '''
    获取一组点绕一点旋转后的位置(get the positions of a set of points after rotating around a point)
    :param center_point:
    :param corners:
    :param angle:
    :return:
    '''
    # points [[x1, y1], [x2, y2]...]
    # 角度(angle)
    M = cv2.getRotationMatrix2D((center_point[0], center_point[1]), angle, 1)
    out_points = rotate(corners, M)
    
    return out_points

def get_longer_side(side, rect):
    '''
    获取矩形较长一边(get the longer side of a rectangle)
    :param side:
    :param rect:
    :return:
    '''
    point1 = rect[0]
    point2 = rect[1]
    point3 = rect[3]
    dis = round(calculate_e_distance(point1, point2))
    if dis == side:
        return point1, point2
    else:
        return point1, point3

def get_cross_angle1(l1, l2):
    '''
    计算两条直线的夹角，取较小(calculate the angle between two lines, taking the smaller one)
    :param l1:
    :param l2:
    :return:
    '''
    a1 = math.atan2(l1[0][0] - l1[1][0], l1[0][1] - l1[1][1])  # 顺时针旋转到和x轴重合的角度(rotate clockwise to align with the x-axis)

    a2 = math.atan2(l2[0][0] - l2[1][0], l2[0][1] - l2[1][1])

    angle = int(round(math.degrees(a1 - a2)))
    if 180 >= angle > 90:
        angle = 180 - angle
    elif 270 >= angle > 180:
        angle = angle - 180
    elif 360 >= angle > 270:
        angle = angle - 270

    if -180 <= angle < -90:
        angle = 180 + angle
    elif -270 <= angle < -180:
        angle = angle + 180
    elif -360 <= angle < -270:
        angle = angle + 270

    return angle

def get_cross_angle(v1, v2):
    # 2个向量模的乘积(product of the magnitudes of two vectors)
    TheNorm = np.linalg.norm(v1)*np.linalg.norm(v2)
    # 叉乘(cross product)
    rho = np.rad2deg(np.arcsin(np.cross(v1, v2)/TheNorm))
    # 点乘(dot product)
    theta = np.rad2deg(np.arccos(np.dot(v1,v2)/TheNorm))
    
    if rho < 0:
        angle1 = -theta
    else:
        angle1 = theta

    angle2 = 180 - abs(angle1)
    if angle1 == 0:
        angle = 0
    elif angle2 > abs(angle1):
        angle = angle1
    else:
        angle = -angle2*abs(angle1)/angle1

    return int(angle)

def get_yaw_angle(image, target, points, world_point, transform):
    '''
    获取较小的转动角(get the smaller rotation angle)
    :param target:
    :param points:
    :param world_x:
    :param world_y:
    :return:
    '''
    world_x = world_point[0]
    world_y = world_point[1]

    target_point = points[target]
    x = target_point[0].x
    y = target_point[0].y
    angle = target_point[2]
    
    if target_point[-1] == 0.03:
        # 将夹持器假设成一个矩形来做碰撞检测(assuming the gripper as a rectangle for collision detection)
        rect_width = 30  # 单位像素(per pixel)
        rect_height = 210  # 单位像素(per pixel)
        rect_width_half = int(rect_width / 2)
        rect_height_half = int(rect_height / 2)

        collision_radius = max(rect_width, rect_height)
    else:
        # 将夹持器假设成一个矩形来做碰撞检测(assuming the gripper to be a rectangle for collision detection)
        rect_width = 30  # 单位像素(per pixel)
        rect_height = 250  # 单位像素(per pixel)
        rect_width_half = int(rect_width / 2)
        rect_height_half = int(rect_height / 2)

        collision_radius = max(rect_width, rect_height)

    # 创建两个矩形(create two rectangles)
    rect1 = ((x, y), (rect_width, rect_height), angle)
    rect2 = ((x, y), (rect_height, rect_width), angle)

    # 获得被碰撞的矩形(get the collided rectangle)
    result = collision_detect(image, target, points, rect1, rect2, collision_radius)

    box_draw = None
    rotation_angle = None

    left_up = [x - rect_width_half, y - rect_height_half]
    right_up = [x + rect_width_half, y - rect_height_half]
    left_down = [x - rect_width_half, y + rect_height_half]
    right_down = [x + rect_width_half, y + rect_height_half]
    
    # 获得矩形的四个角点(get the four corner points of the rectangle)
    out_points1 = rotate_point([x, y], [left_up, right_up, right_down, left_down], -angle)
    out_points2 = rotate_point([x, y], [left_up, right_up, right_down, left_down], -angle - 90)
    
    # 获取矩形的较长边(get the longer side of the rectangle)
    side1 = get_longer_side(collision_radius, out_points1)
    side2 = get_longer_side(collision_radius, out_points2)
    
    point1 = np.array([[side1[0][0], side1[0][1], target_point[-1]]], dtype=np.double)
    world_point1 = transform(point1)
    world_point1 = [world_point1[1], -world_point1[0]] # 画面显示的x轴和y轴不一样(the x-axis and y-axis displayed on the screen are different)
    
    point2 = np.array([[side2[0][0], side2[0][1], target_point[-1]]], dtype=np.double)
    world_point2 = transform(point2)
    world_point2 = [world_point2[1], -world_point2[0]]
    
    if world_y != 0:
        angle1 = get_cross_angle([-world_x, (world_x*world_x + world_y*world_y)/world_y - world_y], [world_point1[0] - world_x, world_point1[1] - world_y])
        angle2 = get_cross_angle([-world_x, (world_x*world_x + world_y*world_y)/world_y - world_y], [world_point2[0] - world_x, world_point2[1] - world_y])
    else:
        angle1 = get_cross_angle([-world_x, world_y - 1], [world_point1[0] - world_x, world_point1[1] - world_y])
        angle2 = get_cross_angle([-world_x, world_y - 1], [world_point2[0] - world_x, world_point2[1] - world_y])
    # print(angle1, angle2, result)
    if result is not None:
        if len(result) == 1:
            if result[0] != rect1:
                box_draw = np.array(out_points1)
                rotation_angle = angle1
            else:
                box_draw = np.array(out_points2)
                rotation_angle = angle2
            # points = rotate_point(side4[0], side4[1], -rotation_angle)
            # if image is not None:
                # cv2.line(image, tuple(side4[0]), (points[0][0], points[0][1]), (0, 255, 0), 1, cv2.LINE_AA)
    else:
        if abs(angle1) > abs(angle2):
            box_draw = np.array(out_points2)
            rotation_angle = angle2
        else:
            box_draw = np.array(out_points1)
            rotation_angle = angle1
        # points = rotate_point(side4[0], side4[1], -rotation_angle)
        # if image is not None:
            # cv2.line(image, tuple(side4[0]), (points[0][0], points[0][1]), (0, 255, 0), 1, cv2.LINE_AA)
    
    if image is not None:
        #cv2.drawContours(image, [np.array([left_up, right_up, right_down, left_down])], -1, (0, 0, 255), 1, cv2.LINE_AA)
        #cv2.line(image, tuple(side3[0]), tuple(side3[1]), (255, 255, 0), 1, cv2.LINE_AA)
        #cv2.line(image, tuple(side4[0]), tuple(side4[1]), (0, 255, 0), 1, cv2.LINE_AA)
        #cv2.line(image, tuple(side1[0]), tuple(side1[1]), (255, 255, 0), 2, cv2.LINE_AA)
        #cv2.line(image, tuple(side2[0]), tuple(side2[1]), (255, 255, 0), 2, cv2.LINE_AA)
        if box_draw is not None:
            cv2.drawContours(image, [box_draw], -1, (0, 255, 255), 1, cv2.LINE_AA)
    # cv2.drawContours(image, [np.array(out_points1)], -1, (0, 255, 255), 1, cv2.LINE_AA)
    # cv2.drawContours(image, [np.array(out_points2)], -1, (0, 0, 255), 1, cv2.LINE_AA)
    # print(rotation_angle) 
    
    return rotation_angle, box_draw
