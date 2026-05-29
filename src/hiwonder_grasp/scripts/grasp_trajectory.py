#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/03/01
# @author:aiden
# 简易路径规划(brief path planning)
import rospy
from jetarm_sdk import common, bus_servo_control, gripper_control
from jetarm_kinematics.kinematics_control import set_pose_target

class Grasp():
    def __init__(self, joints_pub, grasp_pub):
        self.target1 = None
        self.target2 = None
        self.target3 = None
        self.grasp = None
        self.approach = None
        self.retreat = None
        self.position = None
        self.pitch = None
        self.joints_pub = joints_pub
        self.grasp_pub = grasp_pub

        # 误差补偿(error compensation)
        #config = common.get_yaml_data('/home/hiwonder/jetarm/src/jetarm_bringup/config/config.yaml')

        self.x_p_offset = 0 #config['x+_offset']
        self.x_n_offset = 0 #config['x-_offset']
        self.y_p_offset = 0 #config['y+_offset']
        self.y_n_offset = 0 #config['y-_offset'] 

    def update_param(self):
        # 误差补偿(error compensation)
        pass
        #config = common.get_yaml_data('/home/hiwonder/jetarm/src/jetarm_bringup/config/config.yaml')

        #self.x_p_offset = config['x+_offset']
        #self.x_n_offset = config['x-_offset']
        #self.y_p_offset = config['y+_offset']
        #self.y_n_offset = config['y-_offset']       
        
    def set_target(self, grasp):
        t0 = rospy.get_time()
        self.position = grasp.position
        self.pitch = grasp.pitch
        self.approach = grasp.grasp_approach
        self.retreat = grasp.grasp_retreat
        self.grasp = grasp
      
        if self.position.x > 0:
            self.position.x += self.x_p_offset
        elif self.position.x < 0:
            self.position.x += self.x_n_offset
        if self.position.y > 0:
            self.position.y += self.y_p_offset
        elif self.position.y < 0:
            self.position.y += self.y_n_offset

        optimal_solution_list = []
        target2 = set_pose_target([self.position.x , self.position.y, self.position.z], self.pitch, [-90, 90], 1)
        #return optimal_solution_list
        if target2[1] == []:
            common.loginfo('unable to reach target')
            return optimal_solution_list
        
        target1 = set_pose_target([self.position.x + self.approach.x, 
                                   self.position.y + self.approach.y, 
                                   self.position.z + self.approach.z], self.pitch, [-90, 90], 1)
        if target1[1] == []:
            common.loginfo('unable to approach target')
            return optimal_solution_list

        target3 = set_pose_target([self.position.x + self.retreat.x, 
                                   self.position.y + self.retreat.y, 
                                   self.position.z + self.retreat.z], self.pitch, [-90, 90], 1)
        if target3[1] == []:
            common.loginfo('unable to retreat')
            return optimal_solution_list

        self.target1 = target1
        self.target2 = target2
        self.target3 = target3
        # print(1111, self.target1, self.target2, self.target3)
        optimal_solution_list = [target1[-1], target1[1], target1[1], target2[1], target2[1], target3[1]]
        common.loginfo('planning cost time: %s'%(rospy.get_time() - t0))
        
        return optimal_solution_list

    def get_queue(self):
        # 夹取和放置的动作构成(the constitute of grasping and placement motion)
        queue_list = [
                [self.move_toward, 1000],
                [self.move_approach, 1000],
                [self.gripper_align, 500],
                [self.move_target, 1000],
                # [self.gripper_move_, 150],
                # [self.gripper_move, 300],

                # [self.gripper_move_, 150],
                [self.gripper_move, 300],
                [self.move_retreat, 500],
                [self.move_toward_init, 1000]
                ]
        if self.grasp.mode == 'place':
            queue_list = [
                    [self.move_toward, 1000],
                    [self.move_approach,1000],
                    [self.gripper_align, 500],
                    [self.move_target, 1000],
                    [self.gripper_move_, 150],
                    # [self.gripper_align, 500],
                    [self.gripper_move, 300],
                    [self.move_retreat, 1000],
                    [self.move_toward_init, 1000],
                    [self.move_init, 1000]
                    ]
        elif self.grasp.mode == 'garbage_place':
            queue_list = [
                    [self.move_toward, 1000],
                    [self.move_approach, 1000],
                    [self.move_target, 1000],
                    [self.gripper_move, 300],
                    [self.move_retreat, 500],
                    [self.move_toward_init, 1000],
                    [self.move_init, 1000]
                    ]
        common.loginfo('mode:%s'%self.grasp.mode)
        return queue_list

    def move_toward(self, t=1000): 
        # 移到朝向物体(move towards the object's direction)
        if self.target2 is not None:
            servo_data = self.target2[1][::-1]
            bus_servo_control.set_servos(self.joints_pub, t, ((10, self.grasp.pre_grasp_posture), (1, servo_data[-1]))) 
            rospy.sleep(t/1000.0)

    def move_approach(self, t=1500): 
        # 移到物体上方(move above the object)
        # 这里再计算一次是因为位置更新了，如果有多解涉及到取变化最小的解(The reason for recalculating is that the position has been updated, and if there are multiple solutions, the one with the smallest change should be selected)
        self.target1 = set_pose_target([self.position.x + self.approach.x, 
                                        self.position.y + self.approach.y, 
                                        self.position.z + self.approach.z], self.pitch, [-90, 90], 1)
        if self.target1 != []:
            servo_data = self.target1[1]
            bus_servo_control.set_servos(self.joints_pub, t, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3])))
            rospy.sleep(t/1000.0 + 0.1)
    
    def move_target(self, t=1800):
        # 移到目标位置(move to the target position)
        self.target2 = set_pose_target([self.position.x , self.position.y, self.position.z], self.pitch, [-90, 90], 1)
        if self.target2 != []:
            servo_data = self.target2[1]
            bus_servo_control.set_servos(self.joints_pub, t, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3])))
            rospy.sleep(t/1000.0 + 0.5)

    def gripper_move_(self, t=150):
        # 夹持器开合一点点(the gripper opens or closes slightly)
        # gripper_control.set_grasp(self.grasp_pub, t, self.grasp.pre_grasp_posture - 30)
        rospy.sleep(t/1000.0 + 0.1)

    def gripper_align(self, t=500):
        # 夹持器对齐(the gripper aligns)
        bus_servo_control.set_servos(self.joints_pub, t, ((5, 500 + int(1000*(self.grasp.align_angle + self.target2[3][-1])/240.0)), ))
        rospy.sleep(t/1000.0 + 0.3)

    def gripper_move(self, t=500):
        # 夹持器开合(the gripper opens and closes)
        gripper_control.set_grasp(self.grasp_pub, t, self.grasp.grasp_posture)
        rospy.sleep(t/1000.0 + 0.3)
    
    def move_retreat(self, t=1000):
        # 远离物体(move away from the object)
        self.target3 = set_pose_target([self.position.x + self.retreat.x, 
                                        self.position.y + self.retreat.y, 
                                        self.position.z + self.retreat.z], self.pitch, [-90, 90], 1)
        if self.target3[1] != []:
            servo_data = self.target3[1]
            bus_servo_control.set_servos(self.joints_pub, t, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3])))
            rospy.sleep(t/1000.0)
        
    
    def move_toward_init(self, t=1000): 
        # 移到朝向物体(move towards the object's direction)
        # print(2222, self.target1, self.target2, self.target3)
        bus_servo_control.set_servos(self.joints_pub, t, ((5, 500), (4, 140), (3, 70), (2, 610)))

    def move_init(self, t=1000): 
        pass
        #bus_servo_control.set_servos(self.joints_pub, t, ((1, 500),))
        #rospy.sleep(t/1000.0)  

if __name__ == '__main__':
    print('pick or place planning')
