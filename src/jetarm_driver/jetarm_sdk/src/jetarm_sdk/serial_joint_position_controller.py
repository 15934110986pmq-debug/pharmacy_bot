#!/usr/bin/env python3
# encoding: utf-8


import os
import math
import rospy
import threading
from jetarm_kinematics import transform
from math import radians


def rad_to_raw(angle, initial_ticks, flipped, encoder_ticks_per_radian):
    """ angle is in radians """
    angle_raw = angle * encoder_ticks_per_radian
    return int(round(initial_ticks - angle_raw if flipped else initial_ticks + angle_raw))


def raw_to_rad(raw, initial_ticks, flipped, radians_per_encoder_tick):
    return (initial_ticks - raw if flipped else raw - initial_ticks) * radians_per_encoder_tick


class JointPositionController():
    def __init__(self, hw_interface, param_ns):
        self.lock = threading.RLock()

        self.hw_interface = hw_interface
        self.param_ns = param_ns
        self.servo_id = rospy.get_param(os.path.join(self.param_ns, 'servo/servo_id'), 0)
        self.global_id = rospy.get_param(os.path.join(self.param_ns, 'servo/global_id'), 0)
        self.type = rospy.get_param(os.path.join(self.param_ns, 'servo/type'), 1)

        self.initial_ticks = rospy.get_param(os.path.join(self.param_ns, 'servo/initial_ticks'), 500)
        self.minimum_ticks = rospy.get_param(os.path.join(self.param_ns, 'servo/minimum_ticks'), 0)
        self.maximum_ticks = rospy.get_param(os.path.join(self.param_ns, 'servo/maximum_ticks'), 1000)

        self.radians_per_encoder_tick = rospy.get_param(os.path.join(self.param_ns, 'servo/radians_per_encoder_tick'), 1)
        self.encoder_ticks_per_radian = rospy.get_param(os.path.join(self.param_ns, 'servo/encoder_ticks_per_radian'), 1)
        self.encoder_resolution = rospy.get_param(os.path.join(self.param_ns, 'servo/encoder_resolution'), 1000)

        self.flipped = self.minimum_ticks > self.maximum_ticks
        if self.flipped:
            self.min_angle = (self.initial_ticks - self.minimum_ticks) * self.radians_per_encoder_tick
            self.max_angle = (self.initial_ticks - self.maximum_ticks) * self.radians_per_encoder_tick
        else:
            self.min_angle = (self.minimum_ticks - self.initial_ticks) * self.radians_per_encoder_tick
            self.max_angle = (self.maximum_ticks - self.initial_ticks) * self.radians_per_encoder_tick
        
        self.current_angle = None
        self.current_tick = None


    def pos_rad_to_raw(self, pos_rad):
        if pos_rad <= math.radians(-120):
            pos_rad = math.pi + math.pi + pos_rad
        pos_rad = self.min_angle if pos_rad < self.min_angle else self.max_angle if pos_rad > self.max_angle else pos_rad
        return rad_to_raw(pos_rad, self.initial_ticks, self.flipped, self.encoder_ticks_per_radian)


    def get_angle_and_tick(self):
        with self.lock:
            return self.current_angle, self.current_tick


    def set_position_in_tick(self, pos, duration):
        # rospy.loginfo(str(pos) + " " + str(duration))
        angle = raw_to_rad(pos, self.initial_ticks, self.flipped, self.radians_per_encoder_tick)
        with self.lock:
            self.current_tick = pos
            self.current_angle =  angle
        self.hw_interface.serial_servo_move(self.servo_id, int(pos), int(duration))


    def set_position_in_rad(self, rad, duration):
        #pos = self.pos_rad_to_raw(rad)
        angles = [0] * 5
        if self.servo_id == 2:
            angles[self.servo_id - 1] = rad - radians(90)
        elif self.servo_id == 4:
            angles[self.servo_id - 1] = rad - radians(90)
        else:
            angles[self.servo_id - 1] = rad
        if self.servo_id == 1:
            pos = int(transform.angle2pulse([angles, ])[0][self.servo_id - 1])
        else:
            pos =  500 + (500 - int(transform.angle2pulse([angles, ])[0][self.servo_id - 1]))
        with self.lock:
            self.current_tick = pos
            self.current_angle = rad
        print(self.servo_id, rad, pos, duration)
        self.hw_interface.serial_servo_move(self.servo_id, pos, duration)


