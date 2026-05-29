#!/usr/bin/env python3
"""
ps2 手柄机体遥控， mode为0为单舵机模式， mode为1为坐标模式(ps2 joystick remote control, when the mode is 0, it is the single servo mode; when the mode is 1, it is the coordinate mode)
"""

import time
import os
import math
import rospy
import numpy as np
import pygame as pg
from enum import Enum
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from math import radians
from jetarm_sdk import bus_servo_control, sdk_client, common
from hiwonder_interfaces.msg import ServoState, ServoStateList, MultiRawIdPosDur
from hiwonder_interfaces.srv import SetRobotPose, GetRobotPose
import actions
import sys
sys.path.append("/home/ubuntu/jetarm/src/stepper/scripts/")
import stepper as Stepper

os.environ["SDL_VIDEODRIVER"] = "dummy"  # For use PyGame without opening a visible display
pg.display.init()

BIG_STEP = 0.1
BIG_ROTATE = math.radians(30)
SMALL_STEP = 0.06
SMALL_ROTATE = math.radians(15)

AXES_MAP = 'ly', 'lx', 'ry', 'rx'
BUTTON_MAP = 'cross', 'circle', '', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', 'hat_xu', 'hat_xd', 'hat_yr', 'hat_yl', 'laxis_l', 'laxis_r', 'laxis_u', 'laxis_d', 'raxis_l', 'raxis_u', 'raxis_u', 'raxis_d'




class ButtonState(Enum):
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3


class JoystickControlNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.sdk = sdk_client.JetArmSDKClient()
        self.count = 0
        self.joy = None
        self.mode = 0

        self.current_servo_position = None
        self.servos_pub = rospy.Publisher('/controllers/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        self.servo_states_sub = rospy.Subscriber('/servo_states', ServoStateList, self.servo_states_callback)
        self.get_current_pose = rospy.ServiceProxy('/kinematics/get_current_pose', GetRobotPose)
        self.set_pose_target = rospy.ServiceProxy('/kinematics/set_pose_target', SetRobotPose)

        self.last_axes = dict(zip(AXES_MAP, [0.0, ] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0, ] * len(BUTTON_MAP)))
        self.update_timer = rospy.Timer(rospy.Duration(0.05), self.joy_callback)


    def servo_states_callback(self, msg):
        servo_positions = []
        for i in msg.servo_states:
            servo_positions.append(i.position)
        self.current_servo_position = np.array(servo_positions)

    def relative_move(self, x, y, z, pitch):
        ret = self.get_current_pose()
        if ret.solution:
            pose = ret.pose
            x += pose.position.x
            y += pose.position.y
            z += pose.position.z
            rpy = [math.degrees(d) for d in common.qua2rpy(pose.orientation)]
            pitch += rpy[1]
            ret = self.set_pose_target(position=[x, y, z], pitch=pitch,  pitch_range=[-50, 50], resolution=1)
            if len(ret.pulse) > 0:
                pulse = [ int(p + 0.5) for p in ret.pulse]
                bus_servo_control.set_servos(self.servos_pub, 40, ((1, pulse[0]), (2, pulse[1]), (3, pulse[2]), (4, pulse[3])))

    def set_slideway(self,pulse):     
        stepper = Stepper.Stepper(1)
        stepper.set_mode(stepper.EN) 
        stepper.goto(pulse)
        rospy.sleep(0.1)
              

    def l1_callback(self, new_state):
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servos(self.servos_pub, 50, ((3, int(self.current_servo_position[2] + 10)), ))
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.relative_move(0, 0, 0.005, 0)

    def l2_callback(self, new_state):
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servos(self.servos_pub, 50, ((3, int(self.current_servo_position[2] - 10)), ))
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.relative_move(0, 0, -0.005, 0)

    def r1_callback(self, new_state):
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servos(self.servos_pub, 50, ((4, int(self.current_servo_position[3] + 10)), ))
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.relative_move(0, 0, 0, -1)

    def r2_callback(self, new_state):
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servos(self.servos_pub, 50, ((4, int(self.current_servo_position[3] - 10)), ))
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.relative_move(0, 0, 0, 1)

    def cross_callback(self, new_state):
        if (self.mode == 0 or self.mode == 1) and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servos(self.servos_pub, 50, ((10, int(self.current_servo_position[-1] - 10)), ))

    def triangle_callback(self, new_state):
        if (self.mode == 0 or self.mode == 1) and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servos(self.servos_pub, 50, ((10, int(self.current_servo_position[-1] + 10)), ))

    def circle_callback(self, new_state):
        if (self.mode == 0 or self.mode == 1) and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servos(self.servos_pub, 50, ((5, int(self.current_servo_position[4] + 10)), ))

    def square_callback(self, new_state):
        if (self.mode == 0 or self.mode == 1) and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servos(self.servos_pub, 50, ((5, int(self.current_servo_position[4] - 10)), ))

    def raxis_u_callback(self, new_state):
        if new_state == ButtonState.Holding:
            pass

    def raxis_d_callback(self, new_state):
        if new_state == ButtonState.Holding:
            pass

    def raxis_l_callback(self, new_state):
        pass

    def raxis_r_callback(self, new_state):
        pass

    def laxis_u_callback(self, new_state):
        pass

    def laxis_d_callback(self, new_state):
        pass

    def laxis_l_callback(self, new_state):
        self.set_slideway(350)       

    def laxis_r_callback(self, new_state):
        self.set_slideway(-350)
      



    def hat_xu_callback(self, new_state):
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servos(self.servos_pub, 50, ((2, int(self.current_servo_position[1] + 10)), ))
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.relative_move(0.005, 0, 0, 0)

    def hat_xd_callback(self, new_state):
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servos(self.servos_pub, 50, ((2, int(self.current_servo_position[1] - 10)), ))
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.relative_move(-0.005, 0, 0, 0)

    def hat_yl_callback(self, new_state):
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servos(self.servos_pub, 50, ((1, int(self.current_servo_position[0] + 10)), ))
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.relative_move(0, 0.005, 0, 0)

    def hat_yr_callback(self, new_state):
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servos(self.servos_pub, 50, ((1, int(self.current_servo_position[0] - 10)), ))
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.relative_move(0, -0.005, 0, 0)

    def start_callback(self, new_state):
        if new_state == ButtonState.Pressed and self.last_buttons['select']:
            if self.mode == 0:
                self.mode = 1
                self.sdk.set_buzzer(1000, 100, 100, 2)
            elif self.mode == 1:
                self.mode = 0
                self.sdk.set_buzzer(1000, 100, 150, 1)
        if new_state == ButtonState.Pressed and not self.last_buttons['select']:
            actions.go_home(self.servos_pub, 1.5)
            

    def axes_callback(self, axes):
        pass

    def joy_callback(self, event):
        # 检查当前是否插入(check if currently inserted)
        self.count += 1
        if self.count > 10:
            if os.path.exists("/dev/input/js0"):
                if self.joy is None:
                    rospy.sleep(1)
                    pg.joystick.init()
                    if pg.joystick.get_count() > 0:
                        try:
                            self.joy = pg.joystick.Joystick(0)
                            self.joy.init()
                        except Exception as e:
                            rospy.logerr(e)
            else:
                if self.joy is not None:
                    try:
                        self.joy.quit()
                        pg.joystick.quit()
                        self.joy = None
                    except Exception as e:
                        rospy.logerr(e)

        if self.joy is None:
            return

        pg.event.pump()
        buttons = list(self.joy.get_button(i) for i in range(self.joy.get_numbuttons()))
        # print(buttons)
        hat = list(self.joy.get_hat(0))
        axes = list(-self.joy.get_axis(i) for i in range(4))

        # 摇杆值(joystick value)
        axes = dict(zip(AXES_MAP, axes))
        # 摇杆 ad 值转为 bool 值(convert the ad value of joystick to bool value)
        laxis_l, laxis_r = 1 if axes['ly'] > 0.5 else 0, 1 if axes['ly'] < -0.5 else 0
        laxis_u, laxis_d = 1 if axes['lx'] > 0.5 else 0, 1 if axes['lx'] < -0.5 else 0
        raxis_l, raxis_r = 1 if axes['ry'] > 0.5 else 0, 1 if axes['ry'] < -0.5 else 0
        raxis_u, raxis_d = 1 if axes['rx'] > 0.5 else 0, 1 if axes['rx'] < -0.5 else 0

        # 方向帽的 ad 值转为 bool 值(convert the ad value of direction cap to bool value)
        hat_y, hat_x = hat
        hat_xl, hat_xr = 1 if hat_x > 0.5 else 0, 1 if hat_x < -0.5 else 0
        hat_yd, hat_yu = 1 if hat_y > 0.5 else 0, 1 if hat_y < -0.5 else 0

        buttons.extend([hat_xl, hat_xr, hat_yu, hat_yd])
        buttons.extend([laxis_l, laxis_r, laxis_u, laxis_d, raxis_l, raxis_r, raxis_u, raxis_d])
        buttons = dict(zip(BUTTON_MAP, buttons))
        # print(buttons)

        # 判断摇杆值的是否改变(determine whether the joystick value is changed)
        axes_changed = False
        for key, value in axes.items():  # 轴的值被改变(the axle value is changed)
            if self.last_axes[key] != value:
                axes_changed = True
        if axes_changed:
            try:
                self.axes_callback(axes)
            except Exception as e:
                rospy.logerr(str(e))

        for key, value in buttons.items():
            if value != self.last_buttons[key]:
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
            else:
                new_state = ButtonState.Holding if value > 0 else ButtonState.Normal
            callback = "".join([key, '_callback'])
            if new_state != ButtonState.Normal:
                # rospy.loginfo(key + ': ' + str(new_state))
                if hasattr(self, callback):
                    try:
                        getattr(self, callback)(new_state)
                    except Exception as e:
                        rospy.logerr(str(e))

        self.last_buttons = buttons
        self.last_axes = axes


if __name__ == "__main__":
    node = JoystickControlNode('joystick_control')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
