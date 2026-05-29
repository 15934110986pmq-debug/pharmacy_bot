import os
import time
import rospy
from jetarm_sdk.serial_joint_position_controller import JointPositionController as SerialJointPositionController
from jetarm_sdk.joint_trajectory_action_controller import JointTrajectoryActionController
from sensor_msgs.msg import JointState
from hiwonder_interfaces.msg import ServoState, ServoStateList, ServoMove, JointMove, MultiRawIdPosDur, RawIdPosDur


class ControllerManager:
    """
    控制器管理器(controller manager)
    功能： 从参数服务读取关节控制器与舵机相关配置信息，生成相关对象(function: read joint controller and servo-related configuration information from the parameter server, and create relevant objects)
          监听相关Topic, 为其他 Node 提供控制关节的接口(listen to relevant topics and provide interfaces for controlling joints to other nodes)
          这些接口包括， 以关节名称及弧度控制的接口和以相应舵机的ID及原始位置数据控制接口(these interfaces include ones for controlling joints by joint name and radians, and controlling by servo ID and raw position data)
          发布当前各个关节角度 JointState, 舵机角度 ServoState 信息(publish current each joint angle JointState and servo angle ServoState)
    """

    def __init__(self, interfaces):
        self.param_ns = '~controllers/'
        self.hw_interfaces = interfaces

        self.joints = {}
        self.servos = {}
        items = rospy.get_param(self.param_ns).items()
        for ctlr_name, ctlr_param in items:
            if ctlr_param['type'] == 'JointPositionController':  # 如果是关节控制器就新建对应关节对象(if it's a joint controller, create a corresponding joint object)
                servo_id = ctlr_param['servo']['servo_id']
                servo_type = ctlr_param['servo']['type']
                if self.hw_interfaces.use_sim:  # 模拟控制，没有真实硬件(simulated control, no real hardware involved)
                    if servo_type == 1:  # 舵机类型 1 为 串行总线舵机(servo type 1 is interface bus servo)
                        rospy.loginfo("USE_SIM!!! Detected serial servo ID:%d for %s and added"
                                      % (servo_id, ctlr_name))
                        ctlr = SerialJointPositionController(self.hw_interfaces,
                                                             os.path.join(self.param_ns, ctlr_name))
                        self.joints[ctlr_name] = ctlr
                        self.servos[ctlr.global_id] = ctlr
                    else:  # 没有定义的舵机类型(undefined servo type)
                        msg = "Undefined servo type {}".format(servo_type)
                        rospy.logerr(msg)
                else: # 真实硬件(real hardware)
                    if servo_type == 1:  # 舵机类型 1 为 串行总线舵机(servo type 1 is interface bus servo)
                        if self.hw_interfaces.serial_servo_id_read(servo_id, 20) == servo_id:  # 通过读ID指令探测舵机是否已经连接到控制器(detect whether the servo is connected to the controller by reading the ID command)
                            rospy.loginfo("Detected serial servo ID:%d for %s and added" % (servo_id, ctlr_name))
                            ctlr = SerialJointPositionController(self.hw_interfaces,
                                                                 os.path.join(self.param_ns, ctlr_name))
                            self.joints[ctlr_name] = ctlr
                            self.servos[ctlr.global_id] = ctlr
                        else:  # 没有在总线上找到该舵机(the servo was not found on the bus)
                            msg = "Could not detect serial servo ID:{} on the bus!!! Node is shutting down.".format(
                                servo_id)
                            rospy.logerr(msg)
                            raise IOError("msg")
                    else:  # 没有定义的舵机类型(undefined servo type)
                        msg = "Undefined servo type {}".format(servo_type)
                        rospy.logerr(msg)

        self.trajectory_controller = None
        if rospy.get_param('~enable_trajectory', True):
            self.trajectory_controller = JointTrajectoryActionController(self.set_joint)

        # 控制是否开启关节、舵机状态发布的 param(control whether to enable publishing of joint and servo states using a parameter)
        enable_joint_state_pub = rospy.get_param('~publish_joint_state', True)
        enable_servo_state_pub = rospy.get_param('~publish_servo_state', True)

        # JointState 发布器(JointState publisher)
        self.joint_state_pub = rospy.Publisher('joint_states', JointState,
                                               queue_size=10) if enable_joint_state_pub else None

        # ServoState 发布器(ServoState publisher)
        self.servo_state_pub = rospy.Publisher('servo_states', ServoStateList,
                                               queue_size=10) if enable_servo_state_pub else None

        # 以舵机ID控制的Topic(Topic controlled by servo ID)
        self.sub_1 = rospy.Subscriber('controllers/set_servo', ServoMove, self.servo_move_callback, queue_size=10)
        self.sub_3 = rospy.Subscriber('controllers/id_pos_dur', RawIdPosDur, self.servo_move_ms_callback, queue_size=10)
        self.sub_4 = rospy.Subscriber('controllers/multi_id_pos_dur', MultiRawIdPosDur, self.servos_move_ms_callback, queue_size=10)

        # 以关节名称控制的Topic(Topic controlled by joint name)
        self.sub_2 = rospy.Subscriber('controllers/set_joint', JointMove, self.joint_move_callback, queue_size=10)

        # 发布 JointState 及 ServoState 的定时器(publish the timer of JointState and ServoState)
        self.start_stamp = time.time()
        pub_freq = rospy.get_param('state_publish_freq', 50)
        if self.servo_state_pub is not None or self.joint_state_pub is not None:
            self.joint_state_timer = rospy.Timer(rospy.Duration(1.0 / pub_freq), self.joint_state_publish_callback)
        else:
            self.joint_state_timer = None
    
    def servo_move_ms_callback(self, msg: RawIdPosDur):
        self.servos[msg.id].set_position_in_tick(msg.position, msg.duration)
    
    def servos_move_ms_callback(self, msg: MultiRawIdPosDur):
        for d in msg.id_pos_dur_list:
            self.servos[d.id].set_position_in_tick(d.position, d.duration)


    def servo_move_callback(self, msg: ServoMove):
        self.servos[msg.id].set_position_in_tick(msg.position, int(msg.duration * 1000))

    def joint_move_callback(self, msg: JointMove):
        self.joints[msg.name].set_position_in_rad(msg.rad, int(msg.duration * 1000))

    def set_joint(self, joint_name, rad, duration):
        self.joints[joint_name].set_position_in_rad(rad, duration)


    def joint_state_publish_callback(self, _):
        joint_state = []
        servo_state = []
        stamp = rospy.Time.now()
        for ctlr_name, ctlr_obj in self.joints.items():
            rad, tick = ctlr_obj.get_angle_and_tick()
            if rad is None or tick is None:  # 舵机位置要被设置后才能发布数据(the servo position must be set before data can be published)
                if time.time() - self.start_stamp > 10:
                    self.start_stamp = time.time()
                    rospy.logwarn(
                        "Joint state and servo state needs to be released after at least one control is completed")
                return
            joint_state.append((ctlr_name, rad, 0, 0))
            servo_state.append(ServoState(id=ctlr_obj.servo_id, type=ctlr_obj.type, goal=tick, position=tick, error=0, voltage=0))
            servo_state[-1].header.stamp = stamp

        joint_state = sorted(joint_state, key=lambda j: j[0])
        servo_state = sorted(servo_state, key=lambda s: s.id)

        name, rad, velocity, effort = zip(*joint_state)
        #servo_type, servo_id, tick = zip(*servo_state)
        joint_msg = JointState(name=list(name), position=list(rad), velocity=list(velocity), effort=list(effort))
        servo_msg = ServoStateList(servo_states=servo_state)

        joint_msg.header.stamp = stamp
        servo_msg.header.stamp = stamp

        if self.joint_state_pub is not None:
            self.joint_state_pub.publish(joint_msg)
        if self.servo_state_pub is not None:
            self.servo_state_pub.publish(servo_msg)
