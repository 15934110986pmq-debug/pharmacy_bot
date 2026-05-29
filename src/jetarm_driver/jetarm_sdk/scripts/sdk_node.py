#!/usr/bin/env python3
#
import sys
import rospy
from jetarm_sdk import hw_interfaces, controller_manager
from hiwonder_interfaces.msg import Buzzer, Led, SerialServoBool, SerialServoMove, SerialServoSelect
from hiwonder_interfaces.srv import SerialServoTrigger, SerialServoTriggerResponse


class JetArmSDK:
    def __init__(self, node_name):
        rospy.init_node(node_name, log_level=rospy.INFO)

        serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
        baudrate = rospy.get_param('~baudrate', 10000000)
        use_sim = rospy.get_param('~use_sim', False)
        rospy.loginfo("USE_SIM is {}".format(use_sim))
        self.hw_interfaces = hw_interfaces.HwInterfaces(serial_port, baudrate, use_sim)

        enable_joint_controller = rospy.get_param('~enable_joint_controller', False)
        self.controller_manager = None
        if enable_joint_controller:
            try:
                self.controller_manager = controller_manager.ControllerManager(self.hw_interfaces)
            except IOError as e:
                self.hw_interfaces.set_buzzer(1000, 100, 100, 10)
                rospy.sleep(1)
                sys.exit(-1)

        rospy.Subscriber('~set_buzzer', Buzzer, self.buzzer_callback, queue_size=10)
        rospy.Subscriber('~set_led', Led, self.led_callback, queue_size=10)
        rospy.Subscriber('~serial_servo/move', SerialServoMove, self.ss_position_callback, queue_size=20)
        rospy.Subscriber('~serial_servo/stop', SerialServoSelect, self.ss_stop_callback, queue_size=20)
        rospy.Subscriber('~serial_servo/load_unload', SerialServoBool, self.ss_unload_callback, queue_size=10)
        rospy.Service('~serial_servo/ping', SerialServoTrigger, self.ss_ping_callback)

        rospy.loginfo("SDK node is running ...")

    def buzzer_callback(self, msg):
        self.hw_interfaces.set_buzzer(msg.freq, msg.on_ticks, msg.off_ticks, msg.repeat)

    def led_callback(self, msg):
        self.hw_interfaces.set_led(msg.on_ticks, msg.off_ticks, msg.repeat)

    def ss_position_callback(self, msg):
        self.hw_interfaces.serial_servo_move(msg.servo_id, msg.position, msg.duration)

    def ss_stop_callback(self, msg):
        self.hw_interfaces.serial_servo_stop(msg.servo_id)

    def ss_unload_callback(self, msg):
        self.hw_interfaces.serial_servo_load(msg.servo_id, msg.state)

    def ss_ping_callback(self, req):
        try:
            servo_id = self.hw_interfaces.serial_servo_id_read(req.servo_id)
            if servo_id is not None and servo_id == req.servo_id:
                return SerialServoTriggerResponse(success=True)
        except Exception as e:
            return SerialServoTriggerResponse(success=False, message=str(e))
        return SerialServoTriggerResponse(success=False)


def main(args=None):
    node = JetArmSDK("jetarm_sdk")
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))


if __name__ == '__main__':
    main()
