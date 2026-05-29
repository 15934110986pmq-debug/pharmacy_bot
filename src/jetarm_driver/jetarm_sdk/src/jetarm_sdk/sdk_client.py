import os
import rospy
from hiwonder_interfaces.msg import Buzzer, Led, SerialServoBool, SerialServoMove, SerialServoSelect
from hiwonder_interfaces.srv import SerialServoTrigger


class JetArmSDKClient:
    def __init__(self, namespace='/jetarm_sdk', wait_for_ready=True):

        if wait_for_ready:
            rospy.wait_for_service(os.path.join(namespace, 'serial_servo/ping'))

        self.set_buzzer_pub = rospy.Publisher(os.path.join(namespace, 'set_buzzer'), Buzzer, queue_size=10)
        self.set_led_pub = rospy.Publisher(os.path.join(namespace, 'set_led'), Led, queue_size=10)
        self.ss_move_pub = rospy.Publisher(os.path.join(namespace, 'serial_servo/move'), SerialServoMove, queue_size=10)
        self.ss_stop_pub = rospy.Publisher(os.path.join(namespace, 'serial_servo/stop'), SerialServoSelect, queue_size=10)
        self.ss_load_pub = rospy.Publisher(os.path.join(namespace, 'serial_servo/load_unload'), SerialServoBool, queue_size=10)

        self.ss_ping_proxy = rospy.ServiceProxy(os.path.join(namespace, 'serial_servo/ping'), SerialServoTrigger)


    def set_buzzer(self, freq, on_ticks, off_ticks, repeat):
        msg = Buzzer(freq=freq, on_ticks=on_ticks, off_ticks=off_ticks, repeat=repeat)
        self.set_buzzer_pub.publish(msg)

    def set_led(self, on_ticks, off_ticks, repeat):
        msg = Led(on_ticks=on_ticks, off_ticks=off_ticks, repeat=repeat)
        self.set_led_pub.publish(msg)

    def serial_servo_move(self, servo_id, position, duration):
        msg = SerialServoMove(servo_id=servo_id, position=position, duration=duration)
        self.ss_move_pub.publish(msg)

    def serial_servo_move(self, servo_id):
        msg = SerialServoSelect(servo_id=servo_id)
        self.ss_stop_pub.publish(msg)

    def serial_servo_load(self, servo_id, new_state):
        msg = SerialServoBool(servo_id=servo_id, state=new_state)
        self.ss_load_pub.publish(msg)

    def serial_servo_ping(self, servo_id):
        return self.ss_ping_proxy(servo_id=servo_id).success

