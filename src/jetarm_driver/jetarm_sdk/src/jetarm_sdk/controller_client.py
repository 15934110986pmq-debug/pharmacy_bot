import os
import rospy
from hiwonder_interfaces.msg import JointMove, ServoMove


class JointControllerClient:
    def __init__(self, ns='controllers/', wait_for_service=None, wait_service_timeout=30):

        if wait_for_service is not None:
            rospy.wait_for_service(wait_for_service, timeout=wait_service_timeout)

        self.set_joint_pub = rospy.Publisher(os.path.join(ns, 'set_joint'), JointMove, queue_size=10)
        self.set_servo_pub = rospy.Publisher(os.path.join(ns, 'set_servo'), ServoMove, queue_size=10)

    def set_joint(self, joint_name, rad, duration):
        msg = JointMove()
        msg.name = joint_name
        msg.rad = rad
        msg.duration = duration
        self.set_joint_pub.publish(msg)

    def set_joints(self, rads, duration):
        for i, s in enumerate(rads):
            msg = JointMove()
            msg.name = "joint%d" % (i + 1)
            msg.rad = s
            msg.duration = duration
            self.set_joint_pub.publish(msg)

    def set_servo(self, global_id, position, duration):
        msg = ServoMove()
        msg.id = global_id
        msg.position = position
        msg.duration = duration
        self.set_servo_pub.publish(msg)

