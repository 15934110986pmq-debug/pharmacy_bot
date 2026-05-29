import rospy
from jetarm_sdk.controller_client import JointControllerClient


if __name__ == "__main__":
    rospy.init_node("abcdefg")
    j = JointControllerClient(wait_for_service='/jetarm_sdk/serial_servo/ping')
    rospy.sleep(5)
    for i in range(1, 6):
        j.set_servo(i, 500, 1)
    j.set_servo(10, 500, 1)
    rospy.spin()
