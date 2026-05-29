#!/usr/bin/env python3


import os
import rospy
from jetarm_sdk import sdk_client, tone
import subprocess
import re

if __name__ == "__main__":
    rospy.init_node("bringup_notify", anonymous=True, log_level=rospy.INFO)
    rospy.sleep(2)
    lang = os.environ['ASR_LANGUAGE']
    sdk = None
    ret = False
    def get_usb_audio_device():
        # 执行 `aplay -l` 命令获取音频设备列表
        result = subprocess.run(['aplay', '-l'], stdout=subprocess.PIPE, text=True)
    
        # 解析设备列表
        usb_device = None
        for line in result.stdout.splitlines():
            if 'USB Audio Device' in line:
                match = re.search(r'card (\d+):.*device (\d+)', line)
                if match:
                    card, device = match.groups()
                    usb_device = f'{card}'
                    break
    
        if usb_device:
            return usb_device
        else:
            usb_device = 0
            return usb_device

    try:
        sdk = sdk_client.JetArmSDKClient()
        rospy.wait_for_service("/jetarm_sdk/serial_servo/ping")
        rospy.wait_for_service("/kinematics/set_pose_target")
        rospy.wait_for_service("/finger_trace/enter")
        rospy.wait_for_service("/object_sortting/enter")
        rospy.wait_for_service("/object_tracking/enter")
        rospy.wait_for_service("/tag_stackup/enter")
        rospy.wait_for_service("/waste_classification/enter")
        rospy.sleep(2)
        ret = True
    except:
        pass
    if ret:
        device = get_usb_audio_device()
        sdk.set_buzzer(round(tone.G5), 250, 50, 1)
        rospy.sleep(0.5)
        os.system('amixer -q -D pulse set Master {}%'.format(100))
        os.environ['AUDIODRIVER'] = 'alsa'
        if lang == 'Chinese':
            os.system(f'aplay -q -Dplughw:{device},0 -fS16_LE -r16000 -c1 -N --buffer-size=81920 /home/ubuntu/jetarm/src/jetarm_bringup/voice/Chinese/running.wav')
        else:
            os.system('aplay -q -Dplughw:{device},0 -fS16_LE -r16000 -c1 -N --buffer-size=81920 /home/ubuntu/jetarm/src/jetarm_bringup/voice/running.wav')
    else:
        while not rospy.is_shutdown():
            sdk.set_buzzer(round(tone.C5), 200, 100, 1)
            rospy.sleep(0.5)
    rospy.spin()

