#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/18
# @author:aiden
# 语音控制颜色追踪
import os
import json
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from xf_mic_asr_offline import voice_play
from xf_mic_asr_offline import voice_play_1

VOICE_PATH="/home/ubuntu/jetarm/src/xf_mic_asr_offline/voice"

class VoiceControlColorTrackNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        
        self.language = os.environ['ASR_LANGUAGE']
        rospy.Subscriber('/voice_control/voice_words', String, self.words_callback)

        rospy.wait_for_service('/enable')
        rospy.sleep(5)
        self.play('running')
        rospy.loginfo('唤醒口令: 小幻小幻(Wake up word: hello hiwonder)')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒(No need to wake up within 15 seconds after waking up)')
        rospy.loginfo('控制指令: 移除高度异常目标 停止处理(Voice command: )')

    def play(self, name):
        voice_play.play(name, language=self.language)

    def words_callback(self, msg):
        words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        if self.language == 'Chinese':
            words = words.replace(' ', '')
        print('words:', words)
        if words is not None and words not in ['唤醒成功(wake-up-success)', '休眠(Sleep)', '失败5次(Fail-5-times)', '失败10次(Fail-10-times']:
            if "关" in  words or "停" in words:
                res = rospy.ServiceProxy('/enable', SetBool)(data=False)
            elif "移除" in words:
                res = rospy.ServiceProxy('/enable', SetBool)(data=True)
        elif words == '唤醒成功(wake-up-success)':
            self.play('awake')
        elif words == '休眠(Sleep)':
            rospy.sleep(0.05)

if __name__ == "__main__":
    VoiceControlColorTrackNode('voice_control_shape_recognition')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
        rospy.loginfo("Shutting down")
