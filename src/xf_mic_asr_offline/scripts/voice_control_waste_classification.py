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


class VoiceControlColorTrackNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        
        self.language = os.environ['ASR_LANGUAGE']
        rospy.Subscriber('/voice_control/voice_words', String, self.words_callback)

        rospy.wait_for_service('/waste_classification/enter')
        rospy.sleep(5)
        res = rospy.ServiceProxy('/waste_classification/enter', Trigger)()
        self.play('running')
        if res.success:
            print('open waste_classification')
        else:
            print('open waste_classification fail')
        rospy.loginfo('唤醒口令: 小幻小幻(Wake up word: hello hiwonder)')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒(No need to wake up within 15 seconds after waking up)')
        rospy.loginfo('控制指令: 开始垃圾分类 停止垃圾分类(Voice command: start classification, stop classification)')

    def play(self, name):
        voice_play.play(name, language=self.language)

    def words_callback(self, msg):
        words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        if self.language == 'Chinese':
            words = words.replace(' ', '')
        print('words:', words)
        if words is not None and words not in ['唤醒成功(wake-up-success)', '休眠(Sleep)', '失败5次(Fail-5-times)', '失败10次(Fail-10-times']:
            if "开" in words and "垃圾" in words:
                    self.play("waste_start")
                    res = rospy.ServiceProxy('/waste_classification/enable_transport', SetBool)(data=True)
            if ("关" in  words or "停" in words)  and ("垃圾" in words or "分" in words):
                    res = rospy.ServiceProxy('/waste_classification/enable_transport', SetBool)(data=False)
                    self.play("waste_stop")

if __name__ == "__main__":
    VoiceControlColorTrackNode('voice_control_color_sortting')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
        rospy.loginfo("Shutting down")
