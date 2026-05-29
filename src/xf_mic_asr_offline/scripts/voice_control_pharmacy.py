#!/usr/bin/env python3
# encoding: utf-8
# @data:2026/05/29
# @author:pharmacy_bot
# 语音控制药房分拣
import os
import json
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from xf_mic_asr_offline import voice_play

class VoiceControlPharmacyNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        
        self.language = os.environ['ASR_LANGUAGE']
        rospy.Subscriber('/voice_control/voice_words', String, self.words_callback)

        # 等待分拣服务就绪
        rospy.wait_for_service('/object_sortting/enable_sortting')
        rospy.wait_for_service('/object_sortting/exit')
        rospy.sleep(5)

        # 设置默认模式
        self.mode = '门诊'
        rospy.set_param('/pharmacy_mode', self.mode)

        self.play('running')
        rospy.loginfo('唤醒口令: 小幻小幻(Wake up word: hello hiwonder)')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒(No need to wake up within 15 seconds after waking up)')
        rospy.loginfo('药房控制指令: 开始分拣 停止分拣 紧急暂停 切换门诊/住院/养老院模式')

    def play(self, name):
        voice_play.play(name, language=self.language)

    def words_callback(self, msg):
        words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        if self.language == 'Chinese':
            words = words.replace(' ', '')
        print('words:', words)

        if words is not None and words not in ['唤醒成功(wake-up-success)', '休眠(Sleep)', '失败5次(Fail-5-times)', '失败10次(Fail-10-times']:
            # 开始分拣
            if words == '开始分拣':
                self.play('start_sort')
                res = rospy.ServiceProxy('/object_sortting/enable_sortting', SetBool)(True)
                if res.success:
                    rospy.loginfo('分拣已开启')
                else:
                    rospy.loginfo('开启分拣失败')

            # 停止分拣
            elif words == '停止分拣':
                self.play('stop_sort')
                res = rospy.ServiceProxy('/object_sortting/enable_sortting', SetBool)(False)
                if res.success:
                    rospy.loginfo('分拣已停止')
                else:
                    rospy.loginfo('停止分拣失败')

            # 紧急暂停（退出分拣流程）
            elif words == '紧急暂停':
                self.play('emergency_stop')
                res = rospy.ServiceProxy('/object_sortting/exit', Trigger)()
                if res.success:
                    rospy.loginfo('已紧急暂停')
                else:
                    rospy.loginfo('紧急暂停失败')

            # 切换模式
            elif '切换' in words and ('门诊' in words or '住院' in words or '养老院' in words):
                if '门诊' in words:
                    self.mode = '门诊'
                elif '住院' in words:
                    self.mode = '住院'
                elif '养老院' in words:
                    self.mode = '养老院'
                rospy.set_param('/pharmacy_mode', self.mode)
                rospy.loginfo('切换模式: %s', self.mode)
                self.play('switch_mode')
            else:
                # 未知指令
                rospy.logwarn('无法识别的指令: %s', words)
                self.play('cannot_recognize')

        elif words == '休眠(Sleep)':
            rospy.sleep(0.05)

if __name__ == "__main__":
    VoiceControlPharmacyNode('voice_control_pharmacy')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
        rospy.loginfo("Shutting down")
