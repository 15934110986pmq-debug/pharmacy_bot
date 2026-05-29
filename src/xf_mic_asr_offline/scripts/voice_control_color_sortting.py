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
from hiwonder_interfaces.srv import SetStringBool

class VoiceControlColorTrackNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        
        self.language = os.environ['ASR_LANGUAGE']
        rospy.Subscriber('/voice_control/voice_words', String, self.words_callback)

        rospy.wait_for_service('/object_sortting/enter')
        rospy.sleep(5)
        res = rospy.ServiceProxy('/object_sortting/enter', Trigger)()
        res = rospy.ServiceProxy('/object_sortting/enable_sortting', SetBool)(True)
        self.play('running')
        if res.success:
            print('open color_track')
        else:
            print('open color_track fail')
        rospy.loginfo('唤醒口令: 小幻小幻(Wake up word: hello hiwonder)')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒(No need to wake up within 15 seconds after waking up)')
        rospy.loginfo('控制指令: 分拣红色 分拣绿色 分拣蓝色 停止追踪(Voice command: sortting red/green/blue object)')

    def play(self, name):
        voice_play.play(name, language=self.language)

    def words_callback(self, msg):
        words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        if self.language == 'Chinese':
            words = words.replace(' ', '')
        print('words:', words)
        if words is not None and words not in ['唤醒成功(wake-up-success)', '休眠(Sleep)', '失败5次(Fail-5-times)', '失败10次(Fail-10-times']:
            if words == '分拣红色' or words == 'track red object':
                self.play('start_sort_red')
                res = rospy.ServiceProxy('/object_sortting/set_color_target', SetStringBool)(data_str='red', data_bool=True)
                res = rospy.ServiceProxy('/object_sortting/set_color_target', SetStringBool)(data_str='green', data_bool=False)
                res = rospy.ServiceProxy('/object_sortting/set_color_target', SetStringBool)(data_str='blue', data_bool=False)
            elif words == '分拣绿色' or words == 'sortting green object':
                self.play('start_sort_green')
                res = rospy.ServiceProxy('/object_sortting/set_color_target', SetStringBool)(data_str='red', data_bool=False)
                res = rospy.ServiceProxy('/object_sortting/set_color_target', SetStringBool)(data_str='green', data_bool=True)
                res = rospy.ServiceProxy('/object_sortting/set_color_target', SetStringBool)(data_str='blue', data_bool=False)
            elif words == '分拣蓝色' or words == 'sortting blue object':
                self.play('start_sort_blue')
                res = rospy.ServiceProxy('/object_sortting/set_color_target', SetStringBool)(data_str='red', data_bool=False)
                res = rospy.ServiceProxy('/object_sortting/set_color_target', SetStringBool)(data_str='green', data_bool=False)
                res = rospy.ServiceProxy('/object_sortting/set_color_target', SetStringBool)(data_str='blue', data_bool=True)
            elif words == '停止分拣' or words == '停止颜色分拣' or words == 'stop sortting':
                res1 = rospy.ServiceProxy('/object_sortting/set_color_target', SetStringBool)(data_str='red', data_bool=False)
                res2 = rospy.ServiceProxy('/object_sortting/set_color_target', SetStringBool)(data_str='green', data_bool=False)
                res3 = rospy.ServiceProxy('/object_sortting/set_color_target', SetStringBool)(data_str='blue', data_bool=False)
                if res1.success and res2.success and res3.success:
                    self.play('stop_sort')
                else:
                    self.play('stop_fail')
        elif words == '休眠(Sleep)':
            rospy.sleep(0.05)

if __name__ == "__main__":
    VoiceControlColorTrackNode('voice_control_color_sortting')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
        rospy.loginfo("Shutting down")
