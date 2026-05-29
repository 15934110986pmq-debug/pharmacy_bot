#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/11/21
import os

def play(voice_path, volume=100):
    try:
        os.system('amixer -q -D pulse set Master {}%'.format(volume))
        os.environ['AUDIODRIVER'] = 'alsa'
        os.system('aplay -q -fS16_LE -r16000 -c1 -N ' + voice_path)
    except BaseException as e:
        print('error', e)

