#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/11/21
import os
import subprocess
import re
wav_path = os.path.join(os.path.split(os.path.realpath(__file__))[0], 'feedback_voice')

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
            raise RuntimeError("USB Audio Device not found")

def get_path(f, language='Chinese'):
    if language == 'Chinese':
        return os.path.join(wav_path, f + '.wav')
    else:    
        return os.path.join(wav_path, 'english', f + '.wav')

def play(voice, volume=80, language='Chinese'):
    try:
        device = get_usb_audio_device()
        os.system('amixer -q -D pulse set Master {}%'.format(volume))
        os.environ['AUDIODRIVER'] = 'alsa'
        os.system(f'aplay -q -Dplughw:{device},0 -fS16_LE -r16000 -c1 -N --buffer-size=81920 ' + get_path(voice, language))
    except BaseException as e:
        print('error', e)

if __name__ == '__main__':
    play('ok')
    play('running', language="en")

