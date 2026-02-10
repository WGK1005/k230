'''
K230简单舵机控制
水平：GPIO46 (270度)
垂直：GPIO47 (180度限制)
'''

import time
from machine import Pin, PWM

# 初始化舵机
pan = PWM(Pin(46))   # 水平
tilt = PWM(Pin(47))  # 垂直
pan.freq(50)
tilt.freq(50)

def set_angle(pwm_obj, angle, min_deg=0, max_deg=180, min_pulse=600, max_pulse=2400):
    """
    设置舵机角度
    SG90舵机校准参数：
    - 标准：min_pulse=500, max_pulse=2500
    - 如果角度偏小，尝试：min_pulse=500, max_pulse=2400
    - 如果还不准，可微调：min_pulse=600, max_pulse=2400
    """
    angle = max(min_deg, min(max_deg, angle))
    # 角度转脉宽（微秒）
    pulse = min_pulse + (angle - min_deg) * (max_pulse - min_pulse) / (max_deg - min_deg)
    pwm_obj.duty_ns(int(pulse * 1000))
    return angle

# 简单测试
print("水平舵机测试...")
for angle in [0, 90, 180]:
    deg = set_angle(pan, angle, 0, 180)  # SG90是180度舵机
    print(f"水平: {deg}度")
    time.sleep(2)

print("\n垂直舵机测试...")
for angle in [90, 0, 180, 90]:
    deg = set_angle(tilt, angle, 0, 180)
    print(f"垂直: {deg}度")
    time.sleep(2)

# 停止
pan.duty_ns(int(1500 * 1000))
tilt.duty_ns(int(1500 * 1000))
print("\n测试完成")
