'''
最简单的舵机测试 - 3个位置
'''

import time
from machine import PWM, FPIOA

# 初始化
pwm_io = FPIOA()
pwm_io.set_function(46, FPIOA.PWM2)
servo = PWM(2, freq=50)

print("测试舵机3个位置...")

servo.duty_ns(1050000)
print("90度")
time.sleep(3)

# 90度
servo.duty_ns(15000000)
print("90度")
time.sleep(3)

servo.duty_ns(2000000)
print("180度")
time.sleep(3)

print("完成")
