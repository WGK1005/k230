'''
最简单的PWM舵机控制
'''

import time
from machine import Pin, PWM

# 初始化PWM
servo = PWM(Pin(46))  # 水平舵机接GPIO46
servo.freq(50)        # 舵机标准频率50Hz

print("开始测试...")

# 测试几个关键位置
# 500us = 0度, 1500us = 中间, 2500us = 最大角度
test_positions = [500, 1000, 1500, 2000, 2500]

for pulse in test_positions:
    print(f"脉宽: {pulse}us")
    servo.duty_ns(pulse * 1000)  # 微秒转纳秒
    time.sleep(3)

# 停止信号（保持位置）
servo.duty_ns(1500 * 1000)
print("测试完成")
