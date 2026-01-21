'''
最简单的360度舵机控制
水平舵机: GPIO46
垂直舵机: GPIO47
'''

import time
from machine import Pin, PWM

# 创建PWM对象
pan_pwm = PWM(Pin(46))   # 水平舵机
tilt_pwm = PWM(Pin(47))  # 垂直舵机

# 设置频率为50Hz
pan_pwm.freq(50)
tilt_pwm.freq(50)

def set_servo(pwm_obj, speed):
    """
    控制360度舵机
    speed: -100到100
         负数:逆时针，正数:顺时针，0:停止
    """
    # 限制速度范围
    speed = max(-100, min(100, speed))

    # 计算脉宽（微秒）
    if speed == 0:
        pulse = 1500      # 停止
    elif speed > 0:
        pulse = 1500 - speed * 2  # 顺时针
    else:
        pulse = 1500 - speed * 2  # 逆时针

    # 限制脉宽范围（安全）
    pulse = max(1000, min(2000, pulse))

    # 设置PWM（微秒转纳秒）
    pwm_obj.duty_ns(int(pulse * 1000))

    # 显示状态
    if speed > 0:
        print(f"顺时针 {speed}%")
    elif speed < 0:
        print(f"逆时针 {abs(speed)}%")
    else:
        print("停止")

# 简单测试
print("=== 360度舵机简单测试 ===")
print("水平舵机: GPIO46")
print("垂直舵机: GPIO47")

# 测试1：水平左右转动
print("\n测试水平舵机...")
set_servo(pan_pwm, 50)   # 顺时针转
time.sleep(2)
set_servo(pan_pwm, -50)  # 逆时针转
time.sleep(2)
set_servo(pan_pwm, 0)    # 停止
time.sleep(1)

# 测试2：垂直上下转动
print("\n测试垂直舵机...")
set_servo(tilt_pwm, 50)   # 顺时针转
time.sleep(2)
set_servo(tilt_pwm, -50)  # 逆时针转
time.sleep(2)
set_servo(tilt_pwm, 0)    # 停止
time.sleep(1)

# 测试3：同时转动
print("\n同时转动两个舵机...")
set_servo(pan_pwm, 70)
set_servo(tilt_pwm, 70)
time.sleep(3)

set_servo(pan_pwm, -70)
set_servo(tilt_pwm, -70)
time.sleep(3)

# 停止所有
print("\n停止所有舵机...")
set_servo(pan_pwm, 0)
set_servo(tilt_pwm, 0)

# 关闭PWM
pan_pwm.deinit()
tilt_pwm.deinit()

print("测试完成！")
