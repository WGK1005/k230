'''
K230红色物体跟踪 - 无浮点数
'''

import time
from media.sensor import *
from media.display import *
from media.media import *
from machine import PWM, FPIOA

# 屏幕尺寸
W = 480
H = 800
CX = W // 2
CY = H // 2

# 舵机 - 使用duty_u16 (0-65535) 或 duty_ns
pwm_io1 = FPIOA()
pwm_io1.set_function(47, FPIOA.PWM3)
pwm_ud = PWM(3, freq=50)  # 50Hz

pwm_io2 = FPIOA()
pwm_io2.set_function(46, FPIOA.PWM2)
pwm_lr = PWM(2, freq=50)

# 设置初始位置 (使用duty_ns: 微秒转纳秒)
# 7.5% = 1.5ms = 1500000ns
MID_NS = 1500000
pwm_lr.duty_ns(MID_NS)
pwm_ud.duty_ns(MID_NS)
time.sleep(1)

# 摄像头
sensor = Sensor(width=W, height=H)
sensor.reset()
sensor.set_hmirror(True)
sensor.set_vflip(True)
sensor.set_framesize(width=W, height=H)
sensor.set_pixformat(Sensor.RGB565)

Display.init(Display.ST7701, width=W, height=H,to_ide=True)
MediaManager.init()
sensor.run()

print("红色物体跟踪开始")

# 红色阈值
RED_THRESHOLD = (20, 80, 30, 100, 0, 60)

# 死区
DEAD_ZONE_X = 50
DEAD_ZONE_Y = 50

# 舵机脉宽范围 (纳秒)
# 500us = 500000ns (0度)
# 2500us = 2500000ns (180度)
# 1500us = 1500000ns (90度/中间)
MIN_NS = 500000
MAX_NS = 2500000

# 控制脉宽增量
PAN_LEFT_NS = 1300000   # 1.3ms 左转
PAN_RIGHT_NS = 1700000  # 1.7ms 右转
PAN_MID_NS = 1500000    # 1.5ms 中间

TILT_UP_NS = 1300000    # 1.3ms 上转
TILT_DOWN_NS = 1700000  # 1.7ms 下转
TILT_MID_NS = 1500000   # 1.5ms 中间

while True:
    img = sensor.snapshot()

    # 找红色
    blobs = img.find_blobs([RED_THRESHOLD], pixels_threshold=200)

    if blobs:
        blob = max(blobs, key=lambda b: b.pixels())
        x = blob.cx()
        y = blob.cy()

        # 画框
        img.draw_rectangle(blob.rect(), color=(255,0,0))
        img.draw_cross(x, y, color=(255,0,0))

        # 计算偏移
        offset_x = x - CX
        offset_y = y - CY

        # 显示偏移
        info = f"X:{offset_x:+4d} Y:{offset_y:+4d}"
        img.draw_string_advanced(20, 30, 36, info, color=(255,255,0))

        # 控制逻辑
        pan_ns = PAN_MID_NS
        tilt_ns = TILT_MID_NS

        # 水平控制
        if offset_x > DEAD_ZONE_X:
            pan_ns = PAN_LEFT_NS  # 物体右，舵机左
        elif offset_x < -DEAD_ZONE_X:
            pan_ns = PAN_RIGHT_NS  # 物体左，舵机右

        # 垂直控制
        if offset_y > DEAD_ZONE_Y:
            tilt_ns = TILT_UP_NS    # 物体下，舵机上
        elif offset_y < -DEAD_ZONE_Y:
            tilt_ns = TILT_DOWN_NS  # 物体上，舵机下

        # 控制舵机
        pwm_lr.duty_ns(pan_ns)
        pwm_ud.duty_ns(tilt_ns)

        # 显示状态
        status = "TRACKING"
        img.draw_string_advanced(20, 80, 32, status, color=(0,255,0))

        # 显示舵机状态
        pan_dir = "L" if pan_ns < PAN_MID_NS else "R" if pan_ns > PAN_MID_NS else "M"
        tilt_dir = "U" if tilt_ns < TILT_MID_NS else "D" if tilt_ns > TILT_MID_NS else "M"
        dir_info = f"Pan:{pan_dir} Tilt:{tilt_dir}"
        img.draw_string_advanced(20, 120, 28, dir_info, color=(200,200,255))

    else:
        # 无目标
        img.draw_string_advanced(20, 30, 40, "NO TARGET", color=(255,0,0))

        # 舵机回中
        pwm_lr.duty_ns(PAN_MID_NS)
        pwm_ud.duty_ns(TILT_MID_NS)

    # 画中心十字和死区
    img.draw_cross(CX, CY, color=(0,255,0), size=10, thickness=2)
    img.draw_rectangle(CX-DEAD_ZONE_X, CY-DEAD_ZONE_Y,
                      DEAD_ZONE_X*2, DEAD_ZONE_Y*2,
                      color=(0,100,0), thickness=1)

    # 显示到屏幕
    Display.show_image(img)

    # 延时
    time.sleep_ms(50)

print("停止")
