import time
from machine import PWM, FPIOA
from media.sensor import *
from media.display import *

# 屏幕尺寸
WIDTH = 640
HEIGHT = 480

# 初始化舵机
fpioa = FPIOA()
fpioa.set_function(47, FPIOA.PWM3)  # 垂直
fpioa.set_function(46, FPIOA.PWM2)  # 水平

tilt = PWM(3, freq=50)  # 垂直
pan = PWM(2, freq=50)   # 水平

# 360度舵机控制
STOP = 1500000      # 1.5ms 停止
MIN = 1300000       # 1.3ms 慢速顺时针
MAX = 1700000       # 1.7ms 慢速逆时针

# 初始停止
pan.duty_ns(STOP)
tilt.duty_ns(STOP)
print("舵机初始化完成")

# 初始化摄像头
sensor = Sensor(width=WIDTH, height=HEIGHT)
sensor.reset()
sensor.set_framesize(width=WIDTH, height=HEIGHT)
sensor.set_pixformat(Sensor.RGB565)
sensor.run()

# 初始化显示
Display.init(Display.ST7701, width=WIDTH, height=HEIGHT)

print("系统启动")

# 紫色阈值
purple = (20, 60, 10, 50, -40, 10)

try:
    while True:
        # 获取图像
        img = sensor.snapshot()

        # 找紫色物体
        blobs = img.find_blobs([purple], pixels_threshold=200)

        if blobs:
            blob = max(blobs, key=lambda b: b.pixels())

            # 画框
            img.draw_rectangle(blob.rect(), color=(255, 0, 255))
            img.draw_cross(blob.cx(), blob.cy(), color=(255, 255, 0))

            # 计算偏移
            dx = blob.cx() - WIDTH // 2
            dy = blob.cy() - HEIGHT // 2

            # 简单控制
            if dx > 30:  # 目标在右边
                pan.duty_ns(MIN)  # 顺时针转（向右）
            elif dx < -30:  # 目标在左边
                pan.duty_ns(MAX)  # 逆时针转（向左）
            else:
                pan.duty_ns(STOP)  # 停止

            if dy > 30:  # 目标在下边
                tilt.duty_ns(MIN)  # 顺时针转（向下）
            elif dy < -30:  # 目标在上边
                tilt.duty_ns(MAX)  # 逆时针转（向上）
            else:
                tilt.duty_ns(STOP)  # 停止

        else:
            # 没找到目标，停止
            pan.duty_ns(STOP)
            tilt.duty_ns(STOP)

        # 画中心
        img.draw_cross(WIDTH//2, HEIGHT//2, color=(0, 255, 0))

        # 显示状态
        if blobs:
            img.draw_string(10, 10, "追踪中", color=(0, 255, 0), scale=2)
        else:
            img.draw_string(10, 10, "搜索中", color=(255, 0, 0), scale=2)

        # 显示图像
        Display.show_image(img)

        time.sleep(0.05)

except KeyboardInterrupt:
    print("停止")
finally:
    # 停止舵机
    pan.duty_ns(STOP)
    tilt.duty_ns(STOP)
    print("舵机已停止")
