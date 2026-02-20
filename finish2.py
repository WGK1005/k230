'''
K230 云台追踪 - 打靶模式
高稳定性，物体稳定锁定在中央
'''

import time
from media.sensor import *
from media.display import *
from media.media import *
from machine import PWM, FPIOA

# ========== 基本参数 ==========
W, H = 800, 480
CX, CY = W // 2, H // 2

# 舵机参数
SERVO_MIN_NS = 1000000
SERVO_MID_NS = 1500000
SERVO_MAX_NS = 2000000

# 垂直舵机角度范围(*100)
TILT_MIN, TILT_MAX, TILT_INIT = 4500, 13500, 9000

# 红色阈值
RED_TH = (20, 80, 30, 100, 0, 60)

# ===== 稳定性控制参数 =====
DEADZONE = 10          # 死区(像素) - 在此范围内不动
SMOOTH = 0.4          # 平滑系数(0~1, 越小越稳)
PAN_SCALE = 9.0        # 水平灵敏度
TILT_SCALE = 12.5       # 垂直灵敏度
MIN_MOVE = 75         # 最小移动量

# 距离参数
DIST_REF = 5000        # 参考像素数

# ========== 全局状态 ==========
tilt_angle = TILT_INIT
smooth_x, smooth_y = 0.0, 0.0
pixel_avg = DIST_REF

def angle_to_ns(angle):
    angle = max(TILT_MIN, min(TILT_MAX, angle))
    return SERVO_MIN_NS + (angle * 1000000) // 18000

def speed_to_ns(speed):
    speed = max(-10000, min(10000, speed))
    return SERVO_MID_NS + speed * 50

def init_hw():
    # 舵机
    FPIOA().set_function(46, FPIOA.PWM2)
    FPIOA().set_function(47, FPIOA.PWM3)
    ud = PWM(2, freq=50)
    lr = PWM(3, freq=50)
    ud.duty_ns(angle_to_ns(TILT_INIT))
    lr.duty_ns(SERVO_MID_NS)

    # 摄像头
    s = Sensor(width=W, height=H)
    s.reset()
    s.set_framesize(width=W, height=H)
    s.set_pixformat(Sensor.RGB565)
    Display.init(Display.ST7701, width=W, height=H, to_ide=True)
    MediaManager.init()
    s.run()

    time.sleep(0.5)
    return lr, ud, s

def main():
    global tilt_angle, smooth_x, smooth_y, pixel_avg

    print("实时追踪模式启动")
    lr, ud, cam = init_hw()

    try:
        while True:
            img = cam.snapshot()

            blobs = img.find_blobs([RED_TH], pixels_threshold=200, merge=True)

            if blobs:
                b = max(blobs, key=lambda x: x.pixels())
                x, y, px = b.cx(), b.cy(), b.pixels()

                # 原始误差
                raw_x = x - CX
                raw_y = y - CY

                # 低通滤波平滑
                smooth_x = smooth_x * (1 - SMOOTH) + raw_x * SMOOTH
                smooth_y = smooth_y * (1 - SMOOTH) + raw_y * SMOOTH

                # 像素数平滑(距离)
                pixel_avg = pixel_avg * 0.9 + px * 0.1

                # === 水平控制 ===
                if abs(smooth_x) > DEADZONE:
                    spd = -smooth_x * PAN_SCALE
                    if spd > 0:
                        spd = max(spd, MIN_MOVE)
                    else:
                        spd = min(spd, -MIN_MOVE)
                    lr.duty_ns(speed_to_ns(int(spd)))
                else:
                    lr.duty_ns(SERVO_MID_NS)

                # === 垂直控制 ===
                if abs(smooth_y) > DEADZONE:
                    tilt_angle -= smooth_y * TILT_SCALE * SMOOTH
                    tilt_angle = max(TILT_MIN, min(TILT_MAX, tilt_angle))
                ud.duty_ns(angle_to_ns(int(tilt_angle)))

                # 绘制
                img.draw_rectangle(b.rect(), color=(255,0,0), thickness=2)
                img.draw_cross(x, y, color=(255,0,0), size=10)
                img.draw_line(x, y, CX, CY, color=(255,200,0))

                # 距离显示
                dist_ratio = int(pixel_avg / DIST_REF * 100)
                if dist_ratio > 120:
                    dt, dc = "近", (0,255,0)
                elif dist_ratio < 80:
                    dt, dc = "远", (255,100,100)
                else:
                    dt, dc = "中", (255,255,0)

                img.draw_string_advanced(20, 80, 28, f"距离:{dt} {dist_ratio}%", color=dc)
                img.draw_string_advanced(20, 40, 32, "锁定中", color=(0,255,255))

            else:
                lr.duty_ns(SERVO_MID_NS)
                smooth_x, smooth_y = 0, 0
                img.draw_string_advanced(20, 40, 32, "搜索中", color=(255,150,150))

            # 中心准星
            img.draw_cross(CX, CY, color=(0,255,0), size=25, thickness=3)
            img.draw_circle(CX, CY, 40, color=(0,255,0), thickness=2)

            Display.show_image(img)
            time.sleep_ms(30)

    except KeyboardInterrupt:
        pass
    finally:
        lr.duty_ns(SERVO_MID_NS)
        ud.duty_ns(angle_to_ns(TILT_INIT))
        print("停止")

if __name__ == "__main__":
    main()
