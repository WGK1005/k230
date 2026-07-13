'''
K230 云台追踪 - 电机控制版本
使用电机代替舵机，通过UART控制
'''

import time, utime
from media.sensor import *
from media.display import *
from media.media import *
from machine import Pin, UART
import ustruct

# ========== 基本参数 ==========
W, H = 800, 480
CX, CY = W // 2, H // 2

# 电机参数
MOTOR_ADDR_H = 1  # 水平电机地址
MOTOR_ADDR_V = 2  # 垂直电机地址

# 红色阈值
RED_TH = (20, 80, 30, 100, 0, 60)

# ===== 稳定性控制参数 =====
DEADZONE = 10          # 死区(像素) - 在此范围内不动
SMOOTH = 0.4          # 平滑系数(0~1, 越小越稳)
PAN_SCALE = 0.5        # 水平灵敏度（电机速度缩放）
TILT_SCALE = 0.5       # 垂直灵敏度（电机速度缩放）
MIN_MOVE = 30         # 最小移动量

# 距离参数
DIST_REF = 5000        # 参考像素数

# ========== 全局状态 ==========
smooth_x, smooth_y = 0.0, 0.0
pixel_avg = DIST_REF
uart1 = None

# ========== 电机控制函数 ==========
def Emm_V5_En_Control(addr, state, snF):
    """电机使能"""
    cmd = bytearray(6)
    cmd[0] = addr
    cmd[1] = 0xF3
    cmd[2] = 0xAB
    cmd[3] = 0x01 if state else 0x00
    cmd[4] = 0x01 if snF else 0x00
    cmd[5] = 0x6B
    uart1.write(cmd)

def Emm_V5_Modify_Ctrl_Mode(addr, svF, ctrl_mode):
    """修改控制模式"""
    cmd = bytearray(6)
    cmd[0] = addr
    cmd[1] = 0x46
    cmd[2] = 0x69
    cmd[3] = 0x01 if svF else 0x00
    cmd[4] = ctrl_mode
    cmd[5] = 0x6B
    uart1.write(cmd)

def Emm_V5_Vel_Control(addr, dir, vel, acc, snF):
    """速度控制
    dir: 0为CW(正转)，其余值为CCW(反转)
    vel: 速度(RPM)
    acc: 加速度
    """
    cmd = bytearray(8)
    cmd[0] = addr
    cmd[1] = 0xF6
    cmd[2] = dir
    cmd[3] = (vel >> 8) & 0xFF
    cmd[4] = vel & 0xFF
    cmd[5] = acc
    cmd[6] = 0x01 if snF else 0x00
    cmd[7] = 0x6B
    uart1.write(cmd)

def Emm_V5_Stop_Now(addr, snF):
    """立即停止"""
    cmd = bytearray(5)
    cmd[0] = addr
    cmd[1] = 0xFE
    cmd[2] = 0x98
    cmd[3] = 0x01 if snF else 0x00
    cmd[4] = 0x6B
    uart1.write(cmd)

def motor_control_horizontal(speed):
    """水平方向电机控制
    speed > 0: 右转
    speed < 0: 左转
    """
    if abs(speed) < 10:
        Emm_V5_Stop_Now(MOTOR_ADDR_H, False)
    else:
        direction = 0 if speed > 0 else 1
        rpm = min(abs(int(speed)), 1000)  # 限制最大速度
        Emm_V5_Vel_Control(MOTOR_ADDR_H, direction, rpm, 50, False)

def motor_control_vertical(speed):
    """垂直方向电机控制
    speed > 0: 上升
    speed < 0: 下降
    """
    if abs(speed) < 10:
        Emm_V5_Stop_Now(MOTOR_ADDR_V, False)
    else:
        direction = 0 if speed > 0 else 1
        rpm = min(abs(int(speed)), 1000)  # 限制最大速度
        Emm_V5_Vel_Control(MOTOR_ADDR_V, direction, rpm, 50, False)

def init_hw():
    global uart1
    
    # 初始化UART - 使用 GPIO 03 (TX) 和 GPIO 04 (RX)
    uart1 = UART(1, baudrate=115200)
    time.sleep(0.2)
    
    # 初始化电机
    print("初始化电机...")
    Emm_V5_En_Control(MOTOR_ADDR_H, True, False)
    time.sleep_ms(100)
    Emm_V5_Modify_Ctrl_Mode(MOTOR_ADDR_H, False, 3)  # 速度控制模式
    time.sleep_ms(100)
    
    Emm_V5_En_Control(MOTOR_ADDR_V, True, False)
    time.sleep_ms(100)
    Emm_V5_Modify_Ctrl_Mode(MOTOR_ADDR_V, False, 3)  # 速度控制模式
    time.sleep_ms(100)

    # 摄像头初始化
    print("初始化摄像头...")
    s = Sensor(width=W, height=H)
    s.reset()
    s.set_framesize(width=W, height=H)
    s.set_pixformat(Sensor.RGB565)
    Display.init(Display.ST7701, width=W, height=H, to_ide=True)
    MediaManager.init()
    s.run()

    time.sleep(0.5)
    return s

def main():
    global smooth_x, smooth_y, pixel_avg

    print("电机追踪模式启动")
    cam = init_hw()

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
                    motor_control_horizontal(spd)
                else:
                    motor_control_horizontal(0)

                # === 垂直控制 ===
                if abs(smooth_y) > DEADZONE:
                    spd = smooth_y * TILT_SCALE
                    if spd > 0:
                        spd = max(spd, MIN_MOVE)
                    else:
                        spd = min(spd, -MIN_MOVE)
                    motor_control_vertical(spd)
                else:
                    motor_control_vertical(0)

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
                motor_control_horizontal(0)
                motor_control_vertical(0)
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
        motor_control_horizontal(0)
        motor_control_vertical(0)
        Emm_V5_Stop_Now(MOTOR_ADDR_H, False)
        Emm_V5_Stop_Now(MOTOR_ADDR_V, False)
        print("停止")

if __name__ == "__main__":
    main()
