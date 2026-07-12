# gimbal_track.py
import time, os, sys, math
import cv_lite
from media.sensor import *
from media.display import *
from machine import UART, FPIOA

# ----------------- 串口与引脚底层配置 -----------------
# 使用我们前面为您重构的无冲突 UART1/UART2 配置（根据您的现场线路，可以和 YbUart 分开定义）
fpioa = FPIOA()
fpioa.set_function(3, FPIOA.UART1_TXD)
fpioa.set_function(4, FPIOA.UART1_RXD)
fpioa.set_function(5, FPIOA.UART2_TXD)
fpioa.set_function(6, FPIOA.UART2_RXD)

uart_ud = UART(UART.UART1, baudrate=115200)
uart_lr = UART(UART.UART2, baudrate=115200)

VERTICAL_ADDR = 0x01
HORIZONTAL_ADDR = 0x02

# ----------------- 控制配置参数（已调平滑版） -----------------
DEADZONE_X = 15
DEADZONE_Y = 8
SMOOTH_X = 0.06               # 加强一阶滤波，过滤微小抖动
SMOOTH_Y = 0.10

KP_PAN   = 0.35               # 纯 P 控制，不使用 KD 从而杜绝视觉延迟抖动
KD_PAN   = 0.00
KP_TILT  = 0.45
KD_TILT  = 0.00

V_MIN_PAN  = 350              # 固定电机基础追踪速度
V_MIN_TILT = 350

PAN_LIMIT_MIN = -3500
PAN_LIMIT_MAX = 3500
TILT_LIMIT_MIN = 0
TILT_LIMIT_MAX = 1600
PULSE_PER_REV = 3200
INIT_TILT = PULSE_PER_REV // 4

PAN_DIR  = -1
TILT_DIR = 1

def fast_mode_init(uart, addr, speed=500, acc=10):  # 调低硬加速度 acc=10 缓和物理惯性
    cmd = [addr, 0xF1, (speed >> 8) & 0xff, speed & 0xff, acc, 0x01, 0x00, 0x6B]
    uart.write(bytes(cmd))
    time.sleep_ms(5)

def move_motor(uart, addr, target):
    if target < 0:
        target = (1 << 32) + target
    cmd = [addr, 0xFC, (target >> 24) & 0xff, (target >> 16) & 0xff, (target >> 8) & 0xff, target & 0xff, 0x6B]
    uart.write(bytes(cmd))
    time.sleep_ms(5)

# ----------------- 核心云台追踪循环 -----------------
def run_gimbal_tracking(sensor, tp, key_esc, thresholds, WIDTH=640, HEIGHT=480):
    CX, CY = WIDTH // 2, HEIGHT // 2
    
    # 动态调好的阈值（加载 thresholds 的第一组配置）
    PURPLE_THRESHOLD = tuple(thresholds[0])
    print("启动云台，加载当前调色板阈值:", PURPLE_THRESHOLD)

    # 初始化云台物理速度参数
    fast_mode_init(uart_ud, VERTICAL_ADDR, V_MIN_TILT)
    fast_mode_init(uart_lr, HORIZONTAL_ADDR, V_MIN_PAN)
    time.sleep(0.5)

    # 归零并复位到工作角
    tilt_target = 0
    pan_target = 0
    move_motor(uart_ud, VERTICAL_ADDR, tilt_target)
    move_motor(uart_lr, HORIZONTAL_ADDR, pan_target)
    time.sleep(1.5)
    
    tilt_target = INIT_TILT
    move_motor(uart_ud, VERTICAL_ADDR, tilt_target)
    time.sleep(1.5)

    smooth_x, smooth_y = 0.0, 0.0
    last_smooth_x, last_smooth_y = 0.0, 0.0
    last_pan_control = 0
    last_tilt_control = 0
    
    clock = time.clock()

    while True:
        os.exitpoint()
        clock.tick()
        img = sensor.snapshot()

        # 🎯 触控一键退出：点击屏幕【右上角区域】或按下物理 Esc
        if key_esc.is_pressed():
            time.sleep(0.1)
            # 退出前把电机稳住
            move_motor(uart_lr, HORIZONTAL_ADDR, pan_target)
            return

        target_x, target_y = None, None

        # 用你刚刚调好的最新阈值去识别
        blobs = img.find_blobs([PURPLE_THRESHOLD], pixels_threshold=100, area_threshold=100, merge=True)
        for blob in blobs:
            img.draw_rectangle(blob[0:4], color=(255, 0, 255), thickness=2)
            img.draw_cross(blob.cx(), blob.cy(), color=(255, 0, 255), thickness=1)

        if blobs:
            largest = max(blobs, key=lambda b: b.pixels())
            target_x = largest.cx()
            target_y = largest.cy()

        # 闭环追踪运动计算
        if target_x is not None and target_y is not None:
            raw_x = target_x - CX
            raw_y = target_y - CY

            smooth_x = smooth_x * (1 - SMOOTH_X) + raw_x * SMOOTH_X
            smooth_y = smooth_y * (1 - SMOOTH_Y) + raw_y * SMOOTH_Y

            now = time.ticks_ms()

            # 水平轴控制 (Yaw)
            if time.ticks_diff(now, last_pan_control) >= 40:
                last_pan_control = now
                if abs(smooth_x) > DEADZONE_X:
                    err_diff_x = smooth_x - last_smooth_x
                    pos_delta = int(KP_PAN * smooth_x + KD_PAN * err_diff_x)
                    last_smooth_x = smooth_x
                    
                    # 忽略微小的杂乱震动，防止原地嗡嗡抖
                    if abs(pos_delta) < 6:
                        pos_delta = 0

                    if pos_delta != 0:
                        pan_target += PAN_DIR * pos_delta
                        pan_target = max(PAN_LIMIT_MIN, min(PAN_LIMIT_MAX, pan_target))
                        move_motor(uart_lr, HORIZONTAL_ADDR, pan_target)
                else:
                    last_smooth_x = smooth_x

            # 垂直轴控制 (Tilt)
            if time.ticks_diff(now, last_tilt_control) >= 40:
                last_tilt_control = now
                if abs(smooth_y) > DEADZONE_Y:
                    err_diff_y = smooth_y - last_smooth_y
                    pos_delta_y = int(KP_TILT * smooth_y + KD_TILT * err_diff_y)
                    last_smooth_y = smooth_y

                    if abs(pos_delta_y) < 6:
                        pos_delta_y = 0

                    if pos_delta_y != 0:
                        tilt_target += TILT_DIR * pos_delta_y
                        tilt_target = max(TILT_LIMIT_MIN, min(TILT_LIMIT_MAX, tilt_target))
                        move_motor(uart_ud, VERTICAL_ADDR, tilt_target)
                else:
                    last_smooth_y = smooth_y

            img.draw_cross(target_x, target_y, color=(0, 255, 0), size=15, thickness=2)
            img.draw_line(target_x, target_y, CX, CY, color=(255, 200, 0), thickness=2)
            img.draw_string_advanced(20, 40, 24, "Tracking...", color=(0, 255, 255))
        else:
            smooth_x, smooth_y = 0.0, 0.0
            last_smooth_x, last_smooth_y = 0.0, 0.0
            img.draw_string_advanced(20, 40, 24, "Searching...", color=(255, 150, 150))

        img.draw_cross(CX, CY, color=(0, 255, 0), size=25, thickness=3)
        img.draw_circle(CX, CY, 40, color=(0, 255, 0), thickness=2)
        fps = clock.fps()
        img.draw_string_advanced(10, 10, 20, f"FPS: {fps:.1f}", color=(255, 255, 255))

        Display.show_image(img)