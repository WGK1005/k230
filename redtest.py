# ==========================================================
# K230 CanMV v1.4
# GC2093 + ST7701
# RGB888
# find_rects + cv_lite Hough Circle
# X42S 双轴控制
# Part1
# ==========================================================

import time
import math
import gc
import os

import cv_lite
import ulab.numpy as np

from media.sensor import *
from media.display import *
from media.media import *

from machine import UART
from machine import FPIOA

# ==========================================================
# 图像参数
# ==========================================================

W = 800
H = 480

CX = W // 2
CY = H // 2

FPS = 60

# ==========================================================
# 安全区域
# ==========================================================

SAFE_W = int(W * 3 / 5)
SAFE_H = H

SAFE_X1 = (W - SAFE_W) // 2
SAFE_X2 = SAFE_X1 + SAFE_W

SAFE_Y1 = 0
SAFE_Y2 = H

# ==========================================================
# 矩形检测参数
# ==========================================================

RECT_THRESHOLD = 10000

MIN_RECT_AREA = 4000
MAX_RECT_AREA = 200000

RECT_RATIO_MIN = 0.55
RECT_RATIO_MAX = 1.80

ROI_MARGIN = 20

# ==========================================================
# 霍夫圆参数(cv_lite)
# ==========================================================

DP = 1

MIN_DIST = 30

PARAM1 = 80

PARAM2 = 20

MIN_RADIUS = 10

MAX_RADIUS = 60

# ==========================================================
# PID参数
# ==========================================================

KP_X = 0.70
KI_X = 0.00
KD_X = 0.08

KP_Y = 0.70
KI_Y = 0.00
KD_Y = 0.08

MAX_SPEED = 500

# ==========================================================
# 滤波
# ==========================================================

ALPHA = 0.35

# ==========================================================
# X42S地址
# ==========================================================

VERTICAL_ADDR = 1
HORIZONTAL_ADDR = 2

# ==========================================================
# FPIOA
# ==========================================================

fpioa = FPIOA()

# UART1（垂直）
fpioa.set_function(3, FPIOA.UART1_TXD)
fpioa.set_function(4, FPIOA.UART1_RXD)

# UART2（水平）
fpioa.set_function(5, FPIOA.UART2_TXD)
fpioa.set_function(6, FPIOA.UART2_RXD)

uart_ud = UART(1, 115200)
uart_lr = UART(2, 115200)

# ==========================================================
# 摄像头
# ==========================================================
sensor = Sensor(width=W, height=H)

sensor.reset()

sensor.set_framesize(width=W, height=H)

sensor.set_pixformat(Sensor.RGB888)

sensor.run()

sensor.reset()

sensor.set_framesize(width=W, height=H)

sensor.set_pixformat(Sensor.RGB888)

sensor.reset()

sensor.set_framesize(width=W, height=H)

sensor.set_pixformat(Sensor.RGB888)

Display.init(
    Display.ST7701,
    width=W,
    height=H,
    to_ide=True
)

MediaManager.init()

sensor.run()

clock = time.clock()

IMAGE_SHAPE = [H, W]

# ==========================================================
# 全局变量
# ==========================================================

target_x = CX
target_y = CY

filter_x = CX
filter_y = CY

target_found = False

last_rect = None

last_circle = None

# ==========================================================
# 工具函数
# ==========================================================

def clamp(v, mn, mx):

    if v < mn:
        return mn

    if v > mx:
        return mx

    return v


def lowpass(old, new):

    return old * (1 - ALPHA) + new * ALPHA


def distance(x1, y1, x2, y2):

    dx = x1 - x2
    dy = y1 - y2

    return math.sqrt(dx * dx + dy * dy)


def rect_area(r):

    x, y, w, h = r.rect()

    return w * h


def rect_ratio(r):

    x, y, w, h = r.rect()

    return w / h


def valid_rect(r):

    area = rect_area(r)

    if area < MIN_RECT_AREA:
        return False

    if area > MAX_RECT_AREA:
        return False

    ratio = rect_ratio(r)

    if ratio < RECT_RATIO_MIN:
        return False

    if ratio > RECT_RATIO_MAX:
        return False

    return True


def make_roi(r):

    x, y, w, h = r.rect()

    x -= ROI_MARGIN
    y -= ROI_MARGIN

    w += ROI_MARGIN * 2
    h += ROI_MARGIN * 2

    if x < 0:
        x = 0

    if y < 0:
        y = 0

    if x + w >= W:
        w = W - x - 1

    if y + h >= H:
        h = H - y - 1

    return (x, y, w, h)


def draw_cross(img, x, y):

    img.draw_line(x - 12, y, x + 12, y, color=(255,0,0), thickness=2)
    img.draw_line(x, y - 12, x, y + 12, color=(255,0,0), thickness=2)


def inside_safe(x, y):

    return (
        SAFE_X1 <= x <= SAFE_X2 and
        SAFE_Y1 <= y <= SAFE_Y2
    )

# ==========================================================
# Part2
# 矩形检测 + ROI + cv_lite霍夫圆检测
# ==========================================================

def score_rect(r):
    """
    矩形评分
    越靠近屏幕中心分数越高
    """

    x, y, w, h = r.rect()

    cx = x + w // 2
    cy = y + h // 2

    area = w * h

    dist = distance(cx, cy, CX, CY)

    score = area - dist * 20

    return score


# ==========================================================

def find_target_rect(img):

    rects = img.find_rects(threshold=RECT_THRESHOLD)

    best = None
    best_score = -999999

    for r in rects:

        if not valid_rect(r):
            continue

        s = score_rect(r)

        if s > best_score:

            best_score = s
            best = r

    return best


# ==========================================================

def draw_rect(img, r):

    img.draw_rectangle(
        [v for v in r.rect()],
        color=(255,0,0),
        thickness=2
    )

    for p in r.corners():

        img.draw_circle(
            p[0],
            p[1],
            4,
            color=(0,255,0),
            thickness=2
        )


# ==========================================================

def roi_to_numpy(img, roi):

    x, y, w, h = roi

    crop = img.copy(roi=roi)

    return crop.to_numpy_ref(), [h, w]


# ==========================================================

def find_circle_roi(img, roi):

    x0, y0, w, h = roi

    crop_np, shape = roi_to_numpy(img, roi)

    circles = cv_lite.rgb888_find_circles(

        shape,

        crop_np,

        DP,

        MIN_DIST,

        PARAM1,

        PARAM2,

        MIN_RADIUS,

        MAX_RADIUS

    )

    if len(circles) < 3:

        return None

    best = None

    best_r = 0

    for i in range(0, len(circles), 3):

        cx = circles[i]
        cy = circles[i + 1]
        r = circles[i + 2]

        if r > best_r:

            best_r = r
            best = (
                cx + x0,
                cy + y0,
                r
            )

    return best


# ==========================================================

def detect_target(img):

    global target_found
    global target_x
    global target_y
    global last_rect
    global last_circle

    target_found = False

    rect = find_target_rect(img)

    if rect is None:
        return False

    last_rect = rect

    draw_rect(img, rect)

    roi = make_roi(rect)

    img.draw_rectangle(
        roi,
        color=(0,255,255),
        thickness=2
    )

    circle = find_circle_roi(img, roi)

    if circle is None:
        return False

    last_circle = circle

    x = int(circle[0])
    y = int(circle[1])
    r = int(circle[2])

    target_x = x
    target_y = y

    img.draw_circle(
        x,
        y,
        r,
        color=(255,255,0),
        thickness=2
    )

    draw_cross(img, x, y)

    target_found = True

    return True


# ==========================================================

def update_target():

    global filter_x
    global filter_y

    filter_x = lowpass(filter_x, target_x)

    filter_y = lowpass(filter_y, target_y)


# ==========================================================

def get_error():

    err_x = filter_x - CX
    err_y = filter_y - CY

    return err_x, err_y


# ==========================================================
# Part3
# PID控制
# X42S速度控制
# ==========================================================
# ==========================================================
# Part3
# PID控制 + X42S速度控制
# ==========================================================

last_err_x = 0
last_err_y = 0

sum_err_x = 0
sum_err_y = 0

last_pid_time = time.ticks_ms()

# ==========================================================

def pid_update(err_x, err_y):

    global last_err_x
    global last_err_y

    global sum_err_x
    global sum_err_y

    global last_pid_time

    now = time.ticks_ms()

    dt = time.ticks_diff(now, last_pid_time) / 1000.0

    if dt <= 0:
        dt = 0.01

    last_pid_time = now

    sum_err_x += err_x * dt
    sum_err_y += err_y * dt

    sum_err_x = clamp(sum_err_x, -300, 300)
    sum_err_y = clamp(sum_err_y, -300, 300)

    der_x = (err_x - last_err_x) / dt
    der_y = (err_y - last_err_y) / dt

    last_err_x = err_x
    last_err_y = err_y

    vx = KP_X * err_x + KI_X * sum_err_x + KD_X * der_x
    vy = KP_Y * err_y + KI_Y * sum_err_y + KD_Y * der_y

    vx = int(clamp(vx, -MAX_SPEED, MAX_SPEED))
    vy = int(clamp(vy, -MAX_SPEED, MAX_SPEED))

    return vx, vy

# ==========================================================
# CRC16(Modbus)
# ==========================================================

def crc16(data):

    crc = 0xFFFF

    for b in data:

        crc ^= b

        for _ in range(8):

            if crc & 1:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1

    return crc


# ==========================================================
# X42S C6 限流速度模式
# ==========================================================

ACC = 1000          # RPM/S
CURRENT = 2000      # mA


def motor_speed(uart, addr, rpm):

    if rpm >= 0:
        direction = 0x00      # CW
    else:
        direction = 0x01      # CCW
        rpm = -rpm

    rpm = int(clamp(rpm,0,3000))

    speed = int(rpm * 10)

    packet = bytearray()

    packet.append(addr)

    packet.append(0xC6)

    packet.append(direction)

    packet.append((ACC >> 8) & 0xFF)
    packet.append(ACC & 0xFF)

    packet.append((speed >> 8) & 0xFF)
    packet.append(speed & 0xFF)

    packet.append(0x00)          # 立即执行

    packet.append((CURRENT >> 8) & 0xFF)
    packet.append(CURRENT & 0xFF)

    crc = crc16(packet)

    packet.append(crc & 0xFF)
    packet.append((crc >> 8) & 0xFF)

    uart.write(packet)
# ==========================================================

def stop_motor():

    motor_speed(
        uart_ud,
        VERTICAL_ADDR,
        0
    )

    motor_speed(
        uart_lr,
        HORIZONTAL_ADDR,
        0
    )

# ==========================================================

def track_target():

    if not target_found:

        stop_motor()
        return

    err_x, err_y = get_error()

    vx, vy = pid_update(
        err_x,
        err_y
    )

    dead = 3

    if abs(err_x) < dead:
        vx = 0

    if abs(err_y) < dead:
        vy = 0

    motor_speed(
        uart_lr,
        HORIZONTAL_ADDR,
        vx
    )

    motor_speed(
        uart_ud,
        VERTICAL_ADDR,
        vy
    )

# ==========================================================
# 调试显示
# ==========================================================

def draw_debug(img):

    img.draw_string(
        5,
        5,
        "X:%d" % int(filter_x),
        color=(255,255,0),
        scale=2
    )

    img.draw_string(
        5,
        30,
        "Y:%d" % int(filter_y),
        color=(255,255,0),
        scale=2
    )

    img.draw_string(
        5,
        55,
        "FOUND:%d" % target_found,
        color=(0,255,0),
        scale=2
    )

    img.draw_rectangle(
        SAFE_X1,
        SAFE_Y1,
        SAFE_W,
        SAFE_H,
        color=(0,255,255),
        thickness=2
    )

    draw_cross(img, CX, CY)
    # ==========================================================
    # Part4
    # Main Loop
    # ==========================================================

    while True:

        clock.tick()

        os.exitpoint()

        img = sensor.snapshot()

        target_found = False

        detect_target(img)

        if target_found:

            update_target()

            track_target()

        else:

            stop_motor()

        draw_debug(img)

        img.draw_string(
            5,
            H - 30,
            "FPS: %.1f" % clock.fps(),
            color=(255,255,255),
            scale=2
        )

        Display.show_image(img)

        gc.collect()
