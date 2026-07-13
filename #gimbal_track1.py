# gimbal_track_with_ti.py (K230 上位机端)
import time, os, sys
import math
import struct
import cv_lite  
import ulab.numpy as np  
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA

# --------------------------- 1. 串口与引脚底层映射 ---------------------------
fpioa = FPIOA()

# UART1（云台总线，一驱二：同时控制 0x01 垂直轴 & 0x02 水平轴）
fpioa.set_function(3, FPIOA.UART1_TXD)
fpioa.set_function(4, FPIOA.UART1_RXD)

# UART2（下位机通信：发送偏差与指令给 TI 运动控制板）
fpioa.set_function(11, FPIOA.UART2_TXD)
fpioa.set_function(12, FPIOA.UART2_RXD)

# 实例化串口
uart_gimbal = UART(UART.UART1, baudrate=115200) # 云台电机
uart_ti = UART(UART.UART2, baudrate=115200)     # TI板底盘

VERTICAL_ADDR = 0x01
HORIZONTAL_ADDR = 0x02

# 运动方向极性配置
PAN_DIR  = -1                  
TILT_DIR = 1                   

# --------------------------- 2. 视觉算法常量 ---------------------------
canny_thresh1      = 30        
canny_thresh2      = 90        
approx_epsilon     = 0.04      
area_min_ratio     = 0.002     
max_angle_cos      = 0.55      
gaussian_blur_size = 3  

MIN_AREA = 100               
MAX_AREA = 300000             
MIN_ASPECT_RATIO = 0.3        
MAX_ASPECT_RATIO = 3.0        

BASE_RADIUS = 45              
POINTS_PER_CIRCLE = 50        
RECT_WIDTH = 250              
RECT_HEIGHT = 200             

# --------------------------- 3. 电机硬限位与安全保护 ---------------------------
PAN_LIMIT_MIN = -3500         
PAN_LIMIT_MAX = 3500          
TILT_LIMIT_MIN = 0            
TILT_LIMIT_MAX = 1600         
PULSE_PER_REV = 3200          
INIT_TILT = PULSE_PER_REV // 4 

# --------------------------- 4. 速位结合控制算法参数 ---------------------------
DEADZONE_X = 12               
DEADZONE_Y = 8                

SMOOTH_X = 0.30               
SMOOTH_Y = 0.35

KP_PAN   = 0.20               
KD_PAN   = 0.12               

KP_TILT  = 0.45
KD_TILT  = 0.15

V_MIN_PAN  = 100
V_MAX_PAN  = 800
KV_PAN     = 1.8

V_MIN_TILT = 100
V_MAX_TILT = 800
KV_TILT    = 1.8

PAN_CONTROL_PERIOD = 40       
TILT_CONTROL_PERIOD = 40      

# --------------------------- 5. 电机与下位机通讯函数 ---------------------------
def send_motor_cmd(cmd):
    uart_gimbal.write(bytes(cmd))

def fast_mode_init(addr, speed=350, acc=5):
    cmd = [
        addr,
        0xF1,
        (speed >> 8) & 0xff,
        speed & 0xff,
        acc,
        0x01,  
        0x00,  
        0x6B
    ]
    send_motor_cmd(cmd)

def set_speed_ud(speed, last_speed):
    if abs(speed - last_speed) >= 20:
        fast_mode_init(VERTICAL_ADDR, speed)
        return speed
    return last_speed

def set_speed_lr(speed, last_speed):
    if abs(speed - last_speed) >= 20:
        fast_mode_init(HORIZONTAL_ADDR, speed)
        return speed
    return last_speed

def move_motor(addr, target):
    if target < 0:
        target = (1 << 32) + target
    cmd = [
        addr,
        0xFC,
        (target >> 24) & 0xff,
        (target >> 16) & 0xff,
        (target >> 8) & 0xff,
        target & 0xff,
        0x6B
    ]
    send_motor_cmd(cmd)

# 🎯 核心优化：发送给 TI 下位机的数据打包函数（含帧头、校验和、帧尾）
def send_to_ti(target_type_id, dx, dy, pan, tilt):
    """
    target_type_id: 0-无目标, 1-矩形靶盘, 2-红色色块靶心
    dx, dy: 目标中心相对于相机的像素偏差
    pan, tilt: 当前云台电机的目标脉冲位置（用于TI判断相对角度）
    """
    # 格式：0x55 0xAA (帧头) + 1字节类型 + 4个16位有符号整型(dx, dy, pan, tilt) -> 共11字节
    header = bytes([0x55, 0xAA])
    payload = struct.pack('<Bhhhh', target_type_id, int(dx), int(dy), int(pan), int(tilt))
    
    # 简单加和校验（防EMI电磁干扰导致小车乱跑）
    checksum = sum(payload) & 0xFF
    
    # 最终打包发送
    packet = header + payload + bytes([checksum, 0x0D, 0x0A])
    uart_ti.write(packet)

# --------------------------- 6. 全局辅助数学与拟合函数 ---------------------------
def calculate_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def calculate_center(points):
    if not points:
        return (0, 0)
    sum_x = sum(p[0] for p in points)
    sum_y = sum(p[1] for p in points)
    return (sum_x / len(points), sum_y / len(points))

def is_valid_rect(corners):
    edges = [calculate_distance(corners[i], corners[(i+1)%4]) for i in range(4)]
    ratio1 = edges[0] / max(edges[2], 0.1)
    ratio2 = edges[1] / max(edges[3], 0.1)
    valid_ratio = 0.5 < ratio1 < 1.5 and 0.5 < ratio2 < 1.5

    area = 0
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[(i+1) % 4]
        area += (x1 * y2 - x2 * y1)
    area = abs(area) / 2
    valid_area = MIN_AREA < area < MAX_AREA

    width = max(p[0] for p in corners) - min(p[0] for p in corners)
    height = max(p[1] for p in corners) - min(p[1] for p in corners)
    aspect_ratio = width / max(height, 0.1)
    valid_aspect = MIN_ASPECT_RATIO < aspect_ratio < MAX_ASPECT_RATIO

    return valid_ratio and valid_area and valid_aspect

def get_perspective_matrix(src_pts, dst_pts):
    A = []
    B = []
    for i in range(4):
        x, y = src_pts[i]
        u, v = dst_pts[i]
        A.append([x, y, 1, 0, 0, 0, -u*x, -u*y])
        A.append([0, 0, 0, x, y, 1, -v*x, -v*y])
        B.append(u)
        B.append(v)

    n = 8
    for i in range(n):
        max_row = i
        for j in range(i, len(A)):
            if abs(A[j][i]) > abs(A[max_row][i]):
                max_row = j
        A[i], A[max_row] = A[max_row], A[i]
        B[i], B[max_row] = B[max_row], B[i]

        pivot = A[i][i]
        if abs(pivot) < 1e-8:
            return None
        for j in range(i, n):
            A[i][j] /= pivot
        B[i] /= pivot

        for j in range(len(A)):
            if j != i and A[j][i] != 0:
                factor = A[j][i]
                for k in range(i, n):
                    A[j][k] -= factor * A[i][k]
                B[j] -= factor * B[i]

    return [
        [B[0], B[1], B[2]],
        [B[3], B[4], B[5]],
        [B[6], B[7], 1.0]
    ]

def transform_points(points, matrix):
    transformed = []
    for (x, y) in points:
        x_hom = x * matrix[0][0] + y * matrix[0][1] + matrix[0][2]
        y_hom = x * matrix[1][0] + y * matrix[1][1] + matrix[1][2]
        w_hom = x * matrix[2][0] + y * matrix[2][1] + matrix[2][2]
        if abs(w_hom) > 1e-8:
            transformed.append((x_hom / w_hom, y_hom / w_hom))
    return transformed

def sort_corners(corners):
    center = calculate_center(corners)
    sorted_corners = sorted(corners, key=lambda p: math.atan2(p[1]-center[1], p[0]-center[0]))
    if len(sorted_corners) == 4:
        left_top = min(sorted_corners, key=lambda p: p[0]+p[1])
        index = sorted_corners.index(left_top)
        sorted_corners = sorted_corners[index:] + sorted_corners[:index]
    return sorted_corners

def detect_purple_blobs(img, threshold, roi=None):
    roi_safe = (0, 0, img.width(), img.height())
    if roi:
        rx, ry, rw, rh = roi
        margin = 15
        x = max(0, rx - margin)
        y = max(0, ry - margin)
        w = min(img.width() - x, rw + 2 * margin)
        h = min(img.height() - y, rh + 2 * margin)
        roi_safe = (x, y, w, h)

    blobs = img.find_blobs(
        [threshold],
        roi=roi_safe,
        pixels_threshold=80,
        area_threshold=80,
        merge=True,
        margin=12
    )
    
    if blobs:
        largest_blob = max(blobs, key=lambda b: b.pixels())
        bw, bh = largest_blob[2], largest_blob[3]
        ratio = bw / max(bh, 0.1)
        if 0.4 < ratio < 2.5:
            return [largest_blob]
    return []

# --------------------------- 7. 模块化追踪入口主函数 ---------------------------
def run_gimbal_tracking(sensor, tp, key_esc, thresholds, width=800, height=480):
    
    fpioa = FPIOA()
    fpioa.set_function(3, FPIOA.UART1_TXD) 
    fpioa.set_function(4, FPIOA.UART1_RXD)
    fpioa.set_function(5, FPIOA.UART2_TXD) 
    fpioa.set_function(6, FPIOA.UART2_RXD)

    print("正在等待触摸释放...")
    while True:
        points = tp.read(1)
        if not points:
            break
        time.sleep_ms(20)

    lcd_width = width
    lcd_height = height
    CX = lcd_width // 2
    CY = lcd_height // 2

    PURPLE_THRESHOLD = tuple(thresholds[0])
    print("加载实时 LAB 追踪阈值:", PURPLE_THRESHOLD)

    # --------------------------- 8. 初始化状态变量 ---------------------------
    smooth_x = 0.0
    smooth_y = 0.0
    last_smooth_x = 0.0
    last_smooth_y = 0.0

    last_pan_control = 0
    last_tilt_control = 0

    pan_target = 0
    tilt_target = 0

    last_sent_speed_ud = 350      
    last_sent_speed_lr = 350      

    # ------------------ 9. 云台通电归零与就位 ------------------
    print("正在加载绝对位置驱动配置...")
    
    fast_mode_init(HORIZONTAL_ADDR, 350) 
    fast_mode_init(VERTICAL_ADDR, 350)
    time.sleep_ms(200)

    print("云台双轴归零...")
    move_motor(VERTICAL_ADDR, 0)
    move_motor(HORIZONTAL_ADDR, 0)
    time.sleep(1.5)

    print("俯仰抬升到初始工作高度...")
    tilt_target = INIT_TILT
    move_motor(VERTICAL_ADDR, tilt_target)
    time.sleep(1.5)
    print("安全校准完成，视觉闭环就绪...")

    clock = time.clock()
    
    first_img = sensor.snapshot(chn=CAM_CHN_ID_0)
    image_shape = [first_img.height(), first_img.width()]
    print(f"当前输入图像解析分辨率: {image_shape[1]}x{image_shape[0]}")

    # --------------------------- 10. 闭环控制核心循环 ---------------------------
    try:
        while True:
            os.exitpoint()
            clock.tick()
            
            img = sensor.snapshot(chn=CAM_CHN_ID_0)

            if key_esc.is_pressed():
                print("收到退出信号，云台复位返回...")
                time.sleep_ms(100)
                move_motor(HORIZONTAL_ADDR, 0)
                move_motor(VERTICAL_ADDR, INIT_TILT)
                time.sleep_ms(200)
                return

            target_x = None
            target_y = None
            target_type_id = 0 # 0-无目标, 1-矩形, 2-色块
            target_type_str = "None"

            # 优先矩形检测
            gray_img = img.to_grayscale()
            img_np = gray_img.to_numpy_ref()

            rects = cv_lite.grayscale_find_rectangles_with_corners(
                image_shape, img_np, canny_thresh1, canny_thresh2,
                approx_epsilon, area_min_ratio, max_angle_cos, gaussian_blur_size
            )

            min_area = float('inf')
            smallest_rect = None
            smallest_rect_corners = None

            for rect in rects:
                x_r, y_r, w_r, h_r = rect[0], rect[1], rect[2], rect[3]
                corners = [
                    (rect[4], rect[5]),
                    (rect[6], rect[7]),
                    (rect[8], rect[9]),
                    (rect[10], rect[11])
                ]
                if is_valid_rect(corners):
                    area = w_r * h_r
                    if area < min_area:
                        min_area = area
                        smallest_rect = (x_r, y_r, w_r, h_r)
                        smallest_rect_corners = corners

            # 色块检测（受 ROI 保护）
            purple_blobs = detect_purple_blobs(img, PURPLE_THRESHOLD, roi=smallest_rect)
            for blob in purple_blobs:
                img.draw_rectangle(blob[0:4], color=(255, 0, 255), thickness=1)
                img.draw_cross(blob.cx(), blob.cy(), color=(255, 0, 255), thickness=1)

            # 目标决策
            if smallest_rect and smallest_rect_corners:
                x_r, y_r, w_r, h_r = smallest_rect
                corners = smallest_rect_corners
                sorted_corners = sort_corners(corners)

                for i in range(4):
                    x1, y1 = sorted_corners[i]
                    x2, y2 = sorted_corners[(i+1) % 4]
                    img.draw_line(x1, y1, x2, y2, color=(255, 0, 0), thickness=2)
                for p in sorted_corners:
                    img.draw_circle(p[0], p[1], 5, color=(0, 255, 0), thickness=2)

                virtual_rect = [(0, 0), (RECT_WIDTH, 0), (RECT_WIDTH, RECT_HEIGHT), (0, RECT_HEIGHT)]
                radius_x = BASE_RADIUS
                radius_y = BASE_RADIUS
                virtual_center = (RECT_WIDTH / 2, RECT_HEIGHT / 2)

                virtual_circle_points = []
                for i in range(POINTS_PER_CIRCLE):
                    angle_rad = 2 * math.pi * i / POINTS_PER_CIRCLE
                    x_virt = virtual_center[0] + radius_x * math.cos(angle_rad)
                    y_virt = virtual_center[1] + radius_y * math.sin(angle_rad)
                    virtual_circle_points.append((x_virt, y_virt))

                matrix = get_perspective_matrix(virtual_rect, sorted_corners)
                if matrix:
                    mapped_points = transform_points(virtual_circle_points, matrix)
                    int_points = [(int(round(px)), int(round(py))) for px, py in mapped_points]

                    for (px, py) in int_points:
                        img.draw_circle(px, py, 2, color=(255, 0, 255), thickness=2)

                    mapped_center = transform_points([virtual_center], matrix)
                    if mapped_center:
                        cx, cy = map(int, map(round, mapped_center[0]))
                        img.draw_circle(cx, cy, 3, color=(0, 0, 255), thickness=1)

                        target_x = cx
                        target_y = cy
                        target_type_id = 1
                        target_type_str = "Rectangle"

            elif purple_blobs:
                largest_purple = purple_blobs[0]
                target_x = largest_purple.cx()
                target_y = largest_purple.cy()
                target_type_id = 2
                target_type_str = "Purple Blob"

            # 闭环控制与数据下发
            if target_x is not None and target_y is not None:
                raw_x = target_x - CX
                raw_y = target_y - CY

                smooth_x = smooth_x * (1 - SMOOTH_X) + raw_x * SMOOTH_X
                smooth_y = smooth_y * (1 - SMOOTH_Y) + raw_y * SMOOTH_Y

                now = time.ticks_ms()

                # 云台偏航控制
                if time.ticks_diff(now, last_pan_control) >= PAN_CONTROL_PERIOD:
                    last_pan_control = now
                    if abs(smooth_x) > DEADZONE_X:
                        err_diff_x = smooth_x - last_smooth_x
                        pos_delta = int(KP_PAN * smooth_x + KD_PAN * err_diff_x)
                        last_smooth_x = smooth_x

                        v_pan = int(V_MIN_PAN + KV_PAN * abs(smooth_x))
                        v_pan = max(V_MIN_PAN, min(V_MAX_PAN, v_pan))
                        last_sent_speed_lr = set_speed_lr(v_pan, last_sent_speed_lr)

                        if abs(pos_delta) < 4:
                            pos_delta = 0

                        if pos_delta != 0:
                            pan_target += PAN_DIR * pos_delta
                            pan_target = max(PAN_LIMIT_MIN, min(PAN_LIMIT_MAX, pan_target))
                            move_motor(HORIZONTAL_ADDR, pan_target)
                    else:
                        last_smooth_x = smooth_x

                # 云台俯仰控制
                if time.ticks_diff(now, last_tilt_control) >= TILT_CONTROL_PERIOD:
                    last_tilt_control = now
                    if abs(smooth_y) > DEADZONE_Y:
                        err_diff_y = smooth_y - last_smooth_y
                        pos_delta_y = int(KP_TILT * smooth_y + KD_TILT * err_diff_y)
                        last_smooth_y = smooth_y

                        v_tilt = int(V_MIN_TILT + KV_TILT * abs(smooth_y))
                        v_tilt = max(V_MIN_TILT, min(V_MAX_TILT, v_tilt))
                        last_sent_speed_ud = set_speed_ud(v_tilt, last_sent_speed_ud)

                        tilt_target += TILT_DIR * pos_delta_y
                        tilt_target = max(TILT_LIMIT_MIN, min(TILT_LIMIT_MAX, tilt_target))
                        move_motor(VERTICAL_ADDR, tilt_target)
                    else:
                        last_smooth_y = smooth_y

                # 🎯 下发坐标给 TI 下位机控制底盘行进
                send_to_ti(target_type_id, raw_x, raw_y, pan_target, tilt_target)

                img.draw_cross(target_x, target_y, color=(0, 255, 0), size=15, thickness=2)
                img.draw_line(target_x, target_y, CX, CY, color=(255, 200, 0), thickness=2)
                img.draw_string_advanced(20, 40, 32, f"Tracking: {target_type_str}", color=(0, 255, 255))
            else:
                smooth_x, smooth_y = 0.0, 0.0
                last_smooth_x, last_smooth_y = 0.0, 0.0
                img.draw_string_advanced(20, 40, 32, "Searching...", color=(255, 150, 150))
                # 无目标时发送无目标包
                send_to_ti(0, 0, 0, pan_target, tilt_target)

            img.draw_cross(CX, CY, color=(0, 255, 0), size=25, thickness=3)
            img.draw_circle(CX, CY, 40, color=(0, 255, 0), thickness=2)

            fps = clock.fps()
            img.draw_string_advanced(10, 10, 20, f"FPS: {fps:.1f}", color=(255, 255, 255))

            Display.show_image(img,
                              x=round((lcd_width - sensor.width()) / 2),
                              y=round((lcd_height - sensor.height()) / 2))

    except Exception as e:
        print(f"云台追踪线程中发生致命错误: {e}")
    finally:
        print("正在安全卸载云台控制...")