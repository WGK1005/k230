# gimbal_track.py (丝滑追踪、多目标记忆抗闪烁、激光联动稳定版)
import time, os, sys
import math
import cv_lite  
import ulab.numpy as np  
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA, Pin

# --------------------------- 1. 串口、激光与引脚底层映射 ---------------------------
fpioa = FPIOA()

# UART1（垂直/俯仰轴）
fpioa.set_function(3, FPIOA.UART1_TXD)
fpioa.set_function(4, FPIOA.UART1_RXD)

# UART2（水平/偏航轴）
fpioa.set_function(5, FPIOA.UART2_TXD)
fpioa.set_function(6, FPIOA.UART2_RXD)

# 🎯 激光控制配置（GPIO 18）使用标准的 Pin 对象
fpioa.set_function(18, FPIOA.GPIO18)
laser = Pin(18, Pin.OUT, value=0)  

uart_ud = UART(UART.UART1, baudrate=115200)
uart_lr = UART(UART.UART2, baudrate=115200)

VERTICAL_ADDR = 0x01
HORIZONTAL_ADDR = 0x02

PAN_DIR  = -1                  
TILT_DIR = 1                   

# --------------------------- 2. 视觉算法常量 (已放宽以防边缘断裂丢帧) ---------------------------
canny_thresh1      = 28        # 🎯 下调下限：确保反光、弱光时光滑边缘不会因为断裂被漏检
canny_thresh2      = 85        # 下调上限：提高边缘连通率
approx_epsilon     = 0.04      
area_min_ratio     = 0.002     # 🎯 允许更远的靶标参与检测
max_angle_cos      = 0.55      

gaussian_blur_size = 3  

MIN_AREA = 150                 # 适当放低，防止远距离丢帧
MAX_AREA = 250000             
MIN_ASPECT_RATIO = 0.35        
MAX_ASPECT_RATIO = 2.8        

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

# --------------------------- 4. 控制算法参数 ---------------------------
DEADZONE_X = 10               
DEADZONE_Y = 8                

SMOOTH_X = 0.10               # 🎯 微幅增大滤波，让视觉跳变时的连贯性更好
SMOOTH_Y = 0.12

KP_PAN   = 0.25               
KD_PAN   = -0.10               

KP_TILT  = 0.55
KD_TILT  = -0.18

V_MIN_PAN  = 200              
V_MAX_PAN  = 900             
KV_PAN     = 3.0              

V_MIN_TILT = 120
V_MAX_TILT = 900
KV_TILT    = 2.2

PAN_CONTROL_PERIOD = 50       
TILT_CONTROL_PERIOD = 40      

# --------------------------- 5. 电机串口底层通讯函数 ---------------------------
def send(uart, cmd):
    uart.write(bytes(cmd))

def fast_mode_init(uart, addr, speed=350, acc=5):
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
    send(uart, cmd)

def set_speed_ud(speed, last_speed):
    if abs(speed - last_speed) >= 25:
        fast_mode_init(uart_ud, VERTICAL_ADDR, speed)
        return speed
    return last_speed

def set_speed_lr(speed, last_speed):
    if abs(speed - last_speed) >= 25:
        fast_mode_init(uart_lr, HORIZONTAL_ADDR, speed)
        return speed
    return last_speed

def move_motor(uart, addr, target):
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
    send(uart, cmd)

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
    valid_ratio = 0.45 < ratio1 < 2.2 and 0.45 < ratio2 < 2.2

    area = 0
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[(i+1) % 4]
        area += (x1 * y2 - x2 * y1)
    area = abs(area) / 2
    valid_area = MIN_AREA < area < MAX_AREA

    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    width = max(xs) - min(xs)
    height = max(ys) - min(ys)
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

# --------------------------- 7. 模块化追踪入口主函数 ---------------------------
def run_gimbal_tracking(sensor, tp, key_esc, thresholds, width=800, height=480):
    
    fpioa = FPIOA()
    fpioa.set_function(3, FPIOA.UART1_TXD) 
    fpioa.set_function(4, FPIOA.UART1_RXD)
    fpioa.set_function(5, FPIOA.UART2_TXD) 
    fpioa.set_function(6, FPIOA.UART2_RXD)
    
    fpioa.set_function(18, FPIOA.GPIO18)
    laser.value(0) 

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
    last_sent_speed_lr = 250      

    # 🎯 视觉追踪记忆状态机核心变量
    last_target_x = None
    last_target_y = None
    lost_frame_count = 0          # 靶标丢失的连续帧数计数器
    MAX_LOST_FRAMES = 2           # 🎯 核心升级：允许最大连续 2 帧丢失，利用盲补技术抹平“一段一段”的闪烁感

    # ------------------ 9. 云台通电归零与就位 ------------------
    print("正在加载绝对位置驱动配置...")
    fast_mode_init(uart_lr, HORIZONTAL_ADDR, 250) 
    fast_mode_init(uart_ud, VERTICAL_ADDR, 350)
    time.sleep_ms(200)

    print("云台双轴归零...")
    move_motor(uart_ud, VERTICAL_ADDR, 0)
    move_motor(uart_lr, HORIZONTAL_ADDR, 0)
    time.sleep(1.5)

    print("俯仰抬升到初始工作高度...")
    tilt_target = INIT_TILT
    move_motor(uart_ud, VERTICAL_ADDR, tilt_target)
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
                move_motor(uart_lr, HORIZONTAL_ADDR, 0)
                move_motor(uart_ud, VERTICAL_ADDR, INIT_TILT)
                time.sleep_ms(200)
                return

            target_x = None
            target_y = None
            target_type = "None"

            # (1) 优先使用 cv_lite 引擎进行矩形检测
            gray_img = img.to_grayscale()
            img_np = gray_img.to_numpy_ref()

            rects = cv_lite.grayscale_find_rectangles_with_corners(
                image_shape, img_np, canny_thresh1, canny_thresh2,
                approx_epsilon, area_min_ratio, max_angle_cos, gaussian_blur_size
            )

            best_rect = None
            best_rect_corners = None
            
            # 🎯 升级筛选机制：带有前一帧记忆关联的距离筛选器
            if last_target_x is not None and lost_frame_count == 0:
                # 如果上一帧锁定成功，当前帧优先筛选“靠近上一帧已知目标位置”的矩形，防背景杂物抢锁
                min_dist_to_last = float('inf')
                for rect in rects:
                    corners = [(rect[4], rect[5]), (rect[6], rect[7]), (rect[8], rect[9]), (rect[10], rect[11])]
                    if is_valid_rect(corners):
                        r_cx, r_cy = calculate_center(corners)
                        dist = math.sqrt((r_cx - last_target_x)**2 + (r_cy - last_target_y)**2)
                        # 只有离上一帧目标小于 120 像素的，才被认为是同一个靶标
                        if dist < min_dist_to_last and dist < 120:
                            min_dist_to_last = dist
                            best_rect = (rect[0], rect[1], rect[2], rect[3])
                            best_rect_corners = corners
            
            # 如果记忆定位未触发（初次捕获），退回到离画面中心最近的全局筛选器
            if best_rect is None:
                min_dist_to_center = float('inf')
                for rect in rects:
                    corners = [(rect[4], rect[5]), (rect[6], rect[7]), (rect[8], rect[9]), (rect[10], rect[11])]
                    if is_valid_rect(corners):
                        r_cx, r_cy = calculate_center(corners)
                        dist = math.sqrt((r_cx - CX)**2 + (r_cy - CY)**2)
                        if dist < min_dist_to_center:
                            min_dist_to_center = dist
                            best_rect = (rect[0], rect[1], rect[2], rect[3])
                            best_rect_corners = corners

            # (2) 目标优先级决策
            if best_rect and best_rect_corners:
                lost_frame_count = 0  # 锁定成功，清空丢失计数器
                x_r, y_r, w_r, h_r = best_rect
                corners = best_rect_corners
                sorted_corners = sort_corners(corners)

                for i in range(4):
                    x1, y1 = sorted_corners[i]
                    x2, y2 = sorted_corners[(i+1) % 4]
                    img.draw_line(x1, y1, x2, y2, color=(0, 255, 0), thickness=2) 

                virtual_rect = [(0, 0), (RECT_WIDTH, 0), (RECT_WIDTH, RECT_HEIGHT), (0, RECT_HEIGHT)]
                radius_x = BASE_RADIUS; radius_y = BASE_RADIUS
                virtual_center = (RECT_WIDTH / 2, RECT_HEIGHT / 2)

                virtual_circle_points = []
                for i in range(POINTS_PER_CIRCLE):
                    angle_rad = 2 * math.pi * i / POINTS_PER_CIRCLE
                    virtual_circle_points.append((virtual_center[0] + radius_x * math.cos(angle_rad), virtual_center[1] + radius_y * math.sin(angle_rad)))

                matrix = get_perspective_matrix(virtual_rect, sorted_corners)
                if matrix:
                    mapped_points = transform_points(virtual_circle_points, matrix)
                    for px, py in [(int(round(p[0])), int(round(p[1]))) for p in mapped_points]:
                        img.draw_circle(px, py, 2, color=(255, 0, 255), thickness=1)

                    mapped_center = transform_points([virtual_center], matrix)
                    if mapped_center:
                        target_x, target_y = map(int, map(round, mapped_center[0]))
                        img.draw_circle(target_x, target_y, 4, color=(0, 0, 255), thickness=-1)
                        target_type = "Rectangle"

            else:
                # 如果矩形未检出，降级使用颜色块兜底
                purple_blobs = img.find_blobs([PURPLE_THRESHOLD], pixels_threshold=150, area_threshold=150, merge=True)
                if purple_blobs:
                    largest_purple = max(purple_blobs, key=lambda b: b.pixels())
                    if 0.25 < (largest_purple.w() / max(largest_purple.h(), 0.1)) < 4.0:
                        lost_frame_count = 0
                        target_x = largest_purple.cx()
                        target_y = largest_purple.cy()
                        target_type = "Purple Blob"
                        img.draw_rectangle(largest_purple[0:4], color=(255, 0, 255), thickness=2)

            # 🎯 核心抗丢帧技术：一阶惯性盲补算法
            if target_x is None and last_target_x is not None:
                lost_frame_count += 1
                if lost_frame_count <= MAX_LOST_FRAMES:
                    # 🎯 靶标短暂丢失，强制使用前一帧的记忆坐标进行盲补，假装目标仍然存在
                    target_x = last_target_x
                    target_y = last_target_y
                    target_type = "補帧预测 (Memory)"
                    img.draw_string_advanced(20, 80, 24, f"Frame Lost! Blind tracking...", color=(255, 255, 0))

 # (4) 闭环速位结合跟踪控制 (终极高频丝滑滑入版)
            if target_x is not None and target_y is not None:
                # 稳定维持历史坐标
                last_target_x = target_x
                last_target_y = target_y
                
                # 点亮激光
                laser.value(1)

                raw_x = target_x - CX
                raw_y = target_y - CY

                # 动态自适应跳变重置
                if abs(raw_x - smooth_x) > 150:
                    smooth_x = float(raw_x)
                    last_smooth_x = float(raw_x)

                # 🎯 调整滤波：分辨率小了之后，平滑系数可以适当加大（比如 0.12），滤掉视觉像素跳动
                smooth_x = smooth_x * (1 - 0.12) + raw_x * 0.12
                smooth_y = smooth_y * (1 - 0.15) + raw_y * 0.15

                now = time.ticks_ms()

                # ==========================================
                # 水平轴 (Yaw) 控制 (高频微步、阻尼缓停机制)
                # ==========================================
                # 🎯 核心改变：控制周期从 50ms 缩短到 25ms (40Hz)，大幅提升指令连贯性
                if time.ticks_diff(now, last_pan_control) >= 25:
                    last_pan_control = now
                    
                    if abs(smooth_x) > DEADZONE_X:
                        err_diff_x = smooth_x - last_smooth_x
                        
                        # 降低基础比例系数，把主力交给微步高频更新
                        pos_delta = int(0.18 * smooth_x + 0.10 * err_diff_x)
                        last_smooth_x = smooth_x

                        if abs(pos_delta) < 2: 
                            pos_delta = 0
                        
                        if pos_delta != 0:
                            # 🎯 指数阻尼软着陆：离中心越近，单次步数被压得越低，实现丝滑刹车
                            abs_error_x = abs(smooth_x)
                            if abs_error_x < 35:  # 进入近靶心缓动区
                                pos_delta = int(pos_delta * 0.5)  # 步数强行减半
                                if pos_delta == 0:
                                    pos_delta = 1 if smooth_x > 0 else -1

                            # 严格限制最大单步，防止大偏差时暴冲
                            if pos_delta > 15:  pos_delta = 15
                            if pos_delta < -15: pos_delta = -15

                            # 🎯 动态调速匹配：距离越近，限速越低
                            v_pan = int(V_MIN_PAN + 15 * abs(pos_delta))
                            v_pan = max(V_MIN_PAN, min(V_MAX_PAN, v_pan))
                            last_sent_speed_lr = set_speed_lr(v_pan, last_sent_speed_lr)
                            
                            pan_target += PAN_DIR * pos_delta
                            pan_target = max(PAN_LIMIT_MIN, min(PAN_LIMIT_MAX, pan_target))
                            move_motor(uart_lr, HORIZONTAL_ADDR, pan_target)
                    else:
                        last_smooth_x = smooth_x

                # ==========================================
                # 垂直轴 (Tilt) 控制 (高频微步)
                # ==========================================
                if time.ticks_diff(now, last_tilt_control) >= 25:
                    last_tilt_control = now
                    if abs(smooth_y) > DEADZONE_Y:
                        err_diff_y = smooth_y - last_smooth_y
                        pos_delta_y = int(0.35 * smooth_y + 0.12 * err_diff_y)
                        last_smooth_y = smooth_y

                        if abs(pos_delta_y) < 2: pos_delta_y = 0
                        if pos_delta_y != 0:
                            
                            # 垂直轴近靶心平滑衰减
                            if abs(smooth_y) < 25:
                                pos_delta_y = int(pos_delta_y * 0.5)
                                if pos_delta_y == 0:
                                    pos_delta_y = 1 if smooth_y > 0 else -1

                            if pos_delta_y > 15:  pos_delta_y = 15
                            if pos_delta_y < -15: pos_delta_y = -15

                            v_tilt = int(V_MIN_TILT + 12 * abs(pos_delta_y))
                            v_tilt = max(V_MIN_TILT, min(V_MAX_TILT, v_tilt))
                            last_sent_speed_ud = set_speed_ud(v_tilt, last_sent_speed_ud)
                            
                            tilt_target += TILT_DIR * pos_delta_y
                            tilt_target = max(TILT_LIMIT_MIN, min(TILT_LIMIT_MAX, tilt_target))
                            move_motor(uart_ud, VERTICAL_ADDR, tilt_target)
                    else:
                        last_smooth_y = smooth_y

                img.draw_cross(target_x, target_y, color=(0, 255, 0), size=15, thickness=2)
                img.draw_line(target_x, target_y, CX, CY, color=(255, 200, 0), thickness=2)
                img.draw_string_advanced(20, 40, 32, f"Tracking: {target_type}", color=(0, 255, 255))
            else:
                # 确确实实丢失了目标（连续丢帧超出 MAX_LOST_FRAMES）
                laser.value(0)
                last_target_x = None
                last_target_y = None
                smooth_x, smooth_y = 0.0, 0.0
                last_smooth_x, last_smooth_y = 0.0, 0.0
                img.draw_string_advanced(20, 40, 32, "Searching...", color=(255, 150, 150))

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
        laser.value(0) # 安全退出关闭激光
        print("正在安全卸载云台控制与激光...")