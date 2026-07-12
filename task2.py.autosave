import time, os, sys
import math
import cv_lite  # 导入cv_lite扩展模块
import ulab.numpy as np  # 导入numpy库
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA

# --------------------------- 1. 硬件与屏幕初始化 ---------------------------
lcd_width = 800
lcd_height = 480
CX = lcd_width // 2
CY = lcd_height // 2

# --------------------------- 2. 串口与引脚初始化 ---------------------------
fpioa = FPIOA()

# UART1（垂直/俯仰轴）
fpioa.set_function(3, FPIOA.UART1_TXD)
fpioa.set_function(4, FPIOA.UART1_RXD)

# UART2（水平/偏航轴）
fpioa.set_function(5, FPIOA.UART2_TXD)
fpioa.set_function(6, FPIOA.UART2_RXD)

uart_ud = UART(UART.UART1, baudrate=115200)
uart_lr = UART(UART.UART2, baudrate=115200)

VERTICAL_ADDR = 0x01
HORIZONTAL_ADDR = 0x02

# --------------------------- 3. 视觉配置参数 ---------------------------
canny_thresh1      = 50        # Canny边缘检测低阈值
canny_thresh2      = 150       # Canny边缘检测高阈值
approx_epsilon     = 0.04      # 多边形拟合精度
area_min_ratio     = 0.005     # 最小面积比例
max_angle_cos      = 0.3       # 角度余弦阈值
gaussian_blur_size = 3         # 高斯模糊核尺寸（奇数）

MIN_AREA = 100               # 最小面积阈值
MAX_AREA = 300000             # 最大面积阈值
MIN_ASPECT_RATIO = 0.3        # 最小宽高比
MAX_ASPECT_RATIO = 3.0        # 最大宽高比

BASE_RADIUS = 45              # 基础半径
POINTS_PER_CIRCLE = 50        # 圆形采样点数量
PURPLE_THRESHOLD = (20, 60, 15, 70, -70, -20)  # 紫色色块阈值

RECT_WIDTH = 250              # 固定矩形宽度
RECT_HEIGHT = 200             # 固定矩形高度

# --------------------------- 4. 电机硬限位与安全保护 ---------------------------
PAN_LIMIT_MIN = -3500         # 水平偏航左侧安全极限（负脉冲数）
PAN_LIMIT_MAX = 3500          # 水平偏航右侧安全极限（正脉冲数）

TILT_LIMIT_MIN = 0            # 垂直俯仰下侧极限
TILT_LIMIT_MAX = 1600         # 垂直俯仰上侧极限（一圈为 3200 脉冲，1600 为半圈180度）
PULSE_PER_REV = 3200          # 细分脉冲数

INIT_TILT = PULSE_PER_REV // 4 # 初始化垂直目标点（工作高度约 90 度，800 脉冲）

# --------------------------- 5. 速位结合控制算法调参 ---------------------------
DEADZONE_X = 15               # 水平追踪像素死区
DEADZONE_Y = 8                # 垂直追踪像素死区

SMOOTH_X = 0.05               # 像素坐标一阶滤波系数
SMOOTH_Y = 0.10

# 位置环 (PD 控制器) - 决定每次位置更新量 Delta P
KP_PAN   = 0.40
KD_PAN   = 0.20

KP_TILT  = 0.55
KD_TILT  = 0.18

# 速度环 (动态限速) - 在直通限速位置模式下，限制运动过程的最大速度
KV_PAN     = 1.5              # 水平速度随像素误差的增长系数
V_MIN_PAN  = 120              # 最低追踪限速
V_MAX_PAN  = 800              # 最高速度上限

KV_TILT    = 2.2
V_MIN_TILT = 120
V_MAX_TILT = 900

# 控制刷新周期
PAN_CONTROL_PERIOD = 80       # 水平控制周期 (ms)
TILT_CONTROL_PERIOD = 40      # 垂直控制周期 (ms)

# --------------------------- 6. 控制全局状态变量 ---------------------------
smooth_x = 0.0
smooth_y = 0.0

last_smooth_x = 0.0
last_smooth_y = 0.0

last_pan_control = 0
last_tilt_control = 0

pan_target = 0
tilt_target = 0

last_sent_speed_lr = 350      # 缓存最后下发给偏航轴的限速
last_sent_speed_ud = 350      # 缓存最后下发给俯仰轴的限速

# --------------------------- 7. 电机串口协议底层 ---------------------------
def send(uart, cmd):
    uart.write(bytes(cmd))
   # time.sleep_ms(5)
    #if uart.any():
        #print("RX:", uart.read().hex())

def fast_mode_init(uart, addr, speed=350, acc=5):
    cmd = [
        addr,
        0xF1,
        (speed >> 8) & 0xff,
        speed & 0xff,
        acc,
        0x01,  # 多机同步标志
        0x00,  # 是否保存
        0x6B
    ]
    send(uart, cmd)

def set_speed_lr(speed):
    global last_sent_speed_lr
    if abs(speed - last_sent_speed_lr) >= 20:
        fast_mode_init(uart_lr, HORIZONTAL_ADDR, speed)
        last_sent_speed_lr = speed

def set_speed_ud(speed):
    global last_sent_speed_ud
    if abs(speed - last_sent_speed_ud) >= 20:
        fast_mode_init(uart_ud, VERTICAL_ADDR, speed)
        last_sent_speed_ud = speed

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

# --------------------------- 8. 视觉处理辅助函数 ---------------------------
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

def detect_purple_blobs(img):
    return img.find_blobs(
        [PURPLE_THRESHOLD],
        pixels_threshold=100,
        area_threshold=100,
        merge=True
    )

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

# --------------------------- 9. 电机运动方向极性配置 ---------------------------
PAN_DIR  = -1                  # 水平偏航方向极性（若相反请改为 -1）
TILT_DIR = 1                  # 垂直俯仰方向极性（若相反请改为 -1）

# 定义全局传感器对象
sensor = None

# --------------------------- 10. 主控制逻辑 ---------------------------
def main():
    global sensor
    global smooth_x, smooth_y, last_smooth_x, last_smooth_y
    global pan_target, tilt_target
    global last_pan_control, last_tilt_control

    # 采用标准 try-except-finally 架构防止程序二次运行爆内存和崩溃重启
    try:
        # 使用无参构造，规避部分底层版本不匹配位置参数的问题
        sensor = Sensor()
        sensor.reset()
        # 显式调整输出尺寸和格式
        sensor.set_framesize(width=lcd_width, height=lcd_height)
        sensor.set_pixformat(Sensor.RGB565)

        # 初始化屏幕和多媒体层
        Display.init(Display.ST7701, width=lcd_width, height=lcd_height, to_ide=True)
        MediaManager.init()
        sensor.run()

        print("Zhang Datou Dual X42S Tracking System Start")

        # ------------------ 云台通电归零初始化 ------------------
        print("正在加载绝对位置驱动配置...")
        fast_mode_init(uart_ud, VERTICAL_ADDR, 350)
        fast_mode_init(uart_lr, HORIZONTAL_ADDR, 350)
        time.sleep(0.5)

        # 俯仰回归零点
        tilt_target = 0
        move_motor(uart_ud, VERTICAL_ADDR, tilt_target)
        time.sleep(2)

        # 俯仰抬升到初始工作角
        tilt_target = INIT_TILT
        move_motor(uart_ud, VERTICAL_ADDR, tilt_target)
        time.sleep(2)
        print("安全校准完成，视觉线程就绪...")

        clock = time.clock()
        image_shape = [sensor.height(), sensor.width()]  # [480, 800]

        while True:
            # 捕获 IDE 的停止信号，允许程序干净利落地关闭硬件资源
            os.exitpoint()
            clock.tick()
            img = sensor.snapshot()

            target_x = None
            target_y = None
            target_type = "None"

            # (1) 紫色色块检测
            purple_blobs = detect_purple_blobs(img)
            for blob in purple_blobs:
                img.draw_rectangle(blob[0:4], color=(255, 0, 255), thickness=1)
                img.draw_cross(blob.cx(), blob.cy(), color=(255, 0, 255), thickness=1)

            # (2) 使用 cv_lite 引擎进行矩形检测
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

            # (3) 目标优先级决策
            if smallest_rect and smallest_rect_corners:
                x_r, y_r, w_r, h_r = smallest_rect
                corners = smallest_rect_corners
                sorted_corners = sort_corners(corners)

                # 绘制外框与顶点
                for i in range(4):
                    x1, y1 = sorted_corners[i]
                    x2, y2 = sorted_corners[(i+1) % 4]
                    img.draw_line(x1, y1, x2, y2, color=(255, 0, 0), thickness=2)
                for p in sorted_corners:
                    img.draw_circle(p[0], p[1], 5, color=(0, 255, 0), thickness=2)

                # 投影矩阵计算
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
                        target_type = "Rectangle"

            elif purple_blobs:
                largest_purple = max(purple_blobs, key=lambda b: b.pixels())
                target_x = largest_purple.cx()
                target_y = largest_purple.cy()
                target_type = "Purple Blob"

            # (4) 闭环速位结合跟踪控制
            if target_x is not None and target_y is not None:
                raw_x = target_x - CX
                raw_y = target_y - CY

                smooth_x = smooth_x * (1 - SMOOTH_X) + raw_x * SMOOTH_X
                smooth_y = smooth_y * (1 - SMOOTH_Y) + raw_y * SMOOTH_Y

                now = time.ticks_ms()

               # ==========================================
                # 水平轴 (Yaw) 控制
                # ==========================================
                if time.ticks_diff(now, last_pan_control) >= PAN_CONTROL_PERIOD:
                    last_pan_control = now

                    if abs(smooth_x) > DEADZONE_X:
                        err_diff_x = smooth_x - last_smooth_x
                        pos_delta = int(KP_PAN * smooth_x + KD_PAN * err_diff_x)
                        last_smooth_x = smooth_x

                        # --- 【新增简单过滤】：忽略低于 6 个脉冲的微小修正，防止电机原地高频震动 ---
                        if abs(pos_delta) < 4:
                            pos_delta = 0

                        if pos_delta != 0:
                            pan_target += PAN_DIR * pos_delta
                            pan_target = max(PAN_LIMIT_MIN, min(PAN_LIMIT_MAX, pan_target))
                            move_motor(uart_lr, HORIZONTAL_ADDR, pan_target)
                    else:
                        last_smooth_x = smooth_x

                # ==========================================
                # 垂直轴 (Tilt) 控制
                # ==========================================
                if time.ticks_diff(now, last_tilt_control) >= TILT_CONTROL_PERIOD:
                    last_tilt_control = now

                    if abs(smooth_y) > DEADZONE_Y:
                        err_diff_y = smooth_y - last_smooth_y
                        pos_delta_y = int(KP_TILT * smooth_y + KD_TILT * err_diff_y)
                        last_smooth_y = smooth_y

                        v_tilt = int(V_MIN_TILT + KV_TILT * abs(smooth_y))
                        v_tilt = max(V_MIN_TILT, min(V_MAX_TILT, v_tilt))

                        set_speed_ud(v_tilt)

                        tilt_target += TILT_DIR * pos_delta_y
                        tilt_target = max(TILT_LIMIT_MIN, min(TILT_LIMIT_MAX, tilt_target))

                        move_motor(uart_ud, VERTICAL_ADDR, tilt_target)
                    else:
                        last_smooth_y = smooth_y

                img.draw_cross(target_x, target_y, color=(0, 255, 0), size=15, thickness=2)
                img.draw_line(target_x, target_y, CX, CY, color=(255, 200, 0), thickness=2)
                img.draw_string_advanced(20, 40, 32, f"Tracking: {target_type}", color=(0, 255, 255))
            else:
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

    except KeyboardInterrupt as e:
        print("用户主动退出程序: ", e)
    except BaseException as e:
        print(f"发生异常崩溃: {e}")
    finally:
        # 当点击 CanMV 的“停止”按钮或发生不可预知错误时，强制清理底层硬件，释放VB内存
        print("正在安全释放硬件多媒体及显示资源，请稍候...")
        if isinstance(sensor, Sensor):
            try:
                sensor.stop()
            except:
                pass
        try:
            Display.deinit()
        except:
            pass
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(150)
        try:
            MediaManager.deinit()
        except:
            pass
        print("清理完成，系统已恢复安全状态。")

# --------------------------- 11. 执行入口 ---------------------------
if __name__ == "__main__":
    main()
