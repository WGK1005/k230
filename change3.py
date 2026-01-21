# ============================================================
# MicroPython 基于 cv_lite 的 RGB888 矩形检测测试代码
# 适配MG996R舵机云台（无浮点数版本）
# ============================================================

import time, os, sys, gc
from machine import Pin, PWM
from media.sensor import *     # 摄像头接口 / Camera interface
from media.display import *    # 显示接口 / Display interface
from media.media import *      # 媒体资源管理器 / Media manager
import _thread
import cv_lite                 # cv_lite扩展模块 / cv_lite extension module
import ulab.numpy as np
import math
import json

# -------------------------------
# 图像尺寸 [高, 宽] / Image size [Height, Width]
# -------------------------------
CROP_WIDTH = 400
CROP_HEIGHT = 240
image_shape = [CROP_HEIGHT, CROP_WIDTH]

# -------------------------------
# 初始化摄像头（RGB888格式） / Initialize camera (RGB888 format)
# -------------------------------
sensor = Sensor(id=2, width=1920, height=1080, fps=60)
sensor.reset()
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor_width = sensor.width(None)
sensor_height = sensor.height(None)
sensor.set_framesize(width=CROP_WIDTH, height=CROP_HEIGHT)
sensor.set_pixformat(Sensor.RGB565)

# -------------------------------
# 初始化显示器 / Initialize display
# -------------------------------
Display.init(Display.ST7701, width=800, height=480, to_ide=True)

# -------------------------------
# 初始化媒体资源管理器并启动摄像头 / Init media manager and start camera
# -------------------------------
MediaManager.init()
sensor.run()

# -------------------------------
# MG996R舵机参数配置（无浮点数版本）
# -------------------------------
class MG996R_Servo:
    """MG996R舵机控制类（整数运算）"""
    def __init__(self, pin, freq=50, min_pulse=500, max_pulse=2500, min_angle=0, max_angle=180):
        """
        初始化舵机
        :param pin: 引脚号
        :param freq: PWM频率 (Hz)
        :param min_pulse: 最小脉冲宽度 (us)
        :param max_pulse: 最大脉冲宽度 (us)
        :param min_angle: 最小角度 (度)
        :param max_angle: 最大角度 (度)
        """
        self.pwm = PWM(Pin(pin), freq=freq, duty_ns=0)
        self.freq = freq
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.current_angle = 90  # 默认居中

        # 预计算参数（避免浮点数）
        self.pulse_range = max_pulse - min_pulse
        self.angle_range = max_angle - min_angle
        self.period_us = 1000000 // freq  # 周期(us)

        self.move_to(self.current_angle)

    def pulse_to_duty(self, pulse_us):
        """将脉冲宽度转换为占空比（整数运算）"""
        duty = (pulse_us * 1023) // self.period_us
        if duty < 0:
            return 0
        if duty > 1023:
            return 1023
        return duty

    def angle_to_pulse(self, angle):
        """将角度转换为脉冲宽度（整数运算）"""
        # 限制角度范围
        if angle < self.min_angle:
            angle = self.min_angle
        if angle > self.max_angle:
            angle = self.max_angle

        # 整数运算：角度 -> 脉冲宽度
        pulse = self.min_pulse + ((angle - self.min_angle) * self.pulse_range) // self.angle_range
        return pulse

    def move_to(self, angle):
        """移动到指定角度"""
        pulse = self.angle_to_pulse(angle)
        duty = self.pulse_to_duty(pulse)
        self.pwm.duty(duty)
        self.current_angle = angle
        time.sleep_ms(20)  # 给舵机时间移动

    def move_by(self, delta_angle):
        """相对移动"""
        new_angle = self.current_angle + delta_angle
        self.move_to(new_angle)

    def disable(self):
        """关闭舵机（释放扭矩）"""
        self.pwm.duty(0)

    def enable(self):
        """启用舵机"""
        self.move_to(self.current_angle)

# 初始化舵机云台（使用GPIO46和47）
# X轴舵机（水平旋转）接GPIO46
servo_x = MG996R_Servo(pin=46, min_angle=0, max_angle=180)
# Y轴舵机（垂直旋转）接GPIO47
servo_y = MG996R_Servo(pin=47, min_angle=0, max_angle=180)

# 设置初始位置（居中）
servo_x.move_to(90)
servo_y.move_to(90)

# -------------------------------
# 矩形检测可调参数 / Adjustable rectangle detection parameters
# -------------------------------
canny_thresh1 = 50
canny_thresh2 = 150
approx_epsilon = 0.05
area_min_ratio = 0.0005
max_angle_cos = 0.6
gaussian_blur_size = 3

RECT_WIDTH = 210
RECT_HEIGHT = 95
BASE_RADIUS = 45
POINTS_PER_CIRCLE = 24

# -------------------------------
# PID控制器（整数版本，适配舵机角度控制）
# -------------------------------
class IntegerPID:
    """整数PID控制器"""
    def __init__(self, Kp=100, Ki=0, Kd=100, setpoint=0, dead_zone=2, output_min=-30, output_max=30):
        self.Kp = Kp  # 比例系数（放大100倍）
        self.Ki = Ki  # 积分系数
        self.Kd = Kd  # 微分系数（放大100倍）
        self.setpoint = setpoint
        self.dead_zone = dead_zone
        self.output_min = output_min
        self.output_max = output_max

        # 内部状态
        self.last_error = 0
        self.integral = 0
        self.last_output = 0

    def compute(self, current_value):
        """计算PID输出（返回整数）"""
        error = self.setpoint - current_value

        # 死区处理
        if abs(error) < self.dead_zone:
            return 0

        # 比例项
        P = (self.Kp * error) // 100

        # 积分项（整数运算）
        self.integral += error
        I = (self.Ki * self.integral) // 100

        # 微分项
        D = (self.Kd * (error - self.last_error)) // 100

        # 更新状态
        self.last_error = error

        # 计算输出
        output = P + I + D

        # 输出限幅
        if output < self.output_min:
            output = self.output_min
        if output > self.output_max:
            output = self.output_max

        return output

    def set_limits(self, min_val, max_val):
        """设置输出限制"""
        self.output_min = min_val
        self.output_max = max_val

    def set_setpoint(self, setpoint):
        """设置目标值"""
        self.setpoint = setpoint
        self.integral = 0  # 重置积分
        self.last_error = 0

# 创建整数PID控制器
# X轴PID：图像坐标200对应舵机角度90
pidx = IntegerPID(Kp=50, Ki=0, Kd=20, setpoint=200, dead_zone=3, output_min=-20, output_max=20)
# Y轴PID：图像坐标120对应舵机角度90
pidy = IntegerPID(Kp=50, Ki=0, Kd=20, setpoint=120, dead_zone=3, output_min=-20, output_max=20)

# -------------------------------
# 激光控制引脚
# -------------------------------
from machine import FPIOA
fpioa = FPIOA()
# 假设激光控制使用GPIO32（如果冲突请修改）
fpioa.set_function(32, FPIOA.GPIO32)
LASER_BP = Pin(32, Pin.OUT, pull=Pin.PULL_DOWN, drive=15)

def btn_BP_callback(component, event):
    LASER_BP.value(LASER_BP.value() ^ 1)

# -------------------------------
# UI界面
# -------------------------------
from ui_core import TouchUI

back = image.Image(800, 480, image.RGB888)
back.clear()
ui = TouchUI(800, 480)

# -------------------------------
# 全局变量
# -------------------------------
PID_Flag = False
Loop_Flag = False
EN_Flag = True
turn_R_scan = False
turn_L_scan = False
Find_Flag = False
loop_dis = [1,1,0,1]
loop_index = 0
BlueLaserFlag = True
DisplayFlag = True
right_top = None

# 加载保存的坐标
def load_coordinates():
    """从JSON文件加载激光坐标（返回整数）"""
    try:
        with open("/sdcard/calibration_coordinates.json", "r") as f:
            data = json.load(f)
            return (int(data["laser"]["x"]), int(data["laser"]["y"]))
    except:
        return (200, 120)  # 默认坐标

def save_coordinates(laser_coord):
    """保存激光坐标到JSON文件"""
    coordinates = {
        "laser": {"x": int(laser_coord[0]), "y": int(laser_coord[1])}
    }
    try:
        with open("/sdcard/calibration_coordinates.json", "w") as f:
            json.dump(coordinates, f)
        return True
    except:
        return False

# 初始化坐标
cross = load_coordinates()

# -------------------------------
# 蓝色激光点检测函数
# -------------------------------
def laser_detection(img):
    """检测蓝色激光点（返回整数坐标）"""
    blue_blobs = img.find_blobs([(0, 100, -128, 33, -128, -40)], pixels_threshold=10, area_threshold=5, merge=True)

    if blue_blobs:
        sumX = 0
        sumY = 0
        for blob in blue_blobs:
            sumX += blob.cx()
            sumY += blob.cy()
        num = len(blue_blobs)
        if num:
            return (int(sumX//num), int(sumY//num))
    return None

# -------------------------------
# 矩形检测辅助函数（整数版本）
# -------------------------------
def calculate_distance(p1, p2):
    """计算两点距离（整数）"""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return int(math.sqrt(dx*dx + dy*dy))

def calculate_center(points):
    """计算点集中心（返回整数）"""
    if not points:
        return (0, 0)
    sum_x = 0
    sum_y = 0
    for p in points:
        sum_x += p[0]
        sum_y += p[1]
    return (int(sum_x // len(points)), int(sum_y // len(points)))

def sort_corners(corners):
    """将矩形角点按左上、右上、右下、左下顺序排序"""
    center = calculate_center(corners)

    # 计算每个点到中心的角度并排序（整数运算）
    angles = []
    for p in corners:
        dx = p[0] - center[0]
        dy = p[1] - center[1]
        # 使用atan2的近似计算（避免浮点）
        angle = int(math.atan2(dy, dx) * 1000)
        angles.append((angle, p))

    angles.sort(key=lambda x: x[0])
    sorted_corners = [p for _, p in angles]

    # 找到左上角点
    if len(sorted_corners) == 4:
        left_top = min(sorted_corners, key=lambda p: p[0] + p[1])
        index = sorted_corners.index(left_top)
        sorted_corners = sorted_corners[index:] + sorted_corners[:index]

    return sorted_corners

def is_valid_rect(corners):
    """验证矩形有效性（整数运算）"""
    if len(corners) != 4:
        return False

    # 计算边长
    edges = []
    for i in range(4):
        dx = corners[(i+1)%4][0] - corners[i][0]
        dy = corners[(i+1)%4][1] - corners[i][1]
        length = dx*dx + dy*dy  # 平方距离
        edges.append(length)

    # 检查对边是否相近（使用平方值比较）
    ratio1 = (edges[0] * 100) // max(edges[2], 1)
    ratio2 = (edges[1] * 100) // max(edges[3], 1)

    # 对边长度比应在0.7-1.3之间（平方值对应0.49-1.69）
    if ratio1 < 49 or ratio1 > 169 or ratio2 < 49 or ratio2 > 169:
        return False

    # 计算面积（鞋带公式）
    area = 0
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[(i+1) % 4]
        area += (x1 * y2 - x2 * y1)
    area = abs(area) // 2

    # 面积检查
    MIN_AREA = 1000
    MAX_AREA = 100000
    if area < MIN_AREA or area > MAX_AREA:
        return False

    # 宽高比检查
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    width = max(xs) - min(xs)
    height = max(ys) - min(ys)

    MIN_ASPECT_RATIO = 30  # 0.3 * 100
    MAX_ASPECT_RATIO = 300  # 3.0 * 100
    aspect_ratio = (width * 100) // max(height, 1)

    return MIN_ASPECT_RATIO <= aspect_ratio <= MAX_ASPECT_RATIO

# -------------------------------
# UI回调函数
# -------------------------------
def btn_PID_callback(component, event):
    global PID_Flag
    PID_Flag = not PID_Flag
    text.set_text("PID: " + str(PID_Flag))

def btn_CHECK_callback(component, event):
    global BlueLaserFlag
    BlueLaserFlag = not BlueLaserFlag
    text.set_text("CHECK: " + str(BlueLaserFlag))

def set_z_callback(component, event):
    servo_x.move_to(90)
    servo_y.move_to(90)
    text.set_text("已设置居中")

def back_z_callback(component, event):
    servo_x.move_to(90)
    servo_y.move_to(90)
    PID_Flag = True
    text.set_text("已回原点")

def en_motor_callback(component, event):
    global EN_Flag
    EN_Flag = not EN_Flag
    if EN_Flag:
        servo_x.enable()
        servo_y.enable()
        text.set_text("舵机使能: 已开启")
    else:
        servo_x.disable()
        servo_y.disable()
        text.set_text("舵机使能: 已关闭")

def btn_R_callback(component, event):
    global PID_Flag, turn_R_scan, Find_Flag
    servo_x.move_by(30)  # 向右转30度
    PID_Flag = False
    Find_Flag = False
    turn_R_scan = True
    text.set_text("已开启右扫描")

def btn_L_callback(component, event):
    global PID_Flag, turn_L_scan, Find_Flag
    servo_x.move_by(-30)  # 向左转30度
    PID_Flag = False
    Find_Flag = False
    turn_L_scan = True
    text.set_text("已开启左扫描")

def clear_btn_callback(component, event):
    global turn_R_scan, turn_L_scan
    turn_R_scan = False
    turn_L_scan = False
    text.set_text("已清除扫描")

def loop_line_mode_callback(component, event):
    global PID_Flag, Loop_Flag
    Loop_Flag = not Loop_Flag
    if Loop_Flag:
        text.set_text("巡线模式开启")
        PID_Flag = True
        LASER_BP.value(1)
    else:
        text.set_text("巡线模式关闭")
        PID_Flag = False
        LASER_BP.value(0)

# 创建UI按钮
btn_PID = ui.add_button(0, 420, 100, 50, "PID", btn_PID_callback)
btn_CHECK = ui.add_button(150, 420, 100, 50, "校准", btn_CHECK_callback)
btn_laserBP = ui.add_button(300, 420, 100, 50, "蓝紫", btn_BP_callback)
set_z = ui.add_button(450, 420, 100, 50, "设置Z", set_z_callback)
back_z = ui.add_button(600, 420, 100, 50, "回原点", back_z_callback)
btn_en_motor = ui.add_button(0, 320, 100, 50, "舵机使能", en_motor_callback)
btn_R = ui.add_button(420, 50, 100, 50, "找点R", btn_R_callback)
btn_L = ui.add_button(570, 50, 100, 50, "找点L", btn_L_callback)
clear_btn = ui.add_button(420, 150, 100, 50, "清除", clear_btn_callback)
loop_line_mode = ui.add_button(570, 150, 100, 50, "巡线模式", loop_line_mode_callback)
text = ui.add_static_text(150, 320, 20, "待指示", (255,255,255))

# -------------------------------
# 主循环
# -------------------------------
clock = time.clock()

while True:
    try:
        clock.tick()

        # 1. 读取摄像头图像
        img = sensor.snapshot()

        # 2. 蓝色激光校准
        if not BlueLaserFlag:
            blueLaser = laser_detection(img)
            if blueLaser is not None:
                cross = blueLaser
                BlueLaserFlag = True
                save_coordinates(blueLaser)
                print(f"校准完成: {blueLaser}")

        # 3. 转换为灰度图并进行矩形检测
        gray_img = img.to_grayscale()
        img_np = gray_img.to_numpy_ref()

        rects = cv_lite.grayscale_find_rectangles_with_corners(
            image_shape, img_np,
            canny_thresh1, canny_thresh2,
            approx_epsilon,
            area_min_ratio,
            max_angle_cos,
            gaussian_blur_size
        )

        # 4. 查找最小矩形
        min_area = 0x7FFFFFFF  # 最大整数
        smallest_rect = None
        smallest_rect_corners = None

        for rect in rects:
            x, y, w, h = int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3])
            corners = [
                (int(rect[4]), int(rect[5])),
                (int(rect[6]), int(rect[7])),
                (int(rect[8]), int(rect[9])),
                (int(rect[10]), int(rect[11]))
            ]

            if is_valid_rect(corners):
                area = w * h
                if area < min_area:
                    min_area = area
                    smallest_rect = (x, y, w, h)
                    smallest_rect_corners = corners

        # 5. 处理检测到的矩形
        if smallest_rect and smallest_rect_corners:
            x, y, w, h = smallest_rect
            corners = smallest_rect_corners
            sorted_corners = sort_corners(corners)
            right_top = sorted_corners[1]

            # 绘制矩形
            for i in range(4):
                x1, y1 = sorted_corners[i]
                x2, y2 = sorted_corners[(i+1) % 4]
                img.draw_line(x1, y1, x2, y2, color=(255, 0, 0), thickness=2)

            for p in sorted_corners:
                img.draw_circle(p[0], p[1], 5, color=(0, 255, 0), thickness=2)

            # 计算矩形中心
            rect_center = calculate_center(sorted_corners)
            img.draw_circle(rect_center[0], rect_center[1], 4, color=(0, 255, 255), thickness=2)

            # 简化：直接使用矩形中心作为跟踪点
            pidx.set_setpoint(rect_center[0])
            pidy.set_setpoint(rect_center[1])
            Find_Flag = True

        # 6. PID控制
        if cross is not None and PID_Flag:
            # 计算PID输出（整数）
            x_out = pidx.compute(cross[0])
            y_out = pidy.compute(cross[1])

            # 控制舵机
            if x_out != 0:
                new_x_angle = servo_x.current_angle + x_out
                if 0 <= new_x_angle <= 180:
                    servo_x.move_to(new_x_angle)

            if y_out != 0:
                new_y_angle = servo_y.current_angle + y_out
                if 0 <= new_y_angle <= 180:
                    servo_y.move_to(new_y_angle)

            # 绘制激光点
            img.draw_cross(cross[0], cross[1], color=(255, 0, 0), size=3, thickness=1)

        # 7. 显示图像
        if DisplayFlag:
            Display.show_image(img, layer=Display.LAYER_OSD1)
            ui.update(back)
            Display.show_image(back, layer=Display.LAYER_OSD0)

        # 8. 清理资源
        del img_np, img, gray_img
        gc.collect()

        # 打印FPS（可选）
        # print(f"FPS: {int(clock.fps())}")

    except KeyboardInterrupt:
        break
    except Exception as e:
        print(f"Error: {e}")

# -------------------------------
# 清理退出
# -------------------------------
servo_x.disable()
servo_y.disable()
sensor.stop()
Display.deinit()
os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
time.sleep_ms(100)
MediaManager.deinit()
