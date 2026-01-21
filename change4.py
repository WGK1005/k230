'''
K230红色物体跟踪系统 - 简化版 (无PID控制)
功能: 识别红色物体 → 控制云台舵机 → 发送坐标给下位机
'''

import time
import sys
from media.sensor import *
from media.display import *
from media.media import *
from machine import PWM, FPIOA, UART

# ==================== 舵机配置 ====================
class ServoController:
    """舵机控制器 (水平360度/垂直180度)"""
    def __init__(self, pwm_pin_h, pwm_pin_v, freq=50):
        """
        初始化舵机控制
        pwm_pin_h: 水平舵机PWM引脚 (360度)
        pwm_pin_v: 垂直舵机PWM引脚 (180度)
        """
        # 水平舵机 (左右转, 360度)
        # 360度舵机: 2.5%-占空比 = -90度, 7.5% = 0度, 12.5% = 90度
        pwm_io_h = FPIOA()
        pwm_io_h.set_function(pwm_pin_h, FPIOA.PWM2)
        self.pwm_h = PWM(2, freq, 50, enable=True)
        self.h_min_duty = 2.5     # -90度
        self.h_mid_duty = 7.5     # 0度
        self.h_max_duty = 12.5    # 90度

        # 垂直舵机 (上下转, 180度)
        # 180度舵机: 2.5% = 0度, 7.5% = 90度, 12.5% = 180度
        pwm_io_v = FPIOA()
        pwm_io_v.set_function(pwm_pin_v, FPIOA.PWM3)
        self.pwm_v = PWM(3, freq, 50, enable=True)
        self.v_min_duty = 2.5     # 0度
        self.v_mid_duty = 7.5     # 90度
        self.v_max_duty = 12.5    # 180度

        # 初始化垂直舵机到90度 (占空比7.5%)
        self.pwm_v.duty(self.v_mid_duty)
        print("初始化垂直舵机到90度")
        time.sleep(3)

        self.h_angle = 0
        self.v_angle = 90

    def set_position(self, h_angle, v_angle):
        """
        设置舵机位置
        h_angle: 水平角度 (-90~90, 360度舵机)
        v_angle: 垂直角度 (0~180, 180度舵机)
        """
        # 限制角度范围
        h_angle = max(-90, min(90, h_angle))
        v_angle = max(0, min(180, v_angle))

        # 水平舵机: 将角度映射到占空比 (-90→2.5%, 0→7.5%, 90→12.5%)
        h_duty = self.h_mid_duty + (h_angle / 90.0) * (self.h_max_duty - self.h_mid_duty)
        
        # 垂直舵机: 将角度映射到占空比 (0→2.5%, 90→7.5%, 180→12.5%)
        v_duty = self.v_min_duty + (v_angle / 180.0) * (self.v_max_duty - self.v_min_duty)

        # 设置PWM占空比
        self.pwm_h.duty(h_duty)
        self.pwm_v.duty(v_duty)

        self.h_angle = h_angle
        self.v_angle = v_angle

# ==================== 下位机通信 ====================
class WheelMotorComm:
    """与下位机通信 - 发送目标物体坐标"""
    def __init__(self, uart_id=2, baudrate=115200):
        """初始化UART通信"""
        try:
            self.uart = UART(uart_id, baudrate=baudrate, bits=8, parity=0, stop=1)
            self.uart_enabled = True
            print(f"下位机通信已连接 (UART{uart_id}, {baudrate}bps)")
        except:
            self.uart_enabled = False
            print("警告: 下位机通信初始化失败")

    def send_target_position(self, x, y, valid=True):
        """
        发送目标物体坐标给下位机
        格式: $TARGET,x,y,valid\n
        """
        if not self.uart_enabled:
            return

        try:
            msg = f"$TARGET,{x},{y},{1 if valid else 0}\n"
            self.uart.write(msg.encode())
        except:
            pass

    def send_pan_tilt(self, pan, tilt):
        """发送云台角度"""
        if not self.uart_enabled:
            return

        try:
            msg = f"$SERVO,{int(pan)},{int(tilt)}\n"
            self.uart.write(msg.encode())
        except:
            pass

# ==================== 主程序配置 ====================
# 屏幕尺寸
W = 640
H = 480
CX = W // 2
CY = H // 2

# 初始化舵机控制器 (PWM2: pin46, PWM3: pin47)
servo = ServoController(pwm_pin_h=46, pwm_pin_v=47)

# 初始化下位机通信
wheel_comm = WheelMotorComm(uart_id=2, baudrate=115200)

# 摄像头初始化
sensor = Sensor(width=W, height=H)
sensor.reset()
sensor.set_hmirror(True)
sensor.set_vflip(True)
sensor.set_framesize(width=W, height=H)
sensor.set_pixformat(Sensor.RGB565)

# 使用虚拟显示 (IDE显示)
Display.init(Display.VIRT, width=W, height=H, fps=100)
MediaManager.init()
sensor.run()

print("红色物体跟踪系统已启动")

# 开始识别前恢复到中心位置
servo.set_position(0, 90)
time.sleep(2)

# 红色物体阈值 (L_MIN, L_MAX, A_MIN, A_MAX, B_MIN, B_MAX)
# LAB颜色空间: L(亮度), A(红-绿), B(黄-蓝)
RED_THRESHOLD = (0, 42, 17, 94, -6, 50)

# 控制参数
DEAD_ZONE_X = 30
DEAD_ZONE_Y = 30
MIN_BLOB_PIXELS = 200  # 最小色块像素数
LOST_TARGET_FRAMES = 100  # 未检测到目标的帧数阈值
MAX_PAN_ANGLE = 90  # 最大水平转角
MAX_TILT_ANGLE = 90  # 最大垂直转角

# ==================== 主控制循环 ====================
lost_target_count = 0
frame_count = 0
current_h_angle = 0
current_v_angle = 90

while True:
    frame_count += 1
    img = sensor.snapshot()

    # 红色物体检测
    blobs = img.find_blobs([RED_THRESHOLD], pixels_threshold=MIN_BLOB_PIXELS)

    if blobs:
        # 找最大的红色色块
        blob = max(blobs, key=lambda b: b.pixels())
        x = blob.cx()
        y = blob.cy()
        pixels = blob.pixels()

        # 重置丢失计数
        lost_target_count = 0
        target_found = True

        # 绘制检测结果
        img.draw_rectangle(blob.rect(), color=(255, 0, 0), thickness=2)
        img.draw_cross(x, y, color=(255, 0, 0), size=10, thickness=1)

        # 计算偏移量 (相对于屏幕中心)
        offset_x = x - CX  # 正值=物体右边，负值=物体左边
        offset_y = y - CY  # 正值=物体下边，负值=物体上边

        # ========== 简单比例控制 ==========
        # 根据偏移计算目标角度
        if abs(offset_x) > DEAD_ZONE_X:
            # 将像素偏移量映射到角度 (屏幕宽度640像素对应180度)
            h_angle_delta = (offset_x / CX) * 45  # ±45度范围
            current_h_angle = max(-90, min(90, h_angle_delta))

        if abs(offset_y) > DEAD_ZONE_Y:
            # 将像素偏移量映射到角度 (屏幕高度480像素对应180度)
            v_angle_delta = (offset_y / CY) * 45  # ±45度范围
            current_v_angle = max(45, min(135, 90 + v_angle_delta))

        # 更新舵机位置
        servo.set_position(current_h_angle, current_v_angle)

        # 发送坐标给下位机
        wheel_comm.send_target_position(x, y, valid=True)
        wheel_comm.send_pan_tilt(current_h_angle, current_v_angle)

        # 显示调试信息
        img.draw_string_advanced(0, 0, 28, "TARGET FOUND", color=(0, 255, 0))
        img.draw_string_advanced(0, 35, 24, f"X:{offset_x:+4d} Y:{offset_y:+4d}", color=(255, 255, 0))
        img.draw_string_advanced(0, 60, 24, f"Pan:{int(current_h_angle):+4d}deg Tilt:{int(current_v_angle):+4d}deg", color=(100, 200, 255))
        img.draw_string_advanced(0, 85, 24, f"Pixels:{pixels}", color=(200, 200, 100))

    else:
        # 未检测到目标
        lost_target_count += 1

        if lost_target_count <= LOST_TARGET_FRAMES:
            # 在丢失前的短时间内保持上一个位置
            status = f"LOST {lost_target_count}/{LOST_TARGET_FRAMES}"
            img.draw_string_advanced(0, 0, 28, status, color=(255, 100, 0))
        else:
            # 目标丢失超过阈值，舵机回中
            current_h_angle = 0
            current_v_angle = 90
            servo.set_position(current_h_angle, current_v_angle)
            img.draw_string_advanced(0, 0, 32, "NO TARGET - HOMING", color=(255, 0, 0))

        # 通知下位机目标丢失
        wheel_comm.send_target_position(0, 0, valid=False)

    # ========== 绘制参考线 ==========
    # 屏幕中心十字
    img.draw_cross(CX, CY, color=(0, 255, 0), size=15, thickness=1)

    # 死区矩形
    img.draw_rectangle(CX - DEAD_ZONE_X, CY - DEAD_ZONE_Y,
                      DEAD_ZONE_X * 2, DEAD_ZONE_Y * 2,
                      color=(0, 100, 0), thickness=1)

    # 显示帧率信息
    if frame_count % 30 == 0:
        img.draw_string_advanced(0, H-25, 20, f"Frame:{frame_count}", color=(200, 200, 200))

    # 显示到屏幕
    Display.show_image(img)
