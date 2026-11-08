'''
K230红色物体跟踪系统 - 带PID控制和下位机通信
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
        pwm_io_h = FPIOA()
        pwm_io_h.set_function(pwm_pin_h, FPIOA.PWM2)
        self.pwm_h = PWM(2, freq=freq)

        # 垂直舵机 (上下转, 180度)
        pwm_io_v = FPIOA()
        pwm_io_v.set_function(pwm_pin_v, FPIOA.PWM3)
        self.pwm_v = PWM(3, freq=freq)

        # 舵机脉宽范围 (纳秒)
        # 水平舵机(360度): 500us-2500us (1500us为中点0度)
        self.h_min_ns = 500000    # -180度
        self.h_mid_ns = 1500000   # 中点 (0度)
        self.h_max_ns = 2500000   # +180度
        
        # 垂直舵机(180度): 1000us-2000us (1500us为中点90度)
        self.v_min_ns = 1000000   # 0度
        self.v_mid_ns = 1500000   # 中点 (90度)
        self.v_max_ns = 2000000   # 180度

        # 初始化垂直舵机到90度 (脉宽1500000ns)
        self.pwm_v.duty_ns(1500000)
        print("初始化垂直舵机到90度")
        time.sleep(3)
        #self.set_position(0, 0)
        #time.sleep(1)

    def set_position(self, h_angle, v_angle):
        """
        设置舵机位置
        h_angle: 水平角度 (-180~180, 360度舵机)
        v_angle: 垂直角度 (0~180, 180度舵机)
        """
        # 水平舵机: 将角度映射到脉宽 (-180→500us, 0→1500us, 180→2500us)
        h_ns = self.h_mid_ns + int((h_angle / 180.0) * (self.h_max_ns - self.h_mid_ns))
        
        # 垂直舵机: 将角度映射到脉宽 (0→1000us, 90→1500us, 180→2000us)
        v_ns = self.v_min_ns + int((v_angle / 180.0) * (self.v_max_ns - self.v_min_ns))

        # 限制范围
        h_ns = max(self.h_min_ns, min(self.h_max_ns, h_ns))
        v_ns = max(self.v_min_ns, min(self.v_max_ns, v_ns))

        self.pwm_h.duty_ns(h_ns)
        self.pwm_v.duty_ns(v_ns)

        self.h_angle = h_angle
        self.v_angle = v_angle

# ==================== PID控制器 ====================
class SimplePID:
    """简化的PID控制器"""
    def __init__(self, kp=0.1, ki=0.0, kd=0.05, max_out=180):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.i_sum = 0
        self.last_error = 0

    def compute(self, error):
        """计算PID输出"""
        p = self.kp * error
        self.i_sum += self.ki * error
        self.i_sum = max(-self.max_out, min(self.max_out, self.i_sum))
        d = self.kd * (error - self.last_error)
        self.last_error = error

        output = p + self.i_sum + d
        return max(-self.max_out, min(self.max_out, output))

    def reset(self):
        self.i_sum = 0
        self.last_error = 0

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
W = 480
H = 800
CX = W // 2
CY = H // 2

# 初始化舵机控制器 (PWM2: pin46, PWM3: pin47)
servo = ServoController(pwm_pin_h=46, pwm_pin_v=47)

# 初始化PID控制器
pid_h = SimplePID(kp=0.5, ki=0.05, kd=0.2, max_out=180)
pid_v = SimplePID(kp=0.5, ki=0.05, kd=0.2, max_out=180)

# 初始化下位机通信
wheel_comm = WheelMotorComm(uart_id=2, baudrate=115200)

# 摄像头初始化
sensor = Sensor(width=W, height=H)
sensor.reset()
sensor.set_hmirror(True)
sensor.set_vflip(True)
sensor.set_framesize(width=W, height=H)
sensor.set_pixformat(Sensor.RGB565)

Display.init(Display.ST7701, width=W, height=H,to_ide=True)
MediaManager.init()
sensor.run()

print("红色物体跟踪系统已启动")

# 开始识别前恢复到垂直位置
servo.set_position(0, 0)
pid_h.reset()
pid_v.reset()
time.sleep(2)

# 红色物体阈值 (L_MIN, L_MAX, A_MIN, A_MAX, B_MIN, B_MAX)
# LAB颜色空间: L(亮度), A(红-绿), B(黄-蓝)
RED_THRESHOLD = (20, 80, 30, 100, 0, 60)

# 控制参数
DEAD_ZONE_X = 30
DEAD_ZONE_Y = 30
MIN_BLOB_PIXELS = 200  # 最小色块像素数
LOST_TARGET_FRAMES = 100  # 未检测到目标的帧数阈值 (50ms/帧 × 100帧 = 5秒)

# ==================== 主控制循环 ====================
lost_target_count = 0
frame_count = 0

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

        # ========== PID控制 ==========
        # 如果在死区内，不输出控制信号
        if abs(offset_x) <= DEAD_ZONE_X:
            pan_out = 0
        else:
            pan_out = pid_h.compute(-offset_x)  # 负号因为物体右移→舵机左转

        if abs(offset_y) <= DEAD_ZONE_Y:
            tilt_out = 0
        else:
            tilt_out = pid_v.compute(offset_y)  # 物体下移→舵机上转

        # 更新舵机位置
        servo.set_position(pan_out, tilt_out)

        # 发送坐标给下位机 (用于轮子控制)
        wheel_comm.send_target_position(x, y, valid=True)
        wheel_comm.send_pan_tilt(pan_out, tilt_out)

        # 显示调试信息
        img.draw_string_advanced(10, 10, 28, "TARGET FOUND", color=(0, 255, 0))
        img.draw_string_advanced(10, 50, 24, f"X:{offset_x:+4d} Y:{offset_y:+4d}", color=(255, 255, 0))
        img.draw_string_advanced(10, 85, 24, f"Pan:{int(pan_out):+4d}deg Tilt:{int(tilt_out):+4d}deg", color=(100, 200, 255))
        img.draw_string_advanced(10, 120, 24, f"Pixels:{pixels}", color=(200, 200, 100))

    else:
        # 未检测到目标
        lost_target_count += 1

        if lost_target_count <= LOST_TARGET_FRAMES:
            # 在丢失前的短时间内保持上一个位置
            status = f"LOST {lost_target_count}/{LOST_TARGET_FRAMES}"
            img.draw_string_advanced(10, 10, 28, status, color=(255, 100, 0))
        else:
            # 目标丢失超过阈值，舵机回中
            servo.set_position(0, 0)
            pid_h.reset()
            pid_v.reset()

            img.draw_string_advanced(10, 10, 32, "NO TARGET - HOMING", color=(255, 0, 0))

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
        img.draw_string_advanced(10, 760, 20, f"Frame:{frame_count}", color=(200, 200, 200))

    # 显示到屏幕
    Display.show_image(img)

    # 控制帧率 (目标20Hz)
    time.sleep_ms(50)
