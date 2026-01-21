'''
K230 红色物体跟踪系统 - 全局配置文件
所有可调参数都在这里集中管理
'''

# ==================== 摄像头配置 ====================
CAMERA_WIDTH = 480
CAMERA_HEIGHT = 800
CAMERA_HMIRROR = True
CAMERA_VFLIP = True
CAMERA_PIXFORMAT = "RGB565"

# ==================== 显示配置 ====================
DISPLAY_MODE = "LCD"  # "LCD", "HDMI" 或 "VIRT"
DISPLAY_FPS = 20

# ==================== 舵机配置 ====================
SERVO_PWM_FREQ = 50  # Hz
SERVO_PIN_H = 46     # 水平舵机引脚 (PWM2)
SERVO_PIN_V = 47     # 垂直舵机引脚 (PWM3)

# 舵机脉宽范围 (纳秒)
SERVO_MIN_NS = 500000    # 0°
SERVO_MID_NS = 1500000   # 90° (中点)
SERVO_MAX_NS = 2500000   # 180°

# 舵机行程限制 (度)
SERVO_H_MIN = -180  # 最左
SERVO_H_MAX = 180   # 最右
SERVO_V_MIN = -180  # 最下
SERVO_V_MAX = 180   # 最上

# ==================== 颜色检测配置 ====================
# LAB颜色空间阈值
# 格式: (L_MIN, L_MAX, A_MIN, A_MAX, B_MIN, B_MAX)
# L: 亮度 (0-100)
# A: 红-绿轴 (-128-127)
# B: 黄-蓝轴 (-128-127)
RED_THRESHOLD = (20, 80, 30, 100, 0, 60)

# 最小色块像素数 (过滤噪声)
MIN_BLOB_PIXELS = 200

# ==================== PID控制器配置 ====================
# 水平方向PID参数
PID_H_KP = 0.5    # 比例增益
PID_H_KI = 0.05   # 积分增益
PID_H_KD = 0.2    # 微分增益
PID_H_MAX_OUT = 180  # 最大输出角度

# 垂直方向PID参数
PID_V_KP = 0.5
PID_V_KI = 0.05
PID_V_KD = 0.2
PID_V_MAX_OUT = 180

# ==================== 控制死区 ====================
# 在死区内不发送控制信号，减少震荡
DEAD_ZONE_X = 30  # 水平死区 (像素)
DEAD_ZONE_Y = 30  # 垂直死区 (像素)

# ==================== 目标丢失处理 ====================
LOST_TARGET_FRAMES = 15  # 未检测帧数阈值 (50ms/帧)
# 超过此帧数后，舵机回中，电机停止

# ==================== 串口配置 ====================
UART_ID = 2              # UART接口
UART_BAUDRATE = 115200   # 波特率
UART_BITS = 8
UART_PARITY = 0  # 0=None, 1=Even, 2=Odd
UART_STOP = 1

# UART2 引脚
UART_TX_PIN = 11  # 发送
UART_RX_PIN = 12  # 接收

# ==================== 调试配置 ====================
DEBUG_MODE = True
SHOW_FPS = True
SHOW_BLOB_INFO = True
SHOW_PID_VALUES = False

# ==================== 功能开关 ====================
ENABLE_SERVO_CONTROL = True
ENABLE_WHEEL_CONTROL = True
ENABLE_LCD_DISPLAY = True

# ==================== 高级参数 ====================
# 图像预处理
IMAGE_MEDIAN_FILTER = False  # 中值滤波
IMAGE_GAUSSIAN_BLUR = False  # 高斯模糊

# 多物体追踪
MULTI_OBJECT_TRACKING = False  # 只跟踪最大的色块

# PID积分限制 (防止积分饱和)
PID_I_LIMIT = 100

# 帧缓冲
FRAME_BUFFER_SIZE = 3


# ==================== 函数: 加载配置 ====================
def get_red_threshold():
    """获取红色阈值"""
    return RED_THRESHOLD

def get_servo_config():
    """获取舵机配置"""
    return {
        'freq': SERVO_PWM_FREQ,
        'pin_h': SERVO_PIN_H,
        'pin_v': SERVO_PIN_V,
        'min_ns': SERVO_MIN_NS,
        'mid_ns': SERVO_MID_NS,
        'max_ns': SERVO_MAX_NS,
    }

def get_pid_config():
    """获取PID参数"""
    return {
        'h': {'kp': PID_H_KP, 'ki': PID_H_KI, 'kd': PID_H_KD, 'max_out': PID_H_MAX_OUT},
        'v': {'kp': PID_V_KP, 'ki': PID_V_KI, 'kd': PID_V_KD, 'max_out': PID_V_MAX_OUT},
    }

def get_uart_config():
    """获取UART配置"""
    return {
        'id': UART_ID,
        'baudrate': UART_BAUDRATE,
        'bits': UART_BITS,
        'parity': UART_PARITY,
        'stop': UART_STOP,
    }


# ==================== 调整建议 ====================
"""
根据不同场景调整参数:

【快速跟踪模式】(跟踪运动目标)
- PID_H_KP = 0.8, PID_H_KD = 0.3
- DEAD_ZONE_X = 20
- MIN_BLOB_PIXELS = 100

【精确追踪模式】(跟踪静止目标)
- PID_H_KP = 0.3, PID_H_KI = 0.1
- DEAD_ZONE_X = 10
- MIN_BLOB_PIXELS = 200

【低光环境】
- RED_THRESHOLD = (10, 90, 20, 100, -10, 70)
- IMAGE_GAUSSIAN_BLUR = True

【高光环境】
- RED_THRESHOLD = (30, 70, 40, 90, 10, 50)

【大范围跟踪】
- SERVO_PIN_H/V 配置多舵机
- PID_V_MAX_OUT = 360 (允许360度旋转)
"""
