# gimbal_angle_test.py (双电机角度控制测试)
import time
from machine import UART, FPIOA

# --------------------------- 1. 串口与引脚映射 ---------------------------
fpioa = FPIOA()

# UART1（垂直/俯仰轴 Pitch）—— 引脚 3, 4
fpioa.set_function(3, FPIOA.UART1_TXD)
fpioa.set_function(4, FPIOA.UART1_RXD)

# UART2（水平/偏航轴 Yaw）—— 引脚 5, 6
fpioa.set_function(5, FPIOA.UART2_TXD)
fpioa.set_function(6, FPIOA.UART2_RXD)

uart_pitch = UART(UART.UART1, baudrate=115200)   # 垂直电机
uart_yaw   = UART(UART.UART2, baudrate=115200)   # 水平电机

PITCH_ADDR = 0x01   # 垂直电机地址
YAW_ADDR   = 0x02   # 水平电机地址


# --------------------------- 2. 电机底层驱动函数 ---------------------------
def send(uart, cmd):
    uart.write(bytes(cmd))

def fast_mode_init(uart, addr, speed=350, acc=5):
    """设置电机的运行速度与加速度"""
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

def move_motor(uart, addr, target):
    """驱动电机到绝对位置（脉冲数）"""
    if target < 0:
        target = (1 << 32) + target  # 负数转为32位无符号整数
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


# --------------------------- 3. 角度控制辅助函数 ---------------------------
PULSE_PER_REV = 3200  # 3200 细分下，一圈对应 3200 脉冲

def angle_to_pulse(angle):
    """将物理角度转换为电机脉冲数"""
    return int((angle / 360.0) * PULSE_PER_REV)


# --------------------------- 4. 双电机控制主程序 ---------------------------
def init_motors():
    """初始化两个电机的速度与加速度"""
    print("正在初始化电机速度...")
    fast_mode_init(uart_yaw,   YAW_ADDR,   speed=300, acc=4)
    fast_mode_init(uart_pitch, PITCH_ADDR, speed=300, acc=4)
    time.sleep_ms(200)


def home_all():
    """两个电机同时归零"""
    print("确保电机处于 0 度起始点...")
    move_motor(uart_yaw,   YAW_ADDR,   0)
    move_motor(uart_pitch, PITCH_ADDR, 0)
    time.sleep(2.0)


def move_both(yaw_angle, pitch_angle, wait=2.0):
    """同时驱动水平和垂直电机到指定角度"""
    yaw_pulse   = angle_to_pulse(yaw_angle)
    pitch_pulse = angle_to_pulse(pitch_angle)

    print(f"-> 水平 {yaw_angle}°(脉冲 {yaw_pulse}) | 垂直 {pitch_angle}°(脉冲 {pitch_pulse})")

    move_motor(uart_yaw,   YAW_ADDR,   yaw_pulse)
    move_motor(uart_pitch, PITCH_ADDR, pitch_pulse)

    time.sleep(wait)


def run_angle_test():
    init_motors()
    home_all()

    # 依次测试的角度组合：(水平角度, 垂直角度)
    # 若方向反了，给对应角度加负号即可
    test_points = [
        (30,  15),
        (540,  30),
        (0,  45),


    ]

    try:
        for yaw_angle, pitch_angle in test_points:
            move_both(yaw_angle, pitch_angle, wait=2.0)

        # 自动回零
        print("-> 测试完毕，正在平滑返回 0 度起点...")
        move_both(0, 0, wait=3.0)
        print("测试完成！")

    except KeyboardInterrupt:
        print("\n测试被用户中断...")
    finally:
        # 安全退出：两个轴都回到零点
        move_motor(uart_yaw,   YAW_ADDR,   0)
        move_motor(uart_pitch, PITCH_ADDR, 0)


# 执行测试
run_angle_test()
