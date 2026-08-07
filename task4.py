# gimbal_angle_test.py (极简角度控制测试)
import time
from machine import UART, FPIOA

# --------------------------- 1. 串口与引脚映射 ---------------------------
fpioa = FPIOA()
# UART2（水平/偏航轴 Yaw）
fpioa.set_function(5, FPIOA.UART2_TXD)
fpioa.set_function(6, FPIOA.UART2_RXD)

uart_lr = UART(UART.UART2, baudrate=115200)
HORIZONTAL_ADDR = 0x01


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

# --------------------------- 3. 角度控制主程序 ---------------------------
# 3200 细分下，1度对应的脉冲数约为 8.8889
PULSE_PER_REV = 3200 

def angle_to_pulse(angle):
    """将物理角度转换为电机脉冲数"""
    return int((angle / 360.0) * PULSE_PER_REV)

def run_angle_test():
    print("正在初始化水平轴速度...")
    # 设置一个中等偏慢、运行起来十分优雅丝滑的速度（300脉冲/秒）
    fast_mode_init(uart_lr, HORIZONTAL_ADDR, speed=300, acc=4)
    time.sleep_ms(200)

    # 1. 归零校准
    print("确保电机处于 0 度起始点...")
    move_motor(uart_lr, HORIZONTAL_ADDR, 0)
    time.sleep(2.0)

    # 2. 依次顺时针旋转到指定角度（注意：在你的云台中，顺逆时针由正负号决定。如果反了，请给角度加负号）
    test_angles = [30, 60, 90, 120]

    try:
        for angle in test_angles:
            # 计算对应的绝对脉冲数
            target_pulse = angle_to_pulse(angle)
            print(f"-> 正在旋转至顺时针 {angle}° (对应绝对脉冲位置: {target_pulse})...")
            
            # 发送位置控制指令
            move_motor(uart_lr, HORIZONTAL_ADDR, target_pulse)
            
            # 停顿 2 秒以便观察和测量角度是否精准
            time.sleep(2.0)

        # 3. 自动回零
        print("-> 测试完毕，正在平滑返回 0 度起点...")
        move_motor(uart_lr, HORIZONTAL_ADDR, 0)
        time.sleep(3.0)
        print("测试完成！")

    except KeyboardInterrupt:
        print("\n测试被用户中断...")
    finally:
        # 安全退出
        move_motor(uart_lr, HORIZONTAL_ADDR, 0)

# 执行测试
run_angle_test()