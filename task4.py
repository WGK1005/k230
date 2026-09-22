# gimbal_spin_test.py (水平电机持续旋转)
import time
from machine import UART, FPIOA

# --------------------------- 1. 串口与引脚映射 ---------------------------
fpioa = FPIOA()
fpioa.set_function(5, FPIOA.UART2_TXD)
fpioa.set_function(6, FPIOA.UART2_RXD)

uart_lr = UART(UART.UART2, baudrate=115200)
HORIZONTAL_ADDR = 0x02


# --------------------------- 2. 底层发送 ---------------------------
def send(uart, cmd):
    uart.write(bytes(cmd))


# --------------------------- 3. 速度模式：一直转 ---------------------------
def spin_forever(uart, addr, speed=100, direction=0x00, acc=4):
    """
    让电机以指定速度持续旋转（不停）。
    direction: 0x00 正转, 0x01 反转
    speed: 脉冲/秒
    """
    cmd = [
        addr,
        0xF6,                    # 速度模式
        direction,               # 方向
        (speed >> 8) & 0xff,     # 速度高字节
        speed & 0xff,            # 速度低字节
        acc,                     # 加速度档
        0x00,
        0x6B                     # 固定校验字节
    ]
    send(uart, cmd)


def stop_motor(uart, addr):
    """立即停止电机"""
    cmd = [addr, 0xFE, 0x98, 0x00, 0x6B]
    send(uart, cmd)


# --------------------------- 4. 主程序 ---------------------------
def run_spin_test():
    print("开始持续旋转（水平轴）...")
    print("按 Ctrl+C 停止")

    # 以 300 脉冲/秒 持续正转
    spin_forever(uart_lr, HORIZONTAL_ADDR, speed=300, direction=0x00, acc=4)

    try:
        # 保持运行，直到用户中断
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n用户中断，正在停止电机...")

    finally:
        stop_motor(uart_lr, HORIZONTAL_ADDR)
        print("电机已停止。")


run_spin_test()
