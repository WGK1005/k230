'''
X42S 闭环步进电机简单测试
功能：
1. 读取当前角度/位置
2. 测试转动（相对/绝对位置）
'''

import time
from machine import UART, FPIOA

# ===========================
# 参数配置
# ===========================
PULSE_PER_REV = 3200   # 一圈脉冲数（根据驱动器细分修改）

# 水平电机 (UART2)
# 垂直电机 (UART1)
VERTICAL_ADDR   = 0x01
HORIZONTAL_ADDR = 0x02


# ===========================
# UART 初始化
# ===========================
fpioa = FPIOA()

# UART1（垂直）
fpioa.set_function(3, FPIOA.UART1_TXD)
fpioa.set_function(4, FPIOA.UART1_RXD)

# UART2（水平）
fpioa.set_function(5, FPIOA.UART2_TXD)
fpioa.set_function(6, FPIOA.UART2_RXD)

uart_ud = UART(UART.UART1, baudrate=115200)
uart_lr = UART(UART.UART2, baudrate=115200)


# ===========================
# 底层通信
# ===========================
def send_cmd(uart, cmd, wait_ms=50):
    """发送命令并读取返回"""
    uart.write(bytes(cmd))
    time.sleep_ms(wait_ms)
    if uart.any():
        return uart.read()
    return None


def checksum(data):
    """X42S 校验和：地址+功能码+数据 累加取低8位"""
    return sum(data) & 0xFF


# ===========================
# 回读角度/位置
# ===========================
def read_position(uart, addr):
    """
    读取电机当前位置（脉冲数，带符号32位）
    命令: addr + 0x36 + 0x6B
    返回: 有符号32位位置
    """
    cmd = [addr, 0x36, 0x6B]
    resp = send_cmd(uart, cmd, wait_ms=60)
    if resp is None or len(resp) < 7:
        print("读取失败，无返回")
        return None

    # 返回格式: addr + 0x36 + 4字节位置 + 0x6B
    val = (resp[2] << 24) | (resp[3] << 16) | (resp[4] << 8) | resp[5]
    if val & 0x80000000:
        val -= (1 << 32)
    return val


def read_position_deg(uart, addr):
    """读取位置并转换为角度"""
    pos = read_position(uart, addr)
    if pos is None:
        return None
    return pos * 360.0 / PULSE_PER_REV


# ===========================
# 绝对位置运动
# ===========================
def move_absolute(uart, addr, target):
    """绝对位置运动（脉冲）"""
    if target < 0:
        target = (1 << 32) + target
    cmd = [
        addr,
        0xFC,
        (target >> 24) & 0xFF,
        (target >> 16) & 0xFF,
        (target >> 8)  & 0xFF,
        target & 0xFF,
        0x6B
    ]
    send_cmd(uart, cmd, wait_ms=5)


def move_absolute_deg(uart, addr, deg):
    """绝对位置运动（角度）"""
    target = int(deg * PULSE_PER_REV / 360.0)
    move_absolute(uart, addr, target)


def move_relative(uart, addr, delta):
    """相对位置运动（脉冲）"""
    if delta < 0:
        delta = (1 << 32) + delta
    cmd = [
        addr,
        0xFD,
        (delta >> 24) & 0xFF,
        (delta >> 16) & 0xFF,
        (delta >> 8)  & 0xFF,
        delta & 0xFF,
        0x6B
    ]
    send_cmd(uart, cmd, wait_ms=5)


def move_relative_deg(uart, addr, deg):
    """相对位置运动（角度）"""
    delta = int(deg * PULSE_PER_REV / 360.0)
    move_relative(uart, addr, delta)


# ===========================
# 使能/失能
# ===========================
def enable_motor(uart, addr, en=True):
    """使能/失能电机"""
    cmd = [addr, 0xF3, 0xAB, 0x01 if en else 0x00, 0x00, 0x6B]
    send_cmd(uart, cmd)


# ===========================
# 测试流程
# ===========================
def test_motor(name, uart, addr):
    print("\n========== 测试 %s ==========" % name)

    # 1. 使能
    print("[1] 使能电机...")
    enable_motor(uart, addr, True)
    time.sleep_ms(200)

    # 2. 回读当前角度
    print("[2] 回读当前位置...")
    for i in range(3):
        pos = read_position(uart, addr)
        deg = read_position_deg(uart, addr)
        print("    原始位置: %s  角度: %s" % (pos, deg))
        time.sleep_ms(200)

    # 3. 测试转动 —— 正转 90°
    print("[3] 正转 90°...")
    move_relative_deg(uart, addr, 90)
    time.sleep(2)
    deg = read_position_deg(uart, addr)
    print("    转后角度: %s" % deg)

    # 4. 回零
    print("[4] 回零...")
    move_absolute_deg(uart, addr, 0)
    time.sleep(2)
    deg = read_position_deg(uart, addr)
    print("    回零角度: %s" % deg)

    # 5. 反转 90°
    print("[5] 反转 90°...")
    move_relative_deg(uart, addr, -90)
    time.sleep(2)
    deg = read_position_deg(uart, addr)
    print("    转后角度: %s" % deg)

    # 6. 回零
    print("[6] 回零...")
    move_absolute_deg(uart, addr, 0)
    time.sleep(2)
    deg = read_position_deg(uart, addr)
    print("    回零角度: %s" % deg)

    print("========== %s 测试完成 ==========\n" % name)


# ===========================
# 主程序
# ===========================
if __name__ == "__main__":
    try:
        print("X42S 电机测试开始")

        # 测试垂直电机（UART1, 地址0x01）
        test_motor("垂直电机 UART1", uart_ud, VERTICAL_ADDR)

        time.sleep(0.5)

        # 测试水平电机（UART2, 地址0x02）
        test_motor("水平电机 UART2", uart_lr, HORIZONTAL_ADDR)

        print("全部测试完成")

    except KeyboardInterrupt:
        print("用户中断")
    finally:
        print("Exit")