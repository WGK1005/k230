from machine import UART, FPIOA
import time

# ==========================
# UART1 配置（根据你的接线修改）
# TX -> 电机RX
# RX -> 电机TX
# ==========================

fpioa = FPIOA()
fpioa.set_function(3, FPIOA.UART1_TXD)
fpioa.set_function(4, FPIOA.UART1_RXD)

uart = UART(UART.UART1, baudrate=115200)

ADDR = 0x01


#==============================
# 发送命令
#==============================
def send(cmd):

    uart.write(bytes(cmd))

    print("TX:", bytes(cmd).hex(" "))

    time.sleep_ms(100)

    if uart.any():

        data = uart.read()

        print("RX:", data.hex(" "))

        return data

    return None


#==============================
# 编码器校准
#==============================
def encoder_calibrate():

    print("开始编码器校准...")

    send([
        ADDR,
        0x06,
        0x45,
        0x6B
    ])

    # 等待校准完成
    # 电机会慢慢正转一圈再反转一圈
    time.sleep(12)

    print("校准完成")


#==============================
# 相对位置运动
# angle：角度
#==============================
def move_relative(angle,
                  speed=500,
                  acc=10,
                  dir=0):

    pulse = int(angle * 3200 / 360)

    cmd = [

        ADDR,

        0xFD,

        dir,

        (speed >> 8) & 0xFF,
        speed & 0xFF,

        acc,

        (pulse >> 24) & 0xFF,
        (pulse >> 16) & 0xFF,
        (pulse >> 8) & 0xFF,
        pulse & 0xFF,

        0x00,      # 相对运动

        0x00,      # 不同步

        0x6B
    ]

    send(cmd)


#==============================
# 主程序
#==============================

encoder_calibrate()

time.sleep(2)

print("30°")
move_relative(30)
time.sleep(3)

print("60°")
move_relative(60)
time.sleep(3)

print("90°")
move_relative(90)
time.sleep(3)

print("开始每秒1.8°")

while True:

    move_relative(1.8)

    time.sleep(1)
