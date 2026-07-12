'''
K230 双X42S闭环步进电机红色追踪（绝对位置）
第一次完成校准后，以后运行使用本程序即可
'''
import time
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART,FPIOA

# ===========================
# 图像参数
# ===========================
W=800
H=480
CX=W//2
CY=H//2

RED_TH=(20,80,30,100,0,60)

# ===========================
# 控制参数
# ===========================
DEADZONE_X=20
DEADZONE_Y=10

SMOOTH_X=0.12
SMOOTH_Y=0.30

PAN_CONTROL_PERIOD=100
TILT_CONTROL_PERIOD=50

DIST_REF=5000

PAN_LIMIT=3500
TILT_LIMIT=8000
# 一圈脉冲数（根据你的驱动器细分修改）
PULSE_PER_REV = 3200

# 90°
INIT_TILT = PULSE_PER_REV // 4      # 800

# 垂直运动范围（180°）
TILT_MIN = 0
TILT_MAX = PULSE_PER_REV // 2       # 1600
# ===========================
# 全局变量
# ===========================
smooth_x=0.0
smooth_y=0.0
pixel_avg=DIST_REF

last_pan_control=0
last_tilt_control=0

pan_target=0
tilt_target=0

last_speed_lr=500
last_speed_ud=500

last_pan_error = 0
last_tilt_error = 0

pan_speed = 0
tilt_speed = 0
# ===========================
# UART
# ===========================
fpioa=FPIOA()

# UART1（垂直）
fpioa.set_function(3,FPIOA.UART1_TXD)
fpioa.set_function(4,FPIOA.UART1_RXD)

# UART2（水平）
fpioa.set_function(5,FPIOA.UART2_TXD)
fpioa.set_function(6,FPIOA.UART2_RXD)

uart_ud=UART(UART.UART1,baudrate=115200)
uart_lr=UART(UART.UART2,baudrate=115200)

VERTICAL_ADDR=0x01
HORIZONTAL_ADDR=0x02

# ===========================
# 串口发送
# ===========================
def send(uart,cmd):
    uart.write(bytes(cmd))
    time.sleep_ms(5)
    if uart.any():
        print("RX:",uart.read().hex())

# ===========================
# 进入绝对位置模式
# ===========================
def fast_mode_init(uart,addr,speed=500,acc=20):

    cmd=[
        addr,
        0xF1,
        (speed>>8)&0xff,
        speed&0xff,
        acc,
        0x01,
        0x00,
        0x6B
    ]

    send(uart,cmd)

# ===========================
# 水平速度
# ===========================
def set_speed_lr(speed):

    global last_speed_lr

    if speed!=last_speed_lr:

        fast_mode_init(
            uart_lr,
            HORIZONTAL_ADDR,
            speed
        )

        last_speed_lr=speed

# ===========================
# 垂直速度
# ===========================
def set_speed_ud(speed):

    global last_speed_ud

    if speed!=last_speed_ud:

        fast_mode_init(
            uart_ud,
            VERTICAL_ADDR,
            speed
        )

        last_speed_ud=speed

# ===========================
# 绝对位置控制
# ===========================
def move_motor(uart,addr,target):

    if target<0:
        target=(1<<32)+target

    cmd=[
        addr,
        0xFC,
        (target>>24)&0xff,
        (target>>16)&0xff,
        (target>>8)&0xff,
        target&0xff,
        0x6B
    ]

    send(uart,cmd)

# ===========================
# 摄像头初始化
# ===========================
def init_hw():

    sensor=Sensor(width=W,height=H)

    sensor.reset()

    sensor.set_framesize(
        width=W,
        height=H
    )

    sensor.set_pixformat(
        Sensor.RGB565
    )

    Display.init(
        Display.ST7701,
        width=W,
        height=H,
        to_ide=True
    )

    MediaManager.init()

    sensor.run()

    time.sleep(0.5)

    return sensor
# ============================================
# 主程序
# ============================================

def main():

    global smooth_x
    global smooth_y
    global pixel_avg
    global pan_target
    global tilt_target
    global last_pan_control
    global last_tilt_control

    print("Dual X42S Tracking Start")

    cam = init_hw()
    print("进入绝对位置模式...")

    fast_mode_init(uart_ud,VERTICAL_ADDR,300)
    fast_mode_init(uart_lr,HORIZONTAL_ADDR,300)

    time.sleep(0.5)
    tilt_target = 0
    move_motor(
    uart_ud,
    VERTICAL_ADDR,
    tilt_target
)
    # 等待运动完成
    time.sleep(2)
    tilt_target = INIT_TILT
    move_motor(
    uart_ud,
    VERTICAL_ADDR,
    tilt_target
)
    # 等待运动完成
    time.sleep(2)

    print("Tracking...")

    while True:

        img = cam.snapshot()

        blobs = img.find_blobs(
            [RED_TH],
            pixels_threshold=200,
            merge=True
        )

        if blobs:

            b = max(blobs, key=lambda x: x.pixels())

            x = b.cx()
            y = b.cy()
            px = b.pixels()

            raw_x = x - CX
            raw_y = y - CY

            smooth_x = smooth_x * (1 - SMOOTH_X) + raw_x * SMOOTH_X
            smooth_y = smooth_y * (1 - SMOOTH_Y) + raw_y * SMOOTH_Y

            pixel_avg = pixel_avg * 0.9 + px * 0.1

            now = time.ticks_ms()

            # ===============================
            # 水平控制（100ms）
            # ===============================

            if time.ticks_diff(now, last_pan_control) >= PAN_CONTROL_PERIOD:

                last_pan_control = now

                if abs(smooth_x) > DEADZONE_X:

                    err = abs(smooth_x)

                    if err > 250:
                        gain = 1.2
                        speed = 700
                    elif err > 150:
                        gain = 0.9
                        speed = 500
                    elif err > 80:
                        gain = 0.6
                        speed = 350
                    else:
                        gain = 0.3
                        speed = 200

                    set_speed_lr(abs(speed))

                   # =========================
                   # 位置环
                   # =========================

                    pos_delta = int(smooth_x * gain)


                    # =========================
                    # 速度环
                    # =========================

                    speed = int(abs(smooth_x)*2)

                    speed = max(100,min(800,speed))


                    # 方向
                    if smooth_x < 0:
                     speed = -speed


# =========================
# 速位融合
# =========================

                    delta = int(pos_delta*0.7 + speed*0.003)


                    delta=max(-20,min(20,delta))

                    if abs(delta) >= 5:

                        pan_target += delta

                        if tilt_target > TILT_MAX:
                         tilt_target = TILT_MAX

                        if tilt_target < TILT_MIN:
                         tilt_target = TILT_MIN
                        move_motor(
                            uart_lr,
                            HORIZONTAL_ADDR,
                            pan_target
                        )

            # ===============================
            # 垂直控制（50ms）
            # ===============================

            if time.ticks_diff(now, last_tilt_control) >= TILT_CONTROL_PERIOD:

                last_tilt_control = now

                if abs(smooth_y) > DEADZONE_Y:

                    err = abs(smooth_y)

                    if err > 250:
                        gain = 2.0
                        speed = 700
                    elif err > 150:
                        gain = 1.5
                        speed = 500
                    elif err > 80:
                        gain = 1.0
                        speed = 350
                    else:
                        gain = 0.5
                        speed = 200

                    set_speed_ud(abs(speed))
                    # =========================
# 位置环
# =========================

                    pos_delta=int(smooth_y*gain)


# =========================
# 速度环
# =========================

                    speed=int(abs(smooth_y)*3)

                    speed=max(100,min(900,speed))


                    if smooth_y<0:
                     speed=-speed


# =========================
# 融合
# =========================

                    delta=int(
                       pos_delta*0.6
                        +
                       speed*0.005
                    )


                    delta=max(-80,min(80,delta))

                    if abs(delta) >= 2:

                        tilt_target -= delta

                        if tilt_target > TILT_LIMIT:
                            tilt_target = TILT_LIMIT

                        if tilt_target < -TILT_LIMIT:
                            tilt_target = -TILT_LIMIT

                        move_motor(
                            uart_ud,
                            VERTICAL_ADDR,
                            tilt_target
                        )

            # ===============================
            # 绘图
            # ===============================

            img.draw_rectangle(
                b.rect(),
                color=(255,0,0),
                thickness=2
            )

            img.draw_cross(
                x,
                y,
                color=(255,0,0),
                size=10
            )

            img.draw_line(
                x,
                y,
                CX,
                CY,
                color=(255,200,0)
            )

            dist_ratio = int(pixel_avg / DIST_REF * 100)

            if dist_ratio > 120:
                txt = "Near"
                color = (0,255,0)
            elif dist_ratio < 80:
                txt = "Far"
                color = (255,100,100)
            else:
                txt = "Middle"
                color = (255,255,0)

            img.draw_string_advanced(
                20,
                80,
                28,
                "Dist:%s %d%%"%(txt,dist_ratio),
                color=color
            )

            img.draw_string_advanced(
                20,
                40,
                32,
                "Tracking",
                color=(0,255,255)
            )

        else:

            smooth_x = 0
            smooth_y = 0

            img.draw_string_advanced(
                20,
                40,
                32,
                "Searching",
                color=(255,150,150)
            )

        img.draw_cross(
            CX,
            CY,
            color=(0,255,0),
            size=25,
            thickness=3
        )

        img.draw_circle(
            CX,
            CY,
            40,
            color=(0,255,0),
            thickness=2
        )

        Display.show_image(img)

        time.sleep_ms(30)


# ============================================
# 入口
# ============================================

if __name__ == "__main__":

    try:
        main()

    except KeyboardInterrupt:
        print("Stop")

    finally:
        print("Exit")
