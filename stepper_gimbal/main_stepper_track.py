import time
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA

# ===========================
# 参数配置
# ===========================
W, H = 800, 480
CX, CY = W // 2, H // 2

PULSE_PER_REV = 3200

PITCH_ADDR = 0x01   # 垂直
YAW_ADDR   = 0x02   # 水平

RED_TH = (20, 80, 30, 100, 0, 60)

# ===== 追踪参数 =====
DEADZONE   = 15
SMOOTH     = 0.4
PAN_GAIN   = 0.08     # 度/像素
TILT_GAIN  = 0.08
MAX_STEP   = 2.0      # 单次最大转动角度
MIN_STEP   = 0.2      # 最小转动角度

# ===========================
# UART 初始化
# ===========================
fpioa = FPIOA()
fpioa.set_function(3, FPIOA.UART1_TXD)
fpioa.set_function(4, FPIOA.UART1_RXD)
fpioa.set_function(5, FPIOA.UART2_TXD)
fpioa.set_function(6, FPIOA.UART2_RXD)

uart_pitch = UART(UART.UART1, baudrate=115200)   # 垂直
uart_yaw   = UART(UART.UART2, baudrate=115200)   # 水平


# ===========================
# X42S 底层
# ===========================
def send(uart, cmd):
    uart.write(bytes(cmd))

def fast_mode_init(uart, addr, speed=350, acc=5):
    cmd = [addr, 0xF1,
           (speed >> 8) & 0xff, speed & 0xff,
           acc, 0x01, 0x00, 0x6B]
    send(uart, cmd)

def move_motor(uart, addr, target):
    """绝对位置运动(脉冲)"""
    if target < 0:
        target = (1 << 32) + target
    cmd = [addr, 0xFC,
           (target >> 24) & 0xff,
           (target >> 16) & 0xff,
           (target >> 8) & 0xff,
           target & 0xff,
           0x6B]
    send(uart, cmd)

def angle_to_pulse(angle):
    return int((angle / 360.0) * PULSE_PER_REV)

def pulse_to_angle(pulse):
    return pulse * 360.0 / PULSE_PER_REV


# ===========================
# 追踪状态
# ===========================
# 用软件记录当前目标角度（避免频繁回读）
cur_yaw_deg   = 0.0
cur_pitch_deg = 0.0

def init_motors():
    print("初始化电机速度...")
    fast_mode_init(uart_yaw,   YAW_ADDR,   speed=200, acc=4)
    fast_mode_init(uart_pitch, PITCH_ADDR, speed=200, acc=4)
    time.sleep_ms(200)

    print("归零...")
    move_motor(uart_yaw,   YAW_ADDR,   0)
    move_motor(uart_pitch, PITCH_ADDR, 0)
    time.sleep(2)

    # 垂直电机正转 90°
    print("垂直电机正转 90°...")
    move_motor(uart_pitch, PITCH_ADDR, angle_to_pulse(90))
    time.sleep(2)


# ===========================
# 摄像头初始化
# ===========================
def init_cam():
    s = Sensor(width=W, height=H)
    s.reset()
    s.set_framesize(width=W, height=H)
    s.set_pixformat(Sensor.RGB565)
    Display.init(Display.ST7701, width=W, height=H, to_ide=True)
    MediaManager.init()
    s.run()
    time.sleep(0.5)
    return s


# ===========================
# 追踪主逻辑
# ===========================
def main():
    global cur_yaw_deg, cur_pitch_deg

    print("X42S 红色追踪启动")
    init_motors()
    cam = init_cam()

    smooth_x, smooth_y = 0.0, 0.0

    try:
        while True:
            img = cam.snapshot()
            blobs = img.find_blobs([RED_TH], pixels_threshold=200, merge=True)

            if blobs:
                b = max(blobs, key=lambda x: x.pixels())
                x, y = b.cx(), b.cy()

                raw_x = x - CX
                raw_y = y - CY

                smooth_x = smooth_x * (1 - SMOOTH) + raw_x * SMOOTH
                smooth_y = smooth_y * (1 - SMOOTH) + raw_y * SMOOTH

                # === 水平 Yaw ===
                if abs(smooth_x) > DEADZONE:
                    delta = -smooth_x * PAN_GAIN
                    delta = max(-MAX_STEP, min(MAX_STEP, delta))
                    if abs(delta) >= MIN_STEP:
                        cur_yaw_deg += delta
                        move_motor(uart_yaw, YAW_ADDR, angle_to_pulse(cur_yaw_deg))

                # === 垂直 Pitch ===
                if abs(smooth_y) > DEADZONE:
                    delta = - smooth_y * TILT_GAIN
                    delta = max(-MAX_STEP, min(MAX_STEP, delta))
                    if abs(delta) >= MIN_STEP:
                        cur_pitch_deg += delta
                        move_motor(uart_pitch, PITCH_ADDR, angle_to_pulse(cur_pitch_deg))

                img.draw_rectangle(b.rect(), color=(255, 0, 0), thickness=2)
                img.draw_cross(x, y, color=(255, 0, 0), size=10)
                img.draw_string_advanced(20, 40, 32, "锁定中", color=(0, 255, 255))

            else:
                smooth_x, smooth_y = 0.0, 0.0
                img.draw_string_advanced(20, 40, 32, "搜索中", color=(255, 150, 150))

            img.draw_cross(CX, CY, color=(0, 255, 0), size=25, thickness=3)
            img.draw_circle(CX, CY, 40, color=(0, 255, 0), thickness=2)

            Display.show_image(img)
            time.sleep_ms(50)

    except KeyboardInterrupt:
        pass
    finally:
        move_motor(uart_yaw,   YAW_ADDR,   0)
        move_motor(uart_pitch, PITCH_ADDR, 0)
        print("停止")


if __name__ == "__main__":
    main()
