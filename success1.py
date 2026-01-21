import time, os, gc, sys, math,utime
from machine import PWM, FPIOA, Pin, UART
from media.sensor import *
from media.display import *
from media.media import *
from pid import PID

DETECT_WIDTH = 640
DETECT_HEIGHT = 480

sensor = None

###############################舵机配置#####################################################
# 2.5 = -90度 7.5 = 0度 12.5 = 90度
min_duty = 2      #最小占空比
max_duty = 12    #最大占空比
mid_duty = 7.5      # 中间值，对应于0度
pwm_lr = None

# 配置排针引脚号12，芯片引脚号为47的排针复用为PWM通道3输出
pwm_io1 = FPIOA()
pwm_io1.set_function(47, FPIOA.PWM3)
# 初始化PWM参数
pwm_ud = PWM(3, 50, 50, enable=True)  # 默认频率50Hz,占空比50% 3~12

# 配置排针引脚号32，芯片引脚号为46的排针复用为PWM通道2输出
pwm_io2 = FPIOA()
pwm_io2.set_function(46, FPIOA.PWM2)
# 初始化PWM参数
pwm_lr = PWM(2, 50, 50, enable=True)  # 默认频率50Hz,占空比50% 2~13

pwm_lr.duty(7.5)    #旋转到中间
pwm_ud.duty(4)    #旋转到中间
##########################################################################################

###############################PID配置#####################################################
lr_kp = 0.17
lr_ki = 0.016
lr_kd = 0
lr_max_out = DETECT_WIDTH//2		#PID最大输出
pid_lr = PID(lr_kp, lr_ki, lr_kd, lr_max_out, lr_max_out)

ud_kp = 0.17
ud_ki = 0.016
ud_kd = 0
ud_max_out = DETECT_HEIGHT//2		#PID最大输出
pid_ud = PID(ud_kp, ud_ki, ud_kd, ud_max_out, ud_max_out)
##########################################################################################

## 将Y轴偏移数值转换为占空比的函数
def input_to_duty_cycle(input_min, input_max, input_value):
    # 定义输入输出范围
#    input_min = -(DETECT_HEIGHT // 2)
#    input_max = (DETECT_HEIGHT // 2)
    output_min = min_duty
    output_max = max_duty

    # 检查输入是否越界
    if input_value < input_min or input_value > input_max:
        raise ValueError(f"输入值必须在 {input_min} 和 {input_max} 之间")

    # 计算线性映射公式
    slope = (output_max - output_min) / (input_max - input_min)
    output_value = output_min + (input_value - input_min) * slope

    return output_value

try:
    # 初始化摄像头
    sensor = Sensor(width = DETECT_WIDTH, height = DETECT_HEIGHT)
    # 传感器复位
    sensor.reset()
    # 开启镜像
    sensor.set_hmirror(True)#False
    # sensor vflip
    sensor.set_vflip(True)#False True
    # 设置图像一帧的大小
    sensor.set_framesize(width = DETECT_WIDTH, height = DETECT_HEIGHT)
    # 设置图像输出格式为彩色的RGB565
    sensor.set_pixformat(Sensor.RGB565)
    # 使用IDE显示图像
    Display.init(Display.VIRT, width = DETECT_WIDTH, height = DETECT_HEIGHT, fps = 100)
    # 初始化媒体管理器
    MediaManager.init()
    # 摄像头传感器开启运行
    sensor.run()

    # 定义要识别颜色的阈值，这里需要根据你的具体情况调整
    # 你可以通过尝试不同的阈值来找到最适合你的物体颜色值
    red_threshold = (0, 42, 17, 94, -6, 50)

    while True:
        # 拍摄一张图片
        img = sensor.snapshot()
        # 查找图像中满足红色阈值的区域
        blobs = img.find_blobs([red_threshold], pixels_threshold=200, area_threshold=200, merge=True)

        # 如果找到了至少一个blob
        if blobs:
            # 找到最大的blob
            largest_blob = max(blobs, key=lambda b: b.pixels())
            # 画框
            img.draw_rectangle(largest_blob.rect(), color=(255, 0, 0))
            # 在框内画十字，标记中心点
            img.draw_cross(largest_blob.cx(), largest_blob.cy(), color=(255, 0, 0))

            # 计算相对于屏幕中心的X轴和Y轴的偏移量
            x_offset = largest_blob.cx() - img.width() // 2
            y_offset = largest_blob.cy() - img.height() // 2

            # 屏幕显示位置信息和像素大小，包含正负号
            wz = "x={}, y={}, w={}, h={}".format(x_offset, y_offset, largest_blob.w(), largest_blob.h())
            img.draw_string_advanced(0,0,32,wz)

            # 根据中心偏移量计算PWM的PID
            # 0为目标值，x_offset表示当前设定值
            pid_lr_value = pid_lr.pid_calc(0,x_offset)
            pid_ud_value = pid_ud.pid_calc(0,y_offset)
            ''' 关键点一 '''
            # 将PID值转换为舵机的范围并且输出实际的占空比
            duty_lr_value = input_to_duty_cycle(-(DETECT_WIDTH // 2), (DETECT_WIDTH // 2), pid_lr_value)
            duty_ud_value = input_to_duty_cycle(-(DETECT_HEIGHT // 2), (DETECT_HEIGHT // 2), pid_ud_value)
            print(f"pid_lr_value={pid_lr_value}, pid_ud_value={pid_ud_value}\r\n")
            print(f"duty_lr_value={duty_lr_value}, duty_ud_value={duty_ud_value}\r\n")
            ''' 关键点二 '''
            # 根据计算后的占空比控制舵机动作
            pwm_lr.duty(duty_lr_value)
            pwm_ud.duty(duty_ud_value)
        # 中心画十字
        img.draw_cross(img.width() // 2, img.height() // 2, color=(0, 255, 0), size=10, thickness=3)
        # IDE显示图片
        Display.show_image(img)

except KeyboardInterrupt as e:
    print(f"user stop")
except BaseException as e:
    print(f"Exception &#039;{e}&#039;")
finally:
    # sensor stop run
    if isinstance(sensor, Sensor):
        sensor.stop()
    # deinit display
    Display.deinit()

    if isinstance(pwm_lr, PWM):
        pwm_lr.deinit()

    # release media buffer
    MediaManager.deinit()

    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
