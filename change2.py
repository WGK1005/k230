'''
K230 PID控制云台 - 纯整数运算完整版
'''

import time
from media.sensor import *
from media.display import *
from media.media import *
from machine import PWM, FPIOA, UART

# ============ 配置参数 ============
W = 800      # 屏幕宽度（交换宽高修正旋转）
H = 480      # 屏幕高度
CX = W // 2  # 中心X
CY = H // 2  # 中心Y

# ============ 舵机配置 ============
# 舵机脉宽范围 (纳秒)
SERVO_MIN_NS = 1000000   # 1.0ms
SERVO_MID_NS = 1500000   # 1.5ms (360度停止 / 180度中位90度)
SERVO_MAX_NS = 2000000   # 2.0ms

# 垂直舵机 (MG966R 180度) - 角度控制
# 初始化到90度，运动范围 45-135度
TILT_ANGLE_MIN = 4500    # 45度 * 100
TILT_ANGLE_MAX = 13500   # 135度 * 100
TILT_ANGLE_INIT = 9000   # 90度 * 100 (中位)

# 水平舵机 (MG966R 360度) - 速度控制
# 输出范围：-10000到10000表示速度 (乘以100)
# 0表示停止，负值左转，正值右转
PAN_SPEED_MIN = -10000   # 最大左转速度 * 100
PAN_SPEED_MAX = 10000    # 最大右转速度 * 100
PAN_SPEED_STOP = 0       # 停止

# ============ PID控制器类 ============
class PIDControllerInt:
    def __init__(self, kp, ki, kd, out_min, out_max):
        """
        整数PID控制器
        所有系数乘以10000避免小数
        """
        self.kp = kp      # 比例系数 * 10000
        self.ki = ki      # 积分系数 * 10000
        self.kd = kd      # 微分系数 * 10000
        self.out_min = out_min  # 输出最小值
        self.out_max = out_max  # 输出最大值

        # 状态变量
        self.integral = 0       # 积分项 * 10000
        self.last_error = 0     # 上一次误差
        self.last_time = 0      # 上一次时间戳

        # 积分限幅
        self.integral_max = 1000000  # 最大积分值

    def reset(self):
        """重置PID状态"""
        self.integral = 0
        self.last_error = 0
        self.last_time = 0

    def calculate(self, error):
        """
        计算PID输出
        error: 误差值
        返回: 控制输出
        """
        current_time = time.ticks_ms()

        # 第一次调用
        if self.last_time == 0:
            self.last_time = current_time
            self.last_error = error
            return 0

        # 计算时间差 (毫秒)
        dt = current_time - self.last_time
        if dt < 1:
            dt = 1  # 防止除零

        # ===== 比例项 P =====
        # output_p = kp * error
        p_term = (self.kp * error) // 10000  # 除以10000还原

        # ===== 积分项 I =====
        # 积分累加: integral += error * dt
        self.integral += error * dt

        # 积分限幅防饱和
        if self.integral > self.integral_max:
            self.integral = self.integral_max
        elif self.integral < -self.integral_max:
            self.integral = -self.integral_max

        # output_i = ki * integral
        i_term = (self.ki * self.integral) // 1000000  # 除以1000000 (10000*100)

        # ===== 微分项 D =====
        # 微分: derivative = (error - last_error) / dt
        derivative = ((error - self.last_error) * 1000) // dt  # 乘以1000提高精度

        # output_d = kd * derivative
        d_term = (self.kd * derivative) // 10000  # 除以10000还原

        # ===== PID总和 =====
        output = p_term + i_term + d_term

        # ===== 输出限幅 =====
        if output > self.out_max:
            output = self.out_max
        elif output < self.out_min:
            output = self.out_min

        # ===== 更新状态 =====
        self.last_error = error
        self.last_time = current_time

        return output

    def get_status(self):
        """获取PID状态"""
        return {
            'integral': self.integral,
            'last_error': self.last_error
        }

# ============ 辅助函数 ============
def tilt_angle_to_ns(angle):
    """
    垂直舵机角度转脉宽 (180度舵机)
    angle: 角度 * 100 (4500到13500，对应45-135度)
    返回: 纳秒 (1000000 到 2000000)
    """
    # 限制角度范围
    if angle > TILT_ANGLE_MAX:
        angle = TILT_ANGLE_MAX
    elif angle < TILT_ANGLE_MIN:
        angle = TILT_ANGLE_MIN

    # 正向映射: 0-180度 -> 1.0-2.0ms
    # angle=0 -> 1000000ns, angle=18000 -> 2000000ns
    ns = SERVO_MIN_NS + ((angle * 1000000) // 18000)  # 18000 = 180度*100

    # 限制脉宽范围
    if ns > SERVO_MAX_NS:
        ns = SERVO_MAX_NS
    elif ns < SERVO_MIN_NS:
        ns = SERVO_MIN_NS

    return ns

def pan_speed_to_ns(speed):
    """
    水平舵机速度转脉宽 (360度舵机)
    speed: 速度 * 100 (-10000到10000)
           负值左转，0停止，正值右转
    返回: 纳秒 (1000000 到 2000000)
    """
    # 限制速度范围
    if speed > PAN_SPEED_MAX:
        speed = PAN_SPEED_MAX
    elif speed < PAN_SPEED_MIN:
        speed = PAN_SPEED_MIN

    # 映射: -10000到10000 -> 1.0ms到2.0ms
    # 0 -> 1.5ms (停止)
    # speed=10000 -> 1500000 + 500000 = 2000000ns
    # speed=-10000 -> 1500000 - 500000 = 1000000ns
    ns = SERVO_MID_NS + (speed * 50)  # 修正转换公式

    # 限制脉宽范围
    if ns > SERVO_MAX_NS:
        ns = SERVO_MAX_NS
    elif ns < SERVO_MIN_NS:
        ns = SERVO_MIN_NS

    return ns

def ns_to_tilt_angle(ns):
    """脉宽纳秒转垂直角度"""
    if ns > SERVO_MAX_NS:
        ns = SERVO_MAX_NS
    elif ns < SERVO_MIN_NS:
        ns = SERVO_MIN_NS
    
    angle = ((ns - SERVO_MIN_NS) * 18000) // 1000000
    return angle

def ns_to_pan_speed(ns):
    """脉宽纳秒转水平速度"""
    if ns > SERVO_MAX_NS:
        ns = SERVO_MAX_NS
    elif ns < SERVO_MIN_NS:
        ns = SERVO_MIN_NS
    
    speed = ((ns - SERVO_MID_NS) * 1000) // 500
    return speed

# ============ 初始化函数 ============
def init_servos():
    """初始化舵机"""
    print("初始化舵机...")

    # 配置垂直舵机 (GPIO46 -> PWM2, MG966R 180度) *** 修正 ***
    pwm_io1 = FPIOA()
    pwm_io1.set_function(46, FPIOA.PWM2)
    pwm_ud = PWM(2, freq=50)  # 50Hz

    # 配置水平舵机 (GPIO47 -> PWM3, MG966R 360度) *** 修正 ***
    pwm_io2 = FPIOA()
    pwm_io2.set_function(47, FPIOA.PWM3)
    pwm_lr = PWM(3, freq=50)  # 50Hz

    # 垂直舵机初始化到90度
    pwm_ud.duty_ns(tilt_angle_to_ns(TILT_ANGLE_INIT))
    print(f"垂直舵机初始化(GPIO46): 90度 ({tilt_angle_to_ns(TILT_ANGLE_INIT)}ns)")
    
    # 水平舵机初始化停止 (1.5ms)
    pwm_lr.duty_ns(SERVO_MID_NS)
    print(f"水平舵机初始化(GPIO47): 停止 ({SERVO_MID_NS}ns)")
    
    time.sleep(1)

    return pwm_lr, pwm_ud

def init_camera():
    """初始化摄像头"""
    print("初始化摄像头...")

    sensor = Sensor(width=W, height=H)
    sensor.reset()
    # 不设置镜像和翻转，保持画面正常方向
    # sensor.set_hmirror(True)  # 注释掉水平镜像
    # sensor.set_vflip(True)     # 注释掉垂直翻转
    sensor.set_framesize(width=W, height=H)
    sensor.set_pixformat(Sensor.RGB565)

    Display.init(Display.ST7701, width=W, height=H, to_ide=True)
    MediaManager.init()
    sensor.run()

    return sensor

def init_uart():
    """初始化串口"""
    try:
        uart = UART(1, baudrate=115200, tx=4, rx=5)
        print("串口初始化成功")
        return uart
    except:
        print("串口初始化失败")
        return None

# ============ 主程序 ============
def main():
    print("=" * 40)
    print("K230 PID云台控制 - 整数运算版")
    print("=" * 40)

    # 初始化硬件
    pwm_lr, pwm_ud = init_servos()
    sensor = init_camera()
    uart = init_uart()

    # ===== PID参数配置 =====
    # 所有系数乘以10000
    # 例如: 0.1 -> 1000, 0.01 -> 100

    # 水平PID (控制360度舵机速度)
    # 输出速度值：-10000到10000
    pid_pan = PIDControllerInt(
        kp=5000,     # 0.50 * 10000 (大幅提高响应)
        ki=300,      # 0.03 * 10000
        kd=1000,     # 0.10 * 10000
        out_min=PAN_SPEED_MIN,
        out_max=PAN_SPEED_MAX
    )

    # 垂直PID (控制180度舵机角度偏移)
    # 输出角度偏移：-4500到+4500 (±45度偏移，从90度基准)
    pid_tilt = PIDControllerInt(
        kp=5000,     # 0.50 * 10000 (大幅提高响应)
        ki=200,      # 0.02 * 10000
        kd=1000,     # 0.10 * 10000
        out_min=-4500,  # 最大-45度偏移 (最小45度)
        out_max=4500    # 最大+45度偏移 (最大135度)
    )

    # ===== 跟踪参数 =====
    RED_THRESHOLD = (20, 80, 30, 100, 0, 60)  # 红色阈值
    MIN_PIXELS = 200                           # 最小像素面积
    IDE_DISPLAY_INTERVAL = 10  # IDE显示间隔：每10帧传输1次到IDE

    # ===== 状态变量 =====
    frame_count = 0
    last_frame_time = time.ticks_ms()
    fps = 0
    target_history = []
    MAX_HISTORY = 20
    lost_target_frames = 0  # 丢失目标的帧数计数器
    LOST_TARGET_DELAY = 90  # 3秒延迟(假设30fps,3秒=90帧)

    print("\n开始PID跟踪...")
    print("按Ctrl+C停止\n")

    try:
        while True:
            frame_start = time.ticks_ms()

            # 1. 捕获图像
            img = sensor.snapshot()

            # 2. 寻找红色物块
            blobs = img.find_blobs([RED_THRESHOLD],
                                  pixels_threshold=MIN_PIXELS,
                                  area_threshold=MIN_PIXELS,
                                  merge=True)

            target_found = False
            error_x = 0
            error_y = 0
            target_size = 0

            if blobs:
                target_found = True
                blob = max(blobs, key=lambda b: b.pixels())
                x = blob.cx()
                y = blob.cy()
                target_size = blob.pixels()

                # 3. 计算误差 (乘以100提高精度)
                # 误差方向：目标位置 - 中心位置
                # 但需要反向控制：物体在左，云台左转；物体在上，云台上转
                error_x = (x - CX) * 100  # 正值表示目标在右，需要右转
                error_y = (y - CY) * 100  # 正值表示目标在下，需要下转

                # 记录目标历史
                target_history.append((x, y, frame_count))
                if len(target_history) > MAX_HISTORY:
                    target_history.pop(0)

                # 4. PID计算控制量
                # 水平: 直接比例控制，极低精度追求稳定性
                # 比例系数2：100像素误差 -> 200单位（2%速度）
                pan_speed_simple = (error_x * 2) // 100
                
                # 水平舵机死区补偿：360度舵机需要足够大的速度才能转动
                # 限制速度范围
                if pan_speed_simple > PAN_SPEED_MAX:
                    pan_speed_simple = PAN_SPEED_MAX
                elif pan_speed_simple < PAN_SPEED_MIN:
                    pan_speed_simple = PAN_SPEED_MIN
                
                # 增大死区，降低最小速度
                if abs(pan_speed_simple) < 600:  # 小于6%速度时停止
                    pan_speed = 0
                elif pan_speed_simple > 0:
                    pan_speed = max(pan_speed_simple, 2000)  # 正向最小20%速度
                else:
                    pan_speed = min(pan_speed_simple, -2000)  # 反向最小20%速度
                
                # 垂直: 简单比例控制，极低精度追求稳定性
                # 直接根据误差计算偏移，不经PID
                # 物体在上方(y<CY)时error_y为负，需要角度增加(抬头)
                # 超低比例系数: 0.15，即100像素误差 -> 15度偏移
                tilt_offset_simple = (-error_y * 15) // 100  # 超低精度追求稳定
                # 限制偏离范围
                if tilt_offset_simple > 4500:
                    tilt_offset_simple = 4500
                elif tilt_offset_simple < -4500:
                    tilt_offset_simple = -4500
                
                tilt_offset = tilt_offset_simple  # 使用简单比例控制

                # 5. 控制舵机
                # 水平舵机: 速度控制 (360度舵机)
                pwm_lr.duty_ns(pan_speed_to_ns(pan_speed))
                
                # 垂直舵机: 位置控制 (180度舵机)
                # 目标角度 = 初始90度 + 偏移
                target_tilt_angle = TILT_ANGLE_INIT + tilt_offset
                # 限制角度范围
                if target_tilt_angle > TILT_ANGLE_MAX:
                    target_tilt_angle = TILT_ANGLE_MAX
                elif target_tilt_angle < TILT_ANGLE_MIN:
                    target_tilt_angle = TILT_ANGLE_MIN
                
                # 计算实际脉宽并设置
                tilt_ns = tilt_angle_to_ns(target_tilt_angle)
                pwm_ud.duty_ns(tilt_ns)

                # 6. 绘制目标
                # 目标框
                img.draw_rectangle(blob.rect(), color=(255,0,0), thickness=2)
                img.draw_cross(x, y, color=(255,0,0), size=10, thickness=2)

                # 轨迹线
                if len(target_history) > 1:
                    for i in range(1, len(target_history)):
                        x1, y1, _ = target_history[i-1]
                        x2, y2, _ = target_history[i]
                        img.draw_line(x1, y1, x2, y2, color=(255,100,0), thickness=1)

                # 从目标到中心的线
                img.draw_line(x, y, CX, CY, color=(255,150,0), thickness=1)

                # 重置丢失目标计数器(找到目标时)
                lost_target_frames = 0

            else:
                # 没有目标,增加丢失帧计数
                lost_target_frames += 1

                # 立即停止水平舵机转动
                pwm_lr.duty_ns(SERVO_MID_NS)  # 水平舵机立即停止

                # 只有在丢失目标超过3秒后垂直舵机才返回初始位置
                if lost_target_frames >= LOST_TARGET_DELAY:
                    # 重置PID积分
                    pid_pan.reset()
                    pid_tilt.reset()

                    # 垂直舵机回90度
                    pwm_ud.duty_ns(tilt_angle_to_ns(TILT_ANGLE_INIT))  # 90度

                    # 清空历史
                    target_history = []
                # 否则垂直舵机保持当前位置不变

            # 7. 绘制界面信息

            # 中心十字
            img.draw_cross(CX, CY, color=(0,255,0), size=15, thickness=3)

            # 信息显示区域
            y_pos = 40

            # 标题
            title = "PID云台控制" if target_found else "等待目标"
            title_color = (0,255,255) if target_found else (255,100,100)
            img.draw_string_advanced(20, y_pos, 32, title, color=title_color)
            y_pos += 40

            if target_found:
                # PID状态信息
                pan_speed_val = ns_to_pan_speed(pwm_lr.duty_ns()) // 100
                tilt_angle_val = ns_to_tilt_angle(pwm_ud.duty_ns()) // 100
                
                # 计算垂直舵机的详细调试信息
                tilt_pulse = pwm_ud.duty_ns()
                tilt_offset_val = tilt_offset // 100  # 偏移量（度）
                target_angle_val = target_tilt_angle // 100  # 目标角度
                
                # 水平舵机调试信息
                pan_pulse = pwm_lr.duty_ns()
                pan_speed_output = pan_speed // 100  # 实际输出速度
                pan_speed_calc = pan_speed_simple // 100  # 计算速度

                info_lines = [
                    f"误X:{error_x//100:+4d} 误Y:{error_y//100:+4d}",
                    f"水平:计算{pan_speed_calc:+3d} 输出{pan_speed_output:+3d}",
                    f"水平脉宽:{pan_pulse}ns",
                    f"垂直:偏移{tilt_offset_val:+3d}° 角度{tilt_angle_val:3d}°",
                ]

                for i, line in enumerate(info_lines):
                    color = (255,255,0) if i == 0 else (200,200,255) if i == 1 else (200,255,200)
                    img.draw_string_advanced(20, y_pos, 26, line, color=color)
                    y_pos += 28

                # PID内部状态
                pid_pan_status = pid_pan.get_status()
                pid_tilt_status = pid_tilt.get_status()

                pid_info = [
                    f"积分项: P:{pid_pan_status['integral']//1000} T:{pid_tilt_status['integral']//1000}",
                    f"上次误差: P:{pid_pan_status['last_error']} T:{pid_tilt_status['last_error']}",
                ]

                for line in pid_info:
                    img.draw_string_advanced(20, y_pos, 22, line, color=(150,150,255))
                    y_pos += 24

            else:
                # 无目标信息
                img.draw_string_advanced(20, y_pos, 28, "未检测到红色物块", color=(255,150,150))
                y_pos += 30
                img.draw_string_advanced(20, y_pos, 24, "请将红色物体放入视野", color=(200,200,200))
                y_pos += 26

            # 8. 串口输出数据
            if uart and frame_count % 5 == 0:  # 每5帧输出一次（增加频率）
                if target_found:
                    pan_ns = pwm_lr.duty_ns()
                    tilt_ns = pwm_ud.duty_ns()
                    # 输出更详细的调试信息
                    data = f"X:{error_x//100:+4d} Y:{error_y//100:+4d} Pan:{pan_ns} Tilt:{tilt_ns} Angle:{tilt_angle_val}\n"
                else:
                    data = "NO_TARGET\n"
                try:
                    uart.write(data.encode())
                except:
                    pass

            # 9. 性能统计
            frame_count += 1
            if frame_count % 30 == 0:
                current_time = time.ticks_ms()
                elapsed = current_time - last_frame_time
                fps = 30000 // max(1, elapsed)  # 估算FPS
                last_frame_time = current_time

            # 显示FPS
            fps_text = f"FPS: {fps}"
            img.draw_string_advanced(W-100, H-40, 22, fps_text, color=(100,255,100))

            # 10. 显示到屏幕和IDE
            Display.show_image(img)
            
            # 每隔N帧才传输到IDE（减少IDE传输压力）
            if frame_count % IDE_DISPLAY_INTERVAL == 0:
                Display.show_image(img, to_ide=True)

            # 11. 控制帧率
            frame_time = time.ticks_diff(time.ticks_ms(), frame_start)
            if frame_time < 33:  # 目标30FPS
                time.sleep_ms(33 - frame_time)

    except KeyboardInterrupt:
        print("\n用户中断...")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        # 清理资源
        print("\n清理资源...")
        print("舵机停止/回中...")
        pwm_lr.duty_ns(SERVO_MID_NS)  # 360度舵机停止
        pwm_ud.duty_ns(tilt_angle_to_ns(TILT_ANGLE_INIT))  # 180度舵机回90度
        time.sleep(1)

        print(f"总共处理 {frame_count} 帧")
        print("系统停止")

# ============ 运行主程序 ============
if __name__ == "__main__":
    main()
