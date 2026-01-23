'''
K230 云台红色物体跟踪 - 完整版(含前后距离识别)
'''

import time
from media.sensor import *
from media.display import *
from media.media import *
from machine import PWM, FPIOA

# ========== 配置参数 ==========
W, H = 800, 480  # 屏幕尺寸
CX, CY = W // 2, H // 2  # 中心点

# 安全区设置(面积占屏幕3/5)
SAFE_AREA_RATIO = 0.7746  # sqrt(3/5)
SAFE_AREA_W = int(W * SAFE_AREA_RATIO)  # 安全区宽度
SAFE_AREA_H = int(H * SAFE_AREA_RATIO)  # 安全区高度
SAFE_AREA_X_MIN = (W - SAFE_AREA_W) // 2
SAFE_AREA_X_MAX = SAFE_AREA_X_MIN + SAFE_AREA_W
SAFE_AREA_Y_MIN = (H - SAFE_AREA_H) // 2
SAFE_AREA_Y_MAX = SAFE_AREA_Y_MIN + SAFE_AREA_H

# 舵机脉宽(纳秒)
SERVO_MIN_NS = 1000000   # 1.0ms
SERVO_MID_NS = 1500000   # 1.5ms
SERVO_MAX_NS = 2000000   # 2.0ms

# 垂直舵机(180度) - 角度控制
TILT_ANGLE_MIN = 4500    # 45度*100
TILT_ANGLE_MAX = 13500   # 135度*100
TILT_ANGLE_INIT = 9000   # 90度*100

# 水平舵机(360度) - 速度控制
PAN_SPEED_MIN = -10000   # 最大左转*100
PAN_SPEED_MAX = 10000    # 最大右转*100

# 跟踪参数
RED_THRESHOLD = (20, 80, 30, 100, 0, 60)
MIN_PIXELS = 200
LOST_TARGET_DELAY = 90  # 3秒延迟(30fps)

# 控制系数
PAN_COEF = 0.5    # 水平比例系数 (在安全区内使用)
PAN_COEF_OUTSIDE = 1.5  # 水平比例系数 (离开安全区时使用,更快响应)
TILT_COEF = 15    # 垂直比例系数
PAN_DEADZONE = 800    # 水平死区 (增大以减少抖动)
PAN_MIN_SPEED = 500  # 最小转动速度
PAN_MIN_SPEED_OUTSIDE = 1500  # 最小转动速度 (离开安全区时使用,更快响应)

# 前后距离识别参数
DISTANCE_REFERENCE_PIXELS = 5000  # 参考像素数(标准距离)
DISTANCE_DEADZONE = 500  # 距离死区(像素数)

# ========== 转换函数 ==========
def tilt_angle_to_ns(angle):
    """垂直角度转脉宽"""
    angle = max(TILT_ANGLE_MIN, min(TILT_ANGLE_MAX, angle))
    ns = SERVO_MIN_NS + ((angle * 1000000) // 18000)
    return max(SERVO_MIN_NS, min(SERVO_MAX_NS, ns))

def pan_speed_to_ns(speed):
    """水平速度转脉宽"""
    speed = max(PAN_SPEED_MIN, min(PAN_SPEED_MAX, speed))
    ns = SERVO_MID_NS + (speed * 50)
    return max(SERVO_MIN_NS, min(SERVO_MAX_NS, ns))

def analyze_distance(pixels):
    """
    分析物体前后距离
    :param pixels: 物体像素数
    :return: distance_state ('FORWARD', 'MIDDLE', 'BACKWARD')
    :return: distance_value: 距离差值(像素数)
    """
    diff = pixels - DISTANCE_REFERENCE_PIXELS
    
    if abs(diff) < DISTANCE_DEADZONE:
        return 'MIDDLE', diff  # 接近标准距离
    elif diff > 0:
        return 'FORWARD', diff  # 物体越来越近
    else:
        return 'BACKWARD', diff  # 物体越来越远

# ========== 初始化 ==========
def init_servos():
    """初始化舵机"""
    print("初始化舵机...")
    
    # 垂直舵机(GPIO46 -> PWM2)
    pwm_io1 = FPIOA()
    pwm_io1.set_function(46, FPIOA.PWM2)
    pwm_ud = PWM(2, freq=50)
    pwm_ud.duty_ns(tilt_angle_to_ns(TILT_ANGLE_INIT))
    
    # 水平舵机(GPIO47 -> PWM3)
    pwm_io2 = FPIOA()
    pwm_io2.set_function(47, FPIOA.PWM3)
    pwm_lr = PWM(3, freq=50)
    pwm_lr.duty_ns(SERVO_MID_NS)
    
    print(f"垂直舵机(GPIO46): 90度")
    print(f"水平舵机(GPIO47): 停止")
    time.sleep(1)
    
    return pwm_lr, pwm_ud

def init_camera():
    """初始化摄像头"""
    print("初始化摄像头...")
    
    sensor = Sensor(width=W, height=H)
    sensor.reset()
    sensor.set_framesize(width=W, height=H)
    sensor.set_pixformat(Sensor.RGB565)
    
    Display.init(Display.ST7701, width=W, height=H, to_ide=True)
    MediaManager.init()
    sensor.run()
    
    return sensor

# ========== 主程序 ==========
def main():
    print("=" * 40)
    print("K230 云台红色物体跟踪")
    print("含前后距离识别功能")
    print("=" * 40)
    
    # 初始化硬件
    pwm_lr, pwm_ud = init_servos()
    sensor = init_camera()
    
    # 状态变量
    frame_count = 0
    last_frame_time = time.ticks_ms()
    fps = 0
    target_history = []
    lost_target_frames = 0
    
    print("\n开始跟踪...\n按Ctrl+C停止\n")
    
    try:
        error_count = 0  # 错误计数器
        max_errors = 5   # 最大连续错误次数
        
        while True:
            try:
                frame_start = time.ticks_ms()
                
                # 捕获图像
                img = sensor.snapshot()
                
                # 寻找红色物块
                blobs = img.find_blobs([RED_THRESHOLD],
                                      pixels_threshold=MIN_PIXELS,
                                      area_threshold=MIN_PIXELS,
                                      merge=True)
                
                if blobs:
                    # 找到目标
                    blob = max(blobs, key=lambda b: b.pixels())
                    x, y = blob.cx(), blob.cy()
                    pixels = blob.pixels()  # 物体像素数
                    
                    # 计算原始误差(像素)
                    delta_x = x - CX
                    delta_y = y - CY
                    
                    # 分析前后距离
                    distance_state, distance_value = analyze_distance(pixels)
                    
                    # 检查物体是否在安全区内
                    in_safe_area = (SAFE_AREA_X_MIN <= x <= SAFE_AREA_X_MAX and 
                                   SAFE_AREA_Y_MIN <= y <= SAFE_AREA_Y_MAX)
                    
                    # 水平控制:只有在安全区外才追踪
                    if in_safe_area:
                        # 在安全区内,水平舵机停止转动
                        pan_speed = 0
                    else:
                        # 在安全区外,执行追踪(使用更高的速度)
                        # 物体在中央十字左侧(delta_x < 0) -> 云台向左(-速度)
                        # 物体在中央十字右侧(delta_x > 0) -> 云台向右(+速度)
                        
                        # 死区检查:如果偏差较小,停止转动
                        if abs(delta_x) < PAN_DEADZONE // 100:
                            pan_speed = 0
                        else:
                            # 计算速度(离开安全区时使用更高系数,反向计算)
                            pan_speed_simple = (-delta_x * PAN_COEF_OUTSIDE)
                            pan_speed_simple = max(PAN_SPEED_MIN, min(PAN_SPEED_MAX, pan_speed_simple))
                            
                            if pan_speed_simple > 0:
                                # 右侧偏差:云台向右转
                                pan_speed = max(pan_speed_simple, PAN_MIN_SPEED_OUTSIDE)
                            else:
                                # 左侧偏差:云台向左转
                                pan_speed = min(pan_speed_simple, -PAN_MIN_SPEED_OUTSIDE)
                    
                    # 垂直控制:比例控制
                    tilt_offset = (-delta_y * TILT_COEF)
                    tilt_offset = max(-4500, min(4500, tilt_offset))
                    
                    target_tilt_angle = TILT_ANGLE_INIT + tilt_offset
                    target_tilt_angle = max(TILT_ANGLE_MIN, min(TILT_ANGLE_MAX, target_tilt_angle))
                    
                    # 控制舵机
                    pwm_lr.duty_ns(pan_speed_to_ns(pan_speed))
                    pwm_ud.duty_ns(tilt_angle_to_ns(target_tilt_angle))
                    
                    # 绘制标记
                    img.draw_rectangle(blob.rect(), color=(255,0,0), thickness=2)
                    img.draw_cross(x, y, color=(255,0,0), size=10, thickness=2)
                    img.draw_line(x, y, CX, CY, color=(255,150,0), thickness=1)
                    
                    # 绘制安全区
                    safe_area_color = (0,255,0) if in_safe_area else (255,100,0)
                    img.draw_rectangle((SAFE_AREA_X_MIN, SAFE_AREA_Y_MIN, SAFE_AREA_W, SAFE_AREA_H), 
                                     color=safe_area_color, thickness=2)
                    
                    # 记录轨迹
                    target_history.append((x, y))
                    if len(target_history) > 20:
                        target_history.pop(0)
                    
                    if len(target_history) > 1:
                        for i in range(1, len(target_history)):
                            x1, y1 = target_history[i-1]
                            x2, y2 = target_history[i]
                            img.draw_line(x1, y1, x2, y2, color=(255,100,0), thickness=1)
                    
                    lost_target_frames = 0
                    
                    # 显示前后距离信息
                    if distance_state == 'FORWARD':
                        distance_display = "物体靠近"
                        distance_color = (0,255,0)  # 绿色
                    elif distance_state == 'BACKWARD':
                        distance_display = "物体远离"
                        distance_color = (255,0,0)  # 红色
                    else:
                        distance_display = "距离适中"
                        distance_color = (0,255,255)  # 黄色
                    
                    img.draw_string_advanced(20, 100, 24, distance_display, color=distance_color)
                    img.draw_string_advanced(20, 130, 20, f"像素: {pixels}", color=(200,200,200))
                    
                else:
                    # 未找到目标
                    lost_target_frames += 1
                    
                    # 立即停止水平舵机
                    pwm_lr.duty_ns(SERVO_MID_NS)
                    
                    # 3秒后垂直舵机返回初始位置
                    if lost_target_frames >= LOST_TARGET_DELAY:
                        pwm_ud.duty_ns(tilt_angle_to_ns(TILT_ANGLE_INIT))
                        target_history = []
                
                # 绘制中心十字
                img.draw_cross(CX, CY, color=(0,255,0), size=15, thickness=3)
                
                # 显示状态
                status = "跟踪中" if blobs else "等待目标"
                color = (0,255,255) if blobs else (255,100,100)
                img.draw_string_advanced(20, 40, 32, status, color=color)
                
                # FPS统计
                frame_count += 1
                if frame_count % 30 == 0:
                    current_time = time.ticks_ms()
                    elapsed = current_time - last_frame_time
                    fps = 30000 // max(1, elapsed)
                    last_frame_time = current_time
                
                img.draw_string_advanced(W-100, H-40, 22, f"FPS: {fps}", color=(100,255,100))
                
                # 显示图像
                Display.show_image(img)
                
                # 控制帧率(30FPS)
                frame_time = time.ticks_diff(time.ticks_ms(), frame_start)
                if frame_time < 33:
                    time.sleep_ms(33 - frame_time)
                
                # 重置错误计数(成功处理一帧)
                error_count = 0
                    
            except Exception as frame_error:
                # 单帧错误不立即退出
                error_count += 1
                print(f"\n帧{frame_count}处理错误[{error_count}/{max_errors}]: {frame_error}")
                
                if error_count >= max_errors:
                    print(f"\n连续{max_errors}帧错误,程序退出")
                    raise  # 抛出异常退出主循环
                
                time.sleep_ms(100)  # 短暂延迟后重试
                
    except KeyboardInterrupt:
        print("\n用户中断...")
    except Exception as e:
        print(f"\n!!! 发生错误 !!!")
        print(f"错误类型: {type(e).__name__}")
        print(f"错误信息: {e}")
        print(f"错误帧数: {frame_count}")
        import sys
        sys.print_exception(e)
    finally:
        print("\n清理资源...")
        pwm_lr.duty_ns(SERVO_MID_NS)
        pwm_ud.duty_ns(tilt_angle_to_ns(TILT_ANGLE_INIT))
        time.sleep(1)
        print(f"总共处理 {frame_count} 帧")
        print("系统停止")

if __name__ == "__main__":
    main()
