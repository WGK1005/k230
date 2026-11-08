'''
K230 云台YOLO目标跟踪 + RTSP推流
整合finish1.py的舵机控制和main2.py的推流功能
'''

import time
import os
import _thread
import network
import uctypes
from machine import PWM, FPIOA
from media.sensor import *
from media.media import *
from media.vencoder import *
import multimedia as mm

# ========== WiFi配置 ==========
SSID = "实验"
PASSWORD = "15235105794"

# ========== 舵机配置 ==========
W, H = 640, 480
CX, CY = W // 2, H // 2

SERVO_MIN_NS = 1000000
SERVO_MID_NS = 1500000
SERVO_MAX_NS = 2000000

TILT_ANGLE_MIN = 4500
TILT_ANGLE_MAX = 13500
TILT_ANGLE_INIT = 9000

PAN_SPEED_MIN = -10000
PAN_SPEED_MAX = 10000

PAN_COEF = 0.5
PAN_COEF_OUTSIDE = 1.5
TILT_COEF = 15
PAN_DEADZONE = 800
PAN_MIN_SPEED = 500
PAN_MIN_SPEED_OUTSIDE = 1500

SAFE_AREA_W = int(W * 3 / 5)
SAFE_AREA_H = H
SAFE_AREA_X_MIN = (W - SAFE_AREA_W) // 2
SAFE_AREA_X_MAX = SAFE_AREA_X_MIN + SAFE_AREA_W
SAFE_AREA_Y_MIN = 0
SAFE_AREA_Y_MAX = H

LOST_TARGET_DELAY = 90
MIN_PIXELS = 200

# ========== YOLO配置 ==========
YOLO_MODEL_PATH = r"D:\YOLO\best.pt"
YOLO_CONF_THRESHOLD = 0.5
YOLO_NMS_THRESHOLD = 0.4

# ========== 转换函数 ==========
def tilt_angle_to_ns(angle):
    angle = max(TILT_ANGLE_MIN, min(TILT_ANGLE_MAX, angle))
    ns = SERVO_MIN_NS + ((angle * 1000000) // 18000)
    return max(SERVO_MIN_NS, min(SERVO_MAX_NS, ns))

def pan_speed_to_ns(speed):
    speed = max(PAN_SPEED_MIN, min(PAN_SPEED_MAX, speed))
    ns = SERVO_MID_NS + (speed * 50)
    return max(SERVO_MIN_NS, min(SERVO_MAX_NS, ns))

# ========== 舵机初始化 ==========
def init_servos():
    print("初始化舵机...")
    
    pwm_io1 = FPIOA()
    pwm_io1.set_function(46, FPIOA.PWM2)
    pwm_ud = PWM(2, freq=50)
    pwm_ud.duty_ns(tilt_angle_to_ns(TILT_ANGLE_INIT))
    
    pwm_io2 = FPIOA()
    pwm_io2.set_function(47, FPIOA.PWM3)
    pwm_lr = PWM(3, freq=50)
    pwm_lr.duty_ns(SERVO_MID_NS)
    
    print("舵机初始化完成")
    time.sleep(0.5)
    
    return pwm_lr, pwm_ud

# ========== WiFi初始化 ==========
def init_wifi():
    print("[WIFI] 连接中...")
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        wlan.connect(SSID, PASSWORD)
        while not wlan.isconnected():
            print("[WIFI] 等待连接...")
            time.sleep(1)
    print("[WIFI] 连接成功, IP:", wlan.ifconfig()[0])
    return wlan

# ========== RTSP服务器 ==========
class RtspServer:
    def __init__(self, session_name="tracking", port=8554):
        self.session_name = session_name
        self.port = port
        self.rtspserver = mm.rtsp_server()
        self.venc_chn = VENC_CHN_ID_0
        self.start_stream = False
        self.runthread_over = False
        self.sensor = None
        self.encoder = None
        self.link = None
        
    def _init_stream(self):
        width = 640
        height = 480
        width = ALIGN_UP(width, 16)
        self.sensor = Sensor()
        self.sensor.reset()
        self.sensor.set_framesize(width=width, height=height, alignment=12)
        self.sensor.set_pixformat(Sensor.YUV420SP)
        self.encoder = Encoder()
        self.encoder.SetOutBufs(self.venc_chn, 8, width, height)
        self.link = MediaManager.link(self.sensor.bind_info()['src'], 
                                     (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, self.venc_chn))
        MediaManager.init()
        chnAttr = ChnAttrStr(self.encoder.PAYLOAD_TYPE_H264, 
                            self.encoder.H264_PROFILE_MAIN, width, height)
        self.encoder.Create(self.venc_chn, chnAttr)
        
    def _start_stream(self):
        self.encoder.Start(self.venc_chn)
        self.sensor.run()
        
    def _stop_stream(self):
        self.sensor.stop()
        del self.link
        self.encoder.Stop(self.venc_chn)
        self.encoder.Destroy(self.venc_chn)
        MediaManager.deinit()
        
    def _do_rtsp_stream(self):
        try:
            streamData = StreamData()
            while self.start_stream:
                os.exitpoint()
                self.encoder.GetStream(self.venc_chn, streamData)
                for pack_idx in range(0, streamData.pack_cnt):
                    stream_data = bytes(uctypes.bytearray_at(
                        streamData.data[pack_idx], streamData.data_size[pack_idx]))
                    self.rtspserver.rtspserver_sendvideodata(
                        self.session_name, stream_data, 
                        streamData.data_size[pack_idx], 1000)
                self.encoder.ReleaseStream(self.venc_chn, streamData)
        except Exception as e:
            import sys
            sys.print_exception(e)
        finally:
            self.runthread_over = True
        self.runthread_over = True
        
    def start(self):
        self._init_stream()
        self.rtspserver.rtspserver_init(self.port)
        self.rtspserver.rtspserver_createsession(
            self.session_name, mm.multi_media_type.media_h264, False)
        self.rtspserver.rtspserver_start()
        self._start_stream()
        self.start_stream = True
        _thread.start_new_thread(self._do_rtsp_stream, ())
        
    def stop(self):
        if not self.start_stream:
            return
        self.start_stream = False
        while not self.runthread_over:
            time.sleep(0.1)
        self.runthread_over = False
        self._stop_stream()
        self.rtspserver.rtspserver_stop()
        self.rtspserver.rtspserver_deinit()
        
    def get_rtsp_url(self):
        return self.rtspserver.rtspserver_getrtspurl(self.session_name)

# ========== YOLO检测器 ==========
class YoloDetector:
    def __init__(self, model_path=YOLO_MODEL_PATH):
        self.model_path = model_path
        self.model = None
        self.initialized = False
        
        try:
            # 尝试导入ultralytics YOLO
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            self.initialized = True
            print(f"[YOLO] 模型加载成功: {model_path}")
        except ImportError:
            print("[YOLO] 未安装ultralytics库")
        except Exception as e:
            print(f"[YOLO] 模型加载失败: {e}")
            
    def detect(self, img):
        """
        运行YOLO检测
        返回: [(cx, cy, w, h, conf, class_id), ...]
        cx,cy为中心坐标
        """
        if not self.initialized or self.model is None:
            return []
        
        try:
            # 运行检测
            results = self.model.predict(img, conf=YOLO_CONF_THRESHOLD, verbose=False)
            detections = []
            
            if results and len(results) > 0:
                for result in results:
                    if hasattr(result, 'boxes'):
                        for box in result.boxes:
                            # 获取框的坐标
                            x1, y1, x2, y2 = box.xyxy[0].tolist()
                            cx = (x1 + x2) / 2
                            cy = (y1 + y2) / 2
                            w = x2 - x1
                            h = y2 - y1
                            conf = float(box.conf[0])
                            cls_id = int(box.cls[0])
                            
                            detections.append({
                                'cx': cx,
                                'cy': cy,
                                'w': w,
                                'h': h,
                                'conf': conf,
                                'class_id': cls_id,
                                'area': w * h
                            })
            
            return detections
        except Exception as e:
            print(f"[YOLO] 检测错误: {e}")
            return []
            
    def get_largest_detection(self, detections):
        """获取面积最大的目标"""
        if not detections:
            return None
        return max(detections, key=lambda d: d['area'])

# ========== 主程序 ==========
def main():
    print("=" * 50)
    print("K230 云台YOLO目标跟踪 + RTSP推流")
    print("=" * 50)
    
    os.exitpoint(os.EXITPOINT_ENABLE)
    
    try:
        # 初始化WiFi
        wlan = init_wifi()
        
        # 初始化舵机
        pwm_lr, pwm_ud = init_servos()
        
        # 初始化RTSP推流
        print("\n初始化RTSP推流...")
        rtsp = RtspServer()
        rtsp.start()
        print(f"RTSP推流地址: {rtsp.get_rtsp_url()}")
        
        # 初始化YOLO检测器
        print("\n初始化YOLO检测器...")
        detector = YoloDetector()
        
        # 状态变量
        frame_count = 0
        lost_target_frames = 0
        last_frame_time = time.ticks_ms()
        fps = 0
        
        print("\n开始跟踪...\n按Ctrl+C停止\n")
        
        error_count = 0
        max_errors = 5
        
        # 创建独立的摄像头用于检测
        print("\n初始化检测摄像头...")
        detect_sensor = Sensor(width=W, height=H)
        detect_sensor.reset()
        detect_sensor.set_framesize(width=W, height=H)
        detect_sensor.set_pixformat(Sensor.RGB565)
        detect_sensor.run()
        time.sleep(1)
        
        while True:
            try:
                frame_start = time.ticks_ms()
                
                # 捕获图像用于检测
                img = detect_sensor.snapshot()
                frame_count += 1
                
                # YOLO检测
                detections = detector.detect(img)
                
                if detections:
                    # 获取最大目标
                    target = detector.get_largest_detection(detections)
                    
                    if target:
                        cx = target['cx']
                        cy = target['cy']
                        area = target['area']
                        conf = target['conf']
                        
                        # 计算误差
                        delta_x = cx - CX
                        delta_y = cy - CY
                        
                        # 检查是否在安全区内
                        in_safe_area = (SAFE_AREA_X_MIN <= cx <= SAFE_AREA_X_MAX and 
                                       SAFE_AREA_Y_MIN <= cy <= SAFE_AREA_Y_MAX)
                        
                        # 水平控制
                        if in_safe_area:
                            pan_speed = 0
                        else:
                            if abs(delta_x) < PAN_DEADZONE:
                                pan_speed = 0
                            else:
                                pan_speed_simple = (-delta_x * PAN_COEF_OUTSIDE)
                                pan_speed_simple = max(PAN_SPEED_MIN, min(PAN_SPEED_MAX, pan_speed_simple))
                                
                                if pan_speed_simple > 0:
                                    pan_speed = max(pan_speed_simple, PAN_MIN_SPEED_OUTSIDE)
                                else:
                                    pan_speed = min(pan_speed_simple, -PAN_MIN_SPEED_OUTSIDE)
                        
                        # 垂直控制
                        tilt_offset = (-delta_y * TILT_COEF)
                        tilt_offset = max(-4500, min(4500, tilt_offset))
                        
                        target_tilt_angle = TILT_ANGLE_INIT + tilt_offset
                        target_tilt_angle = max(TILT_ANGLE_MIN, min(TILT_ANGLE_MAX, target_tilt_angle))
                        
                        # 控制舵机
                        pwm_lr.duty_ns(pan_speed_to_ns(pan_speed))
                        pwm_ud.duty_ns(tilt_angle_to_ns(target_tilt_angle))
                        
                        lost_target_frames = 0
                        
                        if frame_count % 30 == 0:
                            print(f"目标检测 - 置信度:{conf:.2f}, 位置:({cx:.0f},{cy:.0f}), 面积:{area:.0f}")
                else:
                    # 未找到目标
                    lost_target_frames += 1
                    pwm_lr.duty_ns(SERVO_MID_NS)
                    
                    if lost_target_frames >= LOST_TARGET_DELAY:
                        pwm_ud.duty_ns(tilt_angle_to_ns(TILT_ANGLE_INIT))
                
                # FPS统计
                if frame_count % 30 == 0:
                    current_time = time.ticks_ms()
                    elapsed = current_time - last_frame_time
                    fps = 30000 // max(1, elapsed)
                    last_frame_time = current_time
                    print(f"FPS: {fps}, 总帧数: {frame_count}")
                
                # 帧率控制(30FPS)
                frame_time = time.ticks_diff(time.ticks_ms(), frame_start)
                if frame_time < 33:
                    time.sleep_ms(33 - frame_time)
                
                error_count = 0
                
            except KeyboardInterrupt:
                break
            except Exception as frame_error:
                error_count += 1
                print(f"帧处理错误[{error_count}/{max_errors}]: {frame_error}")
                if error_count >= max_errors:
                    raise
                time.sleep_ms(100)
        
    except KeyboardInterrupt:
        print("\n用户中断...")
    except Exception as e:
        print(f"\n发生错误: {e}")
        import sys
        sys.print_exception(e)
    finally:
        print("\n清理资源...")
        try:
            detect_sensor.stop()
        except:
            pass
        try:
            pwm_lr.duty_ns(SERVO_MID_NS)
            pwm_ud.duty_ns(tilt_angle_to_ns(TILT_ANGLE_INIT))
            time.sleep(0.5)
        except:
            pass
        try:
            rtsp.stop()
        except:
            pass
        print("系统停止")

if __name__ == "__main__":
    main()
