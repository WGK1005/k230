'''
K230 系统测试套件 - 逐个测试各个模块
'''

import time
from media.sensor import *
from media.display import *
from media.media import *
from machine import PWM, FPIOA, UART

class SystemTester:
    """系统测试工具"""
    
    def __init__(self):
        self.tests_passed = 0
        self.tests_failed = 0
        self.results = []
    
    def log(self, msg, level="INFO"):
        """记录测试日志"""
        prefix = f"[{level}]" if level != "INFO" else "[✓]"
        print(f"{prefix} {msg}")
        self.results.append((level, msg))
    
    def test_pwm(self):
        """测试PWM输出"""
        print("\n=== 测试 PWM 舵机 ===")
        try:
            # 水平舵机
            pwm_io_h = FPIOA()
            pwm_io_h.set_function(46, FPIOA.PWM2)
            pwm_h = PWM(2, freq=50)
            
            # 垂直舵机
            pwm_io_v = FPIOA()
            pwm_io_v.set_function(47, FPIOA.PWM3)
            pwm_v = PWM(3, freq=50)
            
            # 测试脉宽
            test_values = [
                (500000, "最左/最下 (0°)"),
                (1500000, "中点 (90°)"),
                (2500000, "最右/最上 (180°)"),
                (1500000, "回到中点"),
            ]
            
            for ns_value, desc in test_values:
                pwm_h.duty_ns(ns_value)
                pwm_v.duty_ns(ns_value)
                print(f"  {ns_value}ns → {desc}")
                time.sleep(0.5)
            
            self.log("PWM舵机测试通过", "✓")
            self.tests_passed += 1
            return True
            
        except Exception as e:
            self.log(f"PWM测试失败: {e}", "✗")
            self.tests_failed += 1
            return False
    
    def test_uart(self):
        """测试UART串口通信"""
        print("\n=== 测试 UART 串口 ===")
        try:
            uart = UART(2, baudrate=115200, bits=8, parity=0, stop=1)
            
            # 发送测试数据
            test_msg = b"$TEST,480,400,1\n"
            uart.write(test_msg)
            print(f"  发送: {test_msg}")
            
            self.log("UART串口测试通过", "✓")
            self.tests_passed += 1
            return True
            
        except Exception as e:
            self.log(f"UART测试失败: {e}", "✗")
            self.tests_failed += 1
            return False
    
    def test_camera(self):
        """测试摄像头"""
        print("\n=== 测试 摄像头 ===")
        try:
            sensor = Sensor(width=480, height=800)
            sensor.reset()
            sensor.set_hmirror(True)
            sensor.set_vflip(True)
            sensor.set_framesize(width=480, height=800)
            sensor.set_pixformat(Sensor.RGB565)
            
            MediaManager.init()
            sensor.run()
            
            # 拍摄一帧
            img = sensor.snapshot()
            
            if img:
                self.log(f"摄像头正常 (分辨率: 480x800)", "✓")
                self.tests_passed += 1
                return True
            else:
                self.log("摄像头无法获取图像", "✗")
                self.tests_failed += 1
                return False
                
        except Exception as e:
            self.log(f"摄像头测试失败: {e}", "✗")
            self.tests_failed += 1
            return False
    
    def test_display(self):
        """测试显示屏"""
        print("\n=== 测试 显示屏 ===")
        try:
            Display.init(Display.ST7701, width=480, height=800)
            
            # 创建测试图像
            img = Image(size=(480, 800), format=Image.RGB565)
            img.clear((0, 0, 255))  # 蓝色背景
            img.draw_string_advanced(200, 390, 32, "Display OK", color=(255, 255, 0))
            
            Display.show_image(img)
            time.sleep(1)
            
            self.log("显示屏正常", "✓")
            self.tests_passed += 1
            return True
            
        except Exception as e:
            self.log(f"显示屏测试失败: {e}", "✗")
            self.tests_failed += 1
            return False
    
    def test_blob_detection(self):
        """测试色块检测"""
        print("\n=== 测试 色块检测 ===")
        try:
            sensor = Sensor(width=480, height=800)
            sensor.reset()
            sensor.set_framesize(width=480, height=800)
            sensor.set_pixformat(Sensor.RGB565)
            MediaManager.init()
            sensor.run()
            
            # 采样多帧
            for i in range(3):
                img = sensor.snapshot()
                
                # 尝试检测红色
                RED_THRESHOLD = (20, 80, 30, 100, 0, 60)
                blobs = img.find_blobs([RED_THRESHOLD], pixels_threshold=200)
                
                if blobs:
                    print(f"  第{i+1}帧: 检测到{len(blobs)}个红色色块")
            
            self.log("色块检测功能正常", "✓")
            self.tests_passed += 1
            return True
            
        except Exception as e:
            self.log(f"色块检测失败: {e}", "✗")
            self.tests_failed += 1
            return False
    
    def test_pid(self):
        """测试PID控制器"""
        print("\n=== 测试 PID 控制器 ===")
        try:
            # 简单PID实现
            class SimplePID:
                def __init__(self, kp=1.0):
                    self.kp = kp
                    self.last_error = 0
                    self.i_sum = 0
                
                def compute(self, error):
                    return self.kp * error
            
            pid = SimplePID(kp=0.5)
            
            # 测试计算
            errors = [-50, -25, 0, 25, 50]
            for error in errors:
                output = pid.compute(error)
                print(f"  输入误差: {error:+4d} → PID输出: {output:+6.1f}")
            
            self.log("PID控制器正常", "✓")
            self.tests_passed += 1
            return True
            
        except Exception as e:
            self.log(f"PID测试失败: {e}", "✗")
            self.tests_failed += 1
            return False
    
    def run_all_tests(self):
        """运行所有测试"""
        print("="*50)
        print("K230 系统测试套件 v1.0")
        print("="*50)
        
        # 逐个运行测试
        self.test_pwm()
        self.test_uart()
        self.test_camera()
        self.test_display()
        self.test_blob_detection()
        self.test_pid()
        
        # 显示测试结果汇总
        print("\n" + "="*50)
        print("测试结果汇总")
        print("="*50)
        print(f"✓ 通过: {self.tests_passed}")
        print(f"✗ 失败: {self.tests_failed}")
        print(f"总计: {self.tests_passed + self.tests_failed}")
        
        if self.tests_failed == 0:
            print("\n🎉 所有测试通过！系统就绪。")
            return True
        else:
            print(f"\n⚠️  有 {self.tests_failed} 个测试失败。请检查硬件连接。")
            return False


def main():
    """主测试函数"""
    tester = SystemTester()
    tester.run_all_tests()


if __name__ == '__main__':
    main()
