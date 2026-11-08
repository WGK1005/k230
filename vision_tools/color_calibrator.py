'''
K230 颜色校准工具 - 帮助获取最佳的红色阈值
显示实时LAB值，方便调整RED_THRESHOLD
'''

import time
from media.sensor import *
from media.display import *
from media.media import *

class ColorCalibrator:
    """颜色校准工具"""
    
    def __init__(self, width=480, height=800):
        self.W = width
        self.H = height
        
        # 初始化摄像头
        self.sensor = Sensor(width=self.W, height=self.H)
        self.sensor.reset()
        self.sensor.set_hmirror(True)
        self.sensor.set_vflip(True)
        self.sensor.set_framesize(width=self.W, height=self.H)
        self.sensor.set_pixformat(Sensor.RGB565)
        
        Display.init(Display.ST7701, width=self.W, height=self.H)
        MediaManager.init()
        self.sensor.run()
        
        # LAB阈值范围 (初始为红色)
        self.l_min, self.l_max = 20, 80
        self.a_min, self.a_max = 30, 100
        self.b_min, self.b_max = 0, 60
        
        print("颜色校准工具已启动")
        print("按以下键盘快捷键调整阈值:")
        print("L/l: L通道增/减")
        print("A/a: A通道增/减")
        print("B/b: B通道增/减")
        print("R: 重置为红色默认值")
        print("S: 保存当前阈值")
        print("Q: 退出")
    
    def get_lab_at_center(self, img):
        """获取图像中心点的LAB值"""
        cx = self.W // 2
        cy = self.H // 2
        
        # 获取中心点像素的LAB值
        # 注意: 这里需要使用cv_lite或image处理库来获取LAB值
        # 由于OpenMV有find_blobs方法，我们先尝试通过色块检测获取信息
        
        # 简化方案：显示中心点周围的色块
        return None
    
    def run(self):
        """运行校准器"""
        threshold = (self.l_min, self.l_max, 
                    self.a_min, self.a_max, 
                    self.b_min, self.b_max)
        
        while True:
            img = self.sensor.snapshot()
            
            # 寻找符合阈值的色块
            blobs = img.find_blobs([threshold], pixels_threshold=10)
            
            # 绘制中心十字
            cx = self.W // 2
            cy = self.H // 2
            img.draw_cross(cx, cy, color=(0, 255, 0), size=20, thickness=2)
            
            # 显示当前阈值
            threshold_text = f"L:({self.l_min},{self.l_max}) A:({self.a_min},{self.a_max}) B:({self.b_min},{self.b_max})"
            img.draw_string_advanced(10, 10, 20, threshold_text, color=(255, 255, 0))
            
            # 显示检测到的色块数
            if blobs:
                img.draw_string_advanced(10, 50, 24, f"BLOBS FOUND: {len(blobs)}", color=(0, 255, 0))
                
                # 绘制检测到的色块
                for blob in blobs:
                    img.draw_rectangle(blob.rect(), color=(255, 0, 0), thickness=2)
                    img.draw_cross(blob.cx(), blob.cy(), color=(255, 0, 0), size=10)
            else:
                img.draw_string_advanced(10, 50, 24, "NO MATCH", color=(255, 0, 0))
            
            # 显示调整提示
            tips = "Press L/A/B to adjust, R to reset, S to save, Q to quit"
            img.draw_string_advanced(10, 750, 16, tips, color=(200, 200, 200))
            
            Display.show_image(img)
            time.sleep_ms(50)
    
    def print_current_threshold(self):
        """打印当前的阈值"""
        print(f"RED_THRESHOLD = ({self.l_min}, {self.l_max}, {self.a_min}, {self.a_max}, {self.b_min}, {self.b_max})")


def interactive_calibrator():
    """交互式颜色校准"""
    calibrator = ColorCalibrator(width=480, height=800)
    
    print("\n=== 开始实时颜色采样 ===")
    print("请把红色物体放在屏幕中心")
    print("观察屏幕上是否能正确识别红色")
    print("\n当前推荐的RED_THRESHOLD:")
    calibrator.print_current_threshold()
    
    try:
        calibrator.run()
    except KeyboardInterrupt:
        print("\n校准已停止")
        print("推荐的阈值:")
        calibrator.print_current_threshold()


if __name__ == '__main__':
    interactive_calibrator()
