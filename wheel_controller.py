'''
下位机轮子控制模块 - 接收来自K230的目标坐标和云台角度
用于控制轮子朝向目标物体运动
'''

from machine import UART
import time

class WheelController:
    """轮子运动控制 - 接收上位机(K230)的控制信号"""
    
    def __init__(self, uart_id=2, baudrate=115200):
        """初始化UART接收"""
        try:
            self.uart = UART(uart_id, baudrate=baudrate, bits=8, parity=0, stop=1)
            print(f"轮子控制器已初始化 (UART{uart_id}, {baudrate}bps)")
        except Exception as e:
            print(f"UART初始化失败: {e}")
            self.uart = None
    
    def parse_command(self, data):
        """
        解析来自K230的命令
        格式: $TARGET,x,y,valid\n  (目标坐标)
        或    $SERVO,pan,tilt\n     (云台角度)
        """
        try:
            if isinstance(data, bytes):
                data = data.decode('utf-8')
            
            data = data.strip()
            if not data:
                return None
            
            if data.startswith('$TARGET,'):
                # 解析目标坐标
                parts = data.split(',')
                if len(parts) >= 4:
                    x = int(parts[1])
                    y = int(parts[2])
                    valid = bool(int(parts[3]))
                    return {'type': 'TARGET', 'x': x, 'y': y, 'valid': valid}
            
            elif data.startswith('$SERVO,'):
                # 解析云台角度
                parts = data.split(',')
                if len(parts) >= 3:
                    pan = int(parts[1])
                    tilt = int(parts[2])
                    return {'type': 'SERVO', 'pan': pan, 'tilt': tilt}
        
        except Exception as e:
            print(f"命令解析失败: {e}")
            return None
        
        return None
    
    def read_command(self, timeout_ms=100):
        """读取来自K230的命令"""
        if not self.uart:
            return None
        
        start_time = time.time()
        buffer = b''
        
        while (time.time() - start_time) < (timeout_ms / 1000.0):
            if self.uart.any():
                byte = self.uart.read(1)
                if byte:
                    buffer += byte
                    if b'\n' in buffer:
                        # 接收到完整命令
                        cmd_data = buffer.split(b'\n')[0]
                        return self.parse_command(cmd_data)
        
        return None
    
    def calculate_wheel_speed(self, target_x, target_y, screen_width, screen_height, max_speed=255):
        """
        根据目标坐标计算轮子速度
        屏幕中心为目标，偏离越多速度越大
        """
        if not target_x or not target_y:
            return 0, 0  # 目标丢失，停止
        
        cx = screen_width // 2
        cy = screen_height // 2
        
        offset_x = target_x - cx
        offset_y = target_y - cy
        
        # 简单的速度计算：距离越远，速度越大
        # 可以改进为使用云台角度作为方向指导
        left_speed = max_speed // 2
        right_speed = max_speed // 2
        
        # 根据水平偏移调整左右轮速度 (差速转向)
        if abs(offset_x) > 50:
            if offset_x > 0:
                # 目标在右边，减少右轮速度
                right_speed = max(0, max_speed // 2 - abs(offset_x) // 5)
            else:
                # 目标在左边，减少左轮速度
                left_speed = max(0, max_speed // 2 - abs(offset_x) // 5)
        
        return left_speed, right_speed
    
    def control_motors(self, left_speed, right_speed):
        """
        控制电机速度
        这里应该连接到实际的电机驱动电路
        """
        print(f"电机控制: 左={left_speed}, 右={right_speed}")
        # TODO: 实现实际的电机PWM控制


# ==================== 主循环示例 ====================
def main():
    """下位机轮子控制主程序"""
    controller = WheelController(uart_id=2, baudrate=115200)
    
    while True:
        # 尝试读取来自K230的命令
        cmd = controller.read_command(timeout_ms=50)
        
        if cmd:
            if cmd['type'] == 'TARGET':
                # 收到目标坐标
                if cmd['valid']:
                    # 目标有效，计算轮子速度
                    left_spd, right_spd = controller.calculate_wheel_speed(
                        cmd['x'], cmd['y'], 480, 800
                    )
                    controller.control_motors(left_spd, right_spd)
                    print(f"跟踪目标: ({cmd['x']}, {cmd['y']})")
                else:
                    # 目标丢失
                    controller.control_motors(0, 0)
                    print("目标丢失，停止运动")
            
            elif cmd['type'] == 'SERVO':
                # 收到云台角度 (可用于精确方向控制)
                print(f"云台位置: Pan={cmd['pan']}deg, Tilt={cmd['tilt']}deg")
        
        time.sleep_ms(100)


if __name__ == '__main__':
    main()
