'''
K230 参数实时调试工具
在REPL中交互式调整PID参数、颜色阈值等
'''

class ParameterDebugger:
    """实时参数调试工具"""
    
    def __init__(self):
        self.current_config = {
            'red_threshold': (20, 80, 30, 100, 0, 60),
            'pid_h_kp': 0.5,
            'pid_h_ki': 0.05,
            'pid_h_kd': 0.2,
            'pid_v_kp': 0.5,
            'pid_v_ki': 0.05,
            'pid_v_kd': 0.2,
            'dead_zone_x': 30,
            'dead_zone_y': 30,
        }
    
    def show_menu(self):
        """显示菜单"""
        print("\n" + "="*50)
        print("K230 参数调试工具")
        print("="*50)
        print("1. 调整红色阈值 (RED_THRESHOLD)")
        print("2. 调整PID参数 (水平)")
        print("3. 调整PID参数 (垂直)")
        print("4. 调整死区")
        print("5. 显示当前参数")
        print("6. 保存参数到文件")
        print("7. 退出")
        print("="*50)
    
    def adjust_red_threshold(self):
        """调整红色阈值"""
        print("\n调整红色阈值 (LAB颜色空间)")
        print(f"当前: {self.current_config['red_threshold']}")
        print("格式: (L_MIN, L_MAX, A_MIN, A_MAX, B_MIN, B_MAX)")
        
        try:
            l_min = int(input("L_MIN (0-100) [20]: ") or "20")
            l_max = int(input("L_MAX (0-100) [80]: ") or "80")
            a_min = int(input("A_MIN (-128-127) [30]: ") or "30")
            a_max = int(input("A_MAX (-128-127) [100]: ") or "100")
            b_min = int(input("B_MIN (-128-127) [0]: ") or "0")
            b_max = int(input("B_MAX (-128-127) [60]: ") or "60")
            
            self.current_config['red_threshold'] = (l_min, l_max, a_min, a_max, b_min, b_max)
            print(f"✓ 已更新: {self.current_config['red_threshold']}")
            
            # 应用到运行中的程序
            print("\n使用以下代码应用到change1.py:")
            print(f"import change1")
            print(f"change1.RED_THRESHOLD = {self.current_config['red_threshold']}")
            
        except ValueError:
            print("✗ 输入错误，请输入数字")
    
    def adjust_pid_h(self):
        """调整水平PID参数"""
        print("\n调整水平舵机 PID 参数")
        print(f"当前: Kp={self.current_config['pid_h_kp']}, Ki={self.current_config['pid_h_ki']}, Kd={self.current_config['pid_h_kd']}")
        
        try:
            kp = float(input("Kp (0.1-2.0) [0.5]: ") or "0.5")
            ki = float(input("Ki (0.0-0.5) [0.05]: ") or "0.05")
            kd = float(input("Kd (0.0-1.0) [0.2]: ") or "0.2")
            
            self.current_config['pid_h_kp'] = kp
            self.current_config['pid_h_ki'] = ki
            self.current_config['pid_h_kd'] = kd
            print(f"✓ 已更新: Kp={kp}, Ki={ki}, Kd={kd}")
            
            print("\n使用以下代码应用:")
            print(f"import change1")
            print(f"change1.pid_h.kp = {kp}")
            print(f"change1.pid_h.ki = {ki}")
            print(f"change1.pid_h.kd = {kd}")
            
        except ValueError:
            print("✗ 输入错误")
    
    def adjust_pid_v(self):
        """调整垂直PID参数"""
        print("\n调整垂直舵机 PID 参数")
        print(f"当前: Kp={self.current_config['pid_v_kp']}, Ki={self.current_config['pid_v_ki']}, Kd={self.current_config['pid_v_kd']}")
        
        try:
            kp = float(input("Kp (0.1-2.0) [0.5]: ") or "0.5")
            ki = float(input("Ki (0.0-0.5) [0.05]: ") or "0.05")
            kd = float(input("Kd (0.0-1.0) [0.2]: ") or "0.2")
            
            self.current_config['pid_v_kp'] = kp
            self.current_config['pid_v_ki'] = ki
            self.current_config['pid_v_kd'] = kd
            print(f"✓ 已更新: Kp={kp}, Ki={ki}, Kd={kd}")
            
            print("\n使用以下代码应用:")
            print(f"import change1")
            print(f"change1.pid_v.kp = {kp}")
            print(f"change1.pid_v.ki = {ki}")
            print(f"change1.pid_v.kd = {kd}")
            
        except ValueError:
            print("✗ 输入错误")
    
    def adjust_dead_zone(self):
        """调整死区"""
        print("\n调整死区 (像素)")
        print(f"当前: X={self.current_config['dead_zone_x']}, Y={self.current_config['dead_zone_y']}")
        
        try:
            dz_x = int(input("死区X (0-100) [30]: ") or "30")
            dz_y = int(input("死区Y (0-100) [30]: ") or "30")
            
            self.current_config['dead_zone_x'] = dz_x
            self.current_config['dead_zone_y'] = dz_y
            print(f"✓ 已更新: X={dz_x}, Y={dz_y}")
            
            print("\n使用以下代码应用:")
            print(f"import change1")
            print(f"change1.DEAD_ZONE_X = {dz_x}")
            print(f"change1.DEAD_ZONE_Y = {dz_y}")
            
        except ValueError:
            print("✗ 输入错误")
    
    def show_current(self):
        """显示当前配置"""
        print("\n当前参数配置:")
        print("-" * 50)
        print(f"红色阈值: {self.current_config['red_threshold']}")
        print(f"水平PID: Kp={self.current_config['pid_h_kp']}, Ki={self.current_config['pid_h_ki']}, Kd={self.current_config['pid_h_kd']}")
        print(f"垂直PID: Kp={self.current_config['pid_v_kp']}, Ki={self.current_config['pid_v_ki']}, Kd={self.current_config['pid_v_kd']}")
        print(f"死区: X={self.current_config['dead_zone_x']}, Y={self.current_config['dead_zone_y']}")
        print("-" * 50)
    
    def save_to_file(self):
        """保存参数到文件"""
        try:
            with open('debug_config.py', 'w') as f:
                f.write("# K230 调试参数配置\n\n")
                f.write(f"RED_THRESHOLD = {self.current_config['red_threshold']}\n")
                f.write(f"PID_H = {{'kp': {self.current_config['pid_h_kp']}, 'ki': {self.current_config['pid_h_ki']}, 'kd': {self.current_config['pid_h_kd']}}}\n")
                f.write(f"PID_V = {{'kp': {self.current_config['pid_v_kp']}, 'ki': {self.current_config['pid_v_ki']}, 'kd': {self.current_config['pid_v_kd']}}}\n")
                f.write(f"DEAD_ZONE_X = {self.current_config['dead_zone_x']}\n")
                f.write(f"DEAD_ZONE_Y = {self.current_config['dead_zone_y']}\n")
            
            print("✓ 已保存到 debug_config.py")
            
        except Exception as e:
            print(f"✗ 保存失败: {e}")
    
    def run(self):
        """运行调试工具"""
        while True:
            self.show_menu()
            choice = input("请选择 (1-7): ")
            
            if choice == '1':
                self.adjust_red_threshold()
            elif choice == '2':
                self.adjust_pid_h()
            elif choice == '3':
                self.adjust_pid_v()
            elif choice == '4':
                self.adjust_dead_zone()
            elif choice == '5':
                self.show_current()
            elif choice == '6':
                self.save_to_file()
            elif choice == '7':
                print("退出")
                break
            else:
                print("✗ 选择无效")


# 快速调试命令
def quick_test_servo():
    """快速舵机测试"""
    print("舵机快速测试")
    try:
        from change1 import servo
        import time
        
        positions = [
            (0, 0, "中点"),
            (-45, 0, "左转45°"),
            (45, 0, "右转45°"),
            (0, 45, "上转45°"),
            (0, -45, "下转45°"),
            (0, 0, "回到中点"),
        ]
        
        for pan, tilt, desc in positions:
            servo.set_position(pan, tilt)
            print(f"  {desc} → Pan={pan}°, Tilt={tilt}°")
            time.sleep(0.5)
        
        print("✓ 舵机测试完成")
    except Exception as e:
        print(f"✗ 测试失败: {e}")


def quick_test_uart():
    """快速UART测试"""
    print("UART通信快速测试")
    try:
        from change1 import wheel_comm
        
        wheel_comm.send_target_position(240, 400, valid=True)
        print("✓ 已发送: $TARGET,240,400,1")
        
        wheel_comm.send_pan_tilt(30, -20)
        print("✓ 已发送: $SERVO,30,-20")
        
    except Exception as e:
        print(f"✗ 测试失败: {e}")


# 主程序
if __name__ == '__main__':
    debugger = ParameterDebugger()
    debugger.run()
