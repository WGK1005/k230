#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
香橙派4Pro 控制 ZP25S 总线舵机
众灵科技总线舵机指令协议控制
基于 ASCII 字符串命令格式: #IDPXXXTYYYY!
"""

import serial
import time
import os
import sys
import glob

class ZP25SController:
    """ZP25S总线舵机控制器 - 基于ASCII指令协议"""
    
    def __init__(self, port='/dev/ttyS0', baudrate=115200, timeout=1, debug=True):
        """
        初始化舵机控制器
        
        Args:
            port: 串口端口号
            baudrate: 波特率
            timeout: 超时时间
            debug: 是否打印调试信息
        """
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.debug = debug
        
    def open_serial(self):
        """打开串口"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            print(f"串口 {self.port} 打开成功，波特率: {self.baudrate}")
            return True
        except Exception as e:
            print(f"串口打开失败: {e}")
            return False
    
    def close_serial(self):
        """关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("串口已关闭")
    
    def send_command(self, servo_id, command_str, read_response=False):
        """
        发送原始命令字符串
        
        Args:
            servo_id: 舵机ID (0-255, 0通常为广播ID)
            command_str: 命令字符串部分，如 "P1500T1000" 或 "PID"
            read_response: 是否读取响应
            
        Returns:
            响应字符串或 True/False
        """
        try:
            # 构建完整命令: #IDPCOMMAND!
            full_cmd = f"#{servo_id:03d}{command_str}!"
            
            if self.ser and self.ser.is_open:
                if self.debug:
                    print(f"[发送] {full_cmd}")
                
                self.ser.write(full_cmd.encode('ascii'))
                
                if read_response:
                    time.sleep(0.1)
                    if self.ser.in_waiting > 0:
                        response = self.ser.read(self.ser.in_waiting)
                        try:
                            resp_str = response.decode('ascii').strip()
                            if self.debug:
                                print(f"[响应] {resp_str}")
                            return resp_str
                        except:
                            if self.debug:
                                print(f"[响应] {response}")
                            return response
                else:
                    time.sleep(0.05)
                    return True
            else:
                print("串口未打开")
                return False
                
        except Exception as e:
            print(f"发送命令失败: {e}")
            return False
    
    def set_servo_angle(self, servo_id, angle, time_ms=1000):
        """
        设置舵机到指定角度
        命令格式: #IDPXXXTYYYY!
        例如: #000P1500T1000!
        
        Args:
            servo_id: 舵机ID
            angle: 目标角度 (0-240°)
            time_ms: 运动时间 (毫秒)
        """
        # 确保角度在有效范围内 (0-240)
        angle = max(0, min(240, angle))
        # 确保时间在有效范围内 (1-9999ms)
        time_ms = max(1, min(9999, time_ms))
        
        # 角度转换为 PWM 值 (500-2500)
        pwm_val = int(500 + (angle / 240.0) * 2000)
        
        command = f"P{pwm_val:04d}T{time_ms:04d}"
        if self.debug:
            print(f"[设置] ID:{servo_id:03d} 角度:{angle}° PWM:{pwm_val} 时间:{time_ms}ms")
        
        return self.send_command(servo_id, command)
    
    def stop_servo(self, servo_id):
        """
        停止舵机运动
        命令格式: #IDDST!
        """
        if self.debug:
            print(f"[停止] 舵机 {servo_id:03d}")
        return self.send_command(servo_id, "DST")
    
    def pause_servo(self, servo_id):
        """
        暂停舵机运动
        命令格式: #IDDPT!
        """
        if self.debug:
            print(f"[暂停] 舵机 {servo_id:03d}")
        return self.send_command(servo_id, "DPT")
    
    def continue_servo(self, servo_id):
        """
        继续舵机运动
        命令格式: #IDDCT!
        """
        if self.debug:
            print(f"[继续] 舵机 {servo_id:03d}")
        return self.send_command(servo_id, "DCT")
    
    def get_servo_id(self, servo_id):
        """
        读取舵机ID
        命令格式: #IDPID!
        """
        if self.debug:
            print(f"[查询] 舵机ID {servo_id:03d}")
        return self.send_command(servo_id, "PID", read_response=True)
    
    def read_servo_angle(self, servo_id):
        """
        读取舵机当前角度
        命令格式: #IDRAD!
        """
        if self.debug:
            print(f"[读角度] 舵机 {servo_id:03d}")
        return self.send_command(servo_id, "RAD", read_response=True)
    
    def set_servo_id(self, old_id, new_id):
        """
        设置舵机ID
        命令格式: #IDPID XXX!
        """
        command = f"PID{new_id:03d}"
        if self.debug:
            print(f"[设置ID] 舵机 {old_id:03d} -> {new_id:03d}")
        return self.send_command(old_id, command)
    
    def release_servo(self, servo_id):
        """
        释放舵机扭力（断电）
        命令格式: #IDULK!
        """
        if self.debug:
            print(f"[释力] 舵机 {servo_id:03d}")
        return self.send_command(servo_id, "ULK")
    
    def recover_servo(self, servo_id):
        """
        恢复舵机扭力
        命令格式: #IDULR!
        """
        if self.debug:
            print(f"[恢复力] 舵机 {servo_id:03d}")
        return self.send_command(servo_id, "ULR")
    
    def batch_set_angles(self, angles_dict, time_ms=1000):
        """
        批量设置多个舵机的角度
        
        Args:
            angles_dict: 字典 {舵机ID: 角度}
            time_ms: 运动时间 (毫秒)
        """
        for servo_id, angle in angles_dict.items():
            self.set_servo_angle(servo_id, angle, time_ms)
            time.sleep(0.05)
    
    def scan_servos(self, id_range=range(0, 10)):
        """
        扫描舵机ID
        
        Args:
            id_range: 要扫描的ID范围
            
        Returns:
            找到的舵机ID列表
        """
        print("\n[扫描] 正在扫描舵机ID...")
        found_ids = []
        
        for servo_id in id_range:
            try:
                # 先清空缓冲区
                self.ser.reset_input_buffer()
                time.sleep(0.02)
                
                # 发送读取ID命令: #IDPID!
                response = self.get_servo_id(servo_id)
                
                if response:
                    found_ids.append(servo_id)
                    print(f"  ✓ 找到舵机 ID: {servo_id:03d}")
                
            except Exception as e:
                pass
        
        if found_ids:
            print(f"[完成] 共找到 {len(found_ids)} 个舵机: {found_ids}\n")
        else:
            print("[警告] 未找到任何舵机\n")
        
        return found_ids


def find_available_port():
    """自动检测可用的串口"""
    possible_ports = (
        glob.glob('/dev/ttyS*') +
        glob.glob('/dev/ttyUSB*') +
        glob.glob('/dev/ttyAMA*') +
        glob.glob('/dev/ttyMTK*')
    )
    possible_ports = sorted(set(possible_ports))
    return possible_ports


def main(port=None):
    """主函数 - 控制舵机
    
    Args:
        port: 串口设备路径，如 /dev/ttyS0, /dev/ttyUSB0 等
    """
    
    # 如果没有指定端口，尝试自动检测
    if port is None:
        possible_ports = find_available_port()
        
        if possible_ports:
            print(f"检测到可用的串口设备: {possible_ports}")
            port = possible_ports[0]
            print(f"使用第一个设备: {port}\n")
        else:
            print("错误: 未找到任何可用的串口设备!")
            print("请先运行以下命令来诊断可用的串口:")
            print("  python3 check_uart.py")
            print("\n常见的串口设备名称:")
            print("  /dev/ttyS0, /dev/ttyS1  - 标准 UART")
            print("  /dev/ttyUSB0, /dev/ttyUSB1  - USB 转串口")
            print("  /dev/ttyAMA0, /dev/ttyAMA1  - GPIO UART (树莓派/OrangePi)")
            return
    
    # 创建控制器实例
    controller = ZP25SController(port=port, baudrate=115200, timeout=1, debug=True)
    
    if not controller.open_serial():
        print("\n故障排除:")
        print("  1. 检查设备连接")
        print("  2. 验证波特率设置 (当前: 115200)")
        print("  3. 确认串口权限: sudo usermod -a -G dialout $USER")
        print("  4. 运行 check_uart.py 来列出所有可用串口")
        return
    
    try:
        print("\n" + "="*50)
        print("舵机控制系统启动")
        print("="*50 + "\n")
        
        # 扫描舵机
        print("阶段1: 扫描舵机ID (0-15)...")
        found_ids = controller.scan_servos(id_range=range(0, 16))
        
        if not found_ids:
            print("\n警告：未找到舵机！")
            print("可能原因：")
            print("  1. 舵机未上电")
            print("  2. 波特率不匹配 (尝试: 9600, 19200, 38400, 57600)")
            print("  3. 舵机连接不正常")
            print("  4. 串口端口错误")
            return
        
        print(f"找到舵机: {found_ids}\n")
        
        # 选择要控制的舵机
        servo_to_control = found_ids[0]
        print(f"阶段2: 控制舵机 {servo_to_control:03d}")
        print("-" * 50)
        
        # 测试命令序列
        commands = [
            (servo_to_control, 0, 1000, "0° 位置 (最小)"),
            (servo_to_control, 120, 1000, "120° 位置 (中点)"),
            (servo_to_control, 240, 1000, "240° 位置 (最大)"),
            (servo_to_control, 120, 1000, "回到中点"),
        ]
        
        for servo_id, angle, time_ms, description in commands:
            print(f"\n{description}")
            controller.set_servo_angle(servo_id, angle, time_ms)
            time.sleep(time_ms / 1000.0 + 0.3)
        
        # 读取最终角度
        print("\n" + "-" * 50)
        print("阶段3: 读取舵机状态")
        angle = controller.read_servo_angle(servo_to_control)
        print(f"舵机 {servo_to_control:03d} 当前角度: {angle}")
        
        print("\n" + "="*50)
        print("完成")
        print("="*50 + "\n")
        
    except KeyboardInterrupt:
        print("\n\n中断")
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        controller.close_serial()


if __name__ == '__main__':
    import sys
    
    # 检查命令行参数
    port = None
    if len(sys.argv) > 1:
        port = sys.argv[1]
        print(f"使用指定的串口: {port}")
    
    main(port=port)
