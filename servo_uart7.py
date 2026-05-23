#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
香橙派4Pro + ZP25S 舵机控制
使用 UART7 (/dev/ttyS7) @ 115200
众灵科技 P-Bus 协议
"""

import serial
import time
import sys

class ZP25SServo:
    """ZP25S舵机控制器 - UART7版本"""
    
    def __init__(self, port='/dev/ttyS7', baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        
    def open(self):
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
            print(f"✓ 已连接到 {self.port} @ {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"✗ 连接失败: {e}")
            return False
    
    def close(self):
        """关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("✓ 串口已关闭")
    
    def calculate_checksum(self, data):
        """计算异或校验和"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum
    
    def send_command(self, servo_id, angle, time_ms=1000, show_response=True):
        """
        发送舵机控制命令
        
        Args:
            servo_id: 舵机ID (1-254)
            angle: 目标角度 (0-240°)
            time_ms: 运动时间 (毫秒)
            show_response: 是否显示响应
        """
        if not self.ser or not self.ser.is_open:
            print("✗ 串口未打开")
            return False
        
        try:
            # 限制参数范围
            angle = max(0, min(240, angle))
            time_ms = max(0, min(32767, time_ms))
            
            # 构建命令
            cmd = 0x03  # 移动到指定角度命令
            
            # 角度转换 (0-240° -> 0-1023)
            angle_pos = int(angle * 1024 / 240)
            angle_h = (angle_pos >> 8) & 0xFF
            angle_l = angle_pos & 0xFF
            
            # 时间数据
            time_h = (time_ms >> 8) & 0xFF
            time_l = time_ms & 0xFF
            
            # 数据部分
            data = [servo_id, 0x05, cmd, angle_h, angle_l, time_h, time_l]
            
            # 计算校验和
            checksum = self.calculate_checksum(data)
            
            # 构建完整帧
            frame = [0xFF, 0xFF] + data + [checksum, 0xFE]
            
            # 发送
            self.ser.write(bytes(frame))
            
            # 显示信息
            hex_str = ' '.join([f'{b:02X}' for b in frame])
            print(f"[发送] ID:{servo_id} 角度:{angle}° 时间:{time_ms}ms")
            print(f"       {hex_str}")
            
            # 读取响应
            time.sleep(0.1)
            if self.ser.in_waiting > 0 and show_response:
                response = self.ser.read(self.ser.in_waiting)
                hex_response = ' '.join([f'{b:02X}' for b in response])
                print(f"[响应] {hex_response}")
            
            return True
            
        except Exception as e:
            print(f"✗ 命令发送失败: {e}")
            return False
    
    def set_angle(self, servo_id, angle, time_ms=1000):
        """设置舵机角度"""
        return self.send_command(servo_id, angle, time_ms)
    
    def batch_set(self, angles_dict, time_ms=1000):
        """
        批量设置多个舵机
        
        Args:
            angles_dict: {舵机ID: 角度, ...}
            time_ms: 运动时间
        """
        for servo_id, angle in angles_dict.items():
            self.set_angle(servo_id, angle, time_ms)
            time.sleep(0.1)


def test_basic():
    """基础测试"""
    servo = ZP25SServo(port='/dev/ttyS7', baudrate=115200)
    
    if not servo.open():
        return False
    
    try:
        print("\n" + "="*60)
        print("ZP25S 舵机控制测试 - UART7")
        print("="*60)
        
        # 测试1: 单个舵机
        print("\n[测试1] 单个舵机测试")
        print("将舵机1设置到120°(中心位置)...")
        servo.set_angle(1, 120, 1000)
        time.sleep(2)
        
        # 测试2: 移动到不同角度
        print("\n[测试2] 多角度移动")
        for angle in [60, 120, 180, 120]:
            print(f"移动到 {angle}°...")
            servo.set_angle(1, angle, 1500)
            time.sleep(2)
        
        # 测试3: 多舵机控制
        print("\n[测试3] 多舵机控制（如果有）")
        angles = {1: 120, 2: 120, 3: 120}
        servo.batch_set(angles, 1000)
        time.sleep(2)
        
        print("\n✓ 测试完成")
        return True
        
    except KeyboardInterrupt:
        print("\n[中断] 用户停止")
    except Exception as e:
        print(f"\n✗ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        servo.close()


def interactive():
    """交互式控制"""
    servo = ZP25SServo(port='/dev/ttyS7', baudrate=115200)
    
    if not servo.open():
        return
    
    try:
        print("\n" + "="*60)
        print("交互式舵机控制 - UART7")
        print("="*60)
        print("\n命令格式:")
        print("  set <ID> <角度> [时间]  - 设置舵机角度")
        print("  test <ID>               - 快速测试舵机")
        print("  quit                    - 退出")
        print()
        
        while True:
            cmd = input("\n> ").strip().split()
            
            if not cmd:
                continue
            
            if cmd[0].lower() == 'quit':
                break
            
            elif cmd[0].lower() == 'set':
                if len(cmd) < 3:
                    print("用法: set <ID> <角度> [时间]")
                    continue
                
                try:
                    servo_id = int(cmd[1])
                    angle = int(cmd[2])
                    time_ms = int(cmd[3]) if len(cmd) > 3 else 1000
                    servo.set_angle(servo_id, angle, time_ms)
                except:
                    print("参数错误")
            
            elif cmd[0].lower() == 'test':
                if len(cmd) < 2:
                    print("用法: test <ID>")
                    continue
                
                try:
                    servo_id = int(cmd[1])
                    print(f"\n测试舵机 {servo_id}...")
                    servo.set_angle(servo_id, 60, 1000)
                    time.sleep(1.5)
                    servo.set_angle(servo_id, 180, 1000)
                    time.sleep(1.5)
                    servo.set_angle(servo_id, 120, 1000)
                except:
                    print("参数错误")
            
            else:
                print("未知命令")
        
    except KeyboardInterrupt:
        print("\n[中断]")
    finally:
        servo.close()


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'interactive':
        interactive()
    else:
        test_basic()
