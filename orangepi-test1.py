#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
香橙派4Pro 控制 ZP25S 总线舵机
众灵科技 ZP25S 舵机 P-Bus 协议控制
支持微雪总线驱动板
"""

import serial
import time
import struct
import os
import stat
import subprocess

class ZP25SController:
    """ZP25S总线舵机控制器"""
    
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
            print(f"串口 {self.port} 打开成功")
            return True
        except Exception as e:
            print(f"串口打开失败: {e}")
            return False
    
    def close_serial(self):
        """关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("串口已关闭")
    
    def calculate_checksum(self, data):
        """
        计算校验和 (异或校验)
        
        Args:
            data: 数据字节列表
            
        Returns:
            校验和值
        """
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum
    
    def send_command(self, servo_id, angle, time_ms=1000):
        """
        发送舵机控制命令
        
        Args:
            servo_id: 舵机ID (1-254, 通常从1开始编号)
            angle: 目标角度 (0-240 度，中点通常是120°)
            time_ms: 到达目标角度的时间(毫秒)
        """
        try:
            # 确保角度在有效范围内 (0-240)
            angle = max(0, min(240, angle))
            # 确保时间在有效范围内
            time_ms = max(0, min(32767, time_ms))
            
            # 构建命令数据
            # 0x03: 移动到指定角度命令
            cmd = 0x03
            
            # 角度转换为舵机内部格式 (0-240° -> 0-1023)
            angle_pos = int(angle * 1024 / 240)
            angle_h = (angle_pos >> 8) & 0xFF
            angle_l = angle_pos & 0xFF
            
            # 时间数据 (高字节, 低字节)
            time_h = (time_ms >> 8) & 0xFF
            time_l = time_ms & 0xFF
            
            # 构建数据部分 [ID, 长度, 命令, 角度H, 角度L, 时间H, 时间L]
            data = [servo_id, 0x05, cmd, angle_h, angle_l, time_h, time_l]
            
            # 计算校验和 (所有字节异或)
            checksum = 0
            for byte in data:
                checksum ^= byte
            
            # 构建完整帧 [帧头H, 帧头L, ID, 长度, 命令, 参数..., 校验和, 帧尾]
            frame = [0xFF, 0xFF]  # 帧头
            frame.extend(data)
            frame.append(checksum)  # 校验和
            frame.append(0xFE)  # 帧尾
            
            # 发送命令
            if self.ser and self.ser.is_open:
                # 调试信息
                if self.debug:
                    hex_str = ' '.join([f'{b:02X}' for b in frame])
                    print(f"[发送] ID:{servo_id} 角度:{angle}° 时间:{time_ms}ms")
                    print(f"[十六进制] {hex_str}")
                
                self.ser.write(bytes(frame))
                
                # 尝试读取响应（如果有）
                time.sleep(0.05)  # 等待响应
                if self.ser.in_waiting > 0:
                    response = self.ser.read(self.ser.in_waiting)
                    if self.debug:
                        hex_response = ' '.join([f'{b:02X}' for b in response])
                        print(f"[响应] {hex_response}")
                
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
        
        Args:
            servo_id: 舵机ID
            angle: 目标角度 (0-240°)
            time_ms: 运动时间 (毫秒)
        """
        self.send_command(servo_id, angle, time_ms)
    
    def batch_set_angles(self, angles_dict, time_ms=1000):
        """
        批量设置多个舵机的角度
        
        Args:
            angles_dict: 字典 {舵机ID: 角度}
            time_ms: 运动时间 (毫秒)
        """
        for servo_id, angle in angles_dict.items():
            self.set_servo_angle(servo_id, angle, time_ms)
            time.sleep(0.05)  # 命令间隔
    
    def scan_servos(self, id_range=range(1, 5)):
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
                time.sleep(0.05)
                
                # 发送简单命令来测试舵机是否存在
                cmd = 0x03  # 移动命令
                data = [servo_id, 0x05, cmd, 0x02, 0x00, 0x03, 0xE8]  # 120°, 1000ms
                
                checksum = 0
                for byte in data:
                    checksum ^= byte
                
                frame = [0xFF, 0xFF]
                frame.extend(data)
                frame.append(checksum)
                frame.append(0xFE)
                
                self.ser.write(bytes(frame))
                time.sleep(0.1)
                
                # 检查响应 - 必须以FF FF开头
                if self.ser.in_waiting >= 2:
                    response = self.ser.read(self.ser.in_waiting)
                    if len(response) >= 2 and response[0] == 0xFF and response[1] == 0xFF:
                        found_ids.append(servo_id)
                        print(f"  ✓ 找到舵机 ID: {servo_id}")
                
            except Exception as e:
                pass
        
        if found_ids:
            print(f"[完成] 共找到 {len(found_ids)} 个舵机: {found_ids}")
        else:
            print("[警告] 未找到任何舵机")
        
        return found_ids


def main():
    """主函数 - 使用UART7控制舵机"""
    
    # 创建控制器实例，使用UART7
    controller = ZP25SController(port='/dev/ttyS7', baudrate=115200, timeout=1, debug=True)
    
    if not controller.open_serial():
        print("失败：无法打开串口 /dev/ttyS7")
        return
    
    try:
        print("\nUART7 舵机控制开始\n")
        
        # 先扫描舵机
        print("扫描舵机ID (1-10)...")
        found_ids = controller.scan_servos(id_range=range(1, 11))
        
        if not found_ids:
            print("\n警告：未找到舵机！")
            print("可能原因：")
            print("  1. 舵机未上电")
            print("  2. 波特率不匹配（尝试9600、19200等）")
            print("  3. 舵机连接不正常")
            return
        
        print(f"\n找到舵机: {found_ids}\n")
        
        # 舵机控制参数
        commands = {
            2: (150, 1000),   # 002: 顺时针30度 (120+30=150)
            3: (90, 1000),    # 003: 逆时针30度 (120-30=90)
            4: (70, 1000),    # 004: 逆时针50度 (120-50=70)
            5: (120, 3000),   # 005: 360度旋转
        }
        
        for servo_id, (angle, time_ms) in commands.items():
            if servo_id in found_ids:
                print(f"控制舵机 {servo_id}: {angle}° ({time_ms}ms)")
                controller.set_servo_angle(servo_id, angle, time_ms)
                time.sleep(time_ms / 1000 + 0.2)
            else:
                print(f"舵机 {servo_id} 未找到，跳过")
        
        print("\n完成")
        
    except KeyboardInterrupt:
        print("\n中断")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        controller.close_serial()


if __name__ == '__main__':
    main()
