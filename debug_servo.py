#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机调试工具 - 用于诊断连接和通信问题
"""

import serial
import time

def test_serial_ports():
    """测试可用的串口"""
    print("="*50)
    print("   串口诊断工具")
    print("="*50)
    
    ports = ['/dev/ttyS0', '/dev/ttyS1', '/dev/ttyS2', '/dev/ttyS3', 
             '/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyAMA0']
    
    print("\n[检查] 尝试打开各个串口...\n")
    
    for port in ports:
        try:
            ser = serial.Serial(port, 115200, timeout=1)
            print(f"✓ {port} - 可以打开")
            
            # 尝试发送一个测试命令给舵机ID 1
            test_frame = [0xFF, 0xFF, 0x01, 0x05, 0x03, 0x02, 0x00, 0x03, 0xE8]
            checksum = 0
            for b in test_frame[2:]:
                checksum ^= b
            test_frame.append(checksum)
            test_frame.append(0xFE)
            
            print(f"  发送测试命令: {' '.join([f'{b:02X}' for b in test_frame])}")
            ser.write(bytes(test_frame))
            
            time.sleep(0.1)
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                print(f"  收到响应: {' '.join([f'{b:02X}' for b in response])}")
            else:
                print(f"  无响应 (这是正常的，舵机可能不存在或不回复)")
            
            ser.close()
            print()
            
        except Exception as e:
            print(f"✗ {port} - {e}\n")


def test_baudrates():
    """测试不同波特率"""
    print("="*50)
    print("   波特率测试")
    print("="*50)
    
    port = '/dev/ttyS0'
    baudrates = [9600, 19200, 57600, 115200, 230400, 460800]
    
    print(f"\n[检查] 在 {port} 上尝试不同波特率...\n")
    
    for baud in baudrates:
        try:
            ser = serial.Serial(port, baud, timeout=1)
            print(f"✓ 波特率 {baud} - 可以打开")
            
            # 发送简单命令
            test_frame = [0xFF, 0xFF, 0x01, 0x05, 0x03, 0x02, 0x00, 0x03, 0xE8, 0xFC, 0xFE]
            ser.write(bytes(test_frame))
            time.sleep(0.05)
            
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                print(f"  收到数据！响应: {' '.join([f'{b:02X}' for b in response])}")
            
            ser.close()
            print()
            
        except Exception as e:
            print(f"✗ 波特率 {baud} - {e}\n")


def interactive_send():
    """交互式发送命令"""
    print("="*50)
    print("   交互式命令发送")
    print("="*50)
    
    port = input("\n请输入串口号 (默认 /dev/ttyS0): ").strip() or '/dev/ttyS0'
    baud = int(input("请输入波特率 (默认 115200): ").strip() or "115200")
    
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"\n✓ 已连接: {port} @ {baud} bps\n")
        
        while True:
            print("选项:")
            print("1. 发送舵机控制命令")
            print("2. 发送自定义十六进制数据")
            print("3. 读取响应")
            print("4. 退出")
            
            choice = input("\n请选择 (1-4): ").strip()
            
            if choice == '1':
                servo_id = int(input("舵机ID (1-254): "))
                angle = int(input("角度 (0-240): "))
                time_ms = int(input("运动时间(ms, 默认1000): ") or "1000")
                
                # 构建帧
                angle_pos = int(angle * 1024 / 240)
                angle_h = (angle_pos >> 8) & 0xFF
                angle_l = angle_pos & 0xFF
                time_h = (time_ms >> 8) & 0xFF
                time_l = time_ms & 0xFF
                
                data = [servo_id, 0x05, 0x03, angle_h, angle_l, time_h, time_l]
                checksum = 0
                for b in data:
                    checksum ^= b
                
                frame = [0xFF, 0xFF] + data + [checksum, 0xFE]
                hex_str = ' '.join([f'{b:02X}' for b in frame])
                print(f"发送: {hex_str}")
                
                ser.write(bytes(frame))
                
            elif choice == '2':
                hex_input = input("输入十六进制数据 (用空格分开，如: FF FF 01 05 03): ")
                try:
                    data = bytes([int(x, 16) for x in hex_input.split()])
                    print(f"发送: {' '.join([f'{b:02X}' for b in data])}")
                    ser.write(data)
                except:
                    print("格式错误")
                    
            elif choice == '3':
                time.sleep(0.1)
                if ser.in_waiting > 0:
                    response = ser.read(ser.in_waiting)
                    print(f"收到: {' '.join([f'{b:02X}' for b in response])}")
                else:
                    print("无数据")
                    
            elif choice == '4':
                break
        
        ser.close()
        
    except Exception as e:
        print(f"错误: {e}")


if __name__ == '__main__':
    print("\n舵机调试工具\n")
    
    while True:
        print("\n主菜单:")
        print("1. 串口诊断")
        print("2. 波特率测试")
        print("3. 交互式发送命令")
        print("4. 舵机ID扫描")
        print("5. 连接测试")
        print("6. 原始帧发送")
        print("7. 退出")
        
        choice = input("\n请选择 (1-7): ").strip()
        
        if choice == '1':
            test_serial_ports()
        elif choice == '2':
            # 改进的波特率测试
            print("="*50)
            print("   波特率测试")
            print("="*50)
            
            port = '/dev/ttyS0'
            baudrates = [9600, 19200, 57600, 115200, 230400, 460800]
            
            print(f"\n[检查] 在 {port} 上尝试不同波特率...\n")
            
            for baud in baudrates:
                try:
                    ser = serial.Serial(port, baud, timeout=1)
                    print(f"✓ 波特率 {baud} - 可以打开")
                    
                    # 发送简单命令
                    test_frame = [0xFF, 0xFF, 0x01, 0x05, 0x03, 0x02, 0x00, 0x03, 0xE8, 0xFC, 0xFE]
                    ser.write(bytes(test_frame))
                    time.sleep(0.05)
                    
                    if ser.in_waiting > 0:
                        response = ser.read(ser.in_waiting)
                        print(f"  收到数据！响应: {' '.join([f'{b:02X}' for b in response])}")
                    
                    ser.close()
                    print()
                    
                except Exception as e:
                    print(f"✗ 波特率 {baud} - {e}\n")
        elif choice == '3':
            interactive_send()
        elif choice == '4':
            # ID扫描
            print("="*50)
            print("   舵机ID扫描")
            print("="*50)
            
            port = input("\n请输入串口号 (默认 /dev/ttyS0): ").strip() or '/dev/ttyS0'
            baud = int(input("请输入波特率 (默认 115200): ").strip() or "115200")
            
            print(f"\n[扫描] 在 {port} @ {baud} 上扫描舵机ID (0-15)...\n")
            
            try:
                ser = serial.Serial(port, baud, timeout=0.5)
                found_ids = []
                
                for servo_id in range(0, 16):
                    # 构建移动命令
                    angle_pos = int(120 * 1024 / 240)  # 120度
                    angle_h = (angle_pos >> 8) & 0xFF
                    angle_l = angle_pos & 0xFF
                    
                    data = [servo_id, 0x05, 0x03, angle_h, angle_l, 0x03, 0xE8]
                    checksum = 0
                    for b in data:
                        checksum ^= b
                    
                    frame = [0xFF, 0xFF] + data + [checksum, 0xFE]
                    
                    ser.write(bytes(frame))
                    time.sleep(0.1)
                    
                    # 检查响应
                    if ser.in_waiting > 0:
                        response = ser.read(ser.in_waiting)
                        found_ids.append(servo_id)
                        resp_hex = ' '.join([f'{b:02X}' for b in response])
                        print(f"✓ ID {servo_id}: 找到! 响应: {resp_hex}")
                
                ser.close()
                
                if found_ids:
                    print(f"\n[结果] 找到 {len(found_ids)} 个舵机，ID为: {found_ids}")
                else:
                    print("\n[结果] 未找到任何舵机")
                    print("\n[排查建议]")
                    print("1. 检查舵机驱动板是否上电（查看LED指示灯）")
                    print("2. 确认舵机连接是否正确")
                    print("3. 检查USB/UART连接是否稳定")
                    print("4. 尝试用官方工具查看舵机状态")
                    
            except Exception as e:
                print(f"错误: {e}")
        elif choice == '5':
            # 连接测试
            print("="*50)
            print("   连接和协议测试")
            print("="*50)
            
            port = input("\n请输入串口号 (默认 /dev/ttyS0): ").strip() or '/dev/ttyS0'
            baud = int(input("请输入波特率 (默认 115200): ").strip() or "115200")
            servo_id = int(input("请输入舵机ID (默认 1): ") or "1")
            
            print(f"\n[测试] 向 ID {servo_id} 的舵机发送多种命令\n")
            
            try:
                ser = serial.Serial(port, baud, timeout=1)
                
                # 测试1: 基础移动命令
                print("[命令1] 移动到中点 (120°)")
                angle_pos = int(120 * 1024 / 240)
                angle_h = (angle_pos >> 8) & 0xFF
                angle_l = angle_pos & 0xFF
                
                data = [servo_id, 0x05, 0x03, angle_h, angle_l, 0x03, 0xE8]
                checksum = 0
                for b in data:
                    checksum ^= b
                frame = [0xFF, 0xFF] + data + [checksum, 0xFE]
                
                hex_cmd = ' '.join([f'{b:02X}' for b in frame])
                print(f"  发送: {hex_cmd}")
                ser.write(bytes(frame))
                time.sleep(0.1)
                
                if ser.in_waiting > 0:
                    response = ser.read(ser.in_waiting)
                    print(f"  响应: {' '.join([f'{b:02X}' for b in response])}")
                else:
                    print(f"  无响应")
                
                time.sleep(1)
                
                # 测试2: 查询位置
                print("\n[命令2] 查询当前位置")
                query_data = [servo_id, 0x02, 0x04]  # 查询位置命令
                query_checksum = 0
                for b in query_data:
                    query_checksum ^= b
                query_frame = [0xFF, 0xFF] + query_data + [query_checksum, 0xFE]
                
                hex_cmd = ' '.join([f'{b:02X}' for b in query_frame])
                print(f"  发送: {hex_cmd}")
                ser.write(bytes(query_frame))
                time.sleep(0.1)
                
                if ser.in_waiting > 0:
                    response = ser.read(ser.in_waiting)
                    print(f"  响应: {' '.join([f'{b:02X}' for b in response])}")
                else:
                    print(f"  无响应")
                
                ser.close()
                
            except Exception as e:
                print(f"错误: {e}")
        elif choice == '6':
            # 原始帧发送
            print("="*50)
            print("   原始帧发送和监听")
            print("="*50)
            
            port = input("\n请输入串口号 (默认 /dev/ttyS0): ").strip() or '/dev/ttyS0'
            baud = int(input("请输入波特率 (默认 115200): ").strip() or "115200")
            
            try:
                ser = serial.Serial(port, baud, timeout=1)
                print(f"\n✓ 已连接: {port} @ {baud} bps")
                print("按 Ctrl+C 退出\n")
                
                while True:
                    hex_input = input("输入十六进制数据 (例: FF FF 01 05 03 02 00 03 E8 EE FE): ").strip()
                    
                    if not hex_input:
                        continue
                        
                    try:
                        data = bytes([int(x, 16) for x in hex_input.split()])
                        print(f"[发送] {' '.join([f'{b:02X}' for b in data])}")
                        ser.write(data)
                        
                        time.sleep(0.2)
                        if ser.in_waiting > 0:
                            response = ser.read(ser.in_waiting)
                            print(f"[收到] {' '.join([f'{b:02X}' for b in response])}")
                        else:
                            print("[收到] 无数据")
                        print()
                        
                    except ValueError:
                        print("格式错误，请使用十六进制格式（用空格分开）\n")
                        
            except Exception as e:
                print(f"错误: {e}")
        elif choice == '7':
            print("再见")
            break
        else:
            print("无效选择")
