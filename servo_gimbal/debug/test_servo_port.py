#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机串口快速检测工具
自动扫描所有可用串口，找到连接了舵机的串口
"""

import serial
import time
import glob

def test_servo_on_port(port, baudrate=115200, timeout=1):
    """
    在指定串口测试舵机
    
    Args:
        port: 串口设备路径
        baudrate: 波特率
        timeout: 超时时间
    
    Returns:
        找到的舵机ID列表
    """
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        print(f"\n测试 {port} (波特率: {baudrate})...", end='', flush=True)
        
        found_ids = []
        
        # 扫描 ID 0-15
        for servo_id in range(0, 16):
            try:
                ser.reset_input_buffer()
                time.sleep(0.02)
                
                # 发送读取ID命令
                cmd = f"#{servo_id:03d}PID!"
                ser.write(cmd.encode('ascii'))
                time.sleep(0.05)
                
                if ser.in_waiting > 0:
                    response = ser.read(ser.in_waiting)
                    if len(response) > 0:
                        found_ids.append(servo_id)
                        try:
                            resp_str = response.decode('ascii').strip()
                            print(f"\n  ✓ 舵机 ID: {servo_id:03d} (响应: {resp_str})")
                        except:
                            print(f"\n  ✓ 舵机 ID: {servo_id:03d}")
            except:
                pass
        
        ser.close()
        return found_ids
        
    except Exception as e:
        print(f" [失败] {e}")
        return []


def main():
    """主函数"""
    print("\n" + "="*60)
    print("舵机串口自动检测")
    print("="*60)
    
    # 查找所有可用串口
    possible_ports = (
        glob.glob('/dev/ttyS*') +
        glob.glob('/dev/ttyUSB*') +
        glob.glob('/dev/ttyAMA*') +
        glob.glob('/dev/ttyMTK*')
    )
    possible_ports = sorted(set(possible_ports))
    
    if not possible_ports:
        print("\n错误: 未找到任何串口设备")
        return
    
    print(f"\n检测到 {len(possible_ports)} 个串口: {possible_ports}")
    print("\n正在扫描舵机...")
    
    results = {}
    for port in possible_ports:
        found_ids = test_servo_on_port(port)
        if found_ids:
            results[port] = found_ids
    
    print("\n" + "="*60)
    if results:
        print("✓ 检测完成！找到舵机的串口:")
        for port, servo_ids in results.items():
            print(f"  {port}: 舵机 ID {servo_ids}")
        
        print("\n推荐使用命令:")
        for port in results:
            print(f"  python3 orangepi-test1.py {port}")
    else:
        print("✗ 未找到舵机")
        print("\n可能的原因:")
        print("  1. 舵机未上电")
        print("  2. 波特率不匹配（试试其他波特率）")
        print("  3. 串口连接有问题")
    
    print("="*60 + "\n")


if __name__ == '__main__':
    main()
