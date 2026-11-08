#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OrangePi 串口诊断工具
列出所有可用的串口设备
"""

import os
import glob

def check_uart_devices():
    """检查可用的 UART 设备"""
    print("\n" + "="*60)
    print("OrangePi UART 设备诊断")
    print("="*60 + "\n")
    
    # 检查 /dev/ttyS* 设备
    print("[1] 检查 /dev/ttyS* 设备:")
    uart_devices = glob.glob('/dev/ttyS*')
    if uart_devices:
        for device in sorted(uart_devices):
            try:
                # 检查设备是否可读写
                if os.access(device, os.R_OK) and os.access(device, os.W_OK):
                    print(f"  ✓ {device} (可读写)")
                else:
                    print(f"  ✗ {device} (无权限)")
            except:
                print(f"  ? {device}")
    else:
        print("  未找到 /dev/ttyS* 设备")
    
    # 检查 /dev/ttyUSB* 设备
    print("\n[2] 检查 /dev/ttyUSB* 设备 (USB转串口):")
    usb_devices = glob.glob('/dev/ttyUSB*')
    if usb_devices:
        for device in sorted(usb_devices):
            try:
                if os.access(device, os.R_OK) and os.access(device, os.W_OK):
                    print(f"  ✓ {device} (可读写)")
                else:
                    print(f"  ✗ {device} (无权限)")
            except:
                print(f"  ? {device}")
    else:
        print("  未找到 /dev/ttyUSB* 设备")
    
    # 检查 /dev/ttyAMA* 设备 (树莓派/OrangePi 常见)
    print("\n[3] 检查 /dev/ttyAMA* 设备 (UART GPIO):")
    ama_devices = glob.glob('/dev/ttyAMA*')
    if ama_devices:
        for device in sorted(ama_devices):
            try:
                if os.access(device, os.R_OK) and os.access(device, os.W_OK):
                    print(f"  ✓ {device} (可读写)")
                else:
                    print(f"  ✗ {device} (无权限)")
            except:
                print(f"  ? {device}")
    else:
        print("  未找到 /dev/ttyAMA* 设备")
    
    # 检查 /dev/ttyMTK* 设备 (某些ARM板卡)
    print("\n[4] 检查 /dev/ttyMTK* 设备:")
    mtk_devices = glob.glob('/dev/ttyMTK*')
    if mtk_devices:
        for device in sorted(mtk_devices):
            try:
                if os.access(device, os.R_OK) and os.access(device, os.W_OK):
                    print(f"  ✓ {device} (可读写)")
                else:
                    print(f"  ✗ {device} (无权限)")
            except:
                print(f"  ? {device}")
    else:
        print("  未找到 /dev/ttyMTK* 设备")
    
    # 列出 /dev 中所有 tty 设备
    print("\n[5] 所有 /dev/tty* 设备:")
    all_devices = glob.glob('/dev/tty*')
    if all_devices:
        for device in sorted(all_devices)[:20]:  # 只显示前20个
            print(f"  - {device}")
    else:
        print("  未找到任何 tty 设备")
    
    # 检查 dmesg 中的串口信息
    print("\n[6] 系统串口信息 (dmesg 最后 10 行):")
    os.system("dmesg | grep -i -E 'uart|serial|tty' | tail -10")
    
    print("\n" + "="*60)
    print("建议:")
    print("  1. 上方输出中显示 ✓ 的设备是可用的")
    print("  2. 选择你的舵机实际连接到的串口")
    print("  3. 修改 orangepi-test1.py 中的 port 参数")
    print("="*60 + "\n")


if __name__ == '__main__':
    check_uart_devices()
