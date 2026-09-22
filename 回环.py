from machine import UART, FPIOA
import time

fpioa = FPIOA()
fpioa.set_function(3, FPIOA.UART1_TXD)
fpioa.set_function(4, FPIOA.UART1_RXD)

uart = UART(UART.UART1, baudrate=115200)

print("[INFO] 开始回环测试...")
test_msg = b"UART1 Loopback Test"
uart.write(test_msg)
time.sleep_ms(100)

received = uart.read()
print(f"[发送] {test_msg}")
print(f"[接收] {received}")

if received == test_msg:
    print("✅ 回环测试通过，UART1 硬件配置正确")
else:
    print("❌ 回环测试失败，引脚或 UART1 有问题")