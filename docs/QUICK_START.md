# K230 红色物体跟踪系统 - 快速启动指南

## 🚀 5分钟快速启动

### 步骤1: 硬件连接检查

```
✓ 舵机1 (水平) → Pin46 (PWM2)
✓ 舵机2 (垂直) → Pin47 (PWM3)
✓ 舵机电源 → 独立6V电源 (≥2A)
✓ 下位机 → UART2 (TX=Pin11, RX=Pin12)
```

### 步骤2: 上传代码到K230

```bash
# 使用ampy或其他工具上传
ampy --port COM3 put change1.py /sd/change1.py
ampy --port COM3 put config.py /sd/config.py
```

### 步骤3: 启动程序

```python
# 在REPL中运行
import change1
```

### 步骤4: 观察屏幕输出

在K230的LCD屏幕上，你应该看到：
- 摄像头实时画面
- 红色物体识别框
- 舵机位置信息
- PID控制输出值

---

## 📊 系统架构简图

```
┌──────────────────┐
│  K230 摄像头      │
│  RGB565, 480x800 │
└────────┬─────────┘
         │
         ▼
┌──────────────────────────────────┐
│  change1.py (上位机)              │
│  ├─ 红色Blob检测                  │
│  ├─ PID云台控制                   │
│  └─ UART串口发送                  │
└──┬──────────────┬────────────┬───┘
   │              │            │
   ▼              ▼            ▼
 PWM2          PWM3         UART2
   │              │            │
   ▼              ▼            ▼
┌────────┐    ┌────────┐    ┌──────────┐
│水平舵机 │   │垂直舵机 │   │下位机MCU  │
│MG966R  │   │MG966R  │   │(轮子控制)│
└────────┘    └────────┘    └──────────┘
```

---

## 🎮 实时控制与调试

### 舵机测试

```python
# 在REPL中依次执行
from change1 import servo
import time

# 舵机回中
servo.set_position(0, 0)
time.sleep(0.5)

# 左转45°
servo.set_position(-45, 0)
time.sleep(0.5)

# 右转45°
servo.set_position(45, 0)
time.sleep(0.5)

# 上转
servo.set_position(0, 45)
time.sleep(0.5)

# 下转
servo.set_position(0, -45)
```

### PID参数实时调整

```python
# 修改config.py中的PID参数后，重新导入
import importlib
import config

importlib.reload(config)

# 重新创建PID控制器（如果需要）
from change1 import pid_h
pid_h.kp = config.PID_H_KP
pid_h.ki = config.PID_H_KI
pid_h.kd = config.PID_H_KD
```

### 颜色阈值调整

```python
# 动态调整色块检测阈值
# 修改change1.py中的RED_THRESHOLD
RED_THRESHOLD = (20, 80, 30, 100, 0, 60)

# 或在REPL中临时修改
# import change1
# change1.RED_THRESHOLD = (10, 90, 25, 110, -5, 70)
```

---

## 🔧 常见故障排除

### 问题1: 舵机不转

**症状:** 
- 舵机无响应
- 电机发热但不动

**检查清单:**
```python
# 1. 检查PWM输出
from machine import PWM, FPIOA
pwm_io = FPIOA()
pwm_io.set_function(46, FPIOA.PWM2)
pwm = PWM(2, freq=50)

# 手动设置脉宽
pwm.duty_ns(500000)    # 应该转到最左
time.sleep(0.5)
pwm.duty_ns(1500000)   # 应该回到中点
time.sleep(0.5)
pwm.duty_ns(2500000)   # 应该转到最右

# 2. 检查舵机电源
# 用万用表测量舵机电源：应该是6V
# 确保使用独立电源，不能从K230取电

# 3. 检查舵机信号线连接
# PWM2 (Pin46) → 舵机1信号线
# PWM3 (Pin47) → 舵机2信号线
```

### 问题2: 色块检测不工作

**症状:**
- 屏幕显示"NO TARGET"
- 即使红色物体很明显

**调试步骤:**
```python
# 1. 用color_calibrator.py查看实际LAB值
python color_calibrator.py

# 2. 查看原始摄像头数据
from media.sensor import *
from media.display import *
from media.media import *

sensor = Sensor(width=480, height=800)
sensor.reset()
sensor.set_framesize(width=480, height=800)
sensor.set_pixformat(Sensor.RGB565)
Display.init(Display.ST7701)
MediaManager.init()
sensor.run()

while True:
    img = sensor.snapshot()
    Display.show_image(img)
    # 观察摄像头是否正常工作
```

### 问题3: 舵机响应延迟大

**症状:**
- 物体移动，但舵机跟踪延迟明显
- 舵机跟踪不稳定，震荡

**调整建议:**
```python
# 问题A: 响应太慢
# 增加Kp值
PID_H_KP = 0.8  # 从0.5增加到0.8

# 问题B: 响应过快导致震荡
# 增加Kd值
PID_H_KD = 0.3  # 从0.2增加到0.3

# 问题C: 目标移动时跟不上
# 增加Ki值（但要小心，可能导致超调）
PID_H_KI = 0.1  # 从0.05增加到0.1
```

### 问题4: 下位机无法接收数据

**症状:**
- UART通信失败
- 下位机接收不到坐标数据

**检查步骤:**
```python
# 1. 检查波特率是否一致
# K230: UART_BAUDRATE = 115200
# MCU: 应该也设置为115200

# 2. 检查UART引脚
# UART2 TX = Pin11
# UART2 RX = Pin12

# 3. 测试UART通信
from machine import UART
uart = UART(2, baudrate=115200)
uart.write(b"TEST\n")

# 在下位机REPL中：
uart = UART(2, baudrate=115200)
data = uart.read(100)
print(data)  # 应该显示 b'TEST\n'

# 4. 检查波特率、数据位、停止位设置
uart = UART(2, baudrate=115200, bits=8, parity=0, stop=1)
```

---

## 📈 性能优化建议

### 1. 加快帧率

```python
# 增加摄像头帧率（可能需要降低分辨率）
# 或减少处理时间

# 原来: time.sleep_ms(50)  # 20Hz
# 改为: time.sleep_ms(33)  # 30Hz
```

### 2. 减少延迟

```python
# 使用图像缓冲
sensor.set_framebuffers(2)  # 增加到3个缓冲

# 禁用非必要的显示
# 只在调试时显示完整信息
```

### 3. 降低功耗

```python
# 降低舵机更新频率
# 只在偏差大于某个值时更新舵机

if abs(error_h) > 5:  # 只有误差大于5°才更新
    servo.set_position(pan_out, tilt_out)
```

---

## 🧪 测试检查清单

- [ ] 舵机手动控制正常
- [ ] 摄像头显示清晰
- [ ] 色块检测准确
- [ ] PID控制平稳
- [ ] 下位机通信正常
- [ ] LCD显示无闪烁
- [ ] 目标丢失回中功能正常
- [ ] 死区控制生效

---

## 📱 移动调试

可以通过远程REPL进行实时调试：

```bash
# 使用picocom
picocom -b 115200 /dev/ttyUSB0

# 或使用mpremote (MicroPython远程IDE)
mpremote connect COM3
```

---

## 📚 进阶文档

详见:
- [README.md](README.md) - 完整配置指南
- [config.py](config.py) - 参数详解
- [change1.py](change1.py) - 源代码注释

---

**Last Updated:** 2025年1月
**Version:** 1.0
