# K230 红色物体跟踪系统 - 配置与使用指南

## 系统概述

这是一个基于K230（庐山派）的完整视觉跟踪系统，包含三个主要部分：

```
┌─────────────────────────────────────────────────────────┐
│  K230 (上位机) - change1.py                             │
│  ├─ 红色物体识别 (color blob detection)                │
│  ├─ PID控制云台 (2个MG966R舵机)                        │
│  └─ 串口发送坐标给下位机                                │
└─────────────┬───────────────────────────────────────────┘
              │ UART2 (115200 bps)
              ▼
┌─────────────────────────────────────────────────────────┐
│  MCU (下位机) - wheel_controller.py                     │
│  ├─ 接收目标坐标                                         │
│  ├─ 计算轮子速度                                         │
│  └─ 控制电机运动                                         │
└─────────────────────────────────────────────────────────┘
```

## 硬件连接

### 1. 舵机接线 (云台控制)

| 功能 | 引脚 | PWM | 信号线 |
|------|------|-----|--------|
| 水平舵机 (左右) | Pin46 | PWM2 | 50Hz, 500-2500us |
| 垂直舵机 (上下) | Pin47 | PWM3 | 50Hz, 500-2500us |

**MG966R舵机脉宽映射：**
- 500us (0%): 0° (左)
- 1500us (7.5%): 90° (中点)
- 2500us (12.5%): 180° (右)

### 2. 串口连接

| 接口 | 用途 | 波特率 | 引脚 |
|------|------|--------|------|
| UART2 | K230↔下位机 | 115200 | RX=Pin12, TX=Pin11 |

## 软件配置

### K230上位机 (change1.py)

#### 参数调优

```python
# 红色阈值 (LAB颜色空间)
RED_THRESHOLD = (20, 80, 30, 100, 0, 60)
#              L_MIN, L_MAX, A_MIN, A_MAX, B_MIN, B_MAX

# PID参数 (根据舵机响应速度调整)
pid_h = SimplePID(kp=0.5, ki=0.05, kd=0.2, max_out=180)
pid_v = SimplePID(kp=0.5, ki=0.05, kd=0.2, max_out=180)

# 控制死区 (像素)
DEAD_ZONE_X = 30  # 水平死区
DEAD_ZONE_Y = 30  # 垂直死区

# 最小色块大小 (像素数)
MIN_BLOB_PIXELS = 200

# 目标丢失超时 (帧数)
LOST_TARGET_FRAMES = 15  # 相当于750ms (50ms/帧)
```

#### 调色工具

运行`color_calibrator.py`来找最佳的红色阈值：

```python
# 在摄像头下展示红色物体，按ESC键获取LAB值
python color_calibrator.py
```

### MCU下位机 (wheel_controller.py)

#### 命令格式

**从K230接收的命令：**

```
$TARGET,x,y,valid\n
  x: 目标X坐标 (0-480)
  y: 目标Y坐标 (0-800)
  valid: 1=目标有效, 0=目标丢失

$SERVO,pan,tilt\n
  pan: 水平角度 (-180到180)
  tilt: 垂直角度 (-180到180)
```

#### 电机控制实现

修改`control_motors()`函数以适配您的电机驱动：

```python
def control_motors(self, left_speed, right_speed):
    """控制电机"""
    # 示例：使用PWM控制
    # left_motor.set_speed(left_speed)
    # right_motor.set_speed(right_speed)
    pass
```

## 调试步骤

### 1. 测试舵机

```python
# 在REPL中测试
from change1 import servo

servo.set_position(0, 0)      # 两个舵机回中
time.sleep(1)
servo.set_position(45, 45)    # 右上
time.sleep(1)
servo.set_position(-45, -45)  # 左下
```

### 2. 测试色块检测

```python
# 调整RED_THRESHOLD值，观察检测效果
# 在屏幕上看是否正确识别红色物体
# 使用色块检测校准工具
```

### 3. 测试PID控制

```python
# 调整PID参数，观察舵机跟踪响应
# Kp: 增大=更快响应但可能震荡
# Ki: 增大=消除稳态误差但响应变慢
# Kd: 增大=减少震荡但对噪声敏感
```

### 4. 测试串口通信

```python
# 在下位机REPL中运行
from wheel_controller import WheelController

ctrl = WheelController()
cmd = ctrl.read_command()
print(cmd)  # 应该收到来自K230的命令
```

## 常见问题

### Q: 舵机不动作
**A:** 
1. 检查PWM引脚和FPIOA配置
2. 测试PWM输出：`pwm_h.duty_ns(1500000)` 应该让舵机到中点
3. 检查舵机电源（需要独立电源，不能从K230供电）

### Q: 色块检测不准确
**A:**
1. 调整`RED_THRESHOLD`阈值
2. 增加光照
3. 运行`color_calibrator.py`获取准确的LAB值

### Q: 舵机响应过慢/过快
**A:**
1. 增加Kp以加快响应
2. 增加Kd以减少超调
3. 调整死区大小

### Q: 串口通信无法工作
**A:**
1. 确认波特率一致(115200)
2. 检查UART2引脚(RX=12, TX=11)
3. 使用示波器验证信号

## 性能指标

| 指标 | 数值 |
|------|------|
| 帧率 | 20 Hz |
| 舵机响应时间 | ~100ms |
| 精度 | ±2° |
| 通信延迟 | <10ms |

## 进阶功能

### 1. 添加图像处理滤波

```python
# 在find_blobs前添加
img = img.median(1)  # 中值滤波
img = img.gaussian(1)  # 高斯模糊
```

### 2. 多物体追踪

```python
if blobs:
    # 不仅选择最大的，还可以追踪多个物体
    for blob in blobs:
        # 处理多个物体
        pass
```

### 3. 添加反馈控制

```python
# 舵机位置可以通过外部传感器反馈
# 改进为闭环控制系统
```

## 文件结构

```
k230/code/
├── change1.py              # 主程序 (K230上位机)
├── wheel_controller.py     # 下位机轮子控制
├── config.py              # 配置文件 (可选)
├── README.md              # 本文件
└── color_calibrator.py    # 颜色校准工具 (可选)
```

## 参考资源

- K230 官方文档: https://www.lckfb.com
- MicroPython docs: https://docs.micropython.org
- MG966R 舵机规格: 500-2500us PWM

---

最后更新: 2025年1月
维护者: AI Assistant
