# K230 红色物体跟踪系统 - 快速导航

## 📚 文件清单

| 文件 | 大小 | 用途 |
|------|------|------|
| change1.py | ~260行 | K230主程序 (舵机+检测+通信) |
| wheel_controller.py | ~150行 | 下位机轮子控制 |
| config.py | ~150行 | 全局配置参数 |
| test_system.py | ~200行 | 系统自诊断工具 |
| debug_tool.py | ~250行 | 实时参数调试 |
| color_calibrator.py | ~120行 | 颜色校准工具 |
| README.md | 详细配置 | 硬件接线、参数说明 |
| QUICK_START.md | 快速指南 | 5分钟启动、故障排除 |
| SYSTEM_GUIDE.md | 完整说明 | 架构、流程、优化 |
| INDEX.md | 本文件 | 快速导航 |

## 🚀 快速上手

### 第1步: 检查硬件
- 舵机接Pin46/47
- MCU接UART2
- 电源连接正确

### 第2步: 上传代码
```bash
ampy --port COM3 put change1.py /
```

### 第3步: 启动程序
```python
import change1
```

### 第4步: 观察LCD屏幕
应该看到实时摄像头画面和舵机跟踪信息

## 🔧 常用命令

```python
# 舵机测试
from change1 import servo
servo.set_position(45, 45)  # 右上45°

# 系统自检
import test_system
test_system.main()

# 颜色校准
python color_calibrator.py

# 参数调试
import debug_tool
debug_tool.ParameterDebugger().run()
```

## 📖 文档导航

- **[QUICK_START.md](QUICK_START.md)** - 5分钟快速开始
- **[README.md](README.md)** - 详细配置指南
- **[SYSTEM_GUIDE.md](SYSTEM_GUIDE.md)** - 系统架构和优化

## ⚡ 常见问题

| 问题 | 快速解决 |
|------|--------|
| 舵机不转 | 检查Pin46/47连接和电源 |
| 检测无效 | 运行color_calibrator.py调整阈值 |
| 响应延迟 | 增加config.py中的PID_Kp值 |
| 通信失败 | 确认波特率都是115200 |

## 📊 参考数据

- 舵机脉宽: 500-2500us (1500us为中点)
- 红色阈值: (20, 80, 30, 100, 0, 60)
- PID推荐: Kp=0.5, Ki=0.05, Kd=0.2
- 帧率: 20Hz (50ms)

---

**下一步**: 打开[QUICK_START.md](QUICK_START.md)开始吧！
