# STM32轮子控制器集成指南

## 概述
现在的代码已经适配STM32，使用STM32 HAL库实现UART通信和控制。

## 集成步骤

### 1. STM32CubeMX配置

#### UART配置
- 选择要使用的UART(例如USART2)
- 波特率：115200 bps
- 数据位：8
- 停止位：1
- 奇偶校验：无
- **启用UART接收中断**

#### 定时器配置(用于电机PWM控制)
- 选择TIM1, TIM2等定时器
- 配置为PWM模式
- 配置2个通道(左右轮电机)
- 频率：1KHz左右

#### 时钟配置
- 确保系统时钟正常
- 设置HAL滴答定时器

### 2. 项目中添加文件
将以下文件复制到STM32项目中：
- `wheel_controller.h` - 放在 `Inc/` 目录
- `wheel_controller.c` - 放在 `Src/` 目录

### 3. main.c中的集成代码

```c
#include "wheel_controller.h"

/* UART句柄(由CubeMX生成) */
extern UART_HandleTypeDef huart2;

/* 接收缓冲区 */
uint8_t rx_byte;
WheelController *g_controller = NULL;

int main(void) {
    /* HAL初始化 */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();  /* 电机PWM定时器 */
    
    /* 启动PWM输出 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  /* 左轮 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  /* 右轮 */
    
    /* 初始化轮子控制器 */
    g_controller = wheel_controller_init(&huart2);
    if (g_controller == NULL) {
        Error_Handler();
    }
    
    /* 启用UART接收中断 */
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    
    /* 主循环 */
    while (1) {
        /* 定期检查命令(可选，如果使用中断方式) */
        Command cmd = wheel_controller_read_command(g_controller, 10);
        
        if (cmd.type != CMD_TYPE_NONE) {
            if (cmd.type == CMD_TYPE_TARGET) {
                if (cmd.data.target.valid) {
                    uint8_t left_spd, right_spd;
                    wheel_controller_calculate_speed(
                        cmd.data.target.x, cmd.data.target.y,
                        480, 800, 255,
                        &left_spd, &right_spd
                    );
                    
                    /* 更新PWM占空比 */
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, left_spd);
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, right_spd);
                }
            }
        }
        
        HAL_Delay(10);
    }
}

/* UART接收完成中断回调 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2 && g_controller != NULL) {
        wheel_controller_uart_irq_handler(g_controller, rx_byte);
        
        /* 继续接收下一个字节 */
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

/* 错误处理 */
void Error_Handler(void) {
    __disable_irq();
    while (1);
}
```

### 4. 电机PWM控制实现

修改 `wheel_controller.c` 中的 `wheel_controller_control_motors()` 函数：

```c
extern TIM_HandleTypeDef htim1;  /* 电机PWM定时器 */

void wheel_controller_control_motors(uint8_t left_speed, uint8_t right_speed) {
    printf("电机控制: 左=%u, 右=%u\r\n", left_speed, right_speed);
    
    /* 更新PWM占空比(假设ARR=255) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, left_speed);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, right_speed);
}
```

### 5. 编译注意事项

确保在项目中包含以下头文件路径：
- STM32 HAL库路径(由CubeMX自动生成)
- `Inc/` 目录(包含wheel_controller.h)

### 6. UART通信格式

上位机(K230)发送给STM32的格式：
```
$TARGET,200,400,1\n   // 目标坐标(x=200, y=400, valid=1)
$SERVO,45,30\n        // 云台角度(Pan=45°, Tilt=30°)
```

## 常见问题

**Q: 代码无法编译？**
A: 检查：
1. 是否包含了 `stm32lXxx_hal.h`(根据实际型号修改)
2. STM32 HAL库是否正确配置
3. 是否有缺失的中断回调函数

**Q: 没有收到数据？**
A: 检查：
1. UART波特率是否设置为115200
2. 是否启用了接收中断(`HAL_UART_Receive_IT`)
3. RX引脚是否正确连接
4. K230发送的数据格式是否正确

**Q: 电机不动？**
A: 检查：
1. PWM定时器是否已启动(`HAL_TIM_PWM_Start`)
2. 电机驱动引脚是否连接
3. `wheel_controller_control_motors()`中的PWM计数是否设置正确

## STM32型号支持

代码支持所有STM32系列，只需修改头文件：
- STM32L4系列: `#include "stm32l4xx_hal.h"`
- STM32F4系列: `#include "stm32f4xx_hal.h"`
- STM32H7系列: `#include "stm32h7xx_hal.h"`
- STM32G0系列: `#include "stm32g0xx_hal.h"`

## 参考资源

- [STM32 HAL API文档](https://www.st.com/)
- [STM32CubeMX使用指南](https://www.st.com/stm32cubemx)
- [UART中断处理最佳实践](https://www.st.com/)
