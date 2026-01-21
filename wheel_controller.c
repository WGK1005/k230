/*
 * 下位机轮子控制模块实现 - STM32 HAL版本
 */

#include "wheel_controller.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * 初始化轮子控制器
 */
WheelController* wheel_controller_init(UART_HandleTypeDef *huart) {
    WheelController *controller = (WheelController *)malloc(sizeof(WheelController));
    
    if (controller == NULL) {
        printf("内存分配失败\r\n");
        return NULL;
    }
    
    if (huart == NULL) {
        printf("UART句柄为空\r\n");
        free(controller);
        return NULL;
    }
    
    controller->huart = huart;
    controller->rx_len = 0;
    controller->last_cmd_time = HAL_GetTick();
    memset(controller->rx_buffer, 0, sizeof(controller->rx_buffer));
    
    printf("轮子控制器已初始化\r\n");
    
    return controller;
}

/**
 * 释放轮子控制器资源
 */
void wheel_controller_free(WheelController *controller) {
    if (controller != NULL) {
        free(controller);
    }
}

/**
 * 解析命令字符串
 */
Command wheel_controller_parse_command(const uint8_t *data, uint16_t len) {
    Command cmd;
    cmd.type = CMD_TYPE_NONE;
    
    if (data == NULL || len == 0) {
        return cmd;
    }
    
    /* 转换为字符串 */
    char buffer[256];
    if (len >= sizeof(buffer)) {
        len = sizeof(buffer) - 1;
    }
    memcpy(buffer, data, len);
    buffer[len] = '\0';
    
    /* 移除换行符 */
    char *newline = strchr(buffer, '\n');
    if (newline != NULL) {
        *newline = '\0';
    }
    
    /* 移除空格 */
    while (len > 0 && (buffer[len-1] == '\r' || buffer[len-1] == '\n' || buffer[len-1] == ' ')) {
        buffer[--len] = '\0';
    }
    
    /* 解析TARGET命令 */
    if (strncmp(buffer, "$TARGET,", 8) == 0) {
        int x, y, valid;
        if (sscanf(buffer + 8, "%d,%d,%d", &x, &y, &valid) == 3) {
            cmd.type = CMD_TYPE_TARGET;
            cmd.data.target.x = x;
            cmd.data.target.y = y;
            cmd.data.target.valid = (valid != 0);
            return cmd;
        }
    }
    
    /* 解析SERVO命令 */
    if (strncmp(buffer, "$SERVO,", 7) == 0) {
        int pan, tilt;
        if (sscanf(buffer + 7, "%d,%d", &pan, &tilt) == 2) {
            cmd.type = CMD_TYPE_SERVO;
            cmd.data.servo.pan = pan;
            cmd.data.servo.tilt = tilt;
            return cmd;
        }
    }
    
    return cmd;
}

/**
 * 读取来自K230的命令
 */
Command wheel_controller_read_command(WheelController *controller, uint32_t timeout_ms) {
    Command cmd;
    cmd.type = CMD_TYPE_NONE;
    
    if (controller == NULL || controller->huart == NULL) {
        return cmd;
    }
    
    /* 当缓冲区中有完整命令时解析 */
    uint16_t i;
    for (i = 0; i < controller->rx_len; i++) {
        if (controller->rx_buffer[i] == '\n') {
            cmd = wheel_controller_parse_command(controller->rx_buffer, i + 1);
            
            /* 移除已处理的数据 */
            if (i + 1 < controller->rx_len) {
                memmove(controller->rx_buffer, 
                       controller->rx_buffer + i + 1, 
                       controller->rx_len - i - 1);
            }
            controller->rx_len -= (i + 1);
            return cmd;
        }
    }
    
    return cmd;
}

/**
 * UART中断处理回调(在HAL_UART_RxCpltCallback()中调用)
 * 单字节中断接收
 */
void wheel_controller_uart_irq_handler(WheelController *controller, uint8_t byte) {
    if (controller == NULL || controller->rx_len >= sizeof(controller->rx_buffer) - 1) {
        return;
    }
    
    controller->rx_buffer[controller->rx_len++] = byte;
    controller->last_cmd_time = HAL_GetTick();
}

/**
 * 计算轮子速度
 */
void wheel_controller_calculate_speed(int target_x, int target_y,
                                      int screen_width, int screen_height,
                                      uint8_t max_speed,
                                      uint8_t *left_speed, uint8_t *right_speed) {
    if (left_speed == NULL || right_speed == NULL) {
        return;
    }
    
    if (target_x == 0 || target_y == 0) {
        *left_speed = 0;
        *right_speed = 0;
        return;
    }
    
    int cx = screen_width / 2;
    int cy = screen_height / 2;
    
    int offset_x = target_x - cx;
    int offset_y = target_y - cy;
    
    /* 初始速度 */
    int left_spd = max_speed / 2;
    int right_spd = max_speed / 2;
    
    /* 根据水平偏移调整速度(差速转向) */
    if (offset_x > 50) {
        /* 目标在右边，减少右轮速度 */
        right_spd = max_speed / 2 - (offset_x / 5);
        if (right_spd < 0) right_spd = 0;
    } else if (offset_x < -50) {
        /* 目标在左边，减少左轮速度 */
        left_spd = max_speed / 2 - ((-offset_x) / 5);
        if (left_spd < 0) left_spd = 0;
    }
    
    *left_speed = (uint8_t)left_spd;
    *right_speed = (uint8_t)right_spd;
}

/**
 * 控制电机速度
 */
void wheel_controller_control_motors(uint8_t left_speed, uint8_t right_speed) {
    printf("电机控制: 左=%u, 右=%u\r\n", left_speed, right_speed);
    /* TODO: 实现实际的电机PWM控制(使用TIM) */
}

/**
 * 主控制循环
 */
void wheel_controller_main_loop(WheelController *controller) {
    if (controller == NULL) {
        printf("控制器指针为空\r\n");
        return;
    }
    
    printf("开始轮子控制主循环...\r\n");
    
    while (1) {
        /* 尝试读取来自K230的命令 */
        Command cmd = wheel_controller_read_command(controller, 50);
        
        if (cmd.type != CMD_TYPE_NONE) {
            if (cmd.type == CMD_TYPE_TARGET) {
                /* 收到目标坐标 */
                if (cmd.data.target.valid) {
                    uint8_t left_spd, right_spd;
                    wheel_controller_calculate_speed(
                        cmd.data.target.x, cmd.data.target.y,
                        480, 800, 255,
                        &left_spd, &right_spd
                    );
                    wheel_controller_control_motors(left_spd, right_spd);
                    printf("跟踪目标: (%d, %d)\r\n", 
                           cmd.data.target.x, cmd.data.target.y);
                } else {
                    /* 目标丢失 */
                    wheel_controller_control_motors(0, 0);
                    printf("目标丢失，停止运动\r\n");
                }
            } else if (cmd.type == CMD_TYPE_SERVO) {
                /* 收到云台角度 */
                printf("云台位置: Pan=%d度, Tilt=%d度\r\n",
                       cmd.data.servo.pan, cmd.data.servo.tilt);
            }
        }
        
        HAL_Delay(100); /* 延迟100ms */
    }
}

/* ==================== 主程序示例 ==================== */

int main(void) {
    /* 初始化STM32(由CubeMX生成的代码在这里) */
    HAL_Init();
    SystemClock_Config();
    /* ... 其他初始化代码 ... */
    
    /* 初始化UART(由CubeMX生成) */
    /* UART_HandleTypeDef huart2; */
    /* MX_USART2_UART_Init(); */
    
    /* 启用UART接收中断 */
    /* HAL_UART_Receive_IT(&huart2, &rx_byte, 1); */
    
    /* 初始化轮子控制器 */
    WheelController *controller = wheel_controller_init(&huart2);
    
    if (controller == NULL) {
        printf("轮子控制器初始化失败\r\n");
        while (1);
    }
    
    /* 运行主循环 */
    wheel_controller_main_loop(controller);
    
    /* 清理资源(永不到达) */
    wheel_controller_free(controller);
    
    return 0;
}
