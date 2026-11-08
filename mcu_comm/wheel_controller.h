/*
 * 下位机轮子控制模块 - 接收来自K230的目标坐标和云台角度
 * 用于控制轮子朝向目标物体运动
 * STM32 HAL版本
 */

#ifndef WHEEL_CONTROLLER_H
#define WHEEL_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"  /* 根据实际型号修改(如stm32f4xx_hal.h, stm32h7xx_hal.h等) */

/* 命令类型定义 */
typedef enum {
    CMD_TYPE_NONE,
    CMD_TYPE_TARGET,
    CMD_TYPE_SERVO
} CommandType;

/* 命令结构体 */
typedef struct {
    CommandType type;
    union {
        struct {
            int x;
            int y;
            bool valid;
        } target;
        struct {
            int pan;
            int tilt;
        } servo;
    } data;
} Command;

/* 轮子控制器结构体 */
typedef struct {
    UART_HandleTypeDef *huart;  /* UART句柄(由main.c配置) */
    uint8_t rx_buffer[256];     /* 接收缓冲区 */
    uint16_t rx_len;            /* 缓冲区长度 */
    uint32_t last_cmd_time;     /* 上次命令时间(毫秒) */
} WheelController;

/* 函数声明 */

/**
 * 初始化轮子控制器
 * @param huart UART句柄(由HAL_UART_Init()初始化)
 * @return WheelController结构体指针，失败返回NULL
 */
WheelController* wheel_controller_init(UART_HandleTypeDef *huart);

/**
 * 释放轮子控制器资源
 * @param controller 控制器指针
 */
void wheel_controller_free(WheelController *controller);

/**
 * 解析来自K230的命令
 * 格式: $TARGET,x,y,valid\n  (目标坐标)
 * 或    $SERVO,pan,tilt\n     (云台角度)
 * @param data 接收到的数据
 * @return 解析后的命令结构体
 */
Command wheel_controller_parse_command(const uint8_t *data, uint16_t len);

/**
 * 读取来自K230的命令
 * @param controller 控制器指针
 * @param timeout_ms 超时时间(毫秒)
 * @return 接收到的命令
 */
Command wheel_controller_read_command(WheelController *controller, uint32_t timeout_ms);

/**
 * 接收中断回调函数(需要在usart_irq_handler()中调用)
 * @param controller 控制器指针
 * @param byte 接收到的字节
 */
void wheel_controller_uart_irq_handler(WheelController *controller, uint8_t byte);

/**
 * 根据目标坐标计算轮子速度
 * @param target_x 目标X坐标
 * @param target_y 目标Y坐标
 * @param screen_width 屏幕宽度
 * @param screen_height 屏幕高度
 * @param max_speed 最大速度
 * @param left_speed 输出左轮速度
 * @param right_speed 输出右轮速度
 */
void wheel_controller_calculate_speed(int target_x, int target_y,
                                      int screen_width, int screen_height,
                                      uint8_t max_speed,
                                      uint8_t *left_speed, uint8_t *right_speed);

/**
 * 控制电机速度
 * @param left_speed 左轮速度(0-255)
 * @param right_speed 右轮速度(0-255)
 */
void wheel_controller_control_motors(uint8_t left_speed, uint8_t right_speed);

/**
 * 主控制循环
 * @param controller 控制器指针
 */
void wheel_controller_main_loop(WheelController *controller);

#endif /* WHEEL_CONTROLLER_H */
