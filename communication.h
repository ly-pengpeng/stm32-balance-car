#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "stm32f1xx_hal.h"
#include "pid.h"

// 通信缓冲区大小
#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 128

// 命令类型定义
typedef enum {
    CMD_NONE = 0,
    CMD_SET_KP,
    CMD_SET_KI,
    CMD_SET_KD,
    CMD_SET_ANGLE,
    CMD_GET_STATUS,
    CMD_RESET
} CommandType;

// 通信控制器结构体
typedef struct {
    UART_HandleTypeDef *huart;      // UART句柄
    
    // 接收缓冲区
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    uint16_t rx_index;
    
    // 发送缓冲区
    uint8_t tx_buffer[TX_BUFFER_SIZE];
    
    // 命令处理
    CommandType current_cmd;
    float cmd_value;
    
} Communication_HandleTypeDef;

// 函数声明
void Communication_Init(Communication_HandleTypeDef *hcomm, UART_HandleTypeDef *huart);
void Communication_SendData(float angle, float output);
void Communication_SendString(const char *str);
uint8_t Communication_HasCommand(void);
void Communication_ProcessCommand(PID_HandleTypeDef *hpid, float *target_angle);

// 中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif