#include "communication.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>

// 全局通信句柄
Communication_HandleTypeDef *g_comm_handle = NULL;

// 通信初始化
void Communication_Init(Communication_HandleTypeDef *hcomm, UART_HandleTypeDef *huart) {
    hcomm->huart = huart;
    hcomm->rx_index = 0;
    hcomm->current_cmd = CMD_NONE;
    hcomm->cmd_value = 0.0f;
    
    // 清空缓冲区
    memset(hcomm->rx_buffer, 0, RX_BUFFER_SIZE);
    memset(hcomm->tx_buffer, 0, TX_BUFFER_SIZE);
    
    // 保存全局句柄
    g_comm_handle = hcomm;
    
    // 启动接收中断
    HAL_UART_Receive_IT(huart, &hcomm->rx_buffer[0], 1);
    
    // 发送欢迎信息
    Communication_SendString("STM32平衡小车通信就绪\r\n");
}

// 发送传感器数据
void Communication_SendData(float angle, float output) {
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "Angle:%.2f, Output:%.2f\r\n", angle, output);
    
    if (len > 0) {
        HAL_UART_Transmit(g_comm_handle->huart, (uint8_t*)buffer, len, 100);
    }
}

// 发送字符串
void Communication_SendString(const char *str) {
    HAL_UART_Transmit(g_comm_handle->huart, (uint8_t*)str, strlen(str), 100);
}

// 检查是否有命令
uint8_t Communication_HasCommand(void) {
    return (g_comm_handle->current_cmd != CMD_NONE);
}

// 处理命令
void Communication_ProcessCommand(PID_HandleTypeDef *hpid, float *target_angle) {
    if (g_comm_handle->current_cmd == CMD_NONE) {
        return;
    }
    
    switch (g_comm_handle->current_cmd) {
        case CMD_SET_KP:
            hpid->kp = g_comm_handle->cmd_value;
            Communication_SendString("KP参数已更新\r\n");
            break;
            
        case CMD_SET_KI:
            hpid->ki = g_comm_handle->cmd_value;
            Communication_SendString("KI参数已更新\r\n");
            break;
            
        case CMD_SET_KD:
            hpid->kd = g_comm_handle->cmd_value;
            Communication_SendString("KD参数已更新\r\n");
            break;
            
        case CMD_SET_ANGLE:
            *target_angle = g_comm_handle->cmd_value;
            Communication_SendString("目标角度已更新\r\n");
            break;
            
        case CMD_GET_STATUS:
            {
                char status[128];
                snprintf(status, sizeof(status), 
                        "KP:%.2f, KI:%.2f, KD:%.2f, Target:%.2f\r\n", 
                        hpid->kp, hpid->ki, hpid->kd, *target_angle);
                Communication_SendString(status);
            }
            break;
            
        case CMD_RESET:
            PID_Reset(hpid);
            Communication_SendString("PID控制器已重置\r\n");
            break;
            
        default:
            break;
    }
    
    // 清除命令
    g_comm_handle->current_cmd = CMD_NONE;
}

// 解析命令
void ParseCommand(const char *cmd) {
    if (g_comm_handle == NULL) return;
    
    char param[32];
    float value;
    
    if (sscanf(cmd, "set kp %f", &value) == 1) {
        g_comm_handle->current_cmd = CMD_SET_KP;
        g_comm_handle->cmd_value = value;
    }
    else if (sscanf(cmd, "set ki %f", &value) == 1) {
        g_comm_handle->current_cmd = CMD_SET_KI;
        g_comm_handle->cmd_value = value;
    }
    else if (sscanf(cmd, "set kd %f", &value) == 1) {
        g_comm_handle->current_cmd = CMD_SET_KD;
        g_comm_handle->cmd_value = value;
    }
    else if (sscanf(cmd, "set angle %f", &value) == 1) {
        g_comm_handle->current_cmd = CMD_SET_ANGLE;
        g_comm_handle->cmd_value = value;
    }
    else if (strcmp(cmd, "get status") == 0) {
        g_comm_handle->current_cmd = CMD_GET_STATUS;
    }
    else if (strcmp(cmd, "reset") == 0) {
        g_comm_handle->current_cmd = CMD_RESET;
    }
    else {
        Communication_SendString("未知命令\r\n");
    }
}

// UART接收完成回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (g_comm_handle == NULL || huart != g_comm_handle->huart) {
        return;
    }
    
    uint8_t received_char = g_comm_handle->rx_buffer[g_comm_handle->rx_index];
    
    // 处理回车或换行符
    if (received_char == '\r' || received_char == '\n') {
        if (g_comm_handle->rx_index > 0) {
            // 添加字符串结束符
            g_comm_handle->rx_buffer[g_comm_handle->rx_index] = '\0';
            
            // 解析命令
            ParseCommand((char*)g_comm_handle->rx_buffer);
            
            // 清空缓冲区
            g_comm_handle->rx_index = 0;
            memset(g_comm_handle->rx_buffer, 0, RX_BUFFER_SIZE);
        }
    }
    else if (received_char == '\b' || received_char == 127) { // 退格键
        if (g_comm_handle->rx_index > 0) {
            g_comm_handle->rx_index--;
        }
    }
    else if (g_comm_handle->rx_index < RX_BUFFER_SIZE - 1) {
        g_comm_handle->rx_index++;
    }
    else {
        // 缓冲区满，清空
        g_comm_handle->rx_index = 0;
        memset(g_comm_handle->rx_buffer, 0, RX_BUFFER_SIZE);
        Communication_SendString("缓冲区已满，已清空\r\n");
    }
    
    // 继续接收
    HAL_UART_Receive_IT(huart, &g_comm_handle->rx_buffer[g_comm_handle->rx_index], 1);
}

// UART错误回调
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (g_comm_handle != NULL && huart == g_comm_handle->huart) {
        // 清除错误标志并重新启动接收
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
        HAL_UART_Receive_IT(huart, &g_comm_handle->rx_buffer[g_comm_handle->rx_index], 1);
    }
}