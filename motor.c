#include "motor.h"
#include "stm32f1xx_hal.h"
#include <math.h>

// 全局电机句柄（用于编码器中断）
Motor_HandleTypeDef *g_motor_handle = NULL;

// 电机初始化
void Motor_Init(Motor_HandleTypeDef *hmotor, TIM_HandleTypeDef *htim) {
    hmotor->htim = htim;
    hmotor->speed_left = 0;
    hmotor->speed_right = 0;
    hmotor->encoder_left = 0;
    hmotor->encoder_right = 0;
    
    // 保存全局句柄用于中断
    g_motor_handle = hmotor;
    
    // 初始化GPIO方向引脚
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能GPIO时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // 配置方向引脚
    GPIO_InitStruct.Pin = MOTOR_A_DIR_PIN | MOTOR_B_DIR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 初始方向设置为正转
    HAL_GPIO_WritePin(GPIOA, MOTOR_A_DIR_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, MOTOR_B_DIR_PIN, GPIO_PIN_SET);
}

// 电机控制（根据PID输出）
void Motor_Control(Motor_HandleTypeDef *hmotor, float output) {
    // 死区处理
    if (fabs(output) < DEAD_ZONE) {
        output = 0;
    }
    
    // 限制输出范围
    if (output > MAX_OUTPUT) {
        output = MAX_OUTPUT;
    } else if (output < -MAX_OUTPUT) {
        output = -MAX_OUTPUT;
    }
    
    // 转换为电机速度
    int16_t speed = (int16_t)output;
    
    // 设置左右电机速度（相同）
    Motor_SetSpeed(hmotor, speed, speed);
}

// 设置电机速度
void Motor_SetSpeed(Motor_HandleTypeDef *hmotor, int16_t left_speed, int16_t right_speed) {
    // 限制速度范围
    if (left_speed > MAX_OUTPUT) left_speed = MAX_OUTPUT;
    if (left_speed < -MAX_OUTPUT) left_speed = -MAX_OUTPUT;
    if (right_speed > MAX_OUTPUT) right_speed = MAX_OUTPUT;
    if (right_speed < -MAX_OUTPUT) right_speed = -MAX_OUTPUT;
    
    hmotor->speed_left = left_speed;
    hmotor->speed_right = right_speed;
    
    // 设置左电机方向和PWM
    if (left_speed >= 0) {
        HAL_GPIO_WritePin(GPIOA, MOTOR_A_DIR_PIN, GPIO_PIN_SET); // 正转
        __HAL_TIM_SET_COMPARE(hmotor->htim, TIM_CHANNEL_1, left_speed);
    } else {
        HAL_GPIO_WritePin(GPIOA, MOTOR_A_DIR_PIN, GPIO_PIN_RESET); // 反转
        __HAL_TIM_SET_COMPARE(hmotor->htim, TIM_CHANNEL_1, -left_speed);
    }
    
    // 设置右电机方向和PWM
    if (right_speed >= 0) {
        HAL_GPIO_WritePin(GPIOA, MOTOR_B_DIR_PIN, GPIO_PIN_SET); // 正转
        __HAL_TIM_SET_COMPARE(hmotor->htim, TIM_CHANNEL_2, right_speed);
    } else {
        HAL_GPIO_WritePin(GPIOA, MOTOR_B_DIR_PIN, GPIO_PIN_RESET); // 反转
        __HAL_TIM_SET_COMPARE(hmotor->htim, TIM_CHANNEL_2, -right_speed);
    }
}

// 停止电机
void Motor_Stop(Motor_HandleTypeDef *hmotor) {
    Motor_SetSpeed(hmotor, 0, 0);
}

// 获取编码器值
int32_t Motor_GetEncoderLeft(Motor_HandleTypeDef *hmotor) {
    return hmotor->encoder_left;
}

int32_t Motor_GetEncoderRight(Motor_HandleTypeDef *hmotor) {
    return hmotor->encoder_right;
}

// 重置编码器
void Motor_ResetEncoders(Motor_HandleTypeDef *hmotor) {
    hmotor->encoder_left = 0;
    hmotor->encoder_right = 0;
    
    // 重置硬件计数器
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

// 编码器中断回调函数
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (g_motor_handle == NULL) return;
    
    // 左编码器（TIM2）
    if (htim->Instance == TIM2) {
        int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(htim);
        g_motor_handle->encoder_left += count;
        __HAL_TIM_SET_COUNTER(htim, 0);
    }
    
    // 右编码器（TIM3）
    if (htim->Instance == TIM3) {
        int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(htim);
        g_motor_handle->encoder_right += count;
        __HAL_TIM_SET_COUNTER(htim, 0);
    }
}