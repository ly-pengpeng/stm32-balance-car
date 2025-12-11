#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"
#include "pins.h"
#include "parameters.h"

// 电机控制器结构体
typedef struct {
    TIM_HandleTypeDef *htim;        // PWM定时器句柄
    
    // 电机参数
    int16_t speed_left;             // 左电机速度
    int16_t speed_right;            // 右电机速度
    
    // 编码器值
    int32_t encoder_left;           // 左编码器计数
    int32_t encoder_right;          // 右编码器计数
    
} Motor_HandleTypeDef;

// 函数声明
void Motor_Init(Motor_HandleTypeDef *hmotor, TIM_HandleTypeDef *htim);
void Motor_Control(Motor_HandleTypeDef *hmotor, float output);
void Motor_SetSpeed(Motor_HandleTypeDef *hmotor, int16_t left_speed, int16_t right_speed);
void Motor_Stop(Motor_HandleTypeDef *hmotor);
int32_t Motor_GetEncoderLeft(Motor_HandleTypeDef *hmotor);
int32_t Motor_GetEncoderRight(Motor_HandleTypeDef *hmotor);
void Motor_ResetEncoders(Motor_HandleTypeDef *hmotor);

// 编码器中断回调函数
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif