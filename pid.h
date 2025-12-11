#ifndef PID_H
#define PID_H

#include "stm32f1xx_hal.h"
#include "parameters.h"

// PID控制器结构体
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    
    float setpoint;     // 目标值
    float integral;     // 积分项
    float prev_error;   // 上一次误差
    
    float output;       // 输出值
    float output_min;   // 输出最小值
    float output_max;   // 输出最大值
    
    uint32_t last_time; // 上一次计算时间
    
} PID_HandleTypeDef;

// 函数声明
void PID_Init(PID_HandleTypeDef *hpid, float kp, float ki, float kd);
void PID_SetLimits(PID_HandleTypeDef *hpid, float min, float max);
float PID_Calculate(PID_HandleTypeDef *hpid, float setpoint, float input);
void PID_Reset(PID_HandleTypeDef *hpid);
void PID_SetTunings(PID_HandleTypeDef *hpid, float kp, float ki, float kd);

#endif