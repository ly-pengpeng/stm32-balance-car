#include "pid.h"
#include "stm32f1xx_hal.h"
#include <math.h>

// PID控制器初始化
void PID_Init(PID_HandleTypeDef *hpid, float kp, float ki, float kd) {
    hpid->kp = kp;
    hpid->ki = ki;
    hpid->kd = kd;
    
    hpid->setpoint = 0.0f;
    hpid->integral = 0.0f;
    hpid->prev_error = 0.0f;
    hpid->output = 0.0f;
    
    // 设置默认输出限制
    hpid->output_min = -MAX_OUTPUT;
    hpid->output_max = MAX_OUTPUT;
    
    hpid->last_time = HAL_GetTick();
}

// 设置输出限制
void PID_SetLimits(PID_HandleTypeDef *hpid, float min, float max) {
    hpid->output_min = min;
    hpid->output_max = max;
}

// PID计算
float PID_Calculate(PID_HandleTypeDef *hpid, float setpoint, float input) {
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - hpid->last_time) / 1000.0f; // 转换为秒
    
    if (dt <= 0) {
        return hpid->output; // 时间未变化，返回上次输出
    }
    
    hpid->last_time = current_time;
    hpid->setpoint = setpoint;
    
    // 计算误差
    float error = setpoint - input;
    
    // 比例项
    float proportional = hpid->kp * error;
    
    // 积分项（抗积分饱和）
    hpid->integral += error * dt;
    
    // 积分限幅
    if (hpid->integral > hpid->output_max / hpid->ki) {
        hpid->integral = hpid->output_max / hpid->ki;
    } else if (hpid->integral < hpid->output_min / hpid->ki) {
        hpid->integral = hpid->output_min / hpid->ki;
    }
    
    float integral_term = hpid->ki * hpid->integral;
    
    // 微分项
    float derivative = (error - hpid->prev_error) / dt;
    float derivative_term = hpid->kd * derivative;
    
    // 计算输出
    hpid->output = proportional + integral_term + derivative_term;
    
    // 输出限幅
    if (hpid->output > hpid->output_max) {
        hpid->output = hpid->output_max;
    } else if (hpid->output < hpid->output_min) {
        hpid->output = hpid->output_min;
    }
    
    // 保存误差用于下次计算
    hpid->prev_error = error;
    
    return hpid->output;
}

// 重置PID控制器
void PID_Reset(PID_HandleTypeDef *hpid) {
    hpid->integral = 0.0f;
    hpid->prev_error = 0.0f;
    hpid->output = 0.0f;
    hpid->last_time = HAL_GetTick();
}

// 设置PID参数
void PID_SetTunings(PID_HandleTypeDef *hpid, float kp, float ki, float kd) {
    hpid->kp = kp;
    hpid->ki = ki;
    hpid->kd = kd;
}