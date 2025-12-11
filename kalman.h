#ifndef KALMAN_H
#define KALMAN_H

#include "stm32f1xx_hal.h"
#include "parameters.h"

// 卡尔曼滤波器结构体
typedef struct {
    float Q_angle;      // 过程噪声协方差（角度）
    float Q_gyro;       // 过程噪声协方差（陀螺仪）
    float R_angle;      // 测量噪声协方差
    
    float angle;        // 估计角度
    float bias;         // 估计偏差
    float rate;         // 无偏差的速率
    
    float P[2][2];      // 误差协方差矩阵
    float K[2];         // 卡尔曼增益
    float y;            // 角度差
    float S;            // 估计误差
    
    uint32_t last_time; // 上一次更新时间
    
} Kalman_HandleTypeDef;

// 函数声明
void Kalman_Init(Kalman_HandleTypeDef *hkalman);
float Kalman_Update(Kalman_HandleTypeDef *hkalman, float newAngle, float newRate);
void Kalman_SetAngle(Kalman_HandleTypeDef *hkalman, float angle);
float Kalman_GetRate(Kalman_HandleTypeDef *hkalman);

#endif