#include "kalman.h"
#include "stm32f1xx_hal.h"
#include <math.h>

// 卡尔曼滤波器初始化
void Kalman_Init(Kalman_HandleTypeDef *hkalman) {
    hkalman->Q_angle = Q_ANGLE;
    hkalman->Q_gyro = Q_GYRO;
    hkalman->R_angle = R_ANGLE;
    
    hkalman->angle = 0.0f;
    hkalman->bias = 0.0f;
    hkalman->rate = 0.0f;
    
    // 初始化误差协方差矩阵
    hkalman->P[0][0] = 0.0f;
    hkalman->P[0][1] = 0.0f;
    hkalman->P[1][0] = 0.0f;
    hkalman->P[1][1] = 0.0f;
    
    hkalman->last_time = HAL_GetTick();
}

// 卡尔曼滤波更新
float Kalman_Update(Kalman_HandleTypeDef *hkalman, float newAngle, float newRate) {
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - hkalman->last_time) / 1000.0f; // 转换为秒
    
    if (dt <= 0) {
        return hkalman->angle; // 时间未变化，返回上次估计
    }
    
    hkalman->last_time = current_time;
    
    // 预测步骤
    hkalman->rate = newRate - hkalman->bias;
    hkalman->angle += dt * hkalman->rate;
    
    // 更新误差协方差矩阵
    hkalman->P[0][0] += dt * (dt * hkalman->P[1][1] - hkalman->P[0][1] - hkalman->P[1][0] + hkalman->Q_angle);
    hkalman->P[0][1] -= dt * hkalman->P[1][1];
    hkalman->P[1][0] -= dt * hkalman->P[1][1];
    hkalman->P[1][1] += hkalman->Q_gyro * dt;
    
    // 计算卡尔曼增益
    hkalman->S = hkalman->P[0][0] + hkalman->R_angle;
    hkalman->K[0] = hkalman->P[0][0] / hkalman->S;
    hkalman->K[1] = hkalman->P[1][0] / hkalman->S;
    
    // 计算角度差
    hkalman->y = newAngle - hkalman->angle;
    
    // 更新估计
    hkalman->angle += hkalman->K[0] * hkalman->y;
    hkalman->bias += hkalman->K[1] * hkalman->y;
    
    // 更新误差协方差矩阵
    float P00_temp = hkalman->P[0][0];
    float P01_temp = hkalman->P[0][1];
    
    hkalman->P[0][0] -= hkalman->K[0] * P00_temp;
    hkalman->P[0][1] -= hkalman->K[0] * P01_temp;
    hkalman->P[1][0] -= hkalman->K[1] * P00_temp;
    hkalman->P[1][1] -= hkalman->K[1] * P01_temp;
    
    return hkalman->angle;
}

// 设置初始角度
void Kalman_SetAngle(Kalman_HandleTypeDef *hkalman, float angle) {
    hkalman->angle = angle;
}

// 获取无偏差的速率
float Kalman_GetRate(Kalman_HandleTypeDef *hkalman) {
    return hkalman->rate;
}