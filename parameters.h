#ifndef PARAMETERS_H
#define PARAMETERS_H

// PID参数
#define PID_KP 15.0      // 比例系数
#define PID_KI 0.05      // 积分系数
#define PID_KD 0.1       // 微分系数

// 卡尔曼滤波参数
#define Q_ANGLE 0.001    // 过程噪声协方差
#define Q_GYRO 0.003     // 陀螺仪噪声协方差
#define R_ANGLE 0.03     // 测量噪声协方差

// 控制参数
#define MAX_OUTPUT 255   // 最大输出限制
#define DEAD_ZONE 2.0    // 死区范围（度）
#define SAMPLE_TIME 10   // 采样时间（毫秒）

// 安全参数
#define MAX_ANGLE 45.0   // 最大允许角度
#define MIN_VOLTAGE 6.0  // 最低工作电压

#endif