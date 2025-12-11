#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"
#include "pins.h"

// MPU6050寄存器地址定义
#define MPU6050_RA_WHO_AM_I         0x75
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_GYRO_XOUT_H      0x43

// 传感器量程设置
#define MPU6050_GYRO_FS_250         0x00  // ±250°/s
#define MPU6050_ACCEL_FS_2          0x00  // ±2g

// MPU6050数据结构体
typedef struct {
    I2C_HandleTypeDef *hi2c;        // I2C句柄
    
    // 原始数据
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
    
    // 校准数据
    float gyroXoffset, gyroYoffset, gyroZoffset;
    
    // 处理后的数据
    float angleX, angleY;           // 角度（度）
    float gyroX, gyroY;             // 角速度（°/s）
    
    // 时间戳
    uint32_t lastUpdate;
    
} MPU6050_HandleTypeDef;

// 函数声明
uint8_t MPU6050_Init(MPU6050_HandleTypeDef *hmpu, I2C_HandleTypeDef *hi2c);
void MPU6050_ReadData(MPU6050_HandleTypeDef *hmpu);
void MPU6050_Calibrate(MPU6050_HandleTypeDef *hmpu, uint16_t samples);
float MPU6050_GetAngleX(MPU6050_HandleTypeDef *hmpu);
float MPU6050_GetAngleY(MPU6050_HandleTypeDef *hmpu);
float MPU6050_GetGyroX(MPU6050_HandleTypeDef *hmpu);
float MPU6050_GetGyroY(MPU6050_HandleTypeDef *hmpu);

// 底层I2C通信函数
uint8_t MPU6050_WriteByte(uint8_t reg, uint8_t data);
uint8_t MPU6050_ReadByte(uint8_t reg);
void MPU6050_ReadBytes(uint8_t reg, uint8_t *data, uint8_t length);

#endif