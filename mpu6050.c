#include "mpu6050.h"
#include "stm32f1xx_hal.h"
#include <math.h>

// 转换系数
#define ACCEL_SCALE 16384.0f  // ±2g范围
#define GYRO_SCALE 131.0f     // ±250°/s范围
#define RAD_TO_DEG 57.29578f  // 弧度转角度

// MPU6050初始化
uint8_t MPU6050_Init(MPU6050_HandleTypeDef *hmpu, I2C_HandleTypeDef *hi2c) {
    hmpu->hi2c = hi2c;
    
    // 检查设备ID
    uint8_t whoami = MPU6050_ReadByte(MPU6050_RA_WHO_AM_I);
    if (whoami != 0x68) {
        return 0; // 设备未连接
    }
    
    // 唤醒MPU6050
    MPU6050_WriteByte(MPU6050_RA_PWR_MGMT_1, 0x00);
    HAL_Delay(100);
    
    // 配置陀螺仪量程 ±250°/s
    MPU6050_WriteByte(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_250);
    
    // 配置加速度计量程 ±2g
    MPU6050_WriteByte(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_2);
    
    // 初始化变量
    hmpu->gyroXoffset = 0;
    hmpu->gyroYoffset = 0;
    hmpu->gyroZoffset = 0;
    hmpu->angleX = 0;
    hmpu->angleY = 0;
    hmpu->lastUpdate = HAL_GetTick();
    
    // 校准陀螺仪
    MPU6050_Calibrate(hmpu, 1000);
    
    return 1; // 初始化成功
}

// 读取传感器数据
void MPU6050_ReadData(MPU6050_HandleTypeDef *hmpu) {
    uint8_t buffer[14];
    
    // 读取加速度计和陀螺仪数据
    MPU6050_ReadBytes(MPU6050_RA_ACCEL_XOUT_H, buffer, 14);
    
    // 解析加速度数据
    hmpu->accelX = (int16_t)((buffer[0] << 8) | buffer[1]);
    hmpu->accelY = (int16_t)((buffer[2] << 8) | buffer[3]);
    hmpu->accelZ = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    // 解析陀螺仪数据
    hmpu->gyroX = (int16_t)((buffer[8] << 8) | buffer[9]);
    hmpu->gyroY = (int16_t)((buffer[10] << 8) | buffer[11]);
    hmpu->gyroZ = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    // 计算角度（使用加速度计）
    float accelX_g = hmpu->accelX / ACCEL_SCALE;
    float accelY_g = hmpu->accelY / ACCEL_SCALE;
    float accelZ_g = hmpu->accelZ / ACCEL_SCALE;
    
    // 计算俯仰角和横滚角
    hmpu->angleX = atan2(accelY_g, accelZ_g) * RAD_TO_DEG;
    hmpu->angleY = atan2(-accelX_g, sqrt(accelY_g * accelY_g + accelZ_g * accelZ_g)) * RAD_TO_DEG;
    
    // 计算角速度（去除偏移）
    hmpu->gyroX = (hmpu->gyroX / GYRO_SCALE) - hmpu->gyroXoffset;
    hmpu->gyroY = (hmpu->gyroY / GYRO_SCALE) - hmpu->gyroYoffset;
}

// 陀螺仪校准
void MPU6050_Calibrate(MPU6050_HandleTypeDef *hmpu, uint16_t samples) {
    float sumX = 0, sumY = 0, sumZ = 0;
    
    for (uint16_t i = 0; i < samples; i++) {
        MPU6050_ReadData(hmpu);
        sumX += hmpu->gyroX;
        sumY += hmpu->gyroY;
        sumZ += hmpu->gyroZ;
        HAL_Delay(5);
    }
    
    hmpu->gyroXoffset = sumX / samples;
    hmpu->gyroYoffset = sumY / samples;
    hmpu->gyroZoffset = sumZ / samples;
}

// 获取角度数据
float MPU6050_GetAngleX(MPU6050_HandleTypeDef *hmpu) {
    return hmpu->angleX;
}

float MPU6050_GetAngleY(MPU6050_HandleTypeDef *hmpu) {
    return hmpu->angleY;
}

// 获取角速度数据
float MPU6050_GetGyroX(MPU6050_HandleTypeDef *hmpu) {
    return hmpu->gyroX;
}

float MPU6050_GetGyroY(MPU6050_HandleTypeDef *hmpu) {
    return hmpu->gyroY;
}

// I2C写字节
uint8_t MPU6050_WriteByte(uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data};
    return HAL_I2C_Master_Transmit(MPU6050_I2C, MPU6050_ADDR << 1, buffer, 2, 100);
}

// I2C读字节
uint8_t MPU6050_ReadByte(uint8_t reg) {
    uint8_t data;
    HAL_I2C_Master_Transmit(MPU6050_I2C, MPU6050_ADDR << 1, &reg, 1, 100);
    HAL_I2C_Master_Receive(MPU6050_I2C, MPU6050_ADDR << 1, &data, 1, 100);
    return data;
}

// I2C读多个字节
void MPU6050_ReadBytes(uint8_t reg, uint8_t *data, uint8_t length) {
    HAL_I2C_Master_Transmit(MPU6050_I2C, MPU6050_ADDR << 1, &reg, 1, 100);
    HAL_I2C_Master_Receive(MPU6050_I2C, MPU6050_ADDR << 1, data, length, 100);
}