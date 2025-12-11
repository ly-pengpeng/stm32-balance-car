#ifndef PINS_H
#define PINS_H

#include "stm32f1xx_hal.h"

// MPU6050 I2C引脚 (PB6-SCL, PB7-SDA)
#define MPU6050_I2C                I2C1
#define MPU6050_ADDR               0x68

// 电机PWM引脚 (TIM1 CH1-CH4)
#define MOTOR_A_PWM_PIN            GPIO_PIN_8   // PA8 - 左电机PWM
#define MOTOR_A_DIR_PIN            GPIO_PIN_9   // PA9 - 左电机方向
#define MOTOR_B_PWM_PIN            GPIO_PIN_10  // PA10 - 右电机PWM
#define MOTOR_B_DIR_PIN            GPIO_PIN_11  // PA11 - 右电机方向
#define MOTOR_TIM                  TIM1

// 编码器引脚 (TIM2/TIM3)
#define ENCODER_A_PIN_A            GPIO_PIN_0   // PA0 - 左编码器A
#define ENCODER_A_PIN_B            GPIO_PIN_1   // PA1 - 左编码器B
#define ENCODER_B_PIN_A            GPIO_PIN_6   // PA6 - 右编码器A
#define ENCODER_B_PIN_B            GPIO_PIN_7   // PA7 - 右编码器B
#define ENCODER_A_TIM              TIM2
#define ENCODER_B_TIM              TIM3

// 串口调试引脚 (USART1)
#define USART_DEBUG                USART1
#define USART_TX_PIN               GPIO_PIN_9   // PA9
#define USART_RX_PIN               GPIO_PIN_10  // PA10

// LED指示灯
#define LED_PIN                    GPIO_PIN_13  // PC13

// 按键引脚
#define BUTTON_PIN                 GPIO_PIN_0   // PA0

#endif