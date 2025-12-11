#include "main.h"
#include "stm32f1xx_hal.h"
#include "mpu6050.h"
#include "pid.h"
#include "motor.h"
#include "kalman.h"
#include "communication.h"
#include "pins.h"
#include "parameters.h"

// 全局变量
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;

MPU6050_HandleTypeDef hmpu;
PID_HandleTypeDef hpid;
Motor_HandleTypeDef hmotor;
Kalman_HandleTypeDef hkalman;

// 控制变量
float targetAngle = 0.0f;  // 目标平衡角度
float currentAngle = 0.0f; // 当前角度
float output = 0.0f;       // PID输出

uint32_t lastTime = 0;
const uint32_t sampleTime = 10; // 10ms采样周期

// 系统时钟配置
void SystemClock_Config(void);

// 外设初始化
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_USART1_UART_Init(void);

int main(void) {
  // HAL库初始化
  HAL_Init();
  SystemClock_Config();
  
  // 外设初始化
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  
  // 启动PWM和编码器
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  
  // 初始化各模块
  MPU6050_Init(&hmpu, &hi2c1);
  PID_Init(&hpid, PID_KP, PID_KI, PID_KD);
  Motor_Init(&hmotor, &htim1);
  Kalman_Init(&hkalman);
  Communication_Init(&huart1);
  
  // 等待传感器稳定
  HAL_Delay(1000);
  
  // 发送初始化完成信息
  Communication_SendString("STM32平衡小车初始化完成\r\n");
  
  while (1) {
    uint32_t currentTime = HAL_GetTick();
    
    // 定时控制循环
    if (currentTime - lastTime >= sampleTime) {
      lastTime = currentTime;
      
      // 读取传感器数据
      MPU6050_ReadData(&hmpu);
      
      // 使用卡尔曼滤波处理角度数据
      currentAngle = Kalman_Update(&hkalman, hmpu.angleX, hmpu.gyroX);
      
      // PID计算
      output = PID_Calculate(&hpid, targetAngle, currentAngle);
      
      // 电机控制
      Motor_Control(&hmotor, output);
      
      // 串口通信（调试信息）
      Communication_SendData(currentAngle, output);
      
      // 接收串口指令
      if (Communication_HasCommand()) {
        Communication_ProcessCommand(&hpid, &targetAngle);
      }
    }
    
    // 空闲时处理其他任务
    HAL_Delay(1);
  }
}

// 系统时钟配置 - 72MHz
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  
  // 配置HSE振荡器
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  // 配置系统时钟
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}