# STM32F103C8T6 平衡小车项目

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![STM32](https://img.shields.io/badge/STM32-F103C8T6-blue)](https://www.st.com/)
[![Language](https://img.shields.io/badge/language-C-green)](https://en.wikipedia.org/wiki/C_(programming_language))

## 项目简介

这是一个基于STM32F103C8T6微控制器的自平衡小车项目，使用C语言和STM32 HAL库开发。项目实现了完整的双轮自平衡控制系统，包含MPU6050姿态传感器、PID控制算法、电机驱动和串口通信等功能。

## ✨ 特性

- 🚗 **双轮自平衡** - 基于PID控制算法的稳定平衡
- 📊 **实时姿态检测** - MPU6050六轴传感器数据采集
- 🔄 **卡尔曼滤波** - 传感器数据滤波和姿态解算
- ⚡ **高效电机控制** - PWM驱动支持TB6612FNG/L298N
- 📡 **串口通信** - 实时参数调整和状态监控
- 🔧 **模块化设计** - 清晰的代码结构和配置文件
- 📚 **完整文档** - 详细的接线说明和调试指南

## 📁 项目结构

```
平衡小车/
├── Makefile                    # 编译配置
├── STM32F103C8Tx_FLASH.ld      # 链接器脚本
├── README.md                   # 项目说明 (本文件)
├── config/                     # 配置文件
│   ├── parameters.h           # PID参数配置
│   └── pins.h                 # 引脚定义
├── include/                    # 头文件
│   ├── main.h                 # 主头文件
│   ├── mpu6050.h              # MPU6050驱动
│   ├── pid.h                  # PID控制器
│   ├── motor.h                # 电机驱动
│   ├── kalman.h               # 卡尔曼滤波
│   └── communication.h        # 通信功能
├── src/                        # 源文件
│   ├── main.c                 # 主程序
│   ├── mpu6050.c              # MPU6050实现
│   ├── pid.c                  # PID算法实现
│   ├── motor.c                # 电机控制
│   ├── kalman.c               # 卡尔曼滤波实现
│   ├── communication.c        # 通信实现
│   └── peripheral_init.c      # 外设初始化
└── docs/                       # 文档
    ├── wiring.md              # 详细接线说明
    └── tuning.md              # PID调试指南
```

## 🛠️ 硬件要求

| 组件 | 规格 | 数量 |
|------|------|------|
| **主控芯片** | STM32F103C8T6 (蓝桥杯开发板) | 1 |
| **姿态传感器** | MPU6050 六轴传感器 | 1 |
| **电机驱动** | TB6612FNG 或 L298N | 1 |
| **直流电机** | N20减速电机 (带编码器) | 2 |
| **电源** | 7.4V锂电池 | 1 |
| **车架** | 3D打印或自制车架 | 1套 |

## 🚀 快速开始

### 1. 环境准备

确保已安装以下工具：
- **ARM GCC工具链**: `arm-none-eabi-gcc`
- **Make工具**: GNU Make
- **烧录工具**: ST-Link或J-Link
- **串口工具**: PuTTY或Tera Term

### 2. 硬件连接

参考 [接线说明](docs/wiring.md) 完成硬件连接。

### 3. 编译项目

```bash
# 克隆项目
git clone https://github.com/your-username/stm32-balance-car.git
cd stm32-balance-car

# 编译项目
make all
```

### 4. 烧录程序

```bash
# 使用ST-Link烧录
make flash
```

### 5. 调试参数

参考 [PID调试指南](docs/tuning.md) 进行参数调整。

## 📖 详细文档

- [🔌 硬件接线说明](docs/wiring.md) - 完整的硬件连接指南
- [🎛️ PID参数调试](docs/tuning.md) - 参数调整方法和技巧

## 🔧 开发指南

### 代码架构

项目采用模块化设计，主要模块包括：

- **MPU6050驱动**: I2C通信和姿态数据读取
- **PID控制器**: 比例-积分-微分控制算法
- **电机控制**: PWM输出和编码器反馈
- **卡尔曼滤波**: 传感器数据滤波处理
- **通信模块**: 串口命令解析和数据传输

### 自定义配置

在 `config/parameters.h` 中修改PID参数：

```c
#define PID_KP 15.0f      // 比例系数
#define PID_KI 0.05f      // 积分系数  
#define PID_KD 0.1f       // 微分系数
#define MAX_OUTPUT 255    // 最大输出限制
```

### 串口命令

通过串口(115200波特率)发送命令：

```bash
set kp 15.0    # 设置比例系数
set ki 0.05    # 设置积分系数
set kd 0.1     # 设置微分系数
get status     # 获取当前状态
reset          # 重置控制器
```

## 🤝 贡献指南

欢迎提交Issue和Pull Request！

1. Fork本项目
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 打开Pull Request

## 📄 许可证

本项目采用 MIT 许可证 - 查看 [LICENSE](LICENSE) 文件了解详情。

## 🙏 致谢

感谢以下开源项目的参考：
- [立创开源硬件平台 - STM32平衡小车](https://oshwhub.com/)
- [CSDN博客 - STM32F103平衡小车](https://blog.csdn.net/)

## 📞 联系我们

如有问题或建议，请通过以下方式联系：
- 提交 [Issue](https://github.com/your-username/stm32-balance-car/issues)
- 发送邮件至: your-email@example.com

---

⭐ 如果这个项目对你有帮助，请给个Star支持一下！