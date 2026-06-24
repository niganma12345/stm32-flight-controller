# STM32 四轴飞行器 (Quadcopter Flight Controller)

基于 STM32F103 的四轴飞行器项目，包含飞控和遥控器两部分。

## 项目结构

```
├── flight/          # 飞控端代码（STM32F103）
│   ├── Core/        # HAL 驱动 + 应用层
│   │   ├── Inc/     # 头文件
│   │   └── Src/     # 源文件（main, FreeRTOS, 外设驱动）
│   └── Drivers/BSP/ # 板级驱动（MPU6050, BMP280, NRF24L01, 电机, PID, 滤波）
│
└── remote/          # 遥控器端代码（STM32F103）
    ├── Core/        # HAL 驱动 + 应用层
    │   ├── Inc/
    │   └── Src/
    └── Drivers/BSP/ # 板级驱动（NRF24L01, MPU6050, 摇杆, 按键）
```

## 硬件

- **主控**: STM32F103C8T6
- **姿态传感器**: MPU6050 (6轴陀螺仪+加速度计)
- **气压计**: BMP280
- **无线通信**: NRF24L01 (2.4GHz)
- **电机**: 空心杯电机
- **RTOS**: FreeRTOS

## 功能

- 姿态解算与 PID 稳定控制
- 无线遥控（NRF24L01）
- 高度保持（BMP280 气压计）
- 电量检测与回传
- 状态回传（高度、电量、姿态等）

## 开发环境

- **IDE**: Keil MDK-ARM (UVision 5)
- **HAL 库**: STM32F1xx HAL Driver
- **RTOS**: FreeRTOS
- **代码生成**: STM32CubeMX

## 编译与烧录

1. 使用 Keil MDK 打开 `flight/` 或 `remote/` 下的 `.uvprojx` 工程文件
2. 编译（Build）生成 `.hex` 文件
3. 通过 ST-Link / J-Link 烧录到 STM32

## 更新日志

- 增加电源管理任务
- 新增电量回传、高度回传、状态回传
- 灯控任务分离
- 完成回传任务的封装
