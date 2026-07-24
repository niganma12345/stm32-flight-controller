# CLAUDE.md

## 重要规则

### 1. 永远不能更改 CubeMX 生成的文件

CubeMX 生成的文件在下次重新生成配置时会被覆盖，直接修改会导致改动丢失。这些文件包括但不限于：

- `Core/Inc/` 和 `Core/Src/` 下的 `main`、`gpio`、`spi`、`usart`、`tim`、`adc`、`dma`、`i2c`、`stm32f1xx_it`、`stm32f1xx_hal_msp`、`system_stm32f1xx`、`FreeRTOSConfig.h`
- `*.ioc` 文件

**需要修改时的做法：**
- 指导用户在 CubeMX 中修改对应配置
- 或将自定义代码放在 `/* USER CODE BEGIN */` … `/* USER CODE END */` 区域
- 或将逻辑写入 `Drivers/` 下的独立文件中

### 2. 需求不明确时先讨论再动手

当需求存在歧义、有多种可行方案、或改动范围不清晰时，必须先与用户确认细节，再开始写代码。不要自行猜测用户意图。

---

## 项目简介

STM32F1 四轴飞行器，包含两个子项目：
- `flight/` — 飞控（主控板），运行 FreeRTOS
- `remote/` — 遥控器

## 开发工具

- **IDE/编译器**：Keil MDK-ARM（位于 `D:\Keil5\UV4\UV4.exe`）
- **配置工具**：STM32CubeMX（`.ioc` 文件）
- **调试/烧录**：ST-Link

## 代码分层（飞控）

| 层级 | 目录 | 说明 |
|------|------|------|
| 硬件抽象 | `Core/` | CubeMX 生成的外设配置 |
| 驱动层 | `Drivers/BSP/` | 传感器和外设驱动 |
| 计算层 | `Drivers/COM/` | PID、滤波、光流计算、高度融合、姿态解算 |
| 应用层 | `Drivers/APP/` | 飞控任务、遥控数据接收 |

## 传感器

- MPU6050（IMU 姿态）
- BMP280（气压高度）
- PMW3901（光流传感器，水平位移）
- VL53L1X（激光测距，低空高度）
- NRF24L01（无线通信）
- 蓝牙串口模块
