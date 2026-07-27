# CLAUDE.md

## 重要规则

### 0. 修改代码前先 git 备份

每次修改任何代码文件之前，先执行 `git add -A && git commit -m "备份: <简短描述>"` 将当前状态提交到 git，确保随时可回滚。

### 1. 永远不能更改 CubeMX 生成的文件

CubeMX 生成的文件（`Core/` 下除 `USER CODE` 区域外、`*.ioc`）会被 CubeMX 覆盖。需要修改时指导用户在 CubeMX 中操作，或将代码写入 `Drivers/` 独立文件。

### 2. 需求不明确时先讨论再动手

当需求存在歧义、有多种可行方案、或改动范围不清晰时，必须先与用户确认细节，再开始写代码。不要自行猜测用户意图。

---

## 项目简介

STM32F1 四轴飞行器：`flight/` 飞控（FreeRTOS），`remote/` 遥控器。

- **IDE**：`D:\Keil5\UV4\UV4.exe`（Keil MDK-ARM），**配置**：STM32CubeMX，**烧录**：ST-Link
- **分层**：`Core/` 硬件抽象 → `Drivers/BSP/` 驱动 → `Drivers/COM/` 计算（PID/滤波/光流/高度融合/姿态）→ `Drivers/APP/` 应用（飞控任务/遥控接收）
- **传感器**：MPU6050、BMP280、PMW3901、VL53L1X、NRF24L01、蓝牙串口
