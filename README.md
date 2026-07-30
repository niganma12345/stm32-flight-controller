# STM32 四轴飞行器 (Quadcopter Flight Controller)

基于 STM32F103C8T6 的四轴飞行器项目，包含飞控端和遥控器端，通过 NRF24L01 2.4GHz 无线通信，运行 FreeRTOS 实时系统。

## 项目结构

```
├── flight/                # 飞控端（STM32F103C8T6, FreeRTOS）
│   ├── Core/              # HAL 硬件抽象层 + FreeRTOS 任务入口（CubeMX 生成）
│   ├── Drivers/
│   │   ├── BSP/           # 板级驱动（传感器、电机、无线、OLED）
│   │   ├── COM/           # 计算模块（姿态解算、PID、滤波、光流、高度融合）
│   │   └── APP/           # 应用任务（飞控主循环、遥控接收、OLED 显示）
│   ├── freeRTOS/          # FreeRTOS V202212.01 内核源码
│   ├── MDK-ARM/           # Keil MDK 工程文件
│   └── flight.ioc         # CubeMX 项目配置
│
└── remote/                # 遥控器端（STM32F103C8T6, FreeRTOS）
    ├── Core/              # HAL 层 + FreeRTOS 入口
    ├── Drivers/BSP/       # 板级驱动（摇杆、按键、NRF24L01、MPU6050、OLED）
    ├── freeRTOS/          # FreeRTOS 内核
    ├── MDK-ARM/           # Keil MDK 工程文件
    └── remote.ioc         # CubeMX 项目配置
```

### 分层架构

```
APP  (应用层)   → flight_task / nrf24l01_task / led_task / oled_task / power_mgmt_task
COM  (计算层)   → 姿态解算(Mahony) / 串级PID / 数字滤波 / 光流处理 / 高度融合
BSP  (驱动层)   → MPU6050 / PMW3901 / SPA06 / VL53L1X / QMC5883P / NRF24L01 / Motor / OLED
Core (HAL层)    → CubeMX 生成（外设初始化、时钟树、FreeRTOS 配置）
```

## 硬件配置

### 主控

| 参数 | 值 |
|---|---|
| MCU | STM32F103C8T6 (Cortex-M3) |
| 主频 | 72MHz (HSE 8MHz × PLL 9) |
| Flash / SRAM | 64KB / 20KB |

### 传感器与模块

| 传感器 | 型号 | 接口 | 用途 |
|---|---|---|---|
| 六轴惯性 | MPU6050 | I2C2 (0x68) | 姿态解算，陀螺仪 ±2000°/s，加计 ±2g，采样 500Hz |
| 光流 | PMW3901 | SPI2 | 水平位移/速度，160 CPI |
| 激光测距 | VL53L1X | I2C2 (0x52) | 低空定高，最远 4m，分辨率 1mm |
| 气压计 | SPA06 (DPS310) | I2C2 (0xEE) | 气压定高，300~1200hPa，16 倍过采样 |
| 磁力计 | QMC5883P | I2C2 (0x58) | 航向修正，±8Gauss，200Hz |
| 无线通信 | NRF24L01 | SPI1 | 2.4GHz，2Mbps，17 字节数据包 |
| 蓝牙串口 | BlueSerial | USART2 | 调试输出（printf） |
| OLED 显示 | SSD1306 | I2C2 (0x78) | 128×64，0.96 英寸，状态显示 |
| 电机 | 空心杯 ×4 | TIM1/2/3/4 | PWM 18kHz，X 型四旋翼布局 |

## 飞控功能

### 飞行模式

| 模式 | 说明 |
|---|---|
| LOCKED | 上电锁定，摇杆解锁后进入 IDLE |
| IDLE | 已解锁，待命状态 |
| NORMAL | 正常飞行，光流自稳（水平位置保持） |
| FIX_HEIGHT | 定高飞行（激光 + 气压计融合） |
| MANUAL | 手动角度自稳，不使用光流 |
| FAIL | 通信超时 150ms → 自动停转，安全保护 |

### 控制算法

- **姿态解算**：Mahony 互补滤波四元数法，融合 MPU6050 + QMC5883P，输出 Pitch/Roll/Yaw
- **PID 控制**：串级 PID（角度外环 P + 角速度内环 PD），控制周期 6ms
- **高度融合**：VL53L1X 激光 + SPA06 气压计 + MPU6050 Z 轴加速度 → 互补滤波
- **光流定位**：PMW3901 → 坐标映射 → 旋转补偿 → 高度补偿 → 物理速度
- **数字滤波**：一阶低通、卡尔曼滤波、滑动窗口延迟线（传感器时序对齐）
- **积分抗饱和**：输出饱和且误差同向时停止积分累积

### FreeRTOS 任务

| 任务 | 优先级 | 周期 | 功能 |
|---|---|---|---|
| nrf24l01_task | 3 (最高) | 6ms | NRF24L01 收发 + 飞行状态机 + 遥测回传 |
| flight_task | 2 | 6ms | 姿态 → PID → 电机输出 → 光流/高度融合 → OLED 队列投递 |
| led_task | 1 | 6ms | LED 状态指示（通信 + 飞行状态） |
| oled_task | 1 | 事件驱动 | OLED 队列消费渲染，约 100ms 刷新 |
| power_mgmt_task | 1 | 10s/事件 | 电源管理，按键关机（双击）/ 维持供电（单击） |

### PID 参数

| 控制回路 | Kp | Ki | Kd | 输出限幅 |
|---|---|---|---|---|
| 俯仰/横滚角外环 | 7.0 | 0 | 0 | ±30 °/s |
| 俯仰/横滚角内环 | -3.0 | 0 | -0.5 | ±200 |
| 偏航角内环 | 3.5 | 0 | 0 | ±100 |
| 高度位置外环 | 10.5 | 0.05 | 0 | ±2 m/s |
| 垂直速度内环 | 35.0 | 5.0 | 3.0 | ±100 油门 |
| 光流速度环 | -0.02 | 0 | 0.01 | ±15° |

## 遥控器功能

- 摇杆 ADC 采集 + 按键检测（消抖）
- NRF24L01 发送遥控指令（6ms 周期）
- 接收遥测回传并 OLED 显示
- FreeRTOS 多任务

## 数据流

```
遥控器 →(NRF24L01)→ nrf24l01_task → remote_data_queue → flight_task → PID/电机
                                                          ↓
                      遥测回传 ← telemetry_queue ← 姿态/高度/电量
```

## 开发环境

- **IDE**：Keil MDK-ARM (UVision 5)，路径 `D:\Keil5\UV4\UV4.exe`
- **HAL 库**：STM32F1xx HAL Driver
- **RTOS**：FreeRTOS V202212.01
- **配置工具**：STM32CubeMX
- **烧录**：ST-Link

## 编译与烧录

1. 用 Keil MDK 打开 `flight/MDK-ARM/` 或 `remote/MDK-ARM/` 下的 `.uvprojx` 工程
2. 编译（Build）生成 `.hex` 文件
3. 通过 ST-Link 烧录到 STM32

## 注意事项

- `Core/` 目录下的代码由 CubeMX 生成，不可手动修改（USER CODE 区域除外）
- 自定义代码全部写入 `Drivers/` 独立文件
- 修改外设配置请在 CubeMX 中操作后重新生成
