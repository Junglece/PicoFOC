# STM32F103 FOC Motor Control Firmware

基于 STM32F103C8 的磁场定向控制（FOC）无刷电机驱动固件。

Field-Oriented Control (FOC) firmware for BLDC/PMSM motors on STM32F103C8.

---

## 概述 / Overview

本项目实现了一套完整的磁场定向控制（FOC）算法，运行在 STM32F103C8T6（Cortex-M3）上，驱动无刷直流电机（BLDC/PMSM）。核心特点：

- **四层分层架构**：LIB（纯数学）→ BSP（接口定义+实现）→ DRV（外设驱动封装）→ APP（业务逻辑编排）
- **依赖注入模式**：通过函数指针接口（Observer_t、AngleSensor_t、PwmOutput_t、NvStorage_t）实现硬件解耦
- **双通信协议**：CAN（标准帧，1Mbps）+ UART（自定义帧 + CRC16-CCITT）
- **双观测器**：差分观测器 + 龙伯格观测器可选
- **无 RTOS**：纯 TIM1 中断驱动，1kHz 控制频率

This project implements a complete FOC algorithm on STM32F103C8T6 (Cortex-M3) for BLDC/PMSM motors. Features include:

- **4-layer architecture**: LIB (pure math) → BSP (interface + implementation) → DRV (peripheral wrappers) → APP (business logic)
- **Dependency injection pattern**: Hardware decoupling via function pointer interfaces (Observer_t, AngleSensor_t, PwmOutput_t, NvStorage_t)
- **Dual communication**: CAN (standard frame, 1Mbps) + UART (custom frame + CRC16-CCITT)
- **Dual observers**: Difference observer + Luenberger observer
- **No RTOS**: Bare-metal TIM1 interrupt driven, 1kHz control rate

---

## 硬件要求 / Hardware Requirements

| 组件 / Component | 型号 / Model | 说明 / Notes |
|---|---|---|
| **MCU** | STM32F103C8T6 | Blue Pill 或自定义板 |
| **电机驱动** | 3 相逆变器 | DRV8301、L6234 或分立 MOS 桥 |
| **电机** | BLDC / PMSM | 带霍尔或编码器 |
| **编码器** | AS5600 | 12 位 I2C 磁编码器，地址 0x6C |
| **CAN 收发器** | SN65HVD230 | 或兼容的 3.3V CAN 收发器 |
| **调试** | ST-Link | 编程 + 调试 |

### 引脚定义 / Pinout

| 引脚 / Pin | 功能 / Function | 说明 / Notes |
|---|---|---|
| PA0, PA1, PA2 | ADC1_IN0~2 | 电流采样（预留，当前未使用） |
| PA5 | LED | 状态指示灯（PWM 亮度 + 闪烁模式） |
| PA6, PA7, PB0 | TIM3_CH1~3 | 三相 PWM 输出，中心对齐模式 |
| PA9, PA10 | USART1_TX/RX | 串口通信（VOFA+ just-float 调试协议） |
| PA11, PA12 | CAN_RX/TX | CAN 通信（1Mbps） |
| PB1 | GPIO OUT | 电机驱动使能（高电平有效） |
| PB6, PB7 | I2C1_SCL/SDA | AS5600 磁编码器接口 |

---

## 构建指南 / Build Instructions

### 前置条件 / Prerequisites

1. **Keil MDK-ARM 5.x**（推荐 5.38 或更新版本）
2. **ARM Compiler V5.06 update 6 (build 750)**（AC5 模式）
3. **STM32F1xx 器件包**（Keil Pack Installer 中安装）

### 步骤 / Steps

1. 打开 `MDK-ARM/FOC.uvprojx`
2. 在 Project 窗口确认所有源文件已加载
3. 点击 **Build (F7)** 或 **Rebuild All**
4. 预期结果：`0 Error(s), 0 Warning(s)`

---

## 使用方法 / Usage

### 上电流程 / Startup Sequence

1. 上电后 LED 呈心跳闪烁 → 系统初始化完成
2. 发送 **校准命令**（见下方协议）→ 电机注入 Ud=4V 对齐转子，LED 快闪
3. 校准完成后自动进入 **待机模式**，LED 心跳恢复
4. 发送 **位置/速度模式命令** → LED 根据扭矩实时调节亮度

### 状态机 / State Machine

```
POWER_ON → CALIBRATE → STANDBY → RUNNING (position/speed mode)
                ↑          │
                └──────────┘ (abort or complete)
```

---

## 通信协议 / Communication Protocol

### CAN 协议 / CAN Protocol

| 参数 | 值 |
|---|---|
| 波特率 | 1 Mbps |
| 帧格式 | 标准帧，8 字节数据 |
| 控制帧 ID | `0x000 + motor_id` |
| 状态帧 ID | `0x100 + motor_id` |

**8 字节 Payload 格式：**

| 字节 | 内容 | 类型 |
|------|------|------|
| [0] | 模式 (MODE) | uint8 |
| [1-4] | 目标值 (target) | float32 LE |
| [5-6] | Kp | uint16 LE |
| [7] | Kd / Ki | uint8 |

**模式定义：**

| 值 | 模式 | 说明 |
|----|------|------|
| 0x00 | STANDBY | 待机 |
| 0x01 | SPEED | 速度模式 |
| 0x03 | POSITION | 位置模式（角度控制） |
| 0x04 | CALIBRATE | 校准 |

### UART 协议 / UART Protocol

| 参数 | 值 |
|---|---|
| 波特率 | 115200 baud |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验 | 无 |

**帧格式：**

```
[0xAA] [0x08] [8 bytes payload] [CRC16-CCITT LE]
```

- 8 字节 payload 格式与 CAN 协议相同
- CRC16-CCITT（多项式 0x1021），低位在前

**示例 / Examples：**

```
待机 (Standby)    : AA 08 00 00 00 00 00 00 80 00 A6 9A
位置 0° (Position): AA 08 03 00 00 00 00 00 80 00 D3 52
位置 10°          : AA 08 03 C2 B8 32 3E 00 80 00 15 74
校准 (Calibrate)  : AA 08 04 00 00 00 00 00 80 00 CB 95
```

### VOFA+ 调试协议

UART 同时支持 VOFA+ just-float 协议，用于实时波形调试：
- 每次 FOC 控制周期输出 6 个 float（电流 Id/Iq，角度，速度，目标值等）
- 使用 VOFA+ 上位机可直接查看波形

---

## 项目架构 / Architecture

```
FOC/
├── Lib/          纯数学算法，零硬件依赖
│   ├── pid.c/h           PID 控制器
│   ├── foc_math.c/h      坐标变换（Clarke/Park）
│   ├── observer_diff.c/h  差分观测器
│   ├── observer_luenberger.c/h  龙伯格观测器
│   └── data_process.c/h  角度处理工具
│
├── Bsp/          板级支持包（接口定义 + 硬件实现）
│   ├── angle_sensor.h    角度传感器抽象接口
│   ├── angle_as5600.c/h  AS5600 编码器实现
│   ├── pwm_output.h      三相 PWM 抽象接口
│   ├── pwm_3phase.c/h    TIM3 PWM 实现
│   ├── nv_storage.h      非易失存储抽象接口
│   ├── flash_storage.c/h 片内 Flash 存储实现
│   ├── observer.h        观测器抽象接口
│   └── motor_config.h    电机参数配置
│
├── Drv/          驱动层（HAL 外设封装）
│   ├── drv_tim.c/h       定时器时基驱动
│   ├── drv_can.c/h       CAN 原始收发
│   ├── drv_adc.c/h       ADC DMA 电流采样（预留）
│   └── led_indicator.c/h LED PWM + 模式状态机
│
├── App/          应用层（业务逻辑编排）
│   ├── Foc.c/h           FOC 控制器
│   ├── motor_ctrl.c/h    控制环状态机
│   ├── motor_msg.c/h     消息总线（编解码）
│   ├── can_proto.c/h     CAN 协议层
│   └── uart_proto.c/h    UART 协议层（含 CRC16 + VOFA+）
│
├── Core/          CubeMX 生成代码
│   ├── Src/main.c         主函数（用户代码段）
│   ├── Src/*.c            外设初始化代码
│   └── Inc/*.h            外设头文件
│
├── Drivers/       STM32 HAL + CMSIS（第三方，标准库）
│
├── MDK-ARM/       Keil 项目文件
│
└── Tools/         辅助脚本
    ├── gen_uart_cmd.ps1   UART 命令帧生成（PowerShell）
    ├── gen_uart_cmd.py    UART 命令帧生成（Python）
    └── gen_uart_cmd.js    UART 命令帧生成（Node.js）
```

---

## 许可证 / License

MIT License — 详见 [LICENSE](LICENSE) 文件。

本项目包含的第三方代码：
- **STM32 HAL Driver**: BSD-3-Clause © STMicroelectronics
- **CMSIS**: Apache 2.0 © ARM Limited

---

## 相关文档 / Related

- `Tools/gen_uart_cmd.ps1` — 生成带正确 CRC 的 UART 命令帧
- 协议详情见 [通信协议](#通信协议--communication-protocol)
- 如有疑问请提交 Issue
