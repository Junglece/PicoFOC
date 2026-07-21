# PicoFOC

**紧凑型 STM32F103 无刷电机 FOC 驱动平台**

A compact STM32F103 BLDC FOC motor drive platform — firmware, PCB, and enclosure.

![License: MIT](https://img.shields.io/badge/license-MIT-blue)

---

## 项目组成

```
PicoFOC/
├── firmware/      STM32F103 FOC 固件 (Keil MDK-ARM)
│   ├── App/       业务逻辑层
│   ├── Bsp/       板级支持包
│   ├── Drv/       驱动层
│   ├── Lib/       纯数学算法（零硬件依赖）
│   ├── Core/      CubeMX 生成代码
│   ├── Drivers/   STM32 HAL + CMSIS
│   ├── MDK-ARM/   Keil 项目文件
│   └── tools/     辅助脚本
│
├── hardware/      PCB 设计 (EasyEDA / 立创 EDA)
├── mechanical/    3D 打印外壳（适配 2804 电机）
├── LICENSE        MIT License
└── README.md
```

## 固件

四层架构：**LIB**（纯数学，零硬件依赖）→ **BSP**（接口 + 实现）→ **DRV**（HAL 封装）→ **APP**（业务逻辑）。

- 主控 STM32F103C8T6，无 RTOS，纯中断驱动，1 kHz 控制频率
- CAN 1 Mbps + UART 115200 双协议通信
- 双观测器方案可选：差分观测器 / 龙伯格观测器

详见 [firmware/README.md](firmware/README.md)。

## 硬件

- **主控**：STM32F103C8T6
- **驱动芯片**：MP6541（内置三相 MOS 功率桥、驱动电路与电流采样）
- **编码器**：AS5600（I2C，12 位磁编码器）
- **CAN 收发器**：TJA1051，单路 CAN 外设、双物理接口支持设备串联
- **终端电阻**：板载可控 120Ω CAN 终端匹配电阻
- **调试接口**：SWD 下载与在线调试
- **通信接口**：UART 串口 + CAN
- **状态指示**：板载 LED 指示灯
- **接口规格**：PH2.0 / GH1.25
- **电源系统**：12V 直流输入 → DCDC 12V 转 5V → LDO 5V 转 3.3V 全板供电；**无电源防反接保护**

详见 [hardware/README.md](hardware/README.md)。

## 机械

3D 打印外壳，适配 2804 电机，包含三大部件：

- **磁环座** —— 固定径向充磁磁环，配合 AS5600 编码器
- **PCB外壳1** —— 下半部分，连接 PCB 与电机
- **PCB外壳2** —— 上半部分，PCB 上盖

螺丝规格：M3 / M2.5（平头）/ M2（平头）

如需适配不同电机，需自行修改外壳模型。

详见 [mechanical/README.md](mechanical/README.md)。

## 许可证

MIT License，详见 [LICENSE](LICENSE)。

第三方组件：STM32 HAL Driver (BSD-3-Clause) · CMSIS (Apache-2.0)
