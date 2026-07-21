# PicoFOC — Hardware

电机驱动板硬件设计。Motor drive board hardware design.

---

## 设计工具

EasyEDA / 立创 EDA

## 文件清单

| 文件 | 说明 |
|------|------|
| `PicoFOC.epro2` | EDA 工程源文件 |
| `PCB_PCB1_PicoFOC.pdf` | 原理图 + PCB 布局 PDF |

## 硬件规格

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
