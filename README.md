# PicoFOC

**小型电机驱动平台** — 从固件到驱动板到外壳，完整方案。

A compact motor drive platform — firmware, PCB, and enclosure, all in one.

---

## 项目结构 / Structure

```
PicoFOC/
├── firmware/          ← STM32F103 FOC 固件（Keil MDK-ARM 项目）
│   ├── App/              业务逻辑层
│   ├── Bsp/              板级支持包 + 硬件接口
│   ├── Drv/              驱动层（定时器/CAN/ADC/LED）
│   ├── Lib/              纯数学算法（PID/坐标变换/观测器）
│   ├── Core/             CubeMX 生成代码
│   ├── Drivers/          STM32 HAL + CMSIS 驱动库
│   ├── MDK-ARM/          Keil 项目文件
│   └── Tools/            辅助脚本（UART 命令生成器等）
│
├── hardware/          ← PCB / 原理图 / 电路设计
│   └── README.md
│
├── mechanical/        ← 3D 外壳 / 结构件
│   └── README.md
│
├── .gitignore
├── LICENSE            ← MIT License
└── README.md          ← 本文件
```

## 固件 / Firmware

详见 [firmware/README.md](firmware/README.md)。

基于 STM32F103C8T6，四层架构（LIB/BSP/DRV/APP），
支持 CAN + UART 双协议通信，差分 + 龙伯格双观测器。

## 硬件 / Hardware

详见 [hardware/README.md](hardware/README.md)。

## 机械 / Mechanical

详见 [mechanical/README.md](mechanical/README.md)。

## 许可证 / License

MIT — 详见 [LICENSE](LICENSE)。
