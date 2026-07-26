---
name: current-sensing-retrospective
description: ADC 电流采样方案试错经验总结（2026-07 已封存）
metadata:
  type: reference
---

# ADC 三相电流采样 — 试错经验总结

**状态：封存** — 该链路已启用但保留代码，供后续参考或硬件升级后复用。

---

## 背景

PicoFOC 使用 **MP6541** 一体式 BLDC 驱动器，其内部集成电流检测放大器（SOA/SOB/SOC 引脚输出），通过外部偏置网络（3.3V→5.1K→SOx→5.1K→GND → 1.65V + 2.55KΩ 等效）将相电流线性映射为电压信号，由 STM32F103C8T6 的 ADC1（PA0/PA1/PA2）采样。

## 问题现象

从"改对"到"改错"到"放弃"，共经历四次架构迭代。以下是每轮的方案、诊断、根因：

---

### v1：HW 触发 → TIM3 TRGO=OC4REF + CCR4=0

**目标：** 每个 PWM 周期在 V0（CNT=0）时刻硬件触发 ADC，DMA 自动搬运。

**CubeMX 配置：**
- TIM3 MasterOutputTrigger = OC4REF
- CH4 = PWM Generation no output, PWM mode 2, CCR4=0
- ADC1 ExternalTriggerConv = Timer 3 Trigger Out, ContinuousConv = DISABLE

**现象：** 三相电流恒为 -7.12A（(0-2048) × 0.003476）。

**根因：** STM32F1 上 TIMx_CH4 的 OC4REF 内部信号受 CC4E 位控制。CubeMX 中 CH4 设为 "PWM Generation no output" → CC4E=0 → OC4REF 被硬件强制拉低，永远无上升沿 → ADC 永远收不到触发 → DMA 缓冲保持 0。

**教训：** 在 F1 系列上，OC4REF 路径需要 CC4E=1 才能工作。即使引脚不输出，也要在 USER CODE 中手动 `TIM3->CCER |= TIM_CCER_CC4E`。

**修复尝试：** 在 tim.c USER CODE 中加 `TIM3->CCER |= TIM_CCER_CC4E` → AVOID（怕被 CubeMX 重置，但其实 USER CODE 不会被覆盖）。

---

### v2：HW 触发 → TIM3 TRGO=Update

**目标：** 改用 Update Event 触发 ADC（简单粗暴）。

**CubeMX 配置：**
- TIM3 MasterOutputTrigger = Update
- ADC1 ExternalTriggerConv = Timer 3 Trigger Out

**现象：** 能采到正弦波，但：
1. 只有上半周（正半周），负半周被削
2. 三角波变形 + 周期性 dropout（采几周期→全 0→再采到）
3. Id/Iq 形态倒是对的（正弦→Clarke→Park 后信息仍在）

**根因：** 中心对齐 PWM 模式下，Update Event 在 V0（CNT=0）和 V7（CNT=1799）各触发一次 → ADC 以 40kHz 双触发率运行。V7 时刻三个上管导通，SOx 反映的是高侧电流，偏置不同，且电机续流路径不同 → 读数不可用。DMA 缓冲区交替混入 V0 有效值和 V7 无效值 → FOC 1kHz 快照时随机读到 V0/V7 数据 → 波形畸变。

**教训：** 对于三相半桥驱动，电流采样只能在 V0（零矢量，低侧续流）时刻有效。任何在 V7 的采样都会读到错误值。Update Event 双触发无法被 F1 硬件过滤，必须避免。

---

### v3：ADC 连续自由转换 + DMA CIRCULAR + TIM3 CC4 中断

**目标：** ADC 完全自由连续转换（~98kHz，3 通道每 10.5µs 扫一轮），DMA CIRCULAR 持续更新。TIM3 CH4 在 V0 触发 CC4 中断，ISR 中读取 DMA 最新值。

**CubeMX 配置：**
- ADC1 ContinuousConvMode = ENABLE, ExternalTrigConv = ADC_SOFTWARE_START
- TIM3 MasterOutputTrigger = RESET（不参与 ADC 触发）
- CH4 = PWM Generation no output, PWM mode 2, CCR4=0（只用 CC4 中断做 V0 同步）

**现象：**
- 直接读 DMA 缓冲（`s_adc_dma_buf`）：能看到三相 ~120° 正弦波，但只有 2048 以上（正半周），负半周被削
- 断电能看到下降到 2048 以下，但正常运行负半周几乎看不到
- Id/Iq 仍有明显正弦成分

**根因（诊断过程）：**
1. 确认 DMA 缓冲区通过 VOFA+ ch5/6/7 原值读出 → 数据更新、波形存在 → **ADC + DMA 链路正常**
2. 用快照（`ADC_TakeSnapshot` 关中断拷贝） → 结果跟直读一样 → **竞态不是问题**
3. `Motor.current_raw` 一直为 0 → ISR 没执行 → 发现 **CC4E=0 导致 CC4IF 不置位** → 加 `TIM3->CCER |= TIM_CCER_CC4E`
4. CC4E 加上后仍为 0 → 怀疑 **CCR4=0 与更新事件竞争** → 改用 PWM mode 1 + CCR4=18（~1% 占空比）

**根本问题（未解决）：**
ADC 虽然是"自由连续转换"，**但它与 PWM 不同步**。DMA 缓冲区中存储的是最近一次完成的 ADC 扫描，而该扫描可能发生在 PWM 周期的任意时刻。TIM3 CC4 在 V0 附近触发并读取 DMA，读到的是**上一个任意时刻完成的转换结果**——可能来自 V0（偏置正确），也可能来自 V7 附近（电流换路、偏置异常）。因此正半周（V0 附近电流大时偶然对应 V0 采样）可见，负半周不可见。

**教训：**
- 在 STM32F1 上，**连续自由转换 + 定时中断读 DMA** 无法保证采样时刻与 PWM 同步
- 要真正同步，必须用硬件触发 ADC + DMA（回到 v1 思路）——但这需要正确配置 CC4E
- 或者用两个定时器：TIM3 发 PWM，TIM3_CH4 OC4REF 触发 ADC（需要 CC4E=1）
- 另一个方向：用 TIM1（20kHz 中断）触发软件 ADC 启动——但软件触发有抖动

---

### v4：PWM mode 1 + CCR4=18（尝试修复 v3）

**目标：** 用非零 CCR4 避开 CNT=0 的竞争，让 CC4 中断可靠触发。

**配置：** TIM3 CH4 在 CubeMX 初始化后手动覆盖：
- `TIM3->CCMR2 = (old & 0x8FFF) | 0x6000`  → PWM mode 1
- `TIM3->CCR4 = 18`

**结果：** 未验证（在调试中发现并修复 CC4E 后，用户决定放弃电流采样）。

**理论分析：** 即使 CC4 中断可靠触发，v3 的根本问题（ADC 采样与 PWM 不同步）仍然存在。因此这条路线本质上也无法解决波形削波问题。

**结论：** 需要同时做两件事——① CC4E=1 让中断能触，② 回到 HW 触发 ADC（TIM3 TRGO=OC4REF + CC4E=1），让 ADC 在 V0 时刻被硬件启动采样，才能保证同步。

---

## 硬件限制分析

在调试过程中确认的小电机空载/轻载电流特征：
- 小电机（如本项目的 7 对极微型 BLDC）在空载/轻载时，相电流幅度很有限
- MP6541 SOx 输出电压摆幅小，信噪比差
- 偏置网络（1.65V）漂移 + PCB 布局噪声 + ADC 量化噪声 → 有效分辨率不足
- 电流的正弦形态虽然存在，但幅值太小 → 即使在 VOFA 上能看到半周，也无法做可靠的控制反馈

## 关键经验总结

| 方向 | 结论 |
|------|------|
| ADC 触发 | F1 上用 OC4REF 触发 ADC，必须在 USER CODE 中手动 CC4E=1 |
| Update Event 触发 | 中心对齐模式下 V0+V7 双触发，不能使用 |
| 连续转换+ISR读 | 采样时刻与 PWM 不同步，V7 附近读数错误 |
| CC4 中断 | CCR4=0 可能与更新事件竞争导致不触发，建议用非零值 |
| MP6541 精度 | 适合大电流（>1A），微小型电机空载不适用 |
| 调试手段 | VOFA+ 直接发 DMA 原始值 / 快照值 / 重构值，逐级确认比猜测效率高 |

## 迁移建议

如果后续硬件升级（更大电机 / 更高母线电压 / 外部电流传感器），恢复电流环的步骤：
1. 将 CubeMX 回复：ADC1 ExternalTriggerConv = Timer 3 Trigger Out, ContinuousConv = DISABLE
2. tim.c USER CODE 加 `TIM3->CCER |= TIM_CCER_CC4E`
3. TIM3 MasterOutputTrigger = OC4REF, CCR4=0, PWM mode 2
4. 重新使能 `main.c` 中的 `ADC_Init()`、CC4 中断配置、CH4 reconfig
5. 验证：VOFA 看 `current_raw` 是否出现完整正负正弦波
6. 然后做零电流校准、电流环 PI 调参
