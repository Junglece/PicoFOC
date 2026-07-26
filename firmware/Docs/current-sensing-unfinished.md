---
name: current-sensing-unfinished
description: 电流环 Phase 1 剩余工作 + Phase 2 完整计划（已封存）
metadata:
  type: reference
---

# 电流环未完成部分说明

**封存日期：** 2026-07-26
**原因：** 小电机空载/轻载下，MP6541 SOx 电流检测信噪比不足，放弃电流环。

---

## Phase 1 完成项

| 项目 | 状态 | 说明 |
|------|------|------|
| mp6541_current.h 常量和公式 | ✅ | 换算系数、偏置、增益表 |
| drv_adc.h/c 驱动 | ✅ | DMA CIRCULAR + 快照 + ISR 读接口 |
| foc_math.h/c Clarke+Park | ✅ | 纯数学库，不依赖 ADC |
| Foc.h 电流字段 | ✅ | Ia/Ib/Ic/Ialpha/Ibeta/Id/Iq + current_raw |
| Foc.c FOC_ReadCurrents | ✅ | 读取→Clarke→Park 流水线 |
| main.c ADC_Init + VOFA 通道 | ✅ | 初始化 + 8 通道映射 |
| motor_ctrl.c 调 FOC_ReadCurrents | ✅ | 放在 FOC_UpdateSensor 之后 |

## Phase 1 未完成/有问题的项

| 项目 | 状态 | 详细 |
|------|------|------|
| ADC 采样与 PWM 同步 | ❌ 未解决 | 见 [[current-sensing-retrospective]] v3 分析 |
| TIM3 CC4 中断可靠触发 | ⚠️ 修复中 | CC4E 问题已发现，PWM mode 1 + CCR4=18 方案未验证 |
| 全周期电流波形 | ❌ 未验证 | 只能看到正半周，负半周因采样不同步被削 |
| 编译后实测 + VOFA 验证 | ❌ 未进行 | 代码改完未烧录完整验证 |

## Phase 2 所有内容（完全未动）

以下原本计划在 Phase 1 验证通过后实施：

### 2.1 电流环 PI 控制

- **Foc.h** 加字段: `target_iq`, `CurrPID_D`, `CurrPID_Q`
- **Foc.c** 加 `FOC_CurrCtrl()`: D 轴目标=0，Q 轴目标=target_iq，纯 PI
- **motor_ctrl.c** 改流水线为三环串级: **Pos→Spd→Curr**
- 速度环输出从 Uq 改为 target_iq（电流环算出的 Uq 才不会覆盖）
- **main.c** 加 `FOC_SetCurrPID` 配置

### 2.2 零电流校准

- **drv_adc.c** 实现 `ADC_CalibrateOffset()`: 暂停 DMA → 软件触发轮询 128 次取平均
- **motor_msg.h** 加 `CALIBRATE_CURRENT = 5`
- **led_indicator** 加 `CURRENT_CAL_BLINK` (200ms)
- **can_proto.c** mode 校验范围从 >4 改为 >5

### 2.3 Flash 版本迁移

- `CalibrationData_t` 结构体（含 version 字段）
- 旧版（4B float）→ 新版（含 version 判断）迁移逻辑

### 2.4 力矩模式

- mode=1（原 TORQUE）直接设 `target_iq`，不走速度环

## 封存后代码中可保留的部分

以下与电流相关、但不依赖 ADC 采样硬件的代码，**建议保留**：

- **foc_math.h/c** — Clarke/Park 变换是纯数学，后续无论用何种传感器都需用到
- **Foc.h** 中的 Ia/Ib/Ic/Id/Iq/Ialpha/Ibeta 字段 — 数据结构定义无害
- **FOC_ReadCurrents()** 调用链 — 当前已经被置零，FOC 仍然正常跑

## 恢复条件

如果未来以下任一条件满足，可以接续此工作：
1. **更换更大功率的电机**（额定电流 >1A）
2. **提高母线电压**（12V→24V+，电流信噪比提升）
3. **更换外置电流传感器**（ACS712/INA240 等专用芯片）
4. **PCB 改版**（优化 SOx 走线、加差分放大器）

恢复步骤见 [[current-sensing-retrospective#迁移建议]]。
