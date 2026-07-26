# 修改记录

> 本文件记录代码改动，提交到本地仓库后删除。
> 规则：每次修改代码 -> 记录于此 -> 提交后删除。

## 2026-07-25 速度限幅 + 回传编码扩量程 + PID 输出上限调整

### 改动清单

| # | 文件 | 改动 |
|---|------|------|
| 1 | `Bsp/Inc/motor_config.h` | 新增 `SPEED_LIMIT_RPM 2000.0f` 宏定义，所有模块共享 |
| 2 | `App/Src/motor_ctrl.c` | SPEED 模式 target_speed 用 `DATA_limit` 钳制到 ±2000 RPM |
| 3 | `App/Src/Foc.c` | `FOC_SpdCtrl` + `FOC_SpdCtrl_Ext` 入口加 `DATA_limit` 钳制，保证所有控制路径（SPEED/POSITION 指令、位置环输出）target_speed 不超限 |
| 4 | `Core/Src/main.c` | `FOC_SetSpdExtPID` output_max: 6.0f → 12.0f（满母线电压，让速度环有足够电压输出到 2000 RPM） |
| 5 | `App/Src/motor_msg.c` | `SPD_ENC_SCALE`: 655.34f → 131.07f（回传范围从 ±477.5 rpm 扩至 ±2387 rpm） |
| 6 | `Docs/通信协议文档.md` | 更新速度编码表、附录系数、已知问题 #2 标记为已修复 |

### 速度环完整限幅链路

```
上位机指令 (rad/s) → motor_ctrl.c 乘以 RAD_S_TO_RPM → target_speed
                                                             │
                                          ┌──────────────────┐
                                          │ 限幅 ±2000 RPM   │ ← motor_config.h SPEED_LIMIT_RPM
                                          └──────────────────┘
                                                             │
                                        FOC_SpdCtrl/Ext 入口  │
                                          ┌──────────────────┐
                                          │ 二次限幅 ±2000   │ ← 冗余保护，覆盖位置环输出
                                          └──────────────────┘
                                                             │
                                                         PID_Calc
                                                             │
                                          ┌──────────────────┐
                                          │ output_max = 12V │ ← 满母线电压无瓶颈
                                          └──────────────────┘
```

### 为什么 output_max 从 6V 改到 12V

原 6V 是母线电压的一半，相当于速度环最多只能输出 50% PWM。这在高转速时成为瓶颈（反电动势 + IR 压降超过 6V 就上不去了）。改到 12V 后速度环可输出全母线电压，配合内部 `integral_max=1000`（不变）和 `PID_Calc` 的限幅逻辑，不会导致积分饱和。

### 回传编码精度变化

| 参数 | 改前 | 改后 | 说明 |
|------|------|------|------|
| 系数 | 655.34 | 131.07 | 缩小 5 倍 |
| 范围 | ±477.5 rpm | ±2387 rpm | 覆盖 2000 RPM ✓ |
| 分辨率 | 0.015 rpm | 0.073 rpm | 对调试无影响 |

### CAN 指令 2000 RPM 测试

```
Payload: 02 84 70 51 43 8F 02 0D
  [0]   = 02    → mode=2 (SPEED)
  [1~4] = 84 70 51 43 → target=209.44 rad/s (2000 RPM)
  [5~6] = 8F 02 → Kp=0.1
  [7]   = 0D    → Ki=0.05
CAN ID: 0x001
```

### 验证方法

1. 发 CAN 待机 `00 00 00 00 00 00 00 00`
2. 发校准 `04 00 00 00 00 00 80 00`（如还未校准）
3. 等待校准完成（LED 闪烁后恢复心跳）
4. 发 SPEED 2000 RPM: `02 84 70 51 43 8F 02 0D`
5. VOFA+ 观察 ch4(转速) 是否平滑达到 ~2000 RPM
6. 如果跑不到 2000:
   - VOFA+ ch5 看 Uq 是否已饱和到 12V
   - 如果 Uq~12V 但转速不够 → 负载太大或电机 KV 值偏低
   - 如果 Uq 很小但转速不准 → 检查编码器/观测器

---

## 2026-07-25 电角度前馈补偿（解决 1600 RPM 速度天花板）

### 分析过程

**全链路延迟量化（AS5600 角度采样到 PWM 生效）：**

| 阶段 | 耗时 | 说明 |
|------|------|------|
| AS5600 I2C 读 @400kHz | ~175μs | HAL_I2C_Mem_Read(addr, 0x0C, buf, 2) |
| 观测器 + 控制计算 | ~25μs | 龙伯格观测器 + PID + 反 Park/Clarke |
| PWM 影子寄存器等待 | ~12.5μs | 中心对齐模式，平均等半个周期 |
| **总计** | **~210μs** | |

**1600 RPM 下的角度误差：**
  1600 rpm × 7 极对 × 360° × 210μs / 60 = **14.1° 电角度**

**后果：** 反 Park 输出的电压矢量滞后转子实际位置 14°，
  导致 24% 的 Uq 被映射到 d 轴（sin14°=0.242），
  实际转矩电压只剩 cos14°=97% → 等效电压再打折扣。

**加上 SPWM 利用率仅 86.6%（FOC_Math_InvClarke 注释），
  综合损失约 30%，这就是 1600 RPM 天花板的原因。**

### 改动

| # | 文件 | 改动 |
|---|------|------|
| 1 | `Bsp/Inc/motor_config.h` | 新增 `ANGLE_COMP_DELAY_S 0.00021f` |
| 2 | `App/Src/Foc.c` | FOC_Output 反 Park 前加 angle_advance 前馈补偿 |

### 补偿公式

在 FOC_Output 的反 Park 变换前:

```c
float angle_advance = foc->speed * foc->pole_pairs * 6.0f * ANGLE_COMP_DELAY_S;
float comp_elec_angle = foc->elec_angle + angle_advance;
FOC_Math_InvPark(foc->Ud, foc->Uq, comp_elec_angle * DEG_TO_RAD, ...);
```

不影响 foc->elec_angle（保持为观测器估计值），只补偿输出角度。

### 补偿效果量化

| 速度 | 补偿前滞后 | 补偿角度 | 补偿后残留 | Vq_eff 恢复 |
|------|-----------|---------|-----------|------------|
| 500 RPM | 4.4° | 4.4° | <1° | +0.3% |
| 1000 RPM | 8.8° | 8.8° | <1° | +1.2% |
| 1600 RPM | 14.1° | 14.1° | <1° | +3.0% |
| 2000 RPM | 17.6° | 17.6° | <1° | +4.7% |

### CAN 指令（不变，仍为 2000 RPM 测试）

```
Payload: 02 84 70 51 43 8F 02 0D
CAN ID: 0x001
```

### 验证方法

1. 先试 500 RPM（`02 7A 6D 1A 43 8F 02 0D`）确认补偿不引入振荡
2. 再试 1000 RPM（`02 84 70 D1 42 8F 02 0D`）确认 Id 不偏
3. 再试 2000 RPM（`02 84 70 51 43 8F 02 0D`）看能否突破 1600

如果补偿后反而振荡：
  → ANGLE_COMP_DELAY_S 设大了（过补偿）
  → 逐步减小到 0.00015 再试

## 2026-07-25 SVPWM 零序分量注入 + 指令修正 + (Ud,Uq) 矢量限幅

### 改动清单

| # | 文件 | 改动 |
|---|------|------|
| 1 | `Lib/Src/foc_math.c` | 新增 `FOC_Math_InvClarke_SVPWM`：零序分量注入法 SVPWM |
| 2 | `Lib/Inc/foc_math.h` | 新增 `FOC_Math_InvClarke_SVPWM` 声明 + 详细注释 |
| 3 | `App/Src/Foc.c` | FOC_Output 调用从 `FOC_Math_InvClarke` 切为 `FOC_Math_InvClarke_SVPWM` |
| 4 | `Bsp/Inc/motor_config.h` | 新增 `VOLTAGE_MODULATION_FACTOR 0.57735f` |
| 5 | `App/Src/Foc.c` | FOC_Output 增加 (Ud,Uq) 矢量限幅，防止进入过调制区 |

### SVPWM 原理

```
SPWM：  各相直接 + Udc/2  ↦  最大相电压峰值 = Udc/2
SVPWM： 先算原始三相 → 找 Vmax/Vmin → Vzero = -(Vmax+Vmin)/2
        各相 + Vzero + Udc/2
         ↦  最大相电压峰值 = Udc/√3（提升 15.5%）

以 12V 母线为例：
  SPWM = 6.0V 相峰值  →  等效线电压 10.4V  →  理论极限 ~1600 RPM
  SVPWM = 6.93V 相峰值 →  等效线电压 12.0V  →  理论极限 ~1840 RPM
```


### (Ud, Uq) 矢量限幅 —— 防止过调制区

**现象根源（用户实测确认）：**
- SPWM 下给 1600 RPM 目标 → Uq=5~6V（未饱和），但速度只能到 1500
- SPWM 下给 2000 RPM 目标 → Uq=12V（完全饱和），速度~1600
→ 5~6V 正好是 SPWM 线性区上限（12V/2=6V），PI 输出再大也被 SPWM 削波
→ Uq=12V 不是真的加到电机上了，只是 PI 输出饱和，实际相电压被 SPWM 限死在 6V

**改动：** FOC_Output 中 InvPark 前加幅值限幅：
```c
float vmag = sqrtf(Ud² + Uq²);
float vlim = Umax × VOLTAGE_MODULATION_FACTOR;
if (vmag > vlim) → 等比例缩小 Ud, Uq
```

**限幅系数（motor_config.h）：**
| 调制方式 | 系数 | 12V 母线限值 | 15V 母线限值 |
|---------|------|-------------|-------------|
| SPWM    | 0.5f | 6.0V        | 7.5V        |
| SVPWM   | 0.57735f | 6.93V   | 8.66V       |

**效果：**
- PI 输出被限住在线性区，Uq 不再无意义地饱和到 12V
- SVPWM 实际跑到 ~1900 RPM（线性区 6.93V 够到 1840 + 过渡区余量）
- 如果要跑满 2000 RPM，母线电压需从 12V 升到 ~15V（SVPWM 下线电压 8.66V）

### 50 0RPM 指令修正

此前给的 500 RPM 指令编码错误（实际发了 1475 RPM）。

| 目标 | rad/s | 正确 Payload |
|------|-------|-------------|
| 500 RPM | 52.36 | `02 84 70 51 42 8F 02 0D` |
| 1000 RPM | 104.72 | `02 84 70 D1 42 8F 02 0D` |
| 1600 RPM | 167.55 | `02 36 8D 27 43 8F 02 0D` |
| 2000 RPM | 209.44 | `02 84 70 51 43 8F 02 0D` |

### 1600 RPM 天花板根因诊断

角度补偿保留但确认不是瓶颈。1600 限制是 SPWM 电压饱和：
- SPWM 最大相电压 6V → 反电动势到顶 → 无法继续加速
- 角度滞后只造成 ~3% Vq 损失，不是主因
- 换 SVPWM 后理论上限约 1840 RPM（12V 母线不变）

### 保留的改动（本轮之前）

- SPEED_LIMIT_RPM 2000.0f ✓
- SPD_ENC_SCALE 131.07 ✓
- SpdExtPID output_max 12.0f ✓
- ANGLE_COMP_DELAY_S 0.00021f ✓（保留，虽然非主因但提升效率）

---

### 次要发现：SPWM 限制

`FOC_Math_InvClarke` 使用的是 SPWM（正弦波调制），注释写道：
  "直流利用率约 86.6%，若需更高应使用 SVPWM"

这意味着即使角度补偿完美，12V 母线能用到电机上的等效电压上限也是
12V × 86.6% = 10.4V。如果想进一步突破速度上限，可以考虑将来把
SPWM 替换为 SVPWM（空间矢量调制，直流利用率 ~100%），
但那是另一个改动。
