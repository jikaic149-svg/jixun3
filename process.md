# 调试过程记录

> 只记录关键测试、关键参数修改和关键结论，避免后续反复改错。普通解释、临时讨论、无效尝试不写入。

## 记录规则

- 发生参数修改时，记录：修改项、旧值、新值、原因。
- 发生测试时，记录：测试指令、VOFA通道、主要数据结论。
- 发生控制链路变化时，记录：当前使用的数据源、关闭/启用的模块。
- 不记录完整终端日志，不记录重复数据，只保留能指导下一步调参的信息。

## 2026-07-05 速度闭环测试基线

### 控制链路

- 当前测试目标：只调左右轮速度闭环，不上直立环。
- 当前速度反馈源：只使用 QD 电机 CAN 反馈速度 `speed_can_rpm`。
- 已删除 `encoder_rpm` 差分速度链路，避免把 byte6-7 位置字段误当速度。
- `BALANCE_PID_ENABLE = 0U`，按 KEY 后进入 `CHASSIS_MOTOR_SPEED_TEST`。
- `WHEEL_SPEED_TEST_ENABLE = 1U`。

### VOFA通道

当前 `VOFA_DEBUG_STAGE = VOFA_DEBUG_STAGE_SPEED_LOOP`，只输出 6 路：

| 通道 | 名称 |
|---|---|
| I0 | `left_ref_rpm` |
| I1 | `left_can_rpm` |
| I2 | `left_cmd_a` |
| I3 | `right_ref_rpm` |
| I4 | `right_can_rpm` |
| I5 | `right_cmd_a` |

### 初始基线速度环参数

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `WHEEL_SPEED_TEST_REF_RPM` | `0.0f` | KEY 解锁后默认不转 |
| `WHEEL_SPEED_TEST_REF_MAX_ABS_RPM` | `200.0f` | 当前速度测试最大指令 |
| `WHEEL_SPEED_TEST_CURRENT_LIMIT_A` | `0.35f` | 当前测试限流 |
| `WHEEL_SPEED_PID_KP_A_PER_RPM` | `0.00150f` | 速度环 P |
| `WHEEL_SPEED_PID_KI_A_PER_RPM_S` | `0.001000f` | 速度环 I |
| `WHEEL_SPEED_FF_STATIC_A` | `0.015f` | 静摩擦/低速前馈 |
| `WHEEL_SPEED_FF_KV_A_PER_RPM` | `0.00015f` | 转速前馈 |
| `LEFT_WHEEL_SPEED_FEEDBACK_LPF_ALPHA` | `0.25f` | 左轮反馈滤波 |
| `RIGHT_WHEEL_SPEED_FEEDBACK_LPF_ALPHA` | `0.25f` | 右轮反馈滤波 |

### 最近一次数据结论

- `S50/S100/S150/S200` 下速度闭环能跟随目标，说明 CAN 速度反馈和电流控制链路可用。
- 旧参数下高转速实际速度偏高，说明高速前馈偏大。
- 右轮阶跃超调明显，原因包括右轮原先反馈滤波更慢，以及手持悬空时负载不均。
- 电流指令远小于 `0.35A` 限幅，当前不是电流不够的问题。

### 下一步测试

1. 烧录当前参数。
2. 悬空测试 `S50 -> S100 -> S150 -> S200 -> S0`。
3. 重点观察：
   - `left_can_rpm/right_can_rpm` 是否贴近目标。
   - `left_cmd_a/right_cmd_a` 是否平滑、是否接近限幅。
   - 阶跃瞬间是否仍有明显右轮超调。
4. 若 `S200` 响应仍偏慢，再小幅增加 `WHEEL_SPEED_FF_KV_A_PER_RPM`。
5. 若 `S200` 稳态仍偏高或波动变大，则降低 `WHEEL_SPEED_FF_KV_A_PER_RPM` 或降低速度环 P。

## 2026-07-05 速度环 P 微调

### 测试结论

- 新前馈参数下高速响应保留，但 `S150/S200` 稳态仍有小幅来回修正。
- `S200` 稳态均值接近目标，但左轮峰峰值约 `36 rpm`，右轮峰峰值约 `23 rpm`。
- 电流指令平均仍只有约 `0.04A`，没有接近 `0.35A` 限幅。
- 判断：主要不是电流不足，而是速度环 P 略硬，导致 CAN 速度波动被放大成电流修正。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `WHEEL_SPEED_PID_KP_A_PER_RPM` | `0.00150f` | `0.00120f` | 小幅降低 P，减少高转速来回修正，同时保留当前前馈响应 |

### 下一步测试

- 继续测试 `S50/S100/S150/S200/S-50/S-100/S0`。
- 若响应明显变慢但波动改善，可小幅提高前馈 `WHEEL_SPEED_FF_KV_A_PER_RPM`，不要先加 P。
- 若仍然来回抖，再考虑继续小幅降低 P 或降低反馈滤波 alpha。

## 2026-07-05 速度环频率判断

### 测试结论

- 当前 STM32 外部速度 PID 按 `WHEEL_SAFE_TASK_PERIOD_MS = 1U` 运行，即约 `1000Hz`。
- 当前 VOFA 按 `VOFA_OUTPUT_PERIOD_MS = 20U` 输出，即约 `50Hz`，VOFA 只用于观察，不能代表控制频率。
- 本次 `S100/S150/S200` 数据均值能跟随目标，电流指令约 `0.03~0.05A`，远低于 `0.35A` 限幅。
- `S100` 稳态：左轮均值约 `98.93rpm`，右轮约 `97.27rpm`；`S150` 稳态：左轮约 `150.55rpm`，右轮约 `149.16rpm`；`S200` 稳态：左轮约 `202.44rpm`，右轮约 `201.47rpm`。
- 主要问题不是外部 PID 频率低，而是 CAN 速度反馈点有抖动/估计噪声，速度环 P 偏硬时会追着反馈噪声修正电流。

### 下一步判断

- 若怀疑 QD 反馈刷新率不足，临时切到 `VOFA_DEBUG_STAGE_MOTOR_FEEDBACK`，观察左右轮 `rx_count` 每 `20ms` 增量。
- 若每行 `rx_count` 增量明显低于预期，再判断为反馈刷新率问题；否则继续按参数问题处理。
- 参数调整优先级：先小幅降 `WHEEL_SPEED_PID_KP_A_PER_RPM` 或降低反馈 LPF alpha；若响应变慢，再小幅补 `WHEEL_SPEED_FF_KV_A_PER_RPM`，不要直接加 P。

## 2026-07-05 速度环保持响应的抖动处理

### 调整原则

- 平衡车最终需要快速轮速响应，不继续加重反馈滤波。
- 当前抖动不优先靠降低响应速度解决，而是保留偏硬速度环，同时压掉很小的 CAN 速度量化/估计噪声。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `LEFT_WHEEL_SPEED_FEEDBACK_LPF_ALPHA` | `0.25f` | `1.0f` | 关闭左轮速度反馈时间滤波，减少相位滞后 |
| `RIGHT_WHEEL_SPEED_FEEDBACK_LPF_ALPHA` | `0.25f` | `1.0f` | 关闭右轮速度反馈时间滤波，减少相位滞后 |
| `WHEEL_SPEED_PID_ERROR_DEADBAND_RPM` | 无 | `2.0f` | 只对 P 项加入小误差软死区，抑制小幅速度噪声造成的电流抖动，不降低大误差响应 |

### 下一步测试

- 重新烧录后测试 `S50/S100/S150/S200/S0`。
- 重点比较：阶跃响应是否更快、稳态电流是否更碎、速度峰峰值是否下降。
- 若速度响应变快但电流明显抖动，先把 `WHEEL_SPEED_PID_ERROR_DEADBAND_RPM` 从 `2.0f` 调到 `3.0f`，不要重新加重 LPF。

## 2026-07-05 速度环软感排查

### 测试结论

- 关闭 LPF 后响应链路没有时间滤波，但 `2rpm` 的 P 项软死区会降低目标附近的小误差刚度。
- 例如 `5rpm` 误差在 `Kp = 0.00120A/rpm` 下，原本 P 输出约 `0.006A`；加入 `2rpm` 软死区后只剩约 `0.0036A`，目标附近等效 P 降低约 `40%`。
- 本次数据中 `S150` 左轮均值约 `146.79rpm`，偏低约 `3.21rpm`；`S200` 均值能到，但峰峰值仍约 `30rpm`，说明死区没有有效解决大幅抖动，反而可能带来软感。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `WHEEL_SPEED_PID_ERROR_DEADBAND_RPM` | `2.0f` | `0.0f` | 先关闭 P 项软死区，验证软感是否由死区造成；继续保持速度反馈无 LPF |

### 下一步测试

- 重新烧录后复测 `S100/S150/S200/S0`。
- 若软感消失但抖动仍大，说明死区不是合适方案；下一步在无 LPF、无死区基础上考虑小幅调 `Kp` 或排查 QD CAN 反馈速度刷新/估计抖动。

## 2026-07-05 速度反馈显示与轻滤波

### 测试结论

- 无 LPF、无死区后，`S150/S200` 均值基本能到目标，但 raw CAN 速度峰峰值仍约 `25~31rpm`，VOFA 曲线明显抖动。
- 这不是 VOFA 串口慢导致，而是 QD CAN 回传速度本身有较大的瞬时估计抖动。
- 速度环不能长期直接追 raw CAN，否则 P 环会追噪声；但滤波也不能过重，避免直立控制响应变慢。

### 参数/显示修改

| 修改项 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `LEFT_WHEEL_SPEED_FEEDBACK_LPF_ALPHA` | `1.0f` | `0.50f` | 加回很轻的一阶反馈滤波，约毫秒级延迟，降低 raw CAN 抖动 |
| `RIGHT_WHEEL_SPEED_FEEDBACK_LPF_ALPHA` | `1.0f` | `0.50f` | 同上 |
| VOFA速度通道 | raw `speed_can_rpm` | `wheel_speed_actual_*_rpm` | VOFA显示 PID 实际使用的速度，raw CAN 仍可切到 `MOTOR_FEEDBACK` 诊断 |

### 下一步测试

- 复测 `S100/S150/S200/S0`。
- 观察新 VOFA 表头：`left_ref_rpm,left_actual_rpm,left_cmd_a,right_ref_rpm,right_actual_rpm,right_cmd_a`。
- 若曲线仍抖但响应可以接受，再把 alpha 从 `0.50f` 降到 `0.40f`；若响应明显变软，再升回 `0.60f`。

## 2026-07-05 回到硬速度环轻滤波版本

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `LEFT_WHEEL_SPEED_FEEDBACK_LPF_ALPHA` | `0.50f` | `0.25f` | 回到之前更稳的轻滤波版本，兼顾硬度与可观察性 |
| `RIGHT_WHEEL_SPEED_FEEDBACK_LPF_ALPHA` | `0.50f` | `0.25f` | 同上 |

### 当前速度环基准

- `WHEEL_SPEED_PID_KP_A_PER_RPM = 0.00120f`，保持当前硬度。
- `WHEEL_SPEED_PID_ERROR_DEADBAND_RPM = 0.0f`，不使用死区，避免目标附近发软。
- `WHEEL_SPEED_FF_STATIC_A = 0.015f`，`WHEEL_SPEED_FF_KV_A_PER_RPM = 0.00015f`，保持当前前馈响应。
- VOFA 速度通道继续输出 `wheel_speed_actual_*_rpm`，即速度环实际使用的滤波后速度。

## 2026-07-05 进入 IMU 调试

### 阶段判断

- 当前轮速环平均误差与响应速度可以接受，先进入 IMU/陀螺仪调试。
- 直立环仍保持关闭：`BALANCE_PID_ENABLE = 0U`，本阶段不让车尝试直立。
- 调 IMU 时不要按 KEY，避免进入轮速测试导致电机输出。

### 配置修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `VOFA_DEBUG_STAGE` | `VOFA_DEBUG_STAGE_SPEED_LOOP` | `VOFA_DEBUG_STAGE_IMU_ATTITUDE` | VOFA 改为输出 Kalman 角度、陀螺仪角速度、加速度角和三轴 IMU 数据 |

### VOFA通道

| 通道 | 名称 |
|---|---|
| I0 | `kalman_theta_deg` |
| I1 | `gyro_theta_rate_dps` |
| I2 | `accel_theta_deg` |
| I3 | `accel_x_ms2` |
| I4 | `accel_y_ms2` |
| I5 | `accel_z_ms2` |
| I6 | `gyro_x_dps` |
| I7 | `gyro_y_dps` |
| I8 | `gyro_z_dps` |
| I9 | `yaw_deg` |
| I10 | `yaw_rate_dps` |
| I11 | `fault` |

### 下一步测试

- 静止放平：看 `gyro_theta_rate_dps` 是否接近 `0dps`，`kalman_theta_deg` 与 `accel_theta_deg` 是否接近。
- 手动前倾/后仰：确认 `kalman_theta_deg` 和 `gyro_theta_rate_dps` 方向是否符合直立环极性。
- 原地绕竖直方向转动：确认 `yaw_rate_dps` 方向和零漂。

## 2026-07-05 Yaw 零漂处理

### 测试结论

- 静止数据中 `yaw_deg` 近似线性漂移，说明 `yaw_rate_dps` 存在近似常值零偏后被积分。
- 当前代码中 `gyro_bias[3]` 原本只是全 0 数组，没有真正完成零偏估计。
- 平衡车开机时不一定静止，也不一定处在可控姿态，因此不采用开机自动采样零偏。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `CHASSIS_YAW_RATE_BIAS_DPS` | `0.085f` | `0.122f` | 根据本次静止数据中 `gyro_z_dps` 均值补偿 yaw 零偏 |
| `CHASSIS_YAW_RATE_DEADBAND_DPS` | 无 | `0.15f` | 抑制残余极小 yaw 角速度被持续积分成线性漂移 |
| `IMU_GYRO_BIAS_CAL_ENABLE` | 无/临时 `1U` | `0U` | 不做开机自动采样，避免车体开机晃动时把真实运动当零偏 |

### 下一步测试

- 烧录后静止观察 `yaw_rate_dps` 是否围绕 `0`，`yaw_deg` 是否停止明显单向漂移。
- 若静止时仍线性漂移，根据新数据微调 `CHASSIS_YAW_RATE_BIAS_DPS`；不要启用开机自动采样。

## 2026-07-05 KEY 两段式 Yaw 校准

### 控制流程

- 不做开机自动零偏采样，因为平衡车开机时可能倒地、晃动或被手拿着。
- 第一次按 KEY：只启动 yaw 轴零偏校准，电机保持急停，不进入轮速/直立控制。
- 校准期间要求车体倒地或放置静止；校准只使用 yaw 轴角速度均值，不修改 pitch 角度、不修改 pitch 陀螺仪零点。
- 第二次按 KEY：只有在 `yaw_cal_state = 2` 后才允许解锁当前控制模式。

### 灯光提示

| 状态 | 灯光 |
|---|---|
| 未校准，安全等待 | 蓝灯常亮 |
| yaw 校准中 | 蓝灯闪烁 |
| yaw 校准完成，等待第二次 KEY | 蓝灯+绿灯常亮 |
| 控制已解锁 | 绿灯常亮 |
| 电机反馈/使能故障 | 红灯常亮 |

### 参数/接口修改

| 修改项 | 当前值 | 说明 |
|---|---:|---|
| `IMU_YAW_BIAS_KEY_CAL_SAMPLES` | `500U` | 第一次 KEY 后采样 yaw 角速度均值 |
| `IMU_GYRO_BIAS_CAL_ENABLE` | `0U` | 继续禁用开机自动三轴陀螺仪标定 |
| `CHASSIS_YAW_RATE_BIAS_DPS` | `0.122f` | 手动校准前的备用 yaw 零偏 |
| `CHASSIS_YAW_RATE_DEADBAND_DPS` | `0.15f` | 抑制残余小零漂积分 |
| VOFA新增 | `yaw_cal_state` | `0=未校准`，`1=校准中`，`2=校准完成` |

## 2026-07-05 取消 KEY Yaw 校准，回到固定补偿

### 测试结论

- KEY 后 yaw 均值校准对静止要求太高，校准窗口内轻微晃动会把真实转动拟合进零偏，效果可能比固定补偿更差。
- 两组静止数据中 `gyro_z_dps` 均值约为 `0.1224dps` 和 `0.1253dps`，加权后约 `0.124dps`。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `IMU_YAW_BIAS_KEY_CAL_ENABLE` | `1U` | `0U` | 取消 KEY yaw 拟合校准，避免操作晃动污染零偏 |
| `CHASSIS_YAW_RATE_BIAS_DPS` | `0.125f` | `0.124f` | 使用两组静止数据加权均值作为固定 yaw 零偏 |
| `CHASSIS_YAW_RATE_DEADBAND_DPS` | `0.15f` | `0.0f` | 先关闭死区，保留线性漂移特征，便于继续精调固定值 |

### 当前流程

- KEY 恢复为直接解锁/急停切换，不再执行 yaw 校准。
- `yaw_cal_state` 固定为 `2`，仅表示当前不需要 KEY 校准。
- 若后续仍有线性漂移，只微调 `CHASSIS_YAW_RATE_BIAS_DPS`。

## 2026-07-05 Yaw 直线拟合

### 测试结论

- 本次静止数据 `yaw_deg` 线性拟合结果：`yaw = 0.265276 * t + 1.716281`，假设 VOFA 行间隔为 `20ms`，`R^2 = 0.99830`。
- 等价为每行约增加 `0.00531deg`，线性漂移非常明显。
- 同一段数据中 `gyro_z_dps` 均值约 `0.13456dps`，`yaw_rate_dps` 均值约 `0.01056dps`。
- 当前 `CHASSIS_YAW_RATE_BIAS_DPS = 0.124f`，从 `gyro_z_dps` 均值看，下一版固定 yaw 零偏应优先试 `0.135f` 附近。
- 注意：`yaw_deg` 拟合斜率与 `yaw_rate_dps` 均值不一致，不能直接用 `yaw` 斜率反推到 `0.389dps` 这种明显异常的 bias；后续需要确认 VOFA 数据是否等间隔，以及 yaw 积分使用的 `imu_dt_sec` 是否正确。

## 2026-07-05 Yaw 漂移二次拟合与 dt 修正

### 测试结论

- 最新静止数据共 `221` 行；若按 VOFA 目标输出周期 `20ms` 计算，总时长约 `4.4s`。
- `yaw_deg` 直线拟合：`yaw = 0.329520 * t + 0.453970`，`R^2 = 0.999595`，说明漂移仍然高度线性。
- 同一段数据中 `gyro_z_dps` 均值为 `0.163679dps`，当前固定补偿为 `0.124f`，所以固定 yaw 零偏先改为 `0.164f`。
- `yaw_rate_dps` 均值只有 `0.039679dps`，但 `yaw_deg` 按 20ms 反推斜率是 `0.329520dps`，两者相差约 `8.3` 倍；这说明问题不只是 bias，IMU 积分使用的 `imu_dt_sec` 也需要核对。

### 修改内容

| 修改项 | 旧值/旧逻辑 | 新值/新逻辑 | 目的 |
|---|---:|---:|---|
| `CHASSIS_YAW_RATE_BIAS_DPS` | `0.124f` | `0.164f` | 按最新静止数据 `gyro_z_dps` 均值补偿 yaw 零偏 |
| `imu_dt_sec` | DMA 用固定 `BALANCE_IMU_DT_SEC`，fallback 用固定 `IMU_POLL_FALLBACK_DT_SEC` | 每帧 IMU 数据使用 `xTaskGetTickCount()` 计算实际间隔，并限幅到 `0.5ms~20ms` | 避免 yaw 角积分周期和实际任务周期不一致 |
| VOFA IMU 通道 | 13 列 | 14 列，新增 `imu_dt_ms` | 下一次直接确认真实 IMU 积分周期 |

### 下一步测试

- 烧录后静止采集 yaw 数据，重点看最后一列 `imu_dt_ms` 是否稳定。
- 若 `gyro_z_dps` 均值接近 `0.164` 且 `yaw_rate_dps` 围绕 `0`，但 `yaw_deg` 仍漂，则继续检查 `imu_dt_ms` 和 yaw 积分路径。

## 2026-07-05 为什么 yaw 零漂比 pitch 难

### 问题总结

- Pitch 可以用加速度计里的重力方向修正，所以即使陀螺仪有一点零漂，Kalman/互补滤波也能慢慢拉回来。
- Yaw 没有重力参考；当前没有磁力计、视觉定位或外部航向参考，所以 yaw 只能靠 `gyro_z_dps` 积分。只要 `gyro_z_dps` 有很小零偏，`yaw_deg` 就会一直线性漂。
- yaw 零偏很小，通常只有 `0.1~0.2 dps` 量级；但积分 10 秒就是 `1~2 deg`，所以看起来漂得很明显。
- VOFA 输出是降采样观察值，不是 IMU 每 1ms 的完整积分过程。`yaw_deg` 是内部连续积分结果，`yaw_rate_dps` 是被 VOFA 抽样出来的瞬时值；直接拿两者按行号强行拟合，可能得出互相矛盾的结论。
- 因此 yaw 零漂调试不能只看一列 `yaw_deg` 的斜率，还要同时看 `gyro_z_dps` 均值、`yaw_rate_dps` 均值、IMU 积分周期和 VOFA 实际输出周期。

### 当前判断

- 公式本身是清楚的：`yaw_rate_dps = gyro_z_dps - CHASSIS_YAW_RATE_BIAS_DPS`，`yaw_deg += yaw_rate_dps * dt`。
- 最新数据里 `gyro_z_dps` 和 `yaw_rate_dps` 的关系符合当前 bias，但 `yaw_deg` 的行间变化仍与抽样到的 `yaw_rate_dps` 不完全一致。
- 下一步不能继续盲改 bias，需要先让 VOFA 输出真实时间信息，例如 `imu_time_sec` 或 `vofa_period_ms`，再判断到底是 bias 问题、输出降采样问题，还是积分路径问题。

## 2026-07-05 定位到 dt 积分根因并改用 DWT 微秒时钟

### 根因结论

- `yaw_deg` 斜率远大于抽样 `yaw_rate_dps` 的本质是**积分 dt 被系统性放大**，不是单纯 bias 问题。
- 陀螺仪 ODR 配置为 `BMI088_GYRO_1000_116_HZ`，即 DRDY ~1000Hz；而原 `imu_dt_sec` 用 `xTaskGetTickCount()` 计算，tick 只有 1ms 分辨率。
- 陀螺仪 1000Hz 与 SysTick 1000Hz 是两个独立时钟，会拍频：同一个 1ms tick 内常落入多帧陀螺仪数据。此时 `dt_ticks==0`，旧代码跳过 dt 更新却**仍然积分**，并把 `imu_dt_sec` 兜底成 `BALANCE_IMU_DT_SEC = 1ms`。
- 结果：每帧都按 ~1ms 积分，但真实间隔并没有 1ms，`yaw_deg` 被过积分放大，而 VOFA 每 20ms 抽样出的 `yaw_rate_dps` 仍是真实瞬时值 —— 两者对不上正是这个 bug 的指纹。
- Pitch 不受影响，因为有加速度计重力修正把误差拉回；yaw 无绝对参考，dt 误差会一直累积。

### 修改内容

| 修改项 | 旧值/旧逻辑 | 新值/新逻辑 | 目的 |
|---|---|---|---|
| dt 时钟源 | `xTaskGetTickCount()` 整毫秒差值 | 新增 `BSP_DWT_Init()`/`BSP_GetMicros()`，用 DWT->CYCCNT(168MHz) 微秒时间戳算每帧真实间隔 | 保证「积分 dt 之和 == 真实流逝时间」，消除 tick 量化过积分 |
| 首帧 dt | `last_imu_update_tick==0` 判首帧 | `imu_dt_ready` 标志，首帧只播种时间戳不积分 | 避免首帧用巨大/无效 dt |
| dt 下限 | `< 0.5ms` 兜底 1ms | `< 0.2ms` 兜底 `BALANCE_IMU_DT_SEC` | 微秒时钟下真实 dt 约 1ms，仅极端异常才兜底 |
| yaw 速率滤波 | 工作副本被改成 `yaw_rate_kalman = yaw_rate`（无滤波） | 恢复 `KalmanScalar_Update`，并先过死区再滤波 | 压掉抽样噪声与残余零偏积分 |
| `CHASSIS_YAW_RATE_DEADBAND_DPS` | `0.0f` | `0.05f` | 抑制 bias 补偿后残余极小 yaw 角速度被持续积分成线性漂移 |
| `BSP_DWT_Init()` 调用 | 无 | `INS_Init()` 最前面调用一次 | 调度器启动前使能周期计数器 |

### 下一步测试

- 烧录后静止采集 IMU 数据，重点看最后一列 `imu_dt_ms`：现在应稳定在 ~1.0ms 附近（真实微秒 dt），不再是恒定兜底值。
- 观察 `yaw_deg` 是否停止明显单向线性漂移；此时按 20ms 行间隔反推的斜率应与 `yaw_rate_dps` 均值基本一致（不再有 ~8 倍偏差）。
- 若 `imu_dt_ms` 稳定、斜率与 rate 一致但仍有缓慢漂移，则纯属残余固定零偏，只微调 `CHASSIS_YAW_RATE_BIAS_DPS`（当前 `0.164f`）。
- 若静止时 `yaw_rate_dps` 已围绕 0 但偶发跳变被积分，再把 `CHASSIS_YAW_RATE_DEADBAND_DPS` 从 `0.05f` 适当上调。

## 2026-07-06 steps_per_vofa 定位真凶：SysTick 慢 8 倍，不是积分错

### 测试结论（1.txt，新增第15列 steps_per_vofa）

- `steps_per_vofa` 稳定为 **159~160**：每个"20ms" VOFA 周期内实际发生了 160 次积分步。
- `imu_dt_ms`（DWT 真实测量）稳定 ~1.003ms：陀螺仪确实按 1kHz 出数据，每步积分 dt 也是真的 1ms。
- 160 步 × 1ms = **每行真实时长 160ms**，不是 20ms。此前所有"按 20ms/行"的拟合把时间轴压缩了 8 倍，才出现"斜率是 rate 8 倍"的假象。
- 用真实行距 160ms 重新拟合：`yaw_deg` 斜率 = **-0.0366 dps**，与 `yaw_rate_dps` 均值 **-0.0290** 之比 = **1.26 ≈ 1**。
- **结论：yaw 积分路径从头到尾是对的。真正的 bug 是 FreeRTOS 的 tick 走慢了 8 倍**：`pdMS_TO_TICKS(20)` 的"20ms"实际是 160ms。

### SysTick 8 倍慢的根因

- `FreeRTOSConfig.h` 里显式定义了 `configSYSTICK_CLOCK_HZ = 168000000UL`。
- port.c 的规则：**只要该宏被显式定义**（哪怕值等于 CPU 频率），就认为 SysTick 用外部参考时钟，把 `portNVIC_SYSTICK_CLK_BIT` 置 0 → SysTick CLKSOURCE 位清零 → SysTick 实际走 **HCLK/8 = 21MHz**。
- 但重装载值仍按 168MHz/1000 计算 → 每个"1ms" tick 实际 **8ms**。
- 影响全系统：轮速 PID "1kHz" 实际 125Hz、VOFA "20ms" 实际 160ms、所有 vTaskDelay 都慢 8 倍。之前速度环数据的时间轴同样被压缩过。

### 修改内容

| 修改项 | 旧值 | 新值 | 原因 |
|---|---|---|---|
| `configSYSTICK_CLOCK_HZ` | 显式定义 168000000UL | **删除该定义**（加注释警告） | 让 port.c 默认置位 CLKSOURCE，SysTick 真跑 168MHz，tick 恢复真 1ms |
| `CHASSIS_YAW_RATE_BIAS_DPS` | `0.164f` | `0.135f` | 本次数据残余 yaw_rate 均值 -0.029，0.164-0.029=0.135；与 gyro_z 均值 0.1287 一致 |

### 下一步测试

- 烧录后静止采集：`steps_per_vofa` 应从 160 变为 **~20**（tick 修好后 20ms 真的是 20ms）。
- `yaw_deg` 按 20ms/行拟合的斜率应与 `yaw_rate_dps` 均值一致（比值 ~1）。
- 残余漂移应降到 |0.01 dps| 量级；若仍有固定方向小漂移，微调 bias（步长 0.005f）。
- 注意：tick 修复后轮速 PID 真实频率从 125Hz 提高到 1kHz，等效积分增益变化 8 倍，速度环若行为变化属预期，需按新时间基准复测。

## 2026-07-06 VOFA 只保留 Yaw 调试通道

### 测试结论

- 最新静止数据共 `136` 行、`15` 列。
- `steps_per_vofa` 为 `19~20`，均值约 `19.96`，说明 SysTick 修复后 VOFA 的 `20ms` 周期已经恢复正常。
- `imu_dt_ms` 均值约 `1.017ms`，IMU 积分周期正常。
- `gyro_z_dps` 均值约 `0.138dps`，当前 `CHASSIS_YAW_RATE_BIAS_DPS = 0.135f` 接近静止零偏。
- `yaw_rate_dps` 均值约 `-0.0157dps`，`yaw_deg` 在约 `2.715s` 内变化 `-0.061deg`，等效漂移约 `-0.0225dps`。

### 修改内容

| 修改项 | 旧值/旧逻辑 | 新值/新逻辑 | 目的 |
|---|---:|---:|---|
| `VOFA_OUTPUT_CHANNEL_COUNT` (`IMU_ATTITUDE`) | `15U` | `5U` | 当前阶段只看 yaw 相关量，减少 VOFA 曲线干扰 |
| VOFA IMU 通道 | `kalman_theta_deg,gyro_theta_rate_dps,accel_theta_deg,accel_x_ms2,accel_y_ms2,accel_z_ms2,gyro_x_dps,gyro_y_dps,gyro_z_dps,yaw_deg,yaw_rate_dps,fault,yaw_cal_state,imu_dt_ms,steps_per_vofa` | `gyro_z_dps,yaw_deg,yaw_rate_dps,imu_dt_ms,steps_per_vofa` | 保留 raw yaw gyro、yaw 积分角、滤波后 yaw rate 和时间基准诊断 |

### 下一步测试

- 烧录后 VOFA 表头应变为 `gyro_z_dps,yaw_deg,yaw_rate_dps,imu_dt_ms,steps_per_vofa`。
- 先采集一段更长的 yaw-only 静止数据，再比较 `yaw_rate_dps` 均值和 `yaw_deg` 斜率；本次先不继续改 `CHASSIS_YAW_RATE_BIAS_DPS`。

## 2026-07-06 Yaw 角度归一化与最短路径

### 问题判断

- 旧接口 `INS_GetYaw()` 返回的是 `attitude_algorithm` 里的 `Yaw_a`，该值来自没有磁力计约束的旧欧拉角链路，不是当前正在调试的 yaw 积分链路，因此启动后可能直接显示约 `90deg`。
- 当前可信 yaw 链路是 `gyro_z_dps -> CHASSIS_YAW_RATE_BIAS_DPS -> deadband -> Kalman -> 积分`。

### 修改内容

| 修改项 | 旧逻辑 | 新逻辑 | 目的 |
|---|---|---|---|
| `global_chassis_yaw` | 直接使用连续积分角 `yaw_kalman_angle` | 使用 `wrap_angle_360_deg(yaw_kalman_angle)`，范围 `[0,360)` | 对外 yaw 不再超过 360，也不会启动继承旧 90deg |
| VOFA `yaw_deg` | 输出连续积分角 | 输出归一化后的 `global_chassis_yaw` | 曲线/读数统一在 `0~360` |
| `INS_GetYaw()` | 返回旧 `Yaw_a` | 返回 `global_chassis_yaw` | 避免继续使用旧姿态链路的假 yaw |
| `INS_GetChassisYawError(target)` | 无 | 返回 `target-current` 的 `[-180,180)` 最短路径误差 | 后续 yaw 角度闭环避免从 359deg 绕远路转到 1deg |

### 下一步测试

- 烧录后静止开机：VOFA `yaw_deg` 应从接近 `0deg` 或 `360deg` 附近开始，而不是直接 `90deg`。
- 手动转过 `0/360` 边界时，后续控制要使用 `INS_GetChassisYawError(target)` 做角度误差，不要直接 `target - current`。

## 2026-07-06 长时间 Yaw 零漂修正

### 测试结论

- `vofa+.csv` 共 `10955` 行有效 yaw-only 数据，时长约 `218.8s`。
- `steps_per_vofa` 均值约 `19.97`，`imu_dt_ms` 均值约 `1.022ms`，时间基准正常。
- 全段展开 yaw 从 `0.007deg` 到 `2.279deg`，线性拟合漂移约 `+0.0101dps`。
- 后半段漂移略大：最近 `99.9s` 拟合约 `+0.0144dps`，最近 `20.0s` 拟合约 `+0.0173dps`。
- `gyro_z_dps` 全段均值约 `0.1474dps`，后半段均值约 `0.1502dps`，当前 `CHASSIS_YAW_RATE_BIAS_DPS = 0.135f` 偏小。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `CHASSIS_YAW_RATE_BIAS_DPS` | `0.135f` | `0.150f` | 按长时间静止数据的 `gyro_z_dps` 后半段均值修正固定 yaw 零偏，抵消约 `+0.01~0.015dps` 的正向漂移 |

### 下一步测试

- 烧录后再采集 `120s~300s` 静止 yaw-only 数据。
- 目标：展开后的 `yaw_deg` 拟合斜率进入 `|0.005dps|` 以内。
- 若改后变成稳定负漂，再按 `0.002~0.005f` 步长回调 `CHASSIS_YAW_RATE_BIAS_DPS`。

## 2026-07-06 Yaw bias 过补回调

### 测试结论

- 新 `vofa+.csv` 共 `4695` 行有效 yaw-only 数据，时长约 `93.9s`。
- `steps_per_vofa` 均值约 `19.99`，`imu_dt_ms` 均值约 `1.035ms`，时间基准正常。
- 当前 `CHASSIS_YAW_RATE_BIAS_DPS = 0.150f` 时，全段 yaw 从 `359.778deg` 漂到 `358.121deg`，展开后约 `-1.657deg`。
- 全段拟合漂移约 `-0.0202dps`；最近 `60.0s` 拟合约 `-0.0136dps`；最近 `30.0s` 拟合约 `-0.0085dps`。
- 说明 `0.150f` 相比上一版 `0.135f` 已从正漂改成负漂，固定 bias 略微过补。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `CHASSIS_YAW_RATE_BIAS_DPS` | `0.150f` | `0.143f` | 取 `0.135f` 正漂与 `0.150f` 负漂之间的折中值，优先压低末段稳定负漂 |

### 下一步测试

- 烧录后再采集 `120s~300s` 静止 yaw-only 数据。
- 若仍负漂，继续小幅降到 `0.140f`；若回到正漂，再小幅升到 `0.145f`。

## 2026-07-06 Yaw bias 二次回调

### 测试结论

- 新 `vofa+.csv` 共 `4411` 行有效 yaw-only 数据，时长约 `88.2s`。
- 前 `20s` 出现约 `17.6deg` 变化，判断不是稳定静止段，不用于零漂精调。
- 最近 `60.0s` 展开 yaw 从约 `-0.859deg` 到 `-1.703deg`，拟合漂移约 `-0.0145dps`。
- 最近 `30.0s` 展开 yaw 从约 `-1.412deg` 到 `-1.703deg`，拟合漂移约 `-0.0097dps`。
- 时间基准正常：`steps_per_vofa` 均值约 `19.99`，`imu_dt_ms` 均值约 `1.03ms`。
- 结论：`CHASSIS_YAW_RATE_BIAS_DPS = 0.143f` 仍略高，导致稳定段轻微负漂。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `CHASSIS_YAW_RATE_BIAS_DPS` | `0.143f` | `0.138f` | 继续小幅降低固定 yaw 零偏，目标把末段 `-0.01dps` 左右负漂拉回零附近 |

### 下一步测试

- 烧录后采集时尽量保证全程静止，避免开头手动转动混入零漂数据。
- 继续以最近 `60s~120s` 稳定段的展开 yaw 拟合斜率为准；目标仍是 `|0.005dps|` 以内。

## 2026-07-06 Yaw bias 接近零漂微调

### 测试结论

- 新 `vofa+.csv` 共 `6920` 行有效 yaw-only 数据，时长约 `138.3s`。
- `CHASSIS_YAW_RATE_BIAS_DPS = 0.138f` 时，全段展开 yaw 约 `0.084deg -> -1.619deg`，拟合漂移约 `-0.0119dps`。
- 最近 `100s` 拟合漂移约 `-0.0083dps`。
- 最近 `60s` 拟合漂移约 `-0.0061dps`，最近 `30s` 拟合也约 `-0.0061dps`。
- 时间基准正常：`steps_per_vofa` 均值约 `19.98`，`imu_dt_ms` 均值约 `1.03ms`。
- 结论：已接近目标，但稳定段仍有轻微负漂，说明固定 bias 仍略高。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `CHASSIS_YAW_RATE_BIAS_DPS` | `0.138f` | `0.136f` | 小幅降低固定 yaw 零偏，目标把最近稳定段约 `-0.006dps` 的负漂压进 `|0.005dps|` 以内 |

### 下一步测试

- 烧录后再采 `120s~300s` 静止数据。
- 若最近 `60s~120s` 稳定段已经在 `|0.005dps|` 以内，停止继续微调，避免追温漂和噪声。

## 2026-07-06 暂停 Yaw 零漂追参，切到 Pitch 调试

### 判断

- Yaw 固定零偏在 `0.135~0.150dps` 间随采样条件和温度变化，继续手动追固定值会追到温漂和静止条件差异。
- 当前先保留最近参数，不继续调整 yaw bias；后续若要进一步优化，应做静止门控在线 bias 修正，而不是继续手拧常数。

### VOFA 修改

| 修改项 | 旧值/旧逻辑 | 新值/新逻辑 | 目的 |
|---|---:|---:|---|
| `VOFA_OUTPUT_CHANNEL_COUNT` (`IMU_ATTITUDE`) | `5U` | `8U` | 切到 pitch 姿态调试通道 |
| VOFA IMU 通道 | `gyro_z_dps,yaw_deg,yaw_rate_dps,imu_dt_ms,steps_per_vofa` | `kalman_theta_deg,gyro_theta_rate_dps,accel_theta_deg,accel_x_ms2,accel_y_ms2,accel_z_ms2,imu_dt_ms,steps_per_vofa` | 观察 pitch Kalman 角、pitch gyro rate、加速度角、三轴加速度和时间基准 |

### 下一步测试

- 静止放平采集 pitch 数据，看 `kalman_theta_deg` 与 `accel_theta_deg` 是否接近、是否有明显零点偏差。
- 手动前倾/后仰，确认 `kalman_theta_deg` 与 `gyro_theta_rate_dps` 的方向是否符合直立控制极性。

## 2026-07-06 切回速度环复测

### 判断

- 手拿平放 pitch 数据只能验证 IMU 链路和方向，不能定机械直立零点。
- SysTick 已修复后，速度环真实频率从旧数据时期的约 `125Hz` 恢复到约 `1000Hz`，因此必须重新复测左右轮速度闭环。
- 当前继续保持 `BALANCE_PID_ENABLE = 0U`，不上直立环；`WHEEL_SPEED_TEST_ENABLE = 1U`，只测轮速闭环。

### VOFA 修改

| 修改项 | 旧值 | 新值 | 原因 |
|---|---|---|---|
| `VOFA_DEBUG_STAGE` | `VOFA_DEBUG_STAGE_IMU_ATTITUDE` | `VOFA_DEBUG_STAGE_SPEED_LOOP` | 切回轮速闭环 6 路输出，复测 tick 修复后的速度环行为 |

### 当前 VOFA 通道

| 通道 | 名称 |
|---|---|
| I0 | `left_ref_rpm` |
| I1 | `left_actual_rpm` |
| I2 | `left_cmd_a` |
| I3 | `right_ref_rpm` |
| I4 | `right_actual_rpm` |
| I5 | `right_cmd_a` |

### 下一步测试

- 烧录后按 KEY 进入轮速测试。
- 测试 `S50 -> S100 -> S150 -> S200 -> S0 -> S-50 -> S-100 -> S-150 -> S0`。
- 重点观察实际转速是否跟随目标、电流是否抖动或接近限幅、正负速度方向是否正确。

## 2026-07-06 SysTick 修复后速度环复测

### 测试结论

- `vofa+.csv` 前段混有旧 pitch 数据；从约第 `2197` 行开始是速度环数据。
- 正负速度方向正确，`S0` 能停住。
- 各目标转速稳态均值跟随良好：
  - `S50`：左约 `49.19rpm`，右约 `48.92rpm`。
  - `S100`：左右约 `99.65rpm`。
  - `S150`：左约 `149.77rpm`，右约 `149.86rpm`。
  - `S200`：左约 `200.10rpm`，右约 `200.25rpm`。
  - `S-50/S-100/S-150` 均值也基本贴近目标。
- 稳态速度峰峰值偏大：`S200` 左约 `27rpm`、右约 `25rpm`；`S-150` 左约 `29rpm`。
- 电流指令很小且没有接近 `0.35A` 限幅，稳态电流指令峰峰值多在 `0.02~0.03A`，说明不是电流不够。
- 判断：速度均值没问题，但速度环仍在追 CAN 速度反馈抖动；当前主要问题是速度反馈/控制过敏造成的可感震荡。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `WHEEL_SPEED_PID_KP_A_PER_RPM` | `0.00120f` | `0.00100f` | 小幅降低速度 P，减少追反馈噪声带来的电流抖动 |
| `LEFT_WHEEL_SPEED_FEEDBACK_LPF_ALPHA` | `0.25f` | `0.20f` | 略增强左轮速度反馈滤波，降低 CAN 速度抖动进入速度环 |
| `RIGHT_WHEEL_SPEED_FEEDBACK_LPF_ALPHA` | `0.25f` | `0.20f` | 同上 |

### 下一步测试

- 烧录后复测 `S50 -> S100 -> S150 -> S200 -> S0 -> S-50 -> S-100 -> S-150 -> S0`。
- 重点比较稳态速度峰峰值是否下降、手感震荡是否减轻、阶跃响应是否还能接受。
- 若响应明显变软，再优先把 `WHEEL_SPEED_FF_KV_A_PER_RPM` 小幅提高，不要直接把 P 加回去。

## 2026-07-06 速度环低速前馈补偿

### 测试结论

- 新数据在 `Kp=0.00100`、反馈 LPF alpha `0.20` 下，`S100/S150/S200` 约 `200ms` 已基本接近目标，响应没有明显失控变慢。
- 低速段和负向段手感偏软更明显，尤其 `S50/S-50` 和负向右轮。
- 稳态均值仍贴目标，说明整体速度前馈不是严重不足；`S200` 稳态电流也已经接近当前前馈量，不宜优先大幅增加 `WHEEL_SPEED_FF_KV_A_PER_RPM`。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `WHEEL_SPEED_FF_STATIC_A` | `0.015f` | `0.018f` | 小幅增加静摩擦/低速前馈，改善低速和换向初段响应，不放大速度反馈噪声 |

### 下一步测试

- 复测同样速度序列，重点观察 `S50/S-50` 起步是否更干脆。
- 同时确认 `S150/S200` 没有明显超调变大；若高速偏冲，再退回 `0.016~0.017f`。

## 2026-07-06 首次尝试 PID 直立环

### 修改目的

- 轮速环均值跟随已经够进入直立环方向测试，先不追求最终站稳。
- 第一轮只做手扶/架起安全测试，确认 pitch 角度、角速度、轮速输出方向和保护逻辑。
- 清理 `config.h` 中不再参与 PID 路径的旧 LQR、外环、零点搜索配置，避免后续调参混淆。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `WHEEL_SPEED_TEST_ENABLE` | `1U` | `0U` | 退出离地轮速测试模式 |
| `BALANCE_PID_ENABLE` | `0U` | `1U` | 启用 PID 直立环 |
| `VOFA_DEBUG_STAGE` | `VOFA_DEBUG_STAGE_SPEED_LOOP` | `VOFA_DEBUG_STAGE_BALANCE_CASCADE` | 输出直立环 12 通道数据 |
| `BALANCE_PID_CURRENT_LIMIT_A` | `0.35f` | `0.25f` | 首次测试先降低电流上限 |
| `BALANCE_PID_CURRENT_SLEW_A_PER_SEC` | `20.0f` | `10.0f` | 降低电流变化速度，避免方向错误时过猛 |
| `BALANCE_PID_SAFE_ANGLE_DEG` | `45.0f` | `30.0f` | 首次测试更早触发角度保护 |
| `BALANCE_PID_SAFE_RATE_DPS` | `900.0f` | `600.0f` | 首次测试更早触发角速度保护 |
| `BALANCE_PID_BALANCE_SPEED_LIMIT_RPM` | `400.0f` | `200.0f` | 限制直立环给轮速内环的目标转速 |
| `BALANCE_PID_ANGLE_KP_RPM_PER_DEG` | LQR 推导值 | `7.96f` | 改成 PID 显式调参量 |
| `BALANCE_PID_RATE_KD_RPM_PER_DPS` | LQR 推导值 | `0.535f` | 改成 PID 显式调参量 |
| `BALANCE_PID_POLARITY` | `BALANCE_LQR_FORCE_SIGN` | `1.0f` | 方向符号显式放在 PID 配置中 |
| `BALANCE_PID_LEFT_YAW_SIGN` | `BALANCE_LQR_LEFT_YAW_SIGN` | `(-1.0f)` | 删除 LQR 依赖 |
| `BALANCE_PID_RIGHT_YAW_SIGN` | `BALANCE_LQR_RIGHT_YAW_SIGN` | `1.0f` | 删除 LQR 依赖 |

### 当前 VOFA 通道

| 通道 | 名称 |
|---|---|
| I0 | `theta_deg` |
| I1 | `theta_rate_dps` |
| I2 | `accel_deg` |
| I3 | `theta_ref_deg` |
| I4 | `balance_ref_rpm` |
| I5 | `left_ref_rpm` |
| I6 | `left_actual_rpm` |
| I7 | `left_cmd_a` |
| I8 | `right_ref_rpm` |
| I9 | `right_actual_rpm` |
| I10 | `right_cmd_a` |
| I11 | `fault` |

### 首次测试流程

- 烧录后手扶车体或架起，先不要放手。
- 按 KEY 授权进入直立环。
- 小角度前倾/后仰，观察 `theta_deg` 与 `balance_ref_rpm/left_ref_rpm/right_ref_rpm` 的符号关系，并确认轮子是否朝“救车方向”转动。
- 若一倾斜轮子就把车推得更倒，立即按 KEY/断电，下一步只改 `BALANCE_PID_POLARITY` 的符号。
- 采集 `vofa+.csv` 后优先看 `theta_deg`、`theta_rate_dps`、`balance_ref_rpm`、左右轮 `ref/actual/cmd`、`fault`。

## 2026-07-06 直立环极性确认与第二版手扶参数

### 测试结论

- 新 12 路 VOFA 数据显示极性基本正确：
  - `theta_deg < 0` 时，`balance_ref_rpm/left_ref_rpm/right_ref_rpm` 基本为负。
  - `theta_deg > 0` 时，`balance_ref_rpm/left_ref_rpm/right_ref_rpm` 基本为正。
  - 左右轮实际转速与目标转速大多数同号。
- 过零附近少数几行 `theta_deg` 与 `balance_ref_rpm` 短暂反号，是角速度 D 项在刹车，不判断为极性错误。
- `fault=3` 首次出现在 `theta_deg` 约 `-31deg`，与当前 `BALANCE_PID_SAFE_ANGLE_DEG = 30deg` 一致，角度保护正常。
- 手扶反馈：会抖，拉回效果一般。判断第一版角速度 D 偏硬、角度 P 偏弱，且首次限流略保守。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `BALANCE_PID_CURRENT_LIMIT_A` | `0.25f` | `0.30f` | 小幅增加轮速内环可用电流，增强拉回能力 |
| `BALANCE_PID_CURRENT_SLEW_A_PER_SEC` | `10.0f` | `12.0f` | 配合限流略放开，但仍保持温和输出 |
| `BALANCE_PID_BALANCE_SPEED_LIMIT_RPM` | `200.0f` | `250.0f` | 增加直立环最大救车轮速，避免中等角度过早饱和 |
| `BALANCE_PID_ANGLE_KP_RPM_PER_DEG` | `7.96f` | `10.50f` | 提高角度 P，小角度时更主动拉回 |
| `BALANCE_PID_RATE_KD_RPM_PER_DPS` | `0.535f` | `0.350f` | 降低角速度 D 对陀螺仪/手扶扰动的敏感度，减轻抖动 |

### 下一步测试

- 只做手扶小角度测试，尽量控制在 `±3~8deg`。
- 重点观察 `theta_deg` 接近零点时是否还高频抖、`balance_ref_rpm` 是否频繁正负跳变。
- 若仍抖但拉力够：继续降 `BALANCE_PID_RATE_KD_RPM_PER_DPS` 到 `0.25~0.30f`。
- 若不抖但仍拉不回来：优先继续小幅加 `BALANCE_PID_ANGLE_KP_RPM_PER_DEG` 或限流，不改极性。

## 2026-07-06 回到轮速内环重调

### 判断

- 新直立环数据中，`balance_ref_rpm` 已经多次打到 `±250rpm`，但实际轮速平均只达到目标的约 `35%~44%`。
- 说明当前主要瓶颈不是直立环极性，而是轮速内环对快速变化目标的跟随偏慢。
- 因此先关闭直立环，回到独立轮速测试；否则外环目标频繁变化，会干扰内环判断。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `BALANCE_PID_ENABLE` | `1U` | `0U` | 关闭直立环，只测轮速内环 |
| `WHEEL_SPEED_TEST_ENABLE` | `0U` | `1U` | 打开轮速测试模式 |
| `VOFA_DEBUG_STAGE` | `VOFA_DEBUG_STAGE_BALANCE_CASCADE` | `VOFA_DEBUG_STAGE_SPEED_LOOP` | 输出 6 路轮速内环数据 |
| `WHEEL_SPEED_PID_KP_A_PER_RPM` | `0.00100f` | `0.00130f` | 提高轮速误差响应 |
| `WHEEL_SPEED_CURRENT_SLEW_A_PER_SEC` | `8.0f` | `20.0f` | 允许轮速电流更快跟随目标变化 |
| `WHEEL_SPEED_TEST_AUTO_SEQUENCE_ENABLE` | 新增 | `1U` | 按 KEY 后自动执行轮速阶跃序列 |
| `WHEEL_SPEED_TEST_AUTO_STEP_MS` | 新增 | `1500U` | 每档保持 1.5s |

### 当前 VOFA 通道

| 通道 | 名称 |
|---|---|
| I0 | `left_ref_rpm` |
| I1 | `left_actual_rpm` |
| I2 | `left_cmd_a` |
| I3 | `right_ref_rpm` |
| I4 | `right_actual_rpm` |
| I5 | `right_cmd_a` |

### 自动测试序列

- 烧录后按 KEY 授权，只做离地轮速测试，不上直立环。
- 按 KEY 后自动执行：`S50 -> S0 -> S100 -> S0 -> S150 -> S0 -> S200 -> S0 -> S-50 -> S0 -> S-100 -> S0 -> S-150 -> S0`。
- 每段保持 `1.5s`，重点看实际轮速是否能更快接近目标、电流是否长期顶到 `0.35A`、是否出现持续振荡。
- 若实际轮速仍明显跟不上且电流未打满，再继续加 `WHEEL_SPEED_PID_KP_A_PER_RPM`；若电流打满，则考虑提高测试限流或继续优化前馈。

## 2026-07-06 轮速内环自动序列复测结论

### 测试结论

- 自动序列正常执行，VOFA 为 6 路速度环数据。
- 当前 `WHEEL_SPEED_PID_KP_A_PER_RPM = 0.00130f`、`WHEEL_SPEED_CURRENT_SLEW_A_PER_SEC = 20.0f` 下，独立轮速内环表现足够好：
  - `S50` 稳态左约 `47.5rpm`，右约接近 `50rpm`，约 `0.04s` 达到 80%。
  - `S100` 稳态左约 `99.1rpm`，约 `0.06s` 达到 80%。
  - `S150` 稳态左约 `150.9rpm`，约 `0.06s` 达到 80%。
  - `S200` 稳态左约 `202.8rpm`，约 `0.06s` 达到 80%。
  - 负向 `S-50/S-100/S-150` 也能快速贴近目标。
- 全序列电流没有接近 `0.35A` 限幅，说明独立轮速测试下不是电流不够。
- 速度反馈峰峰值仍有约 `13~25rpm`，主要像 CAN 速度反馈抖动/量化，不再优先通过继续加速度 P 解决。
- 结论：独立轮速内环已经可以回到直立环；之前直立时跟不上，主要因为直立路径仍使用更保守的 `BALANCE_PID_CURRENT_LIMIT_A = 0.30f` 和 `BALANCE_PID_CURRENT_SLEW_A_PER_SEC = 12.0f`。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `WHEEL_SPEED_TEST_ENABLE` | `1U` | `0U` | 退出轮速自动测试 |
| `WHEEL_SPEED_TEST_AUTO_SEQUENCE_ENABLE` | `1U` | `0U` | 关闭自动序列，避免后续混淆 |
| `BALANCE_PID_ENABLE` | `0U` | `1U` | 回到直立环手扶测试 |
| `VOFA_DEBUG_STAGE` | `VOFA_DEBUG_STAGE_SPEED_LOOP` | `VOFA_DEBUG_STAGE_BALANCE_CASCADE` | 恢复直立环 12 路观察 |
| `BALANCE_PID_CURRENT_LIMIT_A` | `0.30f` | `0.35f` | 让直立路径有与轮速测试一致的可用电流余量 |
| `BALANCE_PID_CURRENT_SLEW_A_PER_SEC` | `12.0f` | `20.0f` | 让直立路径的轮速内环响应跟上快速变化目标 |
| `BALANCE_PID_RATE_KD_RPM_PER_DPS` | `0.350f` | `0.300f` | 进一步减小角速度项导致的来回切换 |

### 下一步测试

- 继续手扶小角度直立测试，尽量控制在 `±3~8deg`。
- 重点看 `balance_ref_rpm` 和左右轮实际速度是否比上一轮更同步，以及 `theta_deg` 是否还频繁过零摆动。
- 若仍抖且轮速已跟得上，下一步再降 `BALANCE_PID_RATE_KD_RPM_PER_DPS`；若不抖但拉回仍弱，再考虑小幅加 `BALANCE_PID_ANGLE_KP_RPM_PER_DEG`。

## 2026-07-06 直立环发散数据复盘

### 数据判断

- 新 VOFA 数据为 12 路直立级联输出，闭环运行约 `2.02s~31.52s` 后触发角度保护。
- 控制段内 `theta_deg` 平均绝对值约 `4.43deg`，但 `theta_rate_dps` 平均绝对值约 `81.9dps`，并出现 `±300dps` 以上的大幅摆动。
- `balance_ref_rpm` 已经能跟随角度和角速度变化，目标轮速与实际轮速方向大部分一致；独立轮速内环不是当前第一瓶颈。
- 异常点在姿态量：`theta_deg` 与 `accel_deg` 同时刻相关性几乎为零，`d(theta)/dt` 与 `theta_rate_dps` 相关性也很低，说明动态时控制角存在明显错相/被运动加速度拉偏。
- 因此当前“不收敛、越摆越大”优先按姿态角来源问题处理，而不是继续单纯加大直立 P/D 或电流。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `BALANCE_PID_KALMAN_ANGLE_WEIGHT` | `0.85f` | `1.0f` | 控制端只使用 Kalman 角，去掉额外加速度计混合 |
| `BALANCE_PID_ACCEL_ANGLE_WEIGHT` | `0.15f` | `0.0f` | 避免运动时加速度角直接拖动控制角 |
| `BALANCE_KALMAN_R_MEASURE` | `0.110f` | `0.500f` | 降低 Kalman 对加速度角的信任，让陀螺仪短时动态占主导 |

### 下一步测试

- 继续手扶小角度测试，先不要让车自由跑，控制在 `±3~8deg`。
- 重点看 `theta_deg` 是否明显更连续，`d(theta)/dt` 与 `theta_rate_dps` 是否同向，`balance_ref_rpm` 是否不再被加速度角拖着反复抽动。
- 如果仍出现同样的发散，再继续处理 Kalman 加速度门控或进一步提高 `BALANCE_KALMAN_R_MEASURE`，而不是优先加大直立环增益。

## 2026-07-06 直立环小角度拉力不足假设

### 数据判断

- 新数据按 KEY 后约 `0.74s` 触发 `fault=3`，说明当前还没有形成稳定收敛。
- 小角度初段 `|theta| <= 5deg` 时，平均 `|theta|` 约 `2.9deg`，平均 `|balance_ref_rpm|` 约 `43rpm`。
- 初段实际轮速明显滞后，车体角度先继续放大；后面 `balance_ref_rpm` 打到 `±250rpm` 时已经属于补救阶段。
- 本轮先接受“车一直在动，不用把按 KEY 前数据当静止零偏”的判断，不再切 IMU 零偏诊断；优先验证角度 P 是否偏小。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `BALANCE_PID_ANGLE_KP_RPM_PER_DEG` | `10.50f` | `14.00f` | 增强小角度初段给轮速内环的目标速度，避免等角速度起来后才补救 |

### 下一步测试

- 保持手扶小角度，仍然不要直接放手。
- 重点看按 KEY 后前 `0.2s`：`theta_deg` 是否还从 `2~3deg` 快速放大到 `7deg+`。
- 若初段能更早刹住但过零后反向冲得更厉害，再降 `BALANCE_PID_RATE_KD_RPM_PER_DPS` 或限制 `BALANCE_PID_BALANCE_SPEED_LIMIT_RPM`。
- 若初段仍拉不住且 `balance_ref_rpm` 很快打满，再不是继续加 P，而要看轮速实际响应、电流限幅和机械重心。

## 2026-07-06 串口在线调 PID

### 修改目的

- 避免每改一次直立环参数都重新烧录。
- 通过当前 VOFA 串口 RX 命令在线修改 RAM 中的直立环参数；重启后仍回到 `config.h` 默认值。
- 命令解析放在速度命令之前，避免 `P14` 被旧逻辑误识别成速度目标 `14rpm`。

### 支持命令

| 命令示例 | 参数 |
|---|---|
| `P14` / `KP=14` | `balance_pid_runtime_kp`，角度 P |
| `I0` / `KI=0` | `balance_pid_runtime_ki`，角度 I |
| `D0.45` / `KD=0.45` | `balance_pid_runtime_kd`，角速度 D |
| `L250` / `LIMIT=250` / `O250` | `balance_pid_runtime_speed_limit_rpm`，直立环输出目标轮速限幅 |
| `C0.35` / `CUR=0.35` | `balance_pid_runtime_current_limit_a`，直立路径速度内环电流限幅 |
| `R20` / `SLEW=20` | `balance_pid_runtime_current_slew_a_per_sec`，直立路径电流变化率 |

### 使用说明

- VOFA 串口发送 ASCII 命令，末尾带换行。
- 在线修改后会立即作用到直立环，并清掉角度 PID 积分项。
- 当前先不回发长 ACK，避免和周期性 CSV 输出争用发送缓冲；是否生效通过下一段 VOFA 数据判断。
- 轮速测试命令 `S100` 仍保留，用于速度环测试。

### 代码解耦

- 串口命令解析已从 `Application/ins/ins_task.c` 拆出到 `Application/gimbal/chassis_serial_cmd.c/.h`。
- `ins_task.c` 只保留 UART 收行队列，收到一行后调用 `ChassisSerialCmd_HandleLine(line, len)`。
- 新文件已加入 `Makefile` 和 `MDK-ARM/spi_bmi088.uvprojx`，避免不同构建入口漏编译。

## 2026-07-06 定位"小角度甩飞"根因：姿态 Kalman 被轮子加速度污染

### 数据结论（12 路 BALANCE_CASCADE 数据，14349 行；注意：所有"站立"段均为手扶，车不能自立）

- 极性复核（全部关闭怀疑）：
  - 救车方向正确：轮子加速后 60~100ms 车身角加速度转负（滞后相关 -0.18~-0.32），是负反馈指纹，`BALANCE_PID_POLARITY` 不用再改。
  - 速度/位置外环 corr(轮速, theta_ref) = -0.89，级联形式正确；且 theta_ref 限幅 ±2deg，与"立不起来"无关。
  - 差速 YAW 极性未测，但站立阶段 turn_ref=0，不影响当前问题。
- 带载电流→轮加速度实测：回归得 b0 约 `800~1500 rpm/s/A`（r 仅 0.17~0.33，只能给区间），对应 `0.35A` 可救角度约 `5~10deg`。
- 电流限幅不是当前第一瓶颈：`balance_ref` 打满 ±250rpm 时，电流命令只有 10% 时间碰到 `0.35A`，平均只用 `0.27A`；速度环误差均值 167rpm × Kp 0.0013 ≈ 0.217A，对得上——是速度环偏软没把电流用出去。
- **真正根因：姿态估计器在动态下输出假角。**
  - 按 Q=0.012/Qb=0.003/R=0.5 仿真，加速度修正 63% 时间常数仅 ~0.18s，直立摆动 ~1Hz 时污染大部分穿透。
  - 实测 d(theta)/dt 对 gyro rate 回归斜率只有 0.44（陀螺主导应 ~1.0），theta 一多半变化来自加速度修正而非陀螺积分。
  - 轮子加速度折算假角 std = 14.2deg，与 theta 本身波动同量级；高动态时刻 corr(theta, accel_deg) = **-0.36（反相）**。
  - 甩飞回路：前倾 → 轮子加速去救 → 加速度计被污染 → 估计角被拽向反方向 → 控制器反打 → 甩飞。小角度时假信号占比最大，所以恰恰小角度最炸。
- 静止扶稳段 theta 均值 -2.6deg（accel_deg -2.8deg，两源一致），存在 ~2deg 机械零点偏置，`BALANCE_ANGLE_OFFSET_DEG` 尚未补。

### 修改内容

| 修改项 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `BALANCE_KALMAN_Q_ANGLE` | `0.012f` | `0.001f` | 把加速度修正时间常数从 ~0.18s 放慢到 ~1.2s，陀螺积分为主体 |
| `BALANCE_KALMAN_Q_BIAS` | `0.003f` | `0.0001f` | 旧值让污染同时写坏陀螺零漂估计；零漂本身变化很慢 |
| `BALANCE_KALMAN_R_MEASURE` | `0.500f` | `5.000f` | 同上，降低对加速度角的信任 |
| `BALANCE_ACCEL_GATE_ENABLE` | `0`（宏存在但无实现） | `1`（已在 `ins_task.c` 实现） | 加速度模长偏离 `0.75~1.25g` 时只做 `KalmanAngle_Predict` 陀螺积分，跳过加速度修正，拦截轮子加减速瞬态污染 |
| `VOFA_DEBUG_STAGE` | `BALANCE_CASCADE` | `IMU_ATTITUDE` | 先验证估计器，再回直立环 |

- 代码改动：`ins_task.c` Kalman 更新处新增门控分支（`sqrtf` 模长判断 → 越界走 `KalmanAngle_Predict`，在界内走 `KalmanAngle_Update`）。已编译通过。

### 验收测试（关闭闭环：不按 KEY，电机保持急停，手拿车做）

1. 手扶到真正直立平衡附近 ~30s：`kalman_theta` 与 `accel_theta` 应重合并接近 `0deg`；平放/倒下不是平衡零点，应接近 `+90deg`。
2. **水平快速前后推拉（不倾斜）——核心测试**：`accel_theta` 大幅乱摆属正常，但 `kalman_theta` 应基本不动（<2deg）。旧参数下此测试必挂。
3. 倾斜到某角度停住：`kalman_theta` 跟随，停住后 ~1s 内收敛到 `accel_theta`，不振荡。
4. 保持固定倾角 60s：`kalman_theta` 不应慢慢漂走；若漂，说明 R 过大，往回收。

### 估计器通过后的调参路线（先别做，逐条来）

1. 用真正直立平衡附近的数据微调 `BALANCE_ANGLE_OFFSET_DEG`；不能用平放姿态当零点。
2. 回 `BALANCE_CASCADE`，手扶直立。刚度目标：重力需求约 `0.035~0.065 A/deg`，当前 `14×0.0013=0.018 A/deg` 差 2~3.5 倍。优先提直立路径速度内环等效 Kp 至 ~0.003 A/rpm（把 0.35A 真正用满），角度 P 用串口 `P25` 附近，`D0.5` 跟上。
3. `0.35A` 限幅先不动；若之后电流长期顶满仍救不回，再提。

## 2026-07-06 发现陀螺仪符号反向：昨天的 Kalman 修改暴露了它，并放大了危害

### 数据结论（新 vofa+.csv，12 路 BALANCE_CASCADE，3092 行，含大量 fault=3 手动晃车段）

- **决定性证据（电机断开的 fault=3 段，加速度角无污染）**：`d(accel_deg)/dt` 对 `theta_rate_dps` 回归斜率 = **-1.079**（corr -0.76）。幅度 ≈1 说明轴选对了，符号为负说明**陀螺仪与加速度角对同一倾倒方向的符号相反**。
- 全段 corr(theta, accel_deg) = **-0.95**，theta 与 accel_deg 的差从 2.4deg 单调发散到 **144.8deg**：陀螺主导后 theta 朝错误方向积分，加速度修正（又慢又常被门控跳过）拉不回来。
- `d(theta)/dt` 对 rate 斜率 = 1.014：滤波器内部自洽——它忠实地积分了一个符号错误的角速度。
- **必须诚实记录**：这个符号错误一直存在，但旧参数（R=0.5，加速度主导）下 theta ≈ 加速度角，错误的陀螺项只表现为噪声/抖动；昨天把滤波器改成陀螺主导后，theta 变成错误方向的纯积分，问题从"隐性抖动"变成"显性发散"。昨天归因"轮子加速度污染"只对了一半，真正的第一根因是陀螺符号。
- 症状全部对上：
  - "有时极性正确有时反向"：theta 带着巨大漂移偏置，符号与真实倾角随机对不上，轮子方向自然看起来随机。
  - "按 KEY 没反应"：theta 被错误积分卡在 30deg 外，KEY 解锁后 0.1s 内立刻再触发角度保护（数据里 13.5~17.8s 有 8 次解锁→0.1s 内再故障的循环），表现就是"按了没反应"。
  - D 项 = Kd × 反号角速度 = **正反馈注能**，解释了此前"会抖、小角度甩飞"。
- 符号错误的来源推断:`BALANCE_ACCEL_ANGLE_SIGN = -1` 为控制约定翻转了加速度角，但 `BALANCE_GYRO_SIGN` 留在 +1，两源约定不一致。救车方向极性是按加速度角主导的 theta 验证的，所以保持加速度角符号不动、翻转陀螺仪。

### 修改内容

| 修改项 | 旧值/旧逻辑 | 新值/新逻辑 | 原因 |
|---|---|---|---|
| `BALANCE_GYRO_SIGN` | `1.0f` | `(-1.0f)` | 与加速度角符号约定对齐；theta 积分方向、D 项阻尼方向、角速度保护全部恢复正确 |
| KEY 解锁路径 | 直接 `emergency_stop = 0` | 解锁瞬间调用新增 `INS_RequestBalanceAngleReseed()`，用当前加速度角重播种 Kalman | 故障期间累计的陀螺积分误差不带进新一轮控制，消除"解锁即再故障"的死循环 |
| `ins_task.c` | 无 | 新增 `balance_reseed_request` 标志 + `INS_RequestBalanceAngleReseed()`（声明加入 `ins_task.h`） | 跨任务安全触发重播种 |

- 昨天的 Q/R/门控修改**保留**：陀螺符号修正后，陀螺主导 + 加速度门控才是正确组合。已编译通过。

### 验收测试（VOFA 保持 BALANCE_CASCADE，theta/rate/accel_deg 在 I0/I1/I2，故障时也可观察）

1. **不按 KEY**，手拿车缓慢前后倾斜：`theta_deg`（I0）与 `accel_deg`（I2）应同向、幅度接近；`theta_rate_dps`（I1）与 theta 变化方向一致。这一条过了，符号链路才算闭合。
2. 快速水平推拉（不倾斜）：theta 应基本不动（验证昨天的门控+慢修正在符号正确后按预期工作）。
3. 按 KEY 解锁：不应再出现"解锁 0.1s 内立刻 fault=3"；倾斜超 30deg 触发保护后扶正、再按 KEY，应能正常进入控制。
4. 手扶直立：D 项方向已从注能变为阻尼，抖动应明显减轻；极性应稳定不再"时对时反"。

### 测试通过后再做

- 读静止 theta 均值填 `BALANCE_ANGLE_OFFSET_DEG`（旧数据显示约 2deg 量级，但那是符号错误时期的数据，以新数据为准）。
- 再按既定路线调刚度（内环等效 Kp → `P25`/`D0.5`）。

## 2026-07-06 水平推拉测试复盘与 innovation gate

### 数据判断

- 本组 `vofa+.csv` 是 8 路 `IMU_ATTITUDE` 数据，约 `109.7s`。
- 用户确认该组包含“手扶保持角度不变，快速水平前后推拉”测试；不能再要求重复同一测试。
- 最像纯水平推拉的一段约 `40s~45s`：
  - `accel_theta` 峰峰值约 `88.6deg`，说明加速度角被水平加速度严重污染。
  - `kalman_theta` 峰峰值约 `8.1deg`，说明新 Kalman/模长门控已经明显压住污染，但仍残留约 `8deg` 假角。
  - `gyro_rate` 平均绝对值约 `2.9dps`，支持“车体角度没有真实大幅转动”的判断。
- `45s~75s` 段 `gyro_rate` 到数百 `dps`，判断包含真实转动/翻动，不能作为纯水平推拉评价。
- 结论：只靠加速度模长门控还不够。水平推拉时可能 `|a|` 接近 `1g`，但 `accel_theta` 已经是假角，因此需要加一层角度创新量门控。

### 修改内容

| 修改项 | 新值/逻辑 | 原因 |
|---|---:|---|
| `BALANCE_ACCEL_ANGLE_GATE_DEG` | `10.0f` | `accel_angle` 与 gyro 预测角相差超过阈值时，判定为异常测量 |
| `KalmanAngle_Correct()` | 新增 | 将 Kalman 更新拆成 `Predict` 和 `Correct`，避免做 innovation gate 时重复预测 |
| `ins_task.c` Kalman 更新 | 先 `KalmanAngle_Predict()`，再检查 `|a|` 和 `|accel_angle - predicted_angle|`，通过才 `KalmanAngle_Correct()` | 拦截“模长正常但角度是假”的水平加速度污染 |

### 下一步验收

- 继续保持 `VOFA_DEBUG_STAGE_IMU_ATTITUDE`，不按 KEY，电机急停。
- 再做一次同样的手扶水平前后推拉，目标：
  - `accel_theta` 可以大幅乱跳。
  - `kalman_theta` 在纯水平推拉段尽量压到 `±2deg` 内；若仍有 `5deg+` 假角，把 `BALANCE_ACCEL_ANGLE_GATE_DEG` 从 `10deg` 降到 `8deg`。
  - 固定真实倾角停住后，`kalman_theta` 仍应能慢慢回到 `accel_theta`，如果回得太慢或漂移，再讨论放宽阈值或回调 `R`。

## 2026-07-06 innovation gate 后的新 VOFA 数据结论

### 数据判断

- 本组 `vofa+.csv` 为 8 路 `IMU_ATTITUDE` 数据，3427 行，按 `imu_dt_ms * steps_per_vofa` 还原时长约 `70.8s`。
- `0s~10s` 手扶静止/小动作段：
  - `kalman_theta` 峰峰值约 `1.39deg`，均值约 `-1.83deg`。
  - `accel_theta` 峰峰值约 `1.32deg`，均值约 `-2.43deg`。
  - `gyro_rate` 平均绝对值约 `0.84dps`。
  - 说明小角度静态下 Kalman 与加速度角一致，传感器小角度参考可信。
- `18s~35s` 固定角度水平快速前后推拉段：
  - `accel_theta` 峰峰值约 `88.5deg`，说明加速度角被水平加速度严重污染。
  - `kalman_theta` 峰峰值约 `3.1deg`；其中 `21s~35s` 较干净段约 `2.3deg`。
  - `gyro_rate` 平均绝对值约 `2.6dps`。
  - 对比上一组约 `8deg` 假角，本次 innovation gate 已经把水平推拉污染压到可接受范围。
- `13s~18s`、`35s` 之后为用户故意做的大角度转动/接近翻转段；按当前约定，直立平衡点必须定义为 `0deg`，平放/倒下约为 `+90deg`。
- 在这段数据里，旧定义下陀螺积分到约 `+84deg`，而 `accel_theta` 到约 `-90deg`。这里不能直接据此改控制角符号，因为此前 12 路直立环数据已经验证过救车极性正确；该段包含大角度翻转/分支，不适合作为直立小角度控制极性的依据。

### 误改复盘

| 修改项 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `BALANCE_ACCEL_ANGLE_SIGN` | `-1.0f` | 错误改成 `1.0f`，已回滚到 `-1.0f` | 该改动翻转了控制用 `theta` 极性，但 `BALANCE_PID_POLARITY` 仍保持 `1.0f`，导致直立环反打 |
| `BALANCE_ANGLE_OFFSET_DEG` | 错误尝试 `90.0f` | `0.0f` | 直立是 `0deg`、平放是 `+90deg`；offset 只用于之后几度级机械偏置微调 |

### 当前结论

- 零点定义固定为：车体直立、重心过轮轴附近是 `0deg`；平放/倒下是 `+90deg`。
- 水平推拉时加速度污染已经基本被门控拦住，`BALANCE_ACCEL_ANGLE_GATE_DEG = 10deg` 先不需要降到 `8deg`。
- 控制极性以真正闭环数据为准；后续新 12 路数据证明 `BALANCE_PID_POLARITY = 1.0f` 会反打，已改为 `BALANCE_PID_POLARITY = -1.0f`。`BALANCE_ACCEL_ANGLE_SIGN = -1.0f` 保持不动。

### 当前结论与下一步

- 这组采样已经完成 IMU/角度定义判断，不需要再重复做 IMU 验证。
- 最终角度约定固定为：直立平衡附近 `theta = 0deg`，平放/倒下约 `theta = +90deg`。
- 旧问题不是 `BALANCE_ACCEL_ANGLE_SIGN = -1.0f`；把它改成 `+1.0f` 是误改，会导致直立环极性反掉。
- 当前代码设置：
  - `BALANCE_ACCEL_ANGLE_SIGN = -1.0f`
  - `BALANCE_PID_POLARITY = -1.0f`
  - `BALANCE_ANGLE_OFFSET_DEG = 0.0f`
  - `VOFA_DEBUG_STAGE = VOFA_DEBUG_STAGE_BALANCE_CASCADE`
- 下一步直接回直立级联测试，不再停留在 `IMU_ATTITUDE`。
- 直立环测试只看真正直立附近的 `|theta| < 10~15deg` 手扶段；平放/翻倒段不能用于判断 PID。
- 按 KEY 后重点看前 `0.2s`：
  - `theta_deg` 是否继续扩大。
  - `balance_ref_rpm` 是否立刻给出救车方向。
  - `left/right_actual_rpm` 是否跟得上 `left/right_ref_rpm`。
  - `left/right_cmd_a` 是否接近限幅，判断是 PID 外环慢、内环慢，还是电流没用出来。

## 2026-07-06 当前 12 路直立数据：PID 极性仍反

### 数据判断

- 本组 `vofa+.csv` 为 12 路 `BALANCE_CASCADE` 数据，3092 行。
- 第一次 `fault=3` 出现在约 `5.48s`，当时：
  - `theta_deg = +30.075deg`
  - `theta_rate_dps = -29.958dps`
  - `balance_ref_rpm = 0`（因为已经进入故障保护）
  - 上一行仍在闭环时 `theta_deg = +29.896deg`，`balance_ref_rpm = +250rpm`，左右电流均已到 `+0.35A`。
- 闭环有效段中，`theta_deg` 与 `balance_ref_rpm` 几乎始终同号：
  - `theta > 0` 时，`balance_ref_rpm > 0`。
  - `theta < 0` 时，`balance_ref_rpm < 0`。
  - 统计上同号 `1156` 行，反号仅 `1` 行。
- 实测回归显示正轮速会让 `theta` 往正方向变化，因此当前同号输出是正反馈：角度越偏，轮子越把它推向更偏。
- 结论：不是轮速内环没跟上，而是直立 PID 输出极性反了。

### 30 度退出原因

- `fault=3` 对应 `CHASSIS_FAULT_PID_ANGLE`。
- 代码使用 `abs(theta_deg) > BALANCE_PID_SAFE_ANGLE_DEG` 触发保护。
- 当前 `BALANCE_PID_SAFE_ANGLE_DEG = 30.0f`，所以无论正负方向到 `30deg` 都会退出。
- 本组第一次退出是显示角 `+30.075deg`；若现场观察为“负方向”，说明现场物理方向叫法与 VOFA 显示正负相反，但保护本身按绝对值触发。

### 修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `BALANCE_PID_POLARITY` | `1.0f` | `-1.0f` | 让 `theta > 0` 时输出负轮速、`theta < 0` 时输出正轮速，恢复负反馈 |

### 下一步测试

- 保持 `BALANCE_ACCEL_ANGLE_SIGN = -1.0f` 不再动。
- 重新烧录后只做手扶 KEY 小角度测试。
- 若极性正确，按 KEY 后 `theta_deg > 0` 时 `balance_ref_rpm` 应为负，`theta_deg < 0` 时 `balance_ref_rpm` 应为正。
- 暂时不要放宽 `BALANCE_PID_SAFE_ANGLE_DEG`；等负反馈确认后再考虑是否从 `30deg` 提到 `40~45deg`。

## 2026-07-06 直立环极性按物理方向修正

### 现场判断

- 已确认当前 IMU pitch 正方向与轮子转速正方向在物理上是一致的。
- 对平衡车而言，车体向某方向倾倒时，轮子需要向同一方向运动去追重心；因此在该方向约定下，直立环 `theta` 与目标轮速应同号。
- 上一节仅根据失控数据里 `theta` 与 `balance_ref_rpm` 同号就判定为正反馈，忽略了“轮子追重心”这一物理关系，属于极性判断误判。
- 现在 `BALANCE_PID_POLARITY = -1.0f` 会让 `theta > 0` 输出负轮速、`theta < 0` 输出正轮速，实际反向。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `BALANCE_PID_POLARITY` | `-1.0f` | `1.0f` | IMU pitch 与轮速方向一致时，直立环应让倾角误差与目标轮速同号 |

### 下一步测试

- 重新烧录后手扶小角度 KEY 测试。
- 验证方向：`theta_deg > 0` 时 `balance_ref_rpm > 0`，`theta_deg < 0` 时 `balance_ref_rpm < 0`。
- 先只确认极性和是否有阻尼感，不急着放手。

## 2026-07-06 进入直立调参阶段

### 阶段结论

- 速度内环链路已验证可用。
- IMU pitch 与轮速方向极性已按现场物理方向统一。
- 当前直立环极性使用 `BALANCE_PID_POLARITY = 1.0f`：倾角误差与目标轮速同号。
- `VOFA_DEBUG_STAGE` 保持 `VOFA_DEBUG_STAGE_BALANCE_CASCADE`，后续直接用 12 路级联数据调参。

### 调参原则

- 先手扶小角度确认阻尼和极性，再逐步增加刚度。
- 只看真正直立附近 `|theta| < 10~15deg` 的闭环段；平放、翻倒和故障后的数据不用于 PID 判断。
- 若方向正确但拉不住，优先看 `balance_ref_rpm` 是否打满、轮速实际是否跟上、电流是否碰到限幅，再决定调外环 P/D、速度内环等效刚度或电流限幅。
