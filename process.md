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

## 2026-07-06 串口调参链路增强

### 问题判断

- 原 VOFA 串口 RX 只把 `\n` 当作命令结束，收到 `\r` 会直接丢弃。
- 若串口助手或 VOFA 控件只发送回车 `\r`，命令不会入队执行，表现为 `P25/KD=0.5` 等在线调参没有生效。
- 原 12 路 `BALANCE_CASCADE` 没有直接输出运行时 Kp/Kd/限流，是否调参成功只能间接猜。

### 修改内容

| 修改项 | 新逻辑 | 原因 |
|---|---|---|
| 命令结束符 | `\r` 或 `\n` 都触发入队 | 兼容 VOFA/串口助手不同换行设置 |
| 串口命令计数 | 新增 `chassis_serial_cmd_rx_count/ok_count/last_type/last_value` | 便于确认命令是否收到、是否解析成功 |
| BALANCE_CASCADE VOFA | 从 12 路增至 16 路，末尾新增 `runtime_kp,runtime_kd,runtime_current_limit_a,serial_cmd_ok_count` | 在线调参后可直接在 VOFA 看到参数变化和成功计数 |

### 新增 VOFA 通道

- I12 `runtime_kp`
- I13 `runtime_kd`
- I14 `runtime_current_limit_a`
- I15 `serial_cmd_ok_count`

## 2026-07-06 P35/D0.3 直立调参数据

### 数据判断

- 本组 `vofa+.csv` 为 16 路 `BALANCE_CASCADE` 数据，共 `5713` 行，约 `114.26s`。
- 全程 `fault=0`，没有触发角度保护。
- 串口调参链路正常：`serial_cmd_ok_count` 从 `2` 增加到 `6`；最终运行参数为 `runtime_kp=35`、`runtime_kd=0.300`、`runtime_current_limit_a=0.350`。
- `Kp=35` 主要运行段从约 `2.86s` 持续到结束，约 `111s`。
- 全闭环段统计：
  - 平均 `|theta| ≈ 1.13deg`，最大 `|theta| = 24.03deg`。
  - 平均 `|theta_rate| ≈ 6.9dps`。
  - 平均 `|balance_ref_rpm| ≈ 42rpm`。
  - 左右轮平均速度误差约 `17~18rpm`。
  - 电流打满比例很低：左约 `0.2%`，右约 `0.3%`。
- 最大角度出现在约 `20.14s`，`theta=-24.03deg`，同时 `accel=-41.5deg`、`balance_ref=-250rpm`，判断为一次明显外部扰动/快速动作；系统没有 fault，并在随后恢复。
- `33s` 之后多数 10 秒窗口平均 `|theta|` 约 `0.7~1.3deg`，说明当前已进入可调参状态，不再是极性/传感器链路错误。

### 当前结论

- `P35/D0.3` 比 `P50/D0.3` 明显更稳，适合作为下一步调参基线。
- 当前 D 仍偏保守；若想提高抗扰恢复和减少大扰动后的摆动，下一步优先试 `D0.4~D0.45`，不要先继续加 P。
- 当前数据平均角度和 `theta_ref` 都接近 0，不能从这组数据强判机械重心明显偏前；重心偏置仍需结合手感和静止直立均值判断。

## 2026-07-06 P35/D0.4 与 -2deg 偏置测试

### 数据判断

- 本组 `vofa+.csv` 为 16 路 `BALANCE_CASCADE` 数据，共 `8272` 行，约 `165.44s`。
- 最终运行参数为 `runtime_kp=35`、`runtime_kd=0.400`、`runtime_current_limit_a=0.350`。
- 全数据中 `fault=3` 共 `319` 行，分成两段：
  - `6.02s~9.76s`：`Kp=14/D0.3`，触发前 `theta=-25.58deg`、`theta_rate=-212.6dps`，随后角度继续到约 `-81deg`。
  - `26.36s~28.96s`：`Kp=35/D0.3`，触发前 `theta=29.98deg`、`theta_rate=255dps`。
- `D0.4` 生效后没有再出现 fault。
- `P35/D0.4` 在 `62.52s~107.00s` 段：平均 `|theta|≈0.82deg`，最大约 `6.28deg`。
- 最终 `theta_ref=-2deg` 段（约 `148.60s~165.42s`）：
  - 平均 `theta≈-2.02deg`，说明车能跟住 `-2deg` 目标偏置。
  - 平均 `|theta-theta_ref|≈0.62deg`。
  - 平均 `|theta_rate|≈4.1dps`。
  - 平均 `|balance_ref_rpm|≈21.9rpm`。
  - 左右轮平均速度误差约 `9~10rpm`。

### 当前结论

- `P35/D0.4` 明显优于 `P35/D0.3`，阻尼更好，后半段数据更干净。
- `theta_ref=-2deg` 偏置能稳定跟住，说明如果手感上确实需要前/后倾补偿，`B` 偏置链路是可用的。
- 但是否把 `-2deg` 固化为机械零点，还需要现场确认：如果加 `B-2` 后车明显更愿意站住、轮子平均速度/位置漂移更小，再考虑固化；否则只作为手动偏置测试结果。

## 2026-07-06 固化当前直立调参基线

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `BALANCE_PID_ANGLE_KP_RPM_PER_DEG` | `14.00f` | `35.00f` | 固化当前稳定的角度 P 基线 |
| `BALANCE_PID_RATE_KD_RPM_PER_DPS` | `0.300f` | `0.400f` | 固化当前阻尼更好的 D 参数 |
| `BALANCE_ANGLE_OFFSET_DEG` | `0.0f` | `-2.0f` | 将 `B-2` 对应的重心/机械偏置写入固定角度零点；烧录后默认 `theta_ref=0` 即等效于此前手动 `B-2` |

### 烧录后验证

- 不再发送 `B-2`，保持默认 `B0`。
- VOFA 中 `runtime_kp` 应为 `35`，`runtime_kd` 应为 `0.4`。
- 直立静态附近 `theta_deg` 应接近 `0deg`，`balance_ref_rpm` 长期均值应接近 `0rpm`。

## 2026-07-06 固化参数烧录后验证

### 数据判断

- 本组 `vofa+.csv` 为 16 路 `BALANCE_CASCADE` 数据，共 `1709` 行，约 `34.18s`。
- 未发送串口调参命令：`serial_cmd_ok_count=0`。
- 固化参数已生效：`runtime_kp=35`，`runtime_kd=0.400`，`runtime_current_limit_a=0.350`。
- 全程 `fault=0`。
- 全段均值：
  - `theta_avg≈0.18deg`，`theta_ref_avg≈0.09deg`。
  - 平均 `|theta-theta_ref|≈1.01deg`。
  - `balance_ref_avg≈1.69rpm`，长期均值接近 0。
  - 左右轮平均速度误差约 `11.6rpm/12.8rpm`。
- 最后约 `5s` 安静段：
  - `theta_avg≈0.30deg`。
  - 平均 `|theta-theta_ref|≈0.37deg`。
  - `balance_ref_avg≈3.6rpm`。
- 约 `14.3s` 有一次明显外部扰动/快速动作：`accel` 瞬间到约 `55deg`，`theta_rate` 到约 `174dps`，`balance_ref` 打到 `250rpm`，但未触发 fault，并在约 `0.3s` 内回到小角度。

### 当前结论

- `BALANCE_ANGLE_OFFSET_DEG=-2.0f` 固化方向正确，默认 `B0` 已基本等效于此前手动 `B-2`。
- 当前 `P35/D0.4/offset=-2` 可作为直立短站和轻推扰动测试基线。
- 下一步不优先改偏置；若仍有回摆，再小幅试 `D0.45`，若小角度偏软再试 `P38~40`。

## 2026-07-06 转向模式与位置/速度外环阶段

### 转向模式验证

- 最新完整转向测试中，`serial_cmd_ok_count` 按 `0 -> 1 -> 2 -> 3 -> 4` 变化，对应 `left -> stand -> right -> stand` 四条命令均被串口正确解析。
- `left` 模式下左右轮目标出现 `-45/+45rpm` 差速；`right` 模式下出现 `+45/-45rpm` 差速；两次 `stand` 后差速均回到 0。
- 新一轮转向测试全程 `fault=0`，转向段电流未饱和，姿态扰动小；若现场观察物理方向正确，则 `BALANCE_PID_LEFT_YAW_SIGN/BALANCE_PID_RIGHT_YAW_SIGN` 不再调整。

### 用力推车数据判断

- 用力外部推动测试中，出现多次 `fault=3`，均为角度超过安全阈值触发保护，不是反馈丢失。
- 故障前典型状态：
  - `theta_ref_deg` 已经接近原上限 `±2deg`。
  - `balance_ref_rpm` 已经接近原上限 `±250rpm`。
  - 左右轮实际平均速度可到约 `200~250rpm`。
  - 电流没有长期打满 `0.35A`，说明当前首要问题不是轮速方向错误，而是外环/直立输出权限偏小。
- 现有位置/速度外环已经工作：它由轮速估算速度，速度积分得到位置，再由 `position_error` 和 `speed_error` 计算 `theta_ref_deg`。
- `theta_ref_deg` 不是由 pitch 角速度直接计算；pitch 角速度 `theta_rate_dps` 用于内层直立环 D 项阻尼。

### 本次参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `BALANCE_PID_THETA_REF_LIMIT_DEG` | `2.0f` | `3.0f` | 放大位置/速度外环可给直立环的目标重心角权限 |
| `BALANCE_PID_BALANCE_SPEED_LIMIT_RPM` | `250.0f` | `320.0f` | 原数据中 `balance_ref_rpm` 频繁顶到 `±250rpm`，需要给直立环更多速度目标余量 |

### 下一步测试

- 先不改轮速 PID/电流环响应，烧录后继续做 `stand` 模式外力推车测试。
- 重点观察：
  - `theta_ref_deg` 是否能用到新的 `±3deg` 范围。
  - `balance_ref_rpm` 是否还长时间顶到 `±320rpm`。
  - 实际轮速是否能跟随目标，电流是否接近 `0.35A`。
- 若 `balance_ref_rpm` 提高后实际轮速仍明显跟不上、且电流仍偏小，再进入轮速响应调参；若电流已打满，则优先考虑电流/动力余量限制。

## 2026-07-06 最新外环符号判断

### 数据判断

- 最新 `vofa+.csv` 共 `5054` 行，约 `50.5s`，全程 `fault=0`。
- `theta_ref_deg` 未打到新上限：范围约 `-0.77deg ~ +1.11deg`。
- `balance_ref_rpm` 也未打到 `±320rpm`：范围约 `-172rpm ~ +235rpm`。
- 电流几乎未饱和，仅约 2 行触及 `0.35A`，说明这组数据不能支持“轮速输出能力已经不够”的结论。
- 关键现象：
  - 当实际平均轮速为正约 `+107rpm` 时，`theta_ref≈-0.77deg`，`balance_ref≈+136rpm`，控制仍在给正向速度目标。
  - 当实际平均轮速为负约 `-111rpm` 时，`theta_ref≈+1.11deg`，`balance_ref≈-122rpm`，控制仍在给负向速度目标。
- 这说明当前速度/位置外环符号会使 `balance_ref` 与已有速度同向，表现为“越推越跑”，不是单纯的轮速响应慢。

### 当前结论

- 下一步应优先翻转外环测量符号：
  - `BALANCE_PID_POSITION_SIGN` 从 `1.0f` 改为 `(-1.0f)`。
  - `BALANCE_PID_VELOCITY_SIGN` 从 `1.0f` 改为 `(-1.0f)`。
- 暂时不改轮速 PID；修正外环符号后再看实际轮速是否还跟不上目标。

## 2026-07-06 VOFA 改为外环极性诊断视图

### 修改目的

- 暂时不改 `BALANCE_PID_POSITION_SIGN/BALANCE_PID_VELOCITY_SIGN`，先只改 VOFA 输出。
- 下一次 `stand` 模式推车时，直接观察位置/速度误差、`speed_theta_ref_deg`、`balance_ref_rpm` 与实际平均轮速的符号关系。
- 目标是先确认“外环极性”而不是继续混合判断轮速响应。

### 新 16 路通道

| 通道 | 名称 | 含义 |
|---:|---|---|
| I0 | `theta_deg` | 当前车身 pitch 角 |
| I1 | `theta_ref_deg` | 最终给直立环的目标角 |
| I2 | `speed_theta_ref_deg` | 位置/速度外环贡献的目标角 |
| I3 | `position_m` | 符号处理后的估计位置 |
| I4 | `position_ref_m` | 目标位置 |
| I5 | `position_error_m` | `position_ref_m - position_m` |
| I6 | `velocity_mps` | 符号处理后的估计速度 |
| I7 | `velocity_ref_mps` | 目标速度 |
| I8 | `velocity_error_mps` | `velocity_ref_mps - velocity_mps` |
| I9 | `balance_ref_rpm` | 直立角度环输出的平均轮速目标 |
| I10 | `avg_ref_rpm` | 左右轮平均目标转速 |
| I11 | `avg_actual_rpm` | 左右轮平均实际转速 |
| I12 | `avg_cmd_a` | 左右轮平均电流命令 |
| I13 | `fault` | 底盘故障码 |
| I14 | `position_sign` | 编译期 `BALANCE_PID_POSITION_SIGN` |
| I15 | `velocity_sign` | 编译期 `BALANCE_PID_VELOCITY_SIGN` |

### 判断方法

- 只测试 `stand`，不要发 `left/right/forward`。
- 推车后若 `avg_actual_rpm` 为正，理想情况下外环应让 `balance_ref_rpm` 往反向抑制速度；若同向，说明速度/位置外环极性仍错误。
- 看 `velocity_error_mps -> speed_theta_ref_deg -> balance_ref_rpm -> avg_actual_rpm` 这条链，比只看 `theta/theta_ref` 更直接。

## 2026-07-06 翻转位置/速度外环符号

### 数据依据

- 使用外环极性诊断视图后，`position_sign=+1`、`velocity_sign=+1` 时，推车数据出现明确同向加速：
  - `avg_actual_rpm > +15rpm` 的样本中，`balance_ref_rpm` 几乎总是同为正，平均约 `+86rpm`。
  - `avg_actual_rpm < -15rpm` 的样本中，`balance_ref_rpm` 几乎总是同为负，平均约 `-66rpm`。
- 故障前 `avg_actual_rpm` 可到 `+334rpm`，`balance_ref_rpm` 打到 `+320rpm`，说明外环/内环链路在继续推动车速，而不是刹车。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `BALANCE_PID_POSITION_SIGN` | `1.0f` | `(-1.0f)` | 翻转位置测量方向，使位置误差回中方向正确 |
| `BALANCE_PID_VELOCITY_SIGN` | `1.0f` | `(-1.0f)` | 翻转速度测量方向，使速度外环对已有速度产生反向制动 |

### 下一步验证

- 烧录后仍使用外环诊断 VOFA 通道，只测 `stand` 推车。
- 期望现象：
  - `I14 position_sign=-1`，`I15 velocity_sign=-1`。
  - `avg_actual_rpm` 为正时，`balance_ref_rpm` 应倾向为负。
  - `avg_actual_rpm` 为负时，`balance_ref_rpm` 应倾向为正。
- 如果符号正确但速度仍刹不住，再判断是否需要调轮速响应或外环增益。

## 2026-07-06 拆分测试：先只测速度环极性

### 原因

- 双 sign 翻转后，整车数据仍表现为 `balance_ref_rpm` 与 `avg_actual_rpm` 同向，且触发多次 `fault=3`。
- 由于位置误差、车身大角度和速度误差混在一起，不能继续用整车大扰动直接判断两个 sign。
- 先退回原始符号，并关闭位置 P，只保留速度项，逐个确认极性。

### 临时参数

| 参数 | 当前临时值 | 目的 |
|---|---:|---|
| `BALANCE_POSITION_KP_DEG_PER_M` | `0.0f` | 暂时关闭位置回中项，只测试速度外环 |
| `BALANCE_POSITION_KV_DEG_PER_MPS` | `1.50f` | 保留速度项 |
| `BALANCE_PID_POSITION_SIGN` | `1.0f` | 退回原始符号，后续单独验证 |
| `BALANCE_PID_VELOCITY_SIGN` | `1.0f` | 退回原始符号，先验证速度项 |

### 测试判据

- 仍使用外环诊断 VOFA 16 路通道。
- 只在 `stand` 模式下做小幅推车，不做大角度摔车测试。
- 当前只看速度链：
  - `velocity_mps`
  - `velocity_error_mps`
  - `speed_theta_ref_deg`
  - `balance_ref_rpm`
  - `avg_actual_rpm`
- 若 `avg_actual_rpm` 与 `balance_ref_rpm` 仍长期同向，再只翻 `BALANCE_PID_VELOCITY_SIGN` 测一次。
- 速度 sign 确认后，再恢复 `BALANCE_POSITION_KP_DEG_PER_M` 并单独测位置 sign。

## 2026-07-06 速度极性单独测试结果与位置极性测试配置

### 速度极性测试结果

- `vofa+.csv` 为追加文件，前半段是旧的 `position_sign=-1/velocity_sign=-1` 数据，后半段才是本次 `+1/+1`、`KP_position=0` 的速度单独测试数据。
- 最新 `+1/+1` 段从约 `idx=2485` 开始，持续约 `26.8s`，全程 `fault=0`。
- 速度项表现：
  - `avg_actual_rpm > 0` 时，`speed_theta_ref_deg < 0`。
  - `avg_actual_rpm < 0` 时，`speed_theta_ref_deg > 0`。
- 这说明速度外环的目标角方向符合“对已有速度产生反向重心修正”的预期，暂定 `BALANCE_PID_VELOCITY_SIGN=1.0f` 正确。
- 但由于 `BALANCE_POSITION_KP_DEG_PER_M=0`，没有位置回中项，手感会比之前虚、不愿意回中心；这不代表直立环或速度 sign 错。

### 位置极性测试配置

| 参数 | 当前测试值 | 原因 |
|---|---:|---|
| `BALANCE_POSITION_KP_DEG_PER_M` | `1.00f` | 温和恢复位置回中，用于判断位置极性，避免直接回 `2.50f` 太激进 |
| `BALANCE_POSITION_KV_DEG_PER_MPS` | `1.50f` | 保留已初步确认方向正确的速度阻尼 |
| `BALANCE_PID_POSITION_SIGN` | `1.0f` | 先测试原始位置方向 |
| `BALANCE_PID_VELOCITY_SIGN` | `1.0f` | 暂定速度方向正确 |

### 下一步判据

- 仍使用外环诊断 VOFA 通道。
- 只测 `stand`，小幅推动后让车自己回中。
- 若 `position_m` 偏正时，位置项应推动 `speed_theta_ref_deg` 朝让 `position_m` 回 0 的方向变化；实际判断仍以低速段、非大角度摔车段为准。
- 如果恢复 `KP=1.0` 后出现明显越离中心越加速，再单独翻 `BALANCE_PID_POSITION_SIGN`，不要动 `VELOCITY_SIGN`。

## 2026-07-06 关闭位置环并修复速度环

### 修正判断

- 前面对速度 sign 的判断只看了 `avg_actual_rpm` 与 `speed_theta_ref_deg` 的关系，不够完整。
- 更可靠的判断应看完整链路最终输出：`avg_actual_rpm` 与 `balance_ref_rpm` 是否反向。
- `KP_position=1.0/KV=1.5/sign=+1/+1` 的数据中，`avg_actual_rpm` 与 `balance_ref_rpm` 仍大多数同向，说明速度项进入内层 `theta_error = theta - theta_ref` 后，最终仍表现为助推。

### 当前临时参数

| 参数 | 当前值 | 目的 |
|---|---:|---|
| `BALANCE_POSITION_KP_DEG_PER_M` | `0.0f` | 关闭位置项，排除位置回中干扰 |
| `BALANCE_POSITION_KV_DEG_PER_MPS` | `1.50f` | 只保留速度阻尼 |
| `BALANCE_PID_POSITION_SIGN` | `1.0f` | 位置项关闭，暂不测试 |
| `BALANCE_PID_VELOCITY_SIGN` | `(-1.0f)` | 修复速度项符号，验证是否能反向压速度 |

### 下一步判据

- 烧录后清空 VOFA 数据，只测 `stand` 小幅推车。
- 期望：
  - `I14 position_sign=1`。
  - `I15 velocity_sign=-1`。
  - `avg_actual_rpm > 0` 时，`balance_ref_rpm` 应倾向变负或至少明显小于旧版同向助推。
  - `avg_actual_rpm < 0` 时，`balance_ref_rpm` 应倾向变正或至少明显小于旧版同向助推。
- 若此配置手感明显能刹速度，再恢复位置项并单独测试 `POSITION_SIGN`。

## 2026-07-06 打开位置环并加入加速度阻尼

### 修改目的

- 最新速度单独测试中，`VELOCITY_SIGN=-1` 后速度项方向更合理，但 `theta_ref_deg` 幅度仍偏小，推车后回中/刹停能力不足。
- 本次恢复一个温和的位置回中项，同时增大速度阻尼，并加入基于轮速变化率的加速度阻尼，先让外环具备“速度刹住 + 位置回中”的完整能力。

### 当前参数

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `BALANCE_POSITION_KP_DEG_PER_M` | `0.80f` | 温和打开位置环，不直接回到激进值 |
| `BALANCE_POSITION_KV_DEG_PER_MPS` | `2.50f` | 增强速度阻尼，让推车后的目标倾角更明显 |
| `BALANCE_POSITION_KA_DEG_PER_MPS2` | `0.10f` | 新增加速度阻尼，抑制速度继续发散 |
| `BALANCE_POSITION_KI_DEG_PER_M_S` | `0.0f` | 暂不打开位置积分，避免低速慢漂时积累过量 |
| `BALANCE_PID_POSITION_SIGN` | `1.0f` | 先使用原始位置方向 |
| `BALANCE_PID_VELOCITY_SIGN` | `(-1.0f)` | 保持速度单独测试后的方向 |

### 加速度阻尼实现

- 使用已带符号的 `measured_velocity_mps` 做差分：
  - `accel_raw_mps2 = (measured_velocity_mps - last_measured_velocity_mps) / dt`
  - 再使用与速度相同的低通系数滤波。
- 外环新增：
  - `accel_error_mps2 = 0 - accel_lpf_mps2`
  - `accel_theta_ref_deg = BALANCE_POSITION_KA_DEG_PER_MPS2 * accel_error_mps2`
- 最终：
  - `speed_theta_ref = Kp*position_error + Kv*speed_error + Ka*accel_error + Ki*position_integral`

### VOFA 通道变更

仍为 16 路外环诊断，但为了观察加速度阻尼，把两个在 `stand` 下基本恒为 0 的参考通道替换掉：

| 通道 | 当前含义 |
|---|---|
| I0 | `theta_deg` |
| I1 | `theta_ref_deg` |
| I2 | `speed_theta_ref_deg` |
| I3 | `position_m` |
| I4 | `accel_mps2` |
| I5 | `position_error_m` |
| I6 | `velocity_mps` |
| I7 | `accel_theta_ref_deg` |
| I8 | `velocity_error_mps` |
| I9 | `balance_ref_rpm` |
| I10 | `avg_ref_rpm` |
| I11 | `avg_actual_rpm` |
| I12 | `avg_cmd_a` |
| I13 | `fault` |
| I14 | `position_sign` |
| I15 | `velocity_sign` |

### 下一步测试

- 烧录后清空 VOFA 数据，只测 `stand`，先小幅推，再中等力度推。
- 重点看：
  - `fault` 是否仍为 0。
  - `theta_ref_deg/speed_theta_ref_deg` 是否能到约 `1~2deg`，但不要长期打满 `3deg`。
  - 推开后 `position_m` 是否能逐步回到 0 附近。
  - `accel_theta_ref_deg` 是否只在速度变化瞬间起作用，不应长期占主要量。
- 如果车开始越推越跑，优先单独翻 `BALANCE_PID_POSITION_SIGN`；如果只是刹不住，优先继续加 `KV` 或 `KA`。

## 2026-07-06 回退为速度环优先调参

### 原因

- 调参顺序应先确认速度外环能把推车后的速度刹住，再打开位置环做回中。
- 如果速度环和位置环同时打开，推车后的数据会混合位置误差、速度误差和车身姿态误差，不利于判断是速度阻尼不足、位置极性错误，还是位置 P 过大。

### 当前速度环测试参数

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `BALANCE_POSITION_KP_DEG_PER_M` | `0.0f` | 再次关闭位置回中项 |
| `BALANCE_POSITION_KV_DEG_PER_MPS` | `2.50f` | 保留较强速度阻尼 |
| `BALANCE_POSITION_KA_DEG_PER_MPS2` | `0.10f` | 保留加速度阻尼 |
| `BALANCE_POSITION_KI_DEG_PER_M_S` | `0.0f` | 关闭积分 |
| `BALANCE_PID_POSITION_SIGN` | `1.0f` | 位置项关闭，暂不判断 |
| `BALANCE_PID_VELOCITY_SIGN` | `(-1.0f)` | 保持当前速度方向 |

### 测试目标

- 只测 `stand`，不要求回到原点。
- 允许 `position_m` 漂走，因为位置环已关闭。
- 重点看推车后：
  - `avg_actual_rpm` 是否能逐步衰减。
  - `theta_ref_deg/speed_theta_ref_deg` 是否能到约 `1~2deg`，但不要长期打满 `3deg`。
  - `fault` 是否保持 0。
  - `accel_theta_ref_deg` 是否只在速度变化瞬间明显。
- 速度环能稳定刹住后，再把 `BALANCE_POSITION_KP_DEG_PER_M` 从 `0.3~0.5` 小幅打开测试位置回中。

## 2026-07-06 速度环增强一次

### 数据依据

- `KP_position=0/KV=2.5/KA=0.10/VELOCITY_SIGN=-1` 的数据中，`fault=0`，`theta_ref_deg` 未长期打满 `±3deg`，说明外环还有增益余量。
- 近直立区间 `|theta| < 1deg` 时，`theta_ref_deg` 平均只有约 `0.2deg`，不足以明显改变内层直立环输出，推车后速度仍压不住。
- 加速度阻尼瞬时最大可到约 `2.2deg`，说明 `KA=0.10` 对瞬态偏猛，但稳定速度段主要仍缺 `KV`。

### 参数修改

| 参数 | 旧值 | 新值 | 目的 |
|---|---:|---:|---|
| `BALANCE_POSITION_KP_DEG_PER_M` | `0.0f` | `0.0f` | 继续关闭位置回中，只调速度环 |
| `BALANCE_POSITION_KV_DEG_PER_MPS` | `2.50f` | `3.50f` | 增强持续速度阻尼 |
| `BALANCE_POSITION_KA_DEG_PER_MPS2` | `0.10f` | `0.05f` | 降低加速度瞬时尖峰，避免手感发冲 |

### 下一步判据

- 仍只测 `stand`，不要求回原点。
- 期望小推后 `theta_ref_deg` 能更接近 `0.5~1.5deg`，中等推可到 `1.5~2.5deg`，但不要长期顶到 `3deg`。
- 如果速度能衰减但开始前后摆，说明 `KV` 接近偏大或直立环 D 需要微调；如果仍刹不住，再考虑继续小幅加 `KV` 或检查轮速环输出余量。

## 2026-07-06 速度环增强后回退

### 数据/手感

- `KV=3.50/KA=0.05` 后，主观手感“不对”。
- 最新追加数据中出现 `fault=3` 段，车身角度最大到约 `40deg`，说明本次增强没有改善速度刹停，反而更容易把直立环带出安全区。
- 近直立低速段 `theta_ref_deg` 仍只有约 `0.2deg`，但大扰动时车身实际角度远大于目标角，继续单纯加 `KV` 不一定能解决问题。

### 回退参数

| 参数 | 旧值 | 新值 | 目的 |
|---|---:|---:|---|
| `BALANCE_POSITION_KP_DEG_PER_M` | `0.0f` | `0.0f` | 位置环继续关闭 |
| `BALANCE_POSITION_KV_DEG_PER_MPS` | `3.50f` | `2.50f` | 回到较温和速度阻尼 |
| `BALANCE_POSITION_KA_DEG_PER_MPS2` | `0.05f` | `0.0f` | 暂时关闭加速度阻尼，避免瞬态项干扰判断 |

### 下一步

- 用纯速度环再测一组：`KP=0/KV=2.5/KA=0/VELOCITY_SIGN=-1`。
- 若仍表现为越推越跑，应优先重新判断 `VELOCITY_SIGN`，不要继续盲目加 `KV`。
- 判断时优先看小角度区间：`|theta| < 1deg` 时，`theta_ref_deg` 是否正在把 `theta_error = theta - theta_ref` 往减小轮速的方向推。

## 2026-07-06 机械零点偏置微调

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `BALANCE_ANGLE_OFFSET_DEG` | `-2.0f` | `-2.5f` | 现场手感/静态直立均值判断重心偏置补偿不足，继续加大机械零点偏置 |

### 下一步测试

- 烧录后不发 `B` 命令，保持默认 `B0`，观察静止直立时 `theta_deg` 均值是否更接近 0、`balance_ref_rpm` 长期均值是否更接近 0。
- 若出现反方向慢漂（车持续往另一侧走），说明过补，回调到 `-2.2f` 附近。

## 2026-07-06 速度外环极性修正回 +1

### 判断依据

- `balance_ref_rpm` 与 `avg_actual_rpm` 同向不能作为正反馈证据：倒立摆是非最小相位系统，刹车时轮子必须先朝速度同方向加速，把车身甩成反向倾角，再靠倾角减速。此前据此把 `VELOCITY_SIGN` 翻成 `-1` 属于误判。
- 符号推导：`VELOCITY_SIGN=-1` 时，正向速度 `v>0` → `measured_velocity=-v` → `speed_error=+v` → `speed_theta_ref=+KV*v`（前倾目标角）→ 车继续加速，是真正的正反馈。
- 数据指纹吻合：`VELOCITY_SIGN=-1` 下把 `KV` 从 `2.5` 加到 `3.5`，结果更快触发 `fault=3`、角度冲到约 `40deg`——增益越大越发散是符号错误的典型特征。
- `VELOCITY_SIGN=+1` 的单独测试数据本来就正确：`avg_actual_rpm>0` 时 `speed_theta_ref<0`（后仰刹车方向）。当时“刹不住”的真实原因是权限不足：近直立段 `speed_theta_ref` 仅约 `0.2deg`，位置环关闭无回中项，另有 `0.01m/s` 速度死区削弱小速度响应。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `BALANCE_PID_VELOCITY_SIGN` | `(-1.0f)` | `1.0f` | 恢复负反馈：正速度产生负（后仰）目标角 |

### 下一步测试

- 保持 `KP_position=0 / KV=2.5 / KA=0`，只测 `stand` 小幅推车。
- 新判据（不再用 `balance_ref` 与轮速同向与否）：
  - `velocity_mps` 与 `speed_theta_ref_deg` 应反向（正速度 → 负目标角）。
  - 推开后 `avg_actual_rpm` 应在 `1~3s` 内单调衰减。
- 极性确认后若刹车仍弱，加 `KV`；此时加大 `KV` 应表现为刹得更狠而不是发散，本身就是符号正确的交叉验证。之后再以 `0.3~0.5` 小幅打开位置环单独验 `POSITION_SIGN`。

## 2026-07-06 VELOCITY_SIGN=+1 验证通过：连续站立 86s

### 数据判断（16 路外环诊断，5722 行，约 114.4s，`position_sign=+1 / velocity_sign=+1`，`KP_position=0/KV=2.5/KA=0`，offset=-2.5）

- 两段闭环：`0~14.3s` 和 `19.1~105.3s`；第二段**连续闭环 86.2s 无故障**，是目前最长稳定段。
- **速度外环极性判据全部通过**：
  - `|vel|>0.05m/s` 的 2246 行中，`velocity_mps` 与 `speed_theta_ref_deg` 反向比例 **100%**，corr=-1.000，回归斜率 -2.500 deg/(m/s)，正好等于 `-KV`，链路完全按设计工作。
  - 推车事件（峰值速度 ~0.3-0.46m/s）后速度大多在 **1~3s 内衰减到峰值 30% 以下**，能刹住。
- 长段整体质量：平均 `|theta|≈1.09deg`，`theta` 均值 -0.02deg（offset=-2.5 补偿合适）；`speed_theta_ref` 最大 2.28deg 未顶满 3deg；电流饱和仅 1 行。
- 位置环关闭下 `position_m` 顶在 ±0.30m 限幅上来回，属预期（无回中项）。
- 两次 `fault=3`（14.3s、105.3s）都是**强推超出动力权限**，不是极性问题：故障前 `balance_ref` 已顶满 ±320rpm、实际轮速 ~287rpm、电流打到 0.35A，theta 在 0.1s 内从 ~10deg 冲过 30deg 保护线。这是权限极限，行为正常。

### 当前结论

- `BALANCE_PID_VELOCITY_SIGN=1.0f` 定案，不再动；"balance_ref 与轮速同向=正反馈"的旧判据正式作废。
- `BALANCE_ANGLE_OFFSET_DEG=-2.5f` 表现合适（theta 长期均值≈0），暂不再调。
- 下一步按既定路线：`BALANCE_POSITION_KP_DEG_PER_M` 从 `0.3~0.5` 温和打开位置环，单独验证 `POSITION_SIGN`。判据：推开后 `position_m` 应缓慢回向 0，而不是顶在 ±0.3m 限幅上不回。
- 抗强推能力受 0.35A 电流限幅约束；若后续需要抗更大扰动，再讨论提限流，位置环验证阶段不动。

## 2026-07-06 打开位置环验证 POSITION_SIGN

### 打开原因（86s 长段数据的位移分析）

- 直立和速度阻尼已成立，但速度不受控回零：把 `velocity_mps` 积分还原成真实位移，86s 内车在 `-0.36m ~ +2.46m` 间游走，净漂移 `+0.51m`。
- 最长一次同方向移动持续 `7.1s`（另有 `5.3s`），对应现场"位移速度还在变大"的手感。
- `position_m` 有 `22%` 时间顶在 `±0.30m` 限幅上——位置反馈看得见但没人用（`KP_position=0`）。
- 速度阻尼只能衰减速度，无法消除稳态漂移；"直走后停车"必须位置闭环。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `BALANCE_POSITION_KP_DEG_PER_M` | `0.0f` | `0.40f` | 温和打开位置回中项，单独验证 `POSITION_SIGN` 极性 |

其余保持：`KV=2.50 / KA=0 / KI=0`，`POSITION_SIGN=+1 / VELOCITY_SIGN=+1`，offset=-2.5。

### 验证判据

- 只测 `stand`，小幅推开约 `0.2~0.3m` 后放手观察。
- **极性正确**：`position_m` 在数秒~十几秒内缓慢回向 0，不再长期顶在 `±0.30m` 限幅上；`position_error_m` 为正时 `speed_theta_ref_deg` 应含正向（朝误差方向倾）分量。
- **极性错误**：车越离中心越加速跑远、`position_m` 快速打到限幅并伴随速度增大 → 只翻 `BALANCE_PID_POSITION_SIGN`，不动 `VELOCITY_SIGN`。
- 若回中方向正确但来回振荡不收敛，说明 `KP=0.40` 相对 `KV=2.5` 偏大，先降 KP 到 `0.25~0.30`，不要动 KV。
- 位置环极性确认后，再考虑放大 `BALANCE_PID_POSITION_LIMIT_M`（当前 0.30m 对"直走"太小）并测 `forward` 直走→`stand` 停车。

## 2026-07-06 位置环 KP=0.40 验证：POSITION_SIGN 正确，限幅成为新瓶颈

### 数据判断（16 路外环诊断，5098 行，约 102s；连续闭环 93.2s）

- **`POSITION_SIGN=+1` 正确，定案**：
  - 11 次 `|pos|>0.15m` 偏移事件中，10 次都回到 `<0.075m`，典型回中时间 `2~6s`（一次慢至 22.4s，一次被结尾故障打断）。
  - `position_m` 过零仅 6 次/93s，回中是单调爬回，没有位置环振荡，`KP=0.40` 相对 `KV=2.5` 不偏大。
  - 通道自洽验证：`speed_theta_ref = 0.40*position_error + 2.50*velocity_error` 拟合符合率 99.8%，链路按设计工作。
- **新瓶颈是 `POSITION_LIMIT_M=0.30`**：
  - `position_m` 顶限幅时间从上组 22% 升到 **42.4%**（推一下就到 0.30m 饱和）。
  - 位置项最大权限只有 `0.40*0.30=0.12deg`，几乎可以忽略；目前回中主要还是靠速度阻尼+这一点点位置项慢慢蹭回来。
  - 每次积分器顶限幅，"原点"就丢一截：不带限幅还原的真实位移净漂 `-1.13m`（范围 -1.99~0.17m）。车回的是"限幅后的假原点"，物理位置仍在漂。
- 直立质量保持：`theta` 均值 -0.13deg，平均 `|theta|=1.18deg`，与上组相当，位置环打开没有付出直立代价。
- 结尾 `fault=3`（93.2s）仍是强推超权限的老模式：`bref` 顶满 -320、电流 ~0.28A、theta 0.1s 内冲过 30deg，与位置环无关。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `BALANCE_PID_POSITION_LIMIT_M` | `0.30f` | `1.00f` | 位置积分器 42% 时间饱和、原点持续丢失；放大后位置项最大权限从 0.12deg 提到 0.40deg |
| `BALANCE_POSITION_KP_DEG_PER_M` | `0.40f` | `0.80f` | 极性和无振荡已确认，提高回中速度；1m 偏移对应 0.8deg 目标角，仍温和 |

### 下一步测试

- 只测 `stand`：小推后车应更快回中（目标 2~3s 内），且 `position_m` 很少再顶 1.0m 限幅。
- 若出现来回过冲/低频摆动（位置过零明显变多），把 KP 退回 `0.5~0.6`。
- 位置回中满意后，测 `forward` 直走 → `stand` 停车：停车判据是 `position_m` 收敛到停车点附近且 `velocity_mps` 归零。

## 2026-07-06 KP=0.40 第二组数据：极性全部定案，进入纯调参阶段

### 数据判断（16 路外环诊断，4114 行，约 82.3s，全程 fault=0；参数仍为 KP=0.40/LIMIT=0.30）

- 本组为轻推测试（上一组是重推），表现明显更干净：
  - `position_m` 顶 0.30m 限幅时间 **0%**（上一组 42.4%），说明轻扰动下 0.30m 限幅未被触及。
  - 6 次 `|pos|>0.15m` 偏移全部回中，回中时间 **1.0~4.1s**。
  - 真实位移（不带限幅积分）净漂只有 `+0.11m`，范围 `-0.13~0.22m`——位置已经锁得住。
  - 平均 `|theta|=0.50deg`，最大仅 2.5deg；位置过零 5 次，无振荡。
  - `speed_theta_ref = 0.40*perr + 2.50*verr` 符合率 100%。
- 结论：**直立环、速度环、位置环三层极性全部正确并定案**（`POLARITY=+1 / VELOCITY_SIGN=+1 / POSITION_SIGN=+1`），遗留问题只剩参数量级：
  1. 重推时 0.30m 限幅饱和导致原点丢失（上一组已暴露）→ 已放大 `POSITION_LIMIT_M` 到 1.00。
  2. 回中速度偏慢（几秒级）→ 已提 KP 到 0.80，本组数据显示 0.40 无任何振荡迹象，0.80 有裕量。
  3. 强推超 0.35A 动力权限触发 fault=3 属物理极限，暂不处理。
- 当前待验证参数：`KP=0.80 / KV=2.50 / KA=0 / KI=0 / LIMIT=1.00`。

### 下一步测试

- 烧录 KP=0.80/LIMIT=1.00 后先重复 `stand` 轻推+重推：
  - 轻推回中时间应从 ~2-4s 缩短；
  - 重推后 `position_m` 不应长时间贴 1.0m，真实原点不再丢失。
  - 若低频前后摆（位置过零明显增多），KP 退 0.5~0.6。
- 然后进入 `forward` 直走 → `stand` 停车测试。

## 2026-07-07 forward 降为爬行速度并修复裸数字模式命令

### 修改原因

- 现场反馈 `forward` 速度仍偏快，希望降到很慢、随时可以停车的程度。
- 之前发送裸 `1`/`0` 没有切换模式：代码只识别 `m1/m0` 或 `forward/stand`，裸数字会落到最后的浮点解析，变成轮速测试目标值，对当前 `BALANCE_PID` 模式基本无效。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `BALANCE_MODE_FORWARD_VELOCITY_MPS` | `0.18f` | `0.06f` | 将 forward 目标速度降为约 6cm/s 的爬行速度 |
| `BALANCE_MODE_FORWARD_VELOCITY_LIMIT_MPS` | `0.35f` | `0.12f` | 限制 forward 速度目标上限，避免后续误设过快 |
| `BALANCE_MODE_VELOCITY_RAMP_MPS_PER_SEC` | `0.30f` | `0.10f` | 放慢起步斜坡，减少突然前窜 |

### 命令解析修改

- `chassis_serial_cmd.c` 中 `cmd_parse_mode()` 增加裸单字符数字模式识别：
  - `0` = `stand`
  - `1` = `forward`
  - `2` = `stop`
  - `3` = `left`
  - `4` = `right`
- 仍保留原命令：`m0/m1/m2`、`mode 0/1/2`、`stand/forward/stop`。
- 多位数字或小数仍走轮速目标解析，不影响 `50`、`100` 等速度测试指令。

### 验证

- `arm-none-eabi-gcc -fsyntax-only` 已通过：
  - `Application/gimbal/gimbal_task.c`
  - `Application/gimbal/chassis_serial_cmd.c`

### 下一步测试

- 烧录后清空 VOFA，先发 `B0`，再测试：
  - `1`：进入极慢 forward。
  - `0` 或 `2`：随时停车/站立。
- 若 `0.06m/s` 太慢到克服不了地面摩擦，再小幅加到 `0.08f`；若仍感觉快，再降到 `0.04f`。

## 2026-07-07 forward 无明显响应根因：直立执行限幅被压到 20rpm

### 数据结论

- 最新 `vofa+.csv` 中 `1` 命令实际已经生效：
  - `position_error_m` 从约 0 逐步增长到约 `0.93m`。
  - `speed_theta_ref_deg` 从 0 逐步增长到约 `1.07deg`。
  - `position_error_m` 斜率约为 `0.06m/s`，符合当前 `BALANCE_MODE_FORWARD_VELOCITY_MPS=0.06f`。
- 因此问题不是 `forward` 目标没有进入位置/速度外环，而是后级执行权限不足。
- 数据中 `balance_ref_rpm` 长时间被卡在 `±20rpm`，约 `11.7s` 顶限幅；这不是前进速度，而是直立环给轮速内环的目标轮速权限。
- `±20rpm` 对直立/前进都过低，车无法跟上 moving `position_ref`，导致 `position_error_m` 越积越大，最终触发 `fault=3` 大角度保护。
- 触发 `fault=3` 后 `emergency_stop=1`，串口 `0/1/2` 只会改模式，不会重新解锁电机；必须按 KEY 重新解锁。

### 修复内容

| 修改项 | 旧值/旧逻辑 | 新值/新逻辑 | 原因 |
|---|---:|---:|---|
| `BALANCE_PID_RUNTIME_SPEED_LIMIT_MIN_RPM` | 无 | `120.0f` | 防止在线命令 `O20/L20` 把直立执行权限压到无法平衡的 20rpm |
| `Balance_SetPidRuntimeParam(... SPEED_LIMIT ...)` | 运行时限幅可夹到 `20rpm` | 最低夹到 `120rpm` | 保留在线调参能力，但禁止危险低限幅 |
| VOFA I14 | `position_sign` | `chassis_mode` | 直接确认 `0/1/2` 是否切换模式 |
| VOFA I15 | `velocity_sign` | `runtime_speed_limit_rpm` | 直接确认直立环执行限幅是否被压低 |

### 新 VOFA 通道末两列

| 通道 | 含义 |
|---|---|
| I14 | `chassis_mode`：`0=stand`，`1=forward`，`2=stop`，`3=left`，`4=right` |
| I15 | `runtime_speed_limit_rpm`：当前直立环输出轮速限幅，正常应接近 `320`，不应再出现 `20` |

### 验证

- `arm-none-eabi-gcc -fsyntax-only` 已通过：
  - `Application/ins/ins_task.c`
  - `Application/gimbal/gimbal_task.c`
  - `Application/gimbal/chassis_serial_cmd.c`

### 下一步测试

- 烧录后重新上电/复位，清空 VOFA。
- 按 KEY 解锁后发：
  - `B0`
  - `1` 进入慢速 forward
  - `0` 或 `2` 停车/站立
- 判断：
  - I14 应从 `0` 变 `1`，再变回 `0/2`。
  - I15 应为 `320` 附近，至少不应为 `20`。
  - 若触发 `fault=3`，先扶正并按 KEY 重新解锁，再继续测。

## 2026-07-07 forward 指令已生效但结构会攒误差后突然冲

### 最新数据结论

- 新版 VOFA 已确认烧录生效：
  - I14 为 `chassis_mode`。
  - I15 为 `runtime_speed_limit_rpm`。
- `vofa+.csv` 中 `1` 命令确实生效：
  - `t=4.70s` 后 I14 从 `0` 变为 `1`。
  - I15 全程为 `320`，直立执行限幅已恢复，不再是 `20rpm`。
- 但 forward 行为仍不对：
  - `1` 后车最初基本不动，`position_error_m` 持续积累。
  - `position_error_m` 从约 `0` 积到 `1.6m` 量级，`speed_theta_ref_deg` 后期达到 `2.7deg` 左右。
  - 车随后突然大幅动作，最终 `theta` 超过 `30deg`，触发 `fault=3`。
- 根因不是命令没进，而是原 forward 结构把 `position_ref` 按速度目标持续积分；低速/静摩擦下车不动时，位置误差会越攒越大，最终变成一次大动作。

### 控制结构修改

| 修改项 | 旧逻辑 | 新逻辑 | 目的 |
|---|---|---|---|
| `BALANCE_MODE_FORWARD_POSITION_LEAD_M` | 无 | `0.35f` | forward 时只保持一个有限的前方跟随点 |
| forward `position_ref` | `position_ref += velocity_ref * dt`，可长期积累误差 | `position_ref` 以斜坡靠近 `measured_position + 0.35m` | 防止车不动时误差无限积累，避免“先没反应、后突然冲” |
| stand/stop `position_ref` | 保持原逻辑 | 不变 | 停车/站立仍锁当前位置 |

### 预期表现

- 发 `1` 后，I14 应变为 `1`。
- `speed_theta_ref_deg` 应在几秒内上升到约 `0.3~0.5deg`，但不应继续积到 `2~3deg`。
- 车如果克服不了静摩擦，应该只是轻微尝试前走，不应继续攒误差到摔车。
- 若仍完全不走，可小幅增加 `BALANCE_MODE_FORWARD_POSITION_LEAD_M` 到 `0.45f`；若仍太冲，则降到 `0.25f`。

## 2026-07-07 改为按测试 bias 实现前进/后退/停止

### 最新 bias 数据结论

- 最新数据中 `1` 命令仍然已经进模式：
  - I14 从 `0` 变为 `1`，即 `chassis_mode=forward`。
  - I15 大部分为 `120`，说明运行时速度限幅被压到最低钳位，但不再是旧的 `20rpm`。
- 用户测试过的 bias 档位：
  - `B0.5`：稳定，但基本不走，`velocity_mps` 接近 0。
  - `B1.0`：能产生明显动作，是当前可用的前进量级。
  - `B2.0`：过猛，短时间内角度明显增大并接近/触发保护，不适合作为默认行走。
- 原 forward 结构仍不符合当前需求：用户希望按测试 bias 直接实现前进/后退/停止，而不是靠移动位置参考。

### 控制结构修改

| 修改项 | 旧逻辑 | 新逻辑 |
|---|---|---|
| `CHASSIS_MODE_FORWARD` | moving `position_ref`/速度目标 | 固定模式 bias `+1.0deg` |
| `CHASSIS_MODE_BACKWARD` | 无 | 固定模式 bias `-1.0deg` |
| `0`/`stop` | `stand/stop` 混用 | 统一为 `stand`，bias 为 0 |
| `2` | 原 `stop` | 改为 `backward` |
| mode 切换 | 可能叠加之前手动 `B` | 切模式时自动 `Balance_SetManualBiasDeg(0)`，防止手动 bias 残留 |
| 行走模式位置环 | 可能积累位置误差 | `position_ref = measured_position`，`position_error=0`，不再攒误差 |
| 行走模式速度环 | 原速度目标前进 | 目标速度为 0，只保留速度阻尼；实际目标角由 mode bias 提供 |

### 当前命令

| 命令 | 含义 |
|---|---|
| `0` / `stand` / `stop` | 停止并站立，模式 bias 清零 |
| `1` / `forward` / `go` | 前进，模式 bias `+1.0deg` |
| `2` / `backward` / `back` / `reverse` | 后退，模式 bias `-1.0deg` |
| `3` / `left` | 左转 |
| `4` / `right` | 右转 |

### 新参数

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `BALANCE_MODE_FORWARD_BIAS_DEG` | `1.0f` | 根据 `B1.0` 测试作为默认前进 bias |
| `BALANCE_MODE_BACKWARD_BIAS_DEG` | `(-1.0f)` | 后退使用对称 bias |

### 验证

- `arm-none-eabi-gcc -fsyntax-only` 已通过：
  - `Application/gimbal/gimbal_task.c`
  - `Application/gimbal/chassis_serial_cmd.c`
  - `Application/ins/ins_task.c`

### 下一步测试

- 烧录后重新上电/复位，清空 VOFA，按 KEY 解锁。
- 直接测试：
  - `1`：应看到 I14=1，`theta_ref_deg` 约 `+1deg` 附近。
  - `0`：应看到 I14=0，`theta_ref_deg` 回到 0 附近并停车。
  - `2`：应看到 I14=2，`theta_ref_deg` 约 `-1deg` 附近。
- 若 `1deg` 仍嫌猛，把 `BALANCE_MODE_FORWARD_BIAS_DEG` 降到 `0.8f`；若仍不走，再回升到 `1.2f`，不要用 `2.0f` 做默认。

## 2026-07-07 最新 bias 数据结论：B0 保持，B1 前进

### 数据范围

- 最新 `vofa+.csv`：约 `16440` 行，按 20ms 估算约 `328.8s`。
- I14 全程为 `0`，说明本组数据没有进入 `chassis_mode=forward`，只是在 `stand` 模式下测试手动 `B` bias。
- I15 全程为 `320`，说明直立执行限幅正常，没有再被压到 `20/120rpm`。

### bias 结论

| bias | 数据表现 | 结论 |
|---:|---|---|
| `B0` | 前 35.4s 稳定，`theta` 均值约 `-0.60deg`，平均速度绝对值约 `0.016m/s` | 当前保持平衡用 `0deg`，不需要额外手动 bias |
| `B0.5` | 有较多运动/扰动段，且后段接近保护边界；不适合作为明确前进指令 | 太小且不干净，不推荐作为前进默认 |
| `B1.0` | 最长稳定段约 `98.9s`，`theta` 平均绝对值约 `0.59deg`，可产生明显动作但整体不失控 | 当前前进默认使用 `+1.0deg` 合理 |
| `B2.0` | 短段可站住，但过猛，历史数据中也接近/触发保护 | 不作为默认前进 bias |
| `B-1.0` | 可作为后退方向参考，但运动扰动比 `B+1.0` 更明显 | 后退先用 `-1.0deg`，若太猛降到 `-0.8deg` |

### 当前建议

- 保持平衡/停止：`bias = 0.0deg`。
- 前进：`bias = +1.0deg`。
- 后退：`bias = -1.0deg` 先试；若后退更冲，改为 `-0.8deg`。
- 如果用户确实发送了 `1`，但 I14 仍为 `0`，说明 `1` 命令没有进入模式解析；下一步应优先查串口发送是否带换行、是否发到当前 VOFA UART、以及是否烧录了裸数字模式命令版本。

## 2026-07-07 config 清理为 bias-only 行走模式

### 修改内容

- 删除/停用容易误导的旧 forward 速度/位置参数：
  - `BALANCE_MODE_FORWARD_VELOCITY_MPS`
  - `BALANCE_MODE_FORWARD_VELOCITY_LIMIT_MPS`
  - `BALANCE_MODE_FORWARD_POSITION_LEAD_M`
- 当前行走模式只由 bias 参数决定：

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `BALANCE_MODE_STAND_BIAS_DEG` | `0.0f` | 停止/保持平衡 |
| `BALANCE_MODE_FORWARD_BIAS_DEG` | `1.0f` | 前进 |
| `BALANCE_MODE_BACKWARD_BIAS_DEG` | `(-1.0f)` | 后退 |

### 验证

- `Application/gimbal/gimbal_task.c` 已通过 `arm-none-eabi-gcc -fsyntax-only`。

## 2026-07-07 最新短数据：B0 仍优先用于站立

### 数据范围

- 最新 `vofa+.csv`：`3660` 行。
- I14 全程为 `0`，说明本组数据没有真正进入 `chassis_mode=1` 前进模式。
- I15 基本为 `320rpm`，直立执行限幅正常。
- fault 分布：`fault=0` 为 `2806` 行，末尾 `fault=3` 为 `854` 行；末尾摔倒段不能用于评价正常站立 bias。

### 关键段

| 段 | bias | 数据表现 | 结论 |
|---|---:|---|---|
| 开头稳定段 | `0deg` | `984` 行，`theta` 均值约 `-0.046deg`，范围约 `-0.922~0.747deg`，平均速度约 `0.010m/s` | `B0` 本身站立最好 |
| 中间测试段 | `-2deg` | `theta` 均值约 `-2.281deg`，最小到 `-5.025deg` | 偏大，不适合作为停止点 |
| 中间测试段 | `-1deg` | `theta` 均值约 `-0.928deg`，平均速度约 `0.053m/s`，平均轮速约 `16rpm` | 会产生运动趋势，不适合作为停止点 |
| 回零前后 | `-1deg -> 0deg` | 回到 `0deg` 前车体已经有约 `0.9m/s` 速度、角度升到 `5deg+`，随后进入 `fault=3` | 这不是 `B0` 静态立不住，而是高速/大角度状态下回零已来不及救回 |

### 当前判断

- 停止/站立仍选 `B0`，不要用 `B1` 当站立点。
- `B1` 是前进偏置；本组最新数据没有出现 `+1deg` 或 `mode=1` 段，不能用这组数据证明 `1` 比 `0` 更适合站立。
- 如果体感觉得 `0` 有轻微偏倒，应优先微调机械零点或 `BALANCE_MODE_STAND_BIAS_DEG` 的小量 trim，例如 `0.1deg` 级别，不应直接改成 `1deg`。

## 2026-07-07 速度/位置外环减速效果判断

### 数据现象

- 在最新短数据的 `B-1` 后段，速度从约 `0.30m/s` 增加到 `0.94m/s`。
- 同时 `speed_theta_ref_deg` 从约 `-1.28deg` 继续变到 `-3.00deg`，`theta_ref_deg` 已经被 `BALANCE_PID_THETA_REF_LIMIT_DEG = 3.0f` 限死。
- `balance_ref_rpm` 从约 `90rpm` 增加到 `320rpm`，末端已经打到 `BALANCE_PID_BALANCE_SPEED_LIMIT_RPM`。

### 判断

- 速度/位置外环不是完全没起作用；VOFA 上能看到它在不断改变 `speed_theta_ref_deg`。
- 但在当前符号和限幅下，它没有形成有效减速，表现为速度越大，`speed_theta_ref_deg` 越负，最终叠加到更负的 `theta_ref_deg`，内环继续给正向轮速去追车体角度。
- 一旦车体角度已经大、轮速参考接近 `320rpm`，直立优先级会压过速度控制，外环已经没有足够权限把车慢慢刹住。

### 下一步优先级

- 不优先继续加大 `BALANCE_POSITION_KV_DEG_PER_MPS`，因为当前已经会打到 `theta_ref` 限幅。
- 优先验证速度/位置外环符号：当前 `BALANCE_PID_POSITION_SIGN = +1`、`BALANCE_PID_VELOCITY_SIGN = +1` 对这段运动表现像同向加速；下一步可试反外环符号或反外环输出符号。
- 行走 bias 也要加速度保护：速度超过小阈值后应自动削弱/取消 bias，避免一直把车推向限幅。

## 2026-07-07 外环符号严格推导与最新 mode=1 数据

### 不能用直立表现反推外环符号

- `B0` 能站住只能证明直立内环、轮速环、角度反馈在小速度附近没有整体反掉。
- 直立环优先级最高：当车身角度偏了，直立环会优先追 `theta_ref`，因此它可以把外环符号问题暂时掩盖掉。
- 外环符号必须从公式求导判断，不能只看“能不能短时间站住”。

### 当前代码公式

```c
speed_error_mps = balance_velocity_ref_ramp_mps - measured_velocity_mps;
position_error_m = balance_position_ref_m - measured_position_m;

speed_theta_ref_deg =
    BALANCE_POSITION_KP_DEG_PER_M * position_error_m +
    BALANCE_POSITION_KV_DEG_PER_MPS * speed_error_mps +
    accel_theta_ref_deg +
    BALANCE_POSITION_KI_DEG_PER_M_S * balance_speed_integral_m;

theta_ref_deg = manual_bias + mode_bias + speed_theta_ref_deg;
theta_error_deg = theta_deg - theta_ref_deg;

balance_speed_ref_rpm =
    (Kp * theta_error_deg + Ki * integral + Kd * theta_rate_dps) *
    BALANCE_PID_POLARITY;
```

当前参数：

| 参数 | 当前值 |
|---|---:|
| `BALANCE_PID_POLARITY` | `+1` |
| `BALANCE_PID_POSITION_SIGN` | `+1` |
| `BALANCE_PID_VELOCITY_SIGN` | `+1` |
| `BALANCE_POSITION_KP_DEG_PER_M` | `0.80` |
| `BALANCE_POSITION_KV_DEG_PER_MPS` | `2.50` |
| `BALANCE_PID_ANGLE_KP_RPM_PER_DEG` | `35.00` |

### 由公式得到的符号结论

只看 P 项，直立内环输出为：

```text
u = balance_speed_ref_rpm = Kp_angle * (theta - theta_ref)
```

所以：

```text
du / d(theta_ref) = -Kp_angle < 0
```

结论：

- 增大 `theta_ref` 会让轮速参考 `u` 往负方向变化。
- 减小 `theta_ref` 会让轮速参考 `u` 往正方向变化。

如果当前测得车速 `v > 0`，想减速，轮速参考应该往负方向给，也就是需要：

```text
v > 0  =>  theta_ref 修正量应该为正
```

但当前速度外环是：

```text
speed_error = 0 - v = -v
speed_theta_ref = Kv * speed_error = -Kv * v
```

在 `Kv > 0` 时：

```text
v > 0  =>  speed_theta_ref < 0
speed_theta_ref < 0  =>  theta_ref 变小
theta_ref 变小  =>  balance_speed_ref 往正方向变大
```

这和减速需求相反。也就是说，在当前正向速度定义下，速度外环符号是反的。

位置环同理。如果测得位置 `x > 0`，要拉回原点，需要负向轮速，即 `theta_ref` 应为正；但当前：

```text
position_error = 0 - x = -x
position_theta_ref = Kp_pos * position_error = -Kp_pos * x
```

同样会让 `theta_ref` 变小，导致轮速参考往正方向变大，和拉回方向相反。

### 等价修正方式

以下两类改法等价，建议只选一种：

| 改法 | 含义 |
|---|---|
| 把 `BALANCE_PID_POSITION_SIGN` 和 `BALANCE_PID_VELOCITY_SIGN` 从 `+1` 改成 `-1` | 保持外环公式不变，反转进入外环的位置/速度定义 |
| 保持测量 sign 不变，但把外环误差改成 `measured - target` | 反转外环输出符号 |

更小改动是先改：

```c
#define BALANCE_PID_POSITION_SIGN           (-1.0f)
#define BALANCE_PID_VELOCITY_SIGN           (-1.0f)
```

注意：

- 行走 bias 模式里当前 `position_error=0`，所以前进/后退时主要先验证 `VELOCITY_SIGN`。
- 停止/站立模式会用到位置环，因此最终 `POSITION_SIGN` 也应一起改，否则停车拉回方向仍可能反。
- 改完后必须先测 `0` 站立，再测轻推车体，看 `v>0` 时 `speed_theta_ref` 是否变为正、`balance_ref_rpm` 是否往负方向刹车。

### 最新 `vofa+.csv` 数据

- 文件时间：`2026-07-07 11:16:07`。
- 总行数：`8891`。
- I14 分布：`mode=0` 有 `248` 行，`mode=1` 有 `8643` 行。
- fault 分布：`fault=0` 有 `7510` 行，`fault=3` 有 `1381` 行。
- I15 为 `320rpm`，直立输出限幅正常。
- 这次 `1` 命令已经进入控制模式，问题不是“1 没发进去”。

### 最新数据关键段

| 行范围 | mode | 数据表现 | 结论 |
|---:|---:|---|---|
| `0-247` | `0` | `theta` 均值约 `0.017deg`，速度约 `0m/s` | `0` 静态站立正常 |
| `248-1030` | `1` | `theta_ref` 约 `1.000deg`，`theta` 均值约 `0.994deg`，速度约 `0m/s`，轮速参考约 `0rpm` | `1` 已生效，但只是把车体目标角保持在 `1deg`，没有形成稳定前进 |
| `1031-1682` | `1` | 出现速度到约 `0.88m/s` 的运动，随后进入 `fault=3` | 有运动但不可控，不能算完成直走 |
| `3064-4662` | `1` | `theta_ref` 约 `1deg`，速度均值接近 `0`，但有 `-0.8~0.9m/s` 的扰动峰值 | 仍不是稳定直线匀速 |
| `6199-7241` | `1` | `theta_ref` 接近 `3deg`，速度仍接近 `0` | 更大 bias 也可能只是改变静态目标角，不一定产生可控前进 |
| `7243-7304` | `1` | 速度升到约 `0.99m/s`，`balance_ref` 平均约 `199rpm` | 一旦动起来，直立环输出变大，速度没有被温和限制住 |
| `8645-8890` | `1` | 末尾回到约 `1deg`，速度约 `0m/s` | 仍没有稳定直走 |

### 最新判断

- 当前 `mode=1` 命令链路是通的。
- 当前 `+1deg` bias 能改变目标角，但不能可靠地产生稳定直走：要么几乎不走，要么速度起来后进入大扰动/故障。
- 根据公式，速度/位置外环符号在当前测量定义下是反的；这比“看直立能不能站住”更可靠。
- 下一步应先修正外环符号，再继续谈前进 bias 大小；否则加大 bias 只会更容易把直立环推到限幅。

## 2026-07-07 上节符号推导作废：被控对象回归证明三层极性全部正确

### 为什么上节推导是错的（必须记住，第三次踩同一个坑）

- 上节用 `du/d(theta_ref) = -Kp < 0` 推"theta_ref 变小 → 轮速变正 → 与减速需求相反"。这只描述**瞬时**轮速方向，是非最小相位零点：刹车必须先让轮子与车速同向加速，把车身甩成反向倾角，再靠倾角减速。**这与 07-06"速度外环极性修正回 +1"一节作废的旧判据是同一个谬误**。
- 正确推导要分快慢回路：角度环（~0.1s）收敛后 `theta -> theta_ref`；车体保持倾角时物理上必然 `v_dot = g*sin(theta - theta0) - 摩擦`（直立环无法伪造这个关系）。代入外环得慢回路方程 `v_dot = g*c*(bias - theta0) - g*c*Kv*sigma*v`，sigma 为速度测量方向与 theta 方向是否一致——**这是唯一未知量，只能用数据定**。
- sigma=+1 时速度指数收敛，理论时间常数 `tau = 1/(g*c*Kv) = 2.34s`，与 07-06 实测推车衰减 1~3s 定量吻合；sigma=-1 时 86s 站立在数学上不可能。

### 数据计算（8891 行，analyze_polarity.py 可复用）

- 被控对象回归 `a = k1*theta + k2*sgn(v) + b`（n=1302，闭环无故障、|theta|<8、0.03<|v|<0.7）：
  - `k1 = +0.0867 (m/s^2)/deg`，**符号为正 ⇒ sigma=+1，三层符号 `POLARITY/POSITION_SIGN/VELOCITY_SIGN = +1/+1/+1` 全部正确，最终定案**。
  - 真实平衡零点 `theta0 = +2.0deg`（显示坐标）；无摩擦项交叉验证 `theta0 = +1.6deg`。
  - 摩擦等效倾角 `theta_fric = +1.44deg`。
- 独立验证：全数据"轮子不动、电流≈0、v≈0"的停驻段共 108s，theta 中位数 **+2.05deg**，分布 [+0.5, +3.0]，正是以 theta0 为中心的静摩擦死区。
- 直走失败真凶 1（零点）：theta0=+2 时 `B+1` 的有效 bias = -1deg，还在摩擦死区内。数据印证：theta≈1deg 停驻 44.6s（电流 0.025A、实测加速度 +0.004m/s²，若零点真在 0 必须持续 0.17m/s² 加速，物理不可能）；116.5s 段挂着前进命令净位移 **-7.19m**（后溜）。
- 直走失败真凶 2（高速权限天花板）：|v|>0.5 的 619 行中，电流 ≥0.34A 比例 **0%**（摔倒瞬间仅 0.21A）、轮速跟随误差仅 22rpm——内环和电流都不是瓶颈；`theta_ref` 顶 ±3deg 限幅 72%（最大减速度被钉在 g*sin3° ≈ 0.51m/s²）、`balance_ref` 顶 320rpm 30%（轮速上限 1.04m/s 吃不起非最小相位刹车瞬态）。**根本对策不是加强刹车，而是闭环限速不让速度上去。**
- 固定 bias 无法实现爬行的解析原因：巡航速度 `v* = (bias - theta_fric - theta0误差)/Kv`，摩擦（±1.44deg）和零点漂移（±2deg）都比 bias 本身大——这就是"B0.5 不走、B1 时走时停、B2 冲"的公式解。

### 修改内容

| 修改项 | 旧值/旧逻辑 | 新值/新逻辑 | 原因 |
|---|---|---|---|
| `BALANCE_ANGLE_OFFSET_DEG` | `-2.5f` | `-0.5f` | 三重估计零点在显示 +1.6~2.0deg，欠补约 2deg |
| `BALANCE_MODE_FORWARD/BACKWARD_BIAS_DEG` | `±1.0f` | `0.0f` | 弃用开环 bias 行走 |
| `BALANCE_MODE_DRIVE_VELOCITY_MPS` | 无 | `0.10f` | 前进/后退改为速度闭环目标，走现有 ramp（0.10m/s²） |
| `Chassis_ModeTargetVelocityMps()` | 恒 0 | forward/backward 返回 ±0.10 | 同上 |
| `BALANCE_POSITION_KI_DEG_PER_M_S` | `0.0f` | `5.0f` | 行走时积分速度误差，自动收敛到克服摩擦+零点残差所需倾角 |
| `BALANCE_POSITION_INTEGRAL_LIMIT_M_S` | `0.20f` | `0.30f` | 积分权限 = 5.0×0.30 = 1.5deg，略高于摩擦角 1.44deg |
| 积分对象 | drive 清零 / stand 积位置误差（Ki=0 时惰性） | **drive 积速度误差 / stand 清零**；模式切换时清积分 | stand 行为完全不变（P+V 照旧），行走获得摩擦自适应 |
| 限速守卫 | 无 | `GUARD_ON=0.40 / GUARD_OFF=0.15 m/s` 迟滞；触发时 `target_velocity=0` 并冻结积分 | 把车速摁在可回收包络内；只改外环速度目标，不触碰直立角度环任何量 |
| VOFA I7 | `accel_theta_ref_deg`（KA=0 恒 0） | `velocity_ref_mps` | 直接观察守卫动作（超速时 v_ref 被拉 0） |
| VOFA I15 | `runtime_speed_limit_rpm`（恒 320） | `integral_theta_ref_deg` | 观察积分收敛到的倾角 |

- 语法验证通过：`gimbal_task.c / ins_task.c / chassis_serial_cmd.c`。

### 下一步测试

1. 烧录后先测 `stand`：静止直立 theta 均值应接近 0（验证新 offset）；轻推应照旧回中（stand 行为未改）。
2. 发 `1`：车应以约 0.1m/s 匀速爬行。看 I7 `velocity_ref` 爬到 0.10、I6 `velocity_mps` 跟上、I15 积分角收敛到某个稳定小值（预计 0.5~1.5deg）、theta 稳定。
3. 发 `0`：应停车站立，积分清零（I15 回 0）。
4. 行走中人为推快：|v|>0.4 时 I7 应被守卫拉回 0，车刹到 0.15 以下后自动恢复爬行，全程 `fault=0`。
5. 若爬行时积分顶满 1.5deg 仍不走，摩擦角比估计大，把 `INTEGRAL_LIMIT` 提到 0.40；若爬行有低频振荡，把 KI 从 5.0 降到 3.0。
6. 零点验证：stand 长站后若 theta 均值仍有固定偏差 x，把 `BALANCE_ANGLE_OFFSET_DEG` 再加上 x（当前 -0.5 是按 theta0≈+2.0 估计的，允许 ±0.5deg 残差）。

## 2026-07-07 守卫积分冻结漏洞修复：改为向 0 衰减

### 问题（审查发现，未烧录先修）

- 上一版守卫触发时把行走积分**冻结**。但起步顶静摩擦阶段（v=0、误差 +0.1m/s、积分速率 0.5deg/s）积分约 3s 就攒满 +1.5deg，车破摩擦窜出去触发守卫时积分正好停在最大助推值。
- 定量：触发瞬间 v=0.4，速度项 2.5×(-0.4)=-1.0deg，冻结积分 +1.5deg，净 theta_ref=**+0.5deg 仍在向前推**；要到 v=0.6m/s 速度项才压得过积分——守卫 ON 阈值被变相从 0.40 抬到 0.6，蹭到不可回收区边缘。
- 另一个小问题：守卫触发后 v_ref 还要走 1s 斜坡才降到 0，白送 +0.25deg 前倾。

### 修改内容

| 修改项 | 旧逻辑 | 新逻辑 | 原因 |
|---|---|---|---|
| 守卫期间积分 | 冻结（保持原值） | 以 `BALANCE_SPEED_GUARD_INTEGRAL_DECAY_M_PER_S = 0.60f` 向 0 衰减，0.5s 内清完 | 消除同向助推；衰减到 0 即停，不会积成反向，windup 保护仍成立 |
| 守卫触发时 v_ref | 走 0.10m/s² 斜坡降到 0（需 1s） | `balance_velocity_ref_ramp_mps` 立即置 0 | 让 Kv 项立刻全额刹车 |

- 语法验证通过：`gimbal_task.c / ins_task.c`。
- 验收补充：行走中推快触发守卫时，I15 `integral_theta_ref_deg` 应在约 0.5s 内滑到 0，I7 `velocity_ref` 应立刻跳 0（不是斜坡下降）。

## 2026-07-07 新固件首测：零点修好、全程无故障，但爬行是 stick-slip 极限环 → 加轮速前馈

### 数据结论（4102 行，82.0s，全程 fault=0）

- **零点修复验证通过**：stand 静立 60.3s，`theta` 均值 **-0.074deg**；停驻段 theta 中位数 -0.02deg。`BALANCE_ANGLE_OFFSET_DEG=-0.5f` 定案。
- **鲁棒性显著改善**：stand 段被推到 0.97m/s 都救回来了，82s 内 0 次 fault（此前同类推速必 fault=3）。
- **forward 段（56.0~68.9s，12.9s）暴露最后一个问题：静摩擦 stick-slip 极限环**：
  - `v_ref` 正常爬到 0.10，积分攒到 1.40deg（50% 时间顶满 1.5deg 上限），`theta` 实际倾 +1.36deg。
  - 但 `v` 均值仅 **0.007m/s**，净位移 0.09m，车原地抖。
  - `v` 在 ±0.2 间振荡，周期约 1.5s，12.9s 内 17 次过零；corr(积分, v) = -0.47，积分和速度互相追逐。
  - 机理：积分攒倾角→顶破静摩擦→车窜一下(v 到 0.25)→超过 0.1 目标,Kv 刹车+积分回退→轮子重新粘住→v 回 0→再攒。积分权限 1.5deg 与静摩擦角 1.44deg 几乎相等，永远在临界点蹭。
- 守卫未触发（v 最大 0.25 未到 0.4），本组数据不能验证守卫。

### 修改内容：行走轮速前馈

| 修改项 | 逻辑 | 原因 |
|---|---|---|
| `drive_wheel_ff_rpm` | 行走模式把 `v_ref` 换算轮速（`v_ref * VELOCITY_SIGN / RPM_TO_MPS`，0.10m/s≈31rpm）直接加到左右轮目标 | 让轮速内环（自带积分+静摩擦前馈，S50 已验证）破静摩擦持续滚动；外环倾角只做平衡/加减速修正，不再靠攒倾角顶静摩擦 |
| 与守卫的配合 | 守卫触发时 ramp 已被置 0，前馈随之立即消失 | 超速时前馈不会继续推车 |

- 语法验证通过：`gimbal_task.c`。

### 下一步测试

- 烧录后发 `1`：预期 `v`（I6）平滑跟上 0.10 并保持，不再 ±0.2 振荡；`theta` 应接近 0（不再需要 1.4deg 倾角），I15 积分收敛到很小的值（只补动摩擦残差）。
- I10 `avg_ref_rpm` 应能看到 ~31rpm 的前馈基线（平衡修正叠加在上面）。
- 发 `0`：前馈消失，位置锁生效，应平稳停车。
- 若爬行平稳，可把 `BALANCE_MODE_DRIVE_VELOCITY_MPS` 逐步提到 0.15/0.20 测试；若起步仍有一次小窜动，把 `BALANCE_MODE_VELOCITY_RAMP_MPS_PER_SEC` 从 0.10 降到 0.05。
- 顺带补测守卫：行走中推快过 0.4m/s，看 I7 跳 0、I15 在 0.5s 内滑 0、刹回 0.15 后自动恢复爬行。

## 2026-07-07 14:34 数据：offset 不再调，B 恢复 0，轮速前馈仍未测

### 数据结论（6401 行，128s，全程 stand 模式）

- 122s 连续直立无故障，净位移仅 +0.06m。
- 用户中途发过 `B-2`（77s）和 `B-1.5`（93s）；无偏置段（70s）theta 均值 **-0.21deg**。
- 被控对象回归 theta0 = -0.7~-0.8deg（噪声大），与上组（14:21，残差≈0）不一致 → 剩余为会话间漂移（电池/温度/地面），不是固定误差。
- **决定：`BALANCE_ANGLE_OFFSET_DEG = -0.5f` 不再调**。残差 |0.2~0.8deg| 小于静摩擦死区 ±1.3deg，行走 KI 积分和 stand 位置环本来就自动吸收这个量级；追固定值会重蹈 yaw bias 追参的覆辙。
- `B-2` 方向过头（把 theta 拉到 -1.8，比不加偏置离零点更远），手动 B 恢复 0。
- **本组无 forward 段，轮速前馈（治 stick-slip 的关键改动）仍未验证**；下一步直接发 `1` 测爬行。

## 2026-07-07 前馈版首测反馈"太快、倾角收不住"：降速降斜坡

### 现场反馈

- 轮速前馈版烧录后发 `1`，现场感受"跑太快、倾角收不住"。
- 该次数据未导出（vofa+.csv 仍是 14:34 的 stand 数据），具体是超速还是 pitch 振荡待 CSV 确认。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `BALANCE_MODE_DRIVE_VELOCITY_MPS` | `0.10f` | `0.06f` | 爬行目标降到 6cm/s，前馈基线从 ~31rpm 降到 ~18rpm |
| `BALANCE_MODE_VELOCITY_RAMP_MPS_PER_SEC` | `0.10f` | `0.04f` | 起步斜坡放缓到 1.5s，减小前馈建立时车身的非最小相位后仰扰动 |

### 下一步测试

- 烧录后发 `1` 复测；**务必导出这次的 CSV**，若仍"倾角收不住"，用数据区分：
  - v 超过 0.06 多少（速度环问题）；
  - theta 振荡频率/幅度（角度环与前馈交互问题）；
  - I15 积分是否在前馈存在时仍大幅摆动（KI 与前馈打架，则降 KI 5→2）。

## 2026-07-07 现场确认 B-1.5 才立得住：offset 改 -2.0，发现零点会话间漂移

### 现场证据与数据吻合

- 现场：`offset=-0.5` 下 `B0` 立不稳，`B-1.5` 才立得住。
- 与 14:34 数据一致：停驻分布中位数 `-0.76`、p10 `-2.22`，被控对象回归零点 `-0.7~-0.8deg`，用户当时试的 `B-2` 略过头、`B-1.5` 正好。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `BALANCE_ANGLE_OFFSET_DEG` | `-0.5f` | `-2.0f` | 现场 `B-1.5` 等效固化进零点（-0.5 + -1.5 = -2.0） |

### 重要发现：零点存在 ±1~1.5deg 会话间漂移

- 换算到 raw 角度：上午会话真实零点 raw ≈ `-0.5deg`（当时显示 +2.0），下午会话 raw ≈ `-2.0deg`（显示 -1.5）——几小时内漂了约 `1.5deg`。
- 疑似加速度计温漂（1deg 倾角 ≈ 17mg，BMI088 加速度计温漂可达此量级）或电池/线缆等机械变动。
- **约定：若下次开机又漂，不要再重烧固件追常数**（与 yaw bias 追参同一教训）。两个替代方案：
  1. 短期：开机直立后用 `B` 命令做会话级 trim；
  2. 长期：实现 stand 模式慢速零点自学习（静止直立时 theta 均值就是真零点，用很慢的积分伺服内部 bias 跟上去）。

### 下一步测试

- 烧录后 `B0` 应直接立得住（等效旧 `B-1.5`）。
- 然后继续测 `1`（爬行 0.06m/s + 轮速前馈）并导出 CSV。

## 2026-07-07 14:49 数据定位"发1不动"：倾角积分与轮速前馈死锁，KI 归零

### 数据结论（783 行，15.7s，全程 fault=0）

- stand 段 theta 均值 `-0.011deg`：offset=-2.0 本会话完全正确。
- forward 13.8s：**v = 0.000，轮子从未转动**。链路各环节：
  - `v_ref` 正常爬到 0.057（斜坡工作正常）。
  - 积分顶满 1.5deg（59% 时间），`theta_ref` 被抬到 +1.29deg。
  - 车身实际 theta = +0.81deg，theta - theta_ref = -0.48deg。
  - 角度环输出 35 × (-0.48) ≈ **-17rpm**，与前馈 **+17.5rpm** 精确抵消，轮子净指令 ≈ 0，静摩擦锁死。
- 死锁机理：车不动 → 速度误差不消 → 积分抬 theta_ref → 角度环输出更负 → 恰好吃掉前馈 → 车更不动。系统收敛到"谁也不动"的静态平衡。
- 同一矛盾在动态下就是振荡——上次"跑太快倾角收不住"（数据未导出）与此同源。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `BALANCE_POSITION_KI_DEG_PER_M_S` | `5.0f` | `0.0f` | KI 是为"纯倾角顶摩擦"方案设计的；前馈架构下摩擦由轮速内环破，倾角积分与前馈死锁，数据已证明有害。平地爬行不要再打开 |

### 预期行为（KI=0 后）

- 发 `1`：theta_ref 只剩 Kv 项（起步约 2.5×0.06=0.15deg，随速度接近目标趋于 0）。
- 角度环起步输出约 -5rpm，前馈 +17.5rpm，净 +12.5rpm → 轮速内环（静摩擦前馈+积分）破摩擦起转。
- 巡航平衡点：轮子 ~18rpm、v=0.06、速度误差 0、theta_ref≈0、theta≈0，直立姿态爬行。
- 起步瞬间车身会先轻微后仰零点几度再跟上（非最小相位，属正常）。

## 2026-07-07 结构性收尾：零点自学习 + 巡航速度移入平滑滚动窗口

### 判断（回应"不是顶不动摩擦就是倾角大了救不回"）

- 两个正交根源，各给一个结构性解法，不再手拧参数：
  1. **"自己立不住" = 零点会话间温漂**（raw 零点上午 -0.5deg / 下午 -2.0deg，几小时漂 1.5deg）→ 零点自学习。
  2. **"不动或窜" = 目标速度在低速楼板以下**。楼板成分（无刷电机）：齿槽转矩（磁性静摩擦）、减速箱/轴承摩擦、驱动器电流死区、**CAN 速度反馈噪声（峰峰 13~25rpm，18rpm 目标淹没在噪声里，闭环无从谈起）**。0.06m/s(18rpm) 同时踩着摩擦楼板和反馈噪声楼板；0.20m/s(62rpm) 起步电流 0.108A 直接击穿破断力矩，反馈信噪比 3~5 倍。
- 轮胎与地面的摩擦不是楼板（它是"滚动不打滑"的约束，是好东西）；数据里轮子从未打滑，只有"锁死不转"。

### 零点自学习实现

- 原理：静止直立时车实际站住的角度**就是**物理零点（力学定义）。学习不需要"站得好"，只需要"偶尔停驻"——零点错 1deg 的 14:34 数据里停驻时刻累计 79s，样本充足；正向棘轮（每次停驻拉近零点→站得更省力→停驻更多）。
- 实现（`gimbal_task.c`，约 15 行）：`theta_deg -= balance_zero_trim_deg`；门控满足时 `trim += 0.02 * theta * dt`（时间常数 50s，与直立环 ~1s 解耦），限幅 ±2deg，整个会话保留、不随 fault 清零，掉电回到 `BALANCE_ANGLE_OFFSET_DEG` 基准。
- 门控（全部满足才学习）：stand 模式、|theta|<3deg、|theta_rate|<10dps、|v|<0.03m/s、左右轮 |rpm|<5。被推/行走/摔倒全冻结；若零点错到永远无法停驻（±2deg 外），学习保持冻结，安全失效。

### 参数修改

| 参数 | 旧值 | 新值 | 原因 |
|---|---:|---:|---|
| `BALANCE_MODE_DRIVE_VELOCITY_MPS` | `0.06f` | `0.20f` | 移入平滑滚动窗口 [0.15, 0.35] 中间；守卫 0.40 封顶 |
| `BALANCE_MODE_VELOCITY_RAMP_MPS_PER_SEC` | `0.04f` | `0.08f` | 2.5s 到巡航；过慢的斜坡会让起步长时间停留在楼板区 |
| `BALANCE_ZERO_TRIM_*` | 无 | ENABLE=1, RATE=0.02, LIMIT=2.0, 门控 V=0.03/RATE=10/THETA=3/WHEEL=5 | 零点自学习 |
| VOFA I15 | `integral_theta_ref_deg`（KI=0 后恒 0） | `zero_trim_deg` | 观察自学习收敛 |

- 语法验证通过：`gimbal_task.c / ins_task.c / chassis_serial_cmd.c`。

### 下一步测试

1. 烧录后 KEY 解锁站立 1~2 分钟：看 I15 `zero_trim_deg` 从 0 缓慢滑向某个稳定值（量级 ±0.5deg 内），同时 I0 `theta` 均值趋向 0，车踱步减少。
2. 发 `1`：v_ref 2.5s 斜坡到 0.20，预期 v 平滑跟随、theta 接近 0 直立姿态巡航、不再"不动或窜"。
3. 发 `0`：平稳停车。
4. 行走中推快过 0.4：守卫拉 v_ref 到 0，刹回 0.15 后自动恢复。
5. 全程导出 CSV。

## 2026-07-07 15:23 数据：站立/守卫成熟，前馈抵消死锁第二形态（车身后仰替代积分）

### 数据结论（4177 行，83.5s，全程 fault=0）

- 站立成熟：stand 29.6s theta 均值 +0.088deg；零点自学习只学 0.01deg（本会话零点本就准确）。
- **守卫实战验证通过**：63.9s 车被冲到 1.02m/s、theta +5deg、轮子 309rpm 全力追，守卫按住 v_ref=0 后救回，无 fault。历史最大速度偏移且未摔倒。
- **forward 不走，死锁第二形态**（26.9s 准静止段数字自洽）：
  - 车身后仰停在 theta=-1.20deg，theta_ref=+0.46deg（纯 Kv 项）。
  - 角度环输出 35×(-1.66)=-58rpm，前馈 +56rpm，净 -2rpm（实测 avg_ref=-2rpm、轮子 -0.8rpm），落在静摩擦死区。
  - KI 时代由积分抬 theta_ref 提供抵消，KI=0 后由**车身自己后仰**提供抵消——同一个不动点，换了吸收变量。

### 机理定性（本质）

- 角度环单一输出承担两件事（纠正 theta + 消化前馈）不可兼得：扶正后仰车身需要轮子向后滚，前馈持续 +56rpm，两者互相抵消；净指令 ±2rpm 在静摩擦死区内，**死锁点被静摩擦稳定**。
- "直立巡航 0.2m/s（theta≈0、轮 62rpm）"是合法平衡点，但从静止出发到不了——中间隔着死锁坑。
- 教程车前馈加在力矩/PWM 级不会被角度环抵消；我们的前馈加在轮速指令级，与角度环输出同级必然打架。**轮速级速度前馈 + 角度环输出轮速指令的组合存在结构性冲突。**

### 候选修法（未实施，待选）

| 方案 | 思路 | 评价 |
|---|---|---|
| A. 速度解耦 | 轮速指令 = 实测车速换算轮速 + 角度环输出；角度环只管"轮子相对车身"修正，速度调节全交 Kv→theta_ref | 轮速伺服拓扑标准写法，根治；实现前需推一遍 v 正反馈路径稳定性 |
| B. 起步踢脚 | 进 forward 时 0.3~0.5s 反向轮速脉冲把车身甩前倾，跳过死锁坑 | 最小改动，补丁不根治 |
| C. 回纯串级（去前馈） | theta_ref 驱动 | 轮速伺服下巡航需 62/35=1.77deg 恒定角差，速度由摩擦角决定，控不准，不推荐 |

## 2026-07-07 急停/故障后强制回 stand

### 漏洞修复

- 原逻辑在 KEY 急停、角度/角速度保护、反馈/使能故障时只切急停输出，没有把 `chassis_mode` 退回 `CHASSIS_MODE_STAND`。
- 风险：若车在 `forward/backward` 中触发急停或故障，扶正后再次按 KEY 解锁会沿用旧行走模式，可能直接继续前进/后退，而不是先站住。

### 修改内容

| 修改项 | 旧逻辑 | 新逻辑 |
|---|---|---|
| KEY 运行中按下 | `emergency_stop = 1` | 先 `Chassis_SetControlMode(CHASSIS_MODE_STAND)`，再急停 |
| 角度/角速度保护 | 只急停并 reset PID | 先强制回 stand，再急停并 reset PID |
| 电机反馈/使能故障 | 只急停 | 先强制回 stand，再急停 |

### 验证

- `Application/gimbal/gimbal_task.c` 已通过 `arm-none-eabi-gcc -fsyntax-only`。
- 下一次测试：在 `1` 前进中按 KEY 急停，再按 KEY 解锁，I14 应先保持 `0`，不会自动恢复 `1`。
