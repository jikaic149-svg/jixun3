# 平衡式巡逻打靶小车系统方案

## 0. 最终结论

本方案采用三主控节点：

```text
DJI C板：
  底盘平衡、两轮无刷、灰度巡线、底盘位姿估计、底盘任务状态

ESP32-S3：
  yaw云台控制主节点、pitch舵机控制、上层BMI088采样、CAN控制yaw电机、激光控制、无线通信网关

MaixCam2：
  USB摄像头采图、靶子ROI识别、靶心计算、固定映射补偿、score计算、WAV语音播报
```

核心边界：

```text
DJI C板控制底盘，不控制yaw云台内环。
ESP32-S3控制yaw、pitch和激光。
MaixCam2只处理视觉和语音，不控制电机。
MaixCam2到ESP32-S3无线只传坐标，不传图像。
DJI C板和ESP32-S3低频无线同步空间位姿和任务状态，不进入实时闭环。
```

---

## 1. 硬件约束

已确定约束：

```text
1. 两个无刷电机作为平衡车驱动轮。
2. 驱动轮通过CAN与DJI C板通信。
3. DJI C板根据内置BMI088做LQR姿态闭环，实现直立。
4. DJI C板两轮执行层采用全速域统一PI + ESO / LADRC，不做高速/低速控制器切换。
5. 灰度传感器 + BMI088姿态用于巡线和无黑线直走。
6. yaw云台使用无刷电机。
7. ESP32-S3通过外接CAN收发器接入CAN总线，控制yaw电机。
8. ESP32-S3通过SPI以约1000Hz读取上层BMI088。
9. pitch轴使用轻量舵机，由ESP32-S3控制。
10. USB摄像头和激光笔安装在pitch端。
11. USB摄像头连接下层MaixCam2。
12. 没有USB滑环，因此USB线不能穿过滑环。
13. yaw不做360°连续旋转，采用软件±120°、机械约±130°限位。
14. MaixCam2需要语音播报。
15. MaixCam2与ESP32-S3之间使用Wi-Fi传坐标。
16. DJI C板与ESP32-S3之间使用低频无线同步空间位姿和任务状态。
17. 底层允许根据调试结果添加配重块，保证重心居中且偏低。
```

关键修正：

```text
ESP32-S3现在是yaw云台控制主节点。
DJI C板不再负责yaw云台内环，只负责底盘和平衡车任务。
MaixCam2无线传输的是dx/dy/score等低维坐标数据，不是图像数据。
```

---

## 2. 系统拓扑

```text
                         上层pitch端
                 ┌──────────────────────┐
                 │ USB摄像头 + 激光笔    │
                 │ pitch舵机             │
                 └──────────┬───────────┘
                            │ PWM/电源/激光控制
                 ┌──────────▼───────────┐
                 │ ESP32-S3              │
                 │ 上层BMI088            │
                 │ CAN收发器             │
                 │ yaw/pitch/laser控制   │
                 └──────────┬───────────┘
                            │
            USB线 + 电源线 + CAN线，随yaw轴柔性走线
            yaw软件限位±120°，机械限位约±130°
                            │
                 ┌──────────▼───────────┐
                 │ 下层车体              │
                 │ MaixCam2              │
                 │ DJI C板               │
                 │ 两轮无刷              │
                 │ 灰度阵列 + 分电板      │
                 └──────────────────────┘
```

无线关系：

```text
MaixCam2 --Wi-Fi坐标包--> ESP32-S3
DJI C板 --低频无线位姿包--> ESP32-S3
ESP32-S3 --低频无线云台状态--> DJI C板
```

走线原则：

```text
USB线不走滑环。
普通导电滑环只传电源和低速/中速通信。
跨yaw活动区域的线束必须柔软、固定、留余量。
图像原始数据不跨无线和滑环。
底部可加配重块，但必须左右对称、靠近车体中心、固定可靠。
```

---

## 3. 主链路总览

### 3.1 底盘链路

```text
BMI088/编码器/灰度
  -> DJI C板
  -> PID -> LQR -> PI+ESO/LADRC统一执行层
  -> 两轮无刷
```

### 3.2 x轴打靶链路

```text
USB摄像头
  -> MaixCam2本地视觉处理
  -> Wi-Fi发送dx/valid/score/timestamp
  -> ESP32-S3滤波/预测
  -> ESP32-S3 1000Hz读取BMI088
  -> yaw串级闭环
  -> CAN控制yaw无刷
```

### 3.3 y轴打靶链路

```text
USB摄像头
  -> MaixCam2本地视觉处理
  -> Wi-Fi发送dy/valid/timestamp
  -> ESP32-S3滤波
  -> ESP32-S3控制pitch舵机
```

### 3.4 空间位姿同步链路

```text
DJI C板：
  估计底盘x/y/yaw、速度、巡线状态

ESP32-S3：
  估计云台yaw/pitch、目标锁定状态

低频无线：
  只同步空间位姿、任务状态、故障码
```

### 3.5 语音播报链路

```text
MaixCam2计算score
  -> ESP32-S3判断lock状态
  -> ESP32-S3触发MaixCam2播放WAV
  -> MaixCam2回传audio_state
```

---

## 4. 数据分层总览

| 原始数据 | 采样节点 | 处理节点 | 处理结果 | 传输给谁 | 最终用途 |
|---|---|---|---|---|---|
| 底盘BMI088 | DJI C板 | DJI C板 | pitch、pitch_rate、yaw_rate | 本地使用，低频给S3 | LQR平衡、断线直走 |
| 两轮电机反馈 | DJI C板 | DJI C板 | 轮速、电流、异常状态 | 本地使用 | PI+ESO/LADRC执行层、速度估计 |
| 灰度阵列 | DJI C板 | DJI C板 | line_error、line_valid、mark_detected | 本地使用 | 巡线、标识点停车 |
| USB图像帧 | MaixCam2 | MaixCam2 | target_center、laser_est、dx、dy、score、valid | Wi-Fi给S3 | 打靶视觉外环 |
| 上层BMI088 | ESP32-S3 | ESP32-S3 | gyro、rate、disturbance | 本地使用，低频遥测 | yaw增稳、预测辅助 |
| yaw电机反馈 | ESP32-S3 | ESP32-S3 | yaw_angle、yaw_rate | 本地使用 | yaw角度/速度闭环 |
| pitch舵机状态 | ESP32-S3 | ESP32-S3 | pitch_target、pitch_ready | 低频给DJI C板 | y轴闭环和状态同步 |
| 激光控制状态 | ESP32-S3 | ESP32-S3 | laser_on/off | 低频给DJI C板/MaixCam2 | 打靶安全、锁定 |
| 底盘空间位姿 | DJI C板 | DJI C板 | chassis_x/y/yaw | 低频无线给S3 | 三维空间结构 |
| 音频状态 | MaixCam2 | MaixCam2 | audio_state、audio_done | Wi-Fi给S3 | 播报流程推进 |

---

## 5. DJI C板职责

DJI C板负责底盘强实时闭环：

```text
1. 底盘BMI088采样与姿态估计
2. 编码器轮速、电机电流、故障状态读取
3. LQR平衡控制
4. 两个驱动轮无刷电机CAN电流/力矩控制
5. 全速域统一PI + ESO / LADRC执行层扰动抑制
6. 灰度巡线和无黑线直走
7. 底盘空间位姿估计
8. 标识点停车
9. 底盘任务状态同步
```

推荐频率：

```text
BMI088采样：1000Hz
姿态估计：500Hz-1000Hz
LQR平衡环：500Hz-1000Hz
PI+ESO/LADRC执行层：200Hz-500Hz
灰度巡线PID：100Hz-200Hz
底盘位姿同步无线发送：10Hz-30Hz
```

底盘控制总结构：

```text
传感器：
  BMI088 + 编码器 + 灰度

姿态估计：
  Kalman -> theta, theta_dot

外层：
  速度PID / 位移PID / 巡线PID
  -> v_ref, x_ref, yaw_cmd

中层：LQR姿态控制
  输入：theta、theta_dot、v、x、v_ref、x_ref
  输出：u_LQR_L, u_LQR_R

内层：
  全速域统一PI + ESO / LADRC
  -> u_motor_L, u_motor_R

执行：
  CAN电流命令给左右无刷轮
```

核心原则：

```text
不按高速/低速切换PI和ADRC。
ESO始终打开，PI/LADRC始终存在。
积分项根据平衡风险动态削弱。
扰动补偿根据估计可信度动态限幅。
LQR永远拥有最高控制优先级。
当前姿态调试阶段仍禁止给驱动轮输出电流。
```

单轮简化对象：

```text
omega_dot = b0 * u + d

omega：实际轮速
u    ：电机电流/力矩控制命令
b0   ：估计的电机响应增益
d    ：总扰动
```

扰动d包含地面摩擦变化、轮胎打滑、坡度、电池电压下降、左右轮差异、重心变化、机械阻力和电机模型误差。

ESO离散公式：

```text
e_eso = omega - z1

z1 += dt * (z2 + b0 * u_last + beta1 * e_eso)
z2 += dt * (beta2 * e_eso)

beta1 = 2 * wo
beta2 = wo * wo
```

说明：

```text
z1 ≈ omega
z2 ≈ d
wo越大，扰动估计越快，但越吃编码器噪声。
wo越小，估计更稳，但补偿变慢。
```

PI + ESO / LADRC执行律：

```text
risk_theta = abs(theta) / theta_limit
risk_rate  = abs(theta_dot) / theta_dot_limit
risk = clamp(risk_theta + risk_rate, 0, 1)

Ki_eff = Ki * (1 - risk)

e_w = omega_ref - z1
I += e_w * dt * (1 - risk)

u0 = Kp * e_w + Ki_eff * I

z2_used = limit(z2, -d_max, d_max)
u_raw = (u0 - z2_used) / b0
u = limit(u_raw, -u_max, u_max)
```

抗积分饱和：

```text
if abs(u_raw) > u_max:
    I -= e_w * dt * (1 - risk)
```

更平滑的写法：

```text
I_dot = e_w * (1 - risk) - Kaw * (u_raw - u)
I += I_dot * dt
```

LQR和执行层关系：

```text
优先方案：
  LQR直接输出左右轮电流/力矩基准。
  u_motor = limit(u_LQR - z2_used / b0, -u_max, u_max)

备选方案：
  omega_ref = f(u_LQR)
  u_motor = PI_ESO(omega_ref, omega)
```

对平衡车，电流/力矩接口优先于轮速接口。平衡需要快速施加轮端力矩，不能让内层强行死追固定轮速并抢走LQR的救车权限。

推荐最终公式链：

```text
theta, theta_dot = Kalman(acc, gyro)

v = r / 2 * (omega_L + omega_R)

theta_ref = PID_v(v_ref - v)

X = [
  theta - theta_ref
  theta_dot
  v - v_ref
  x - x_ref
]^T

u_balance = -K * X

u_LQR_L = u_balance - yaw_cmd
u_LQR_R = u_balance + yaw_cmd
```

---

## 6. ESP32-S3职责

ESP32-S3是云台控制主节点。

负责：

```text
1. Wi-Fi接收MaixCam2坐标包
2. 对dx/dy做时间戳检查、滤波、预测
3. SPI 1000Hz读取上层BMI088
4. CAN接收yaw电机/编码器反馈
5. CAN控制yaw无刷
6. 控制pitch舵机
7. 控制激光开关
8. 与DJI C板低频无线同步空间位姿和任务状态
9. 与MaixCam2交互语音播报命令
10. 无线遥测和调试
```

不负责：

```text
不控制底盘两轮无刷。
不做底盘平衡闭环。
不处理原始图像。
不把DJI C板低频位姿无线链路放进yaw高频内环。
```

FreeRTOS双核建议：

```text
Core 0：通信核
  MaixCamWifiRxTask     50Hz-100Hz
  DjiPoseWirelessTask   10Hz-30Hz
  TelemetryTask         5Hz-20Hz
  AudioCmdTask          按需

Core 1：实时核
  ImuTask               1000Hz
  YawControlTask        500Hz-1000Hz
  VisionFilterTask      100Hz
  PitchControlTask      50Hz-100Hz
  SafetyTask            50Hz-100Hz
```

---

## 7. MaixCam2职责

MaixCam2负责视觉和语音。

视觉处理：

```text
USB摄像头采图
YOLO识别靶子ROI
ROI内OpenCV经典视觉处理
角点提取
透视变换矫正靶面
靶心识别
根据固定标定关系补偿摄像头-激光安装偏差
laser_est_x, laser_est_y = 标定映射得到的激光等效落点
dx = target_x - laser_est_x
dy = target_y - laser_est_y
score = 根据激光等效落点到靶心距离计算环数
```

坐标传输：

```text
MaixCam2本地视觉真实帧率以实测为准。
如果视觉帧率约80FPS，可以用100Hz发送最新坐标包。
100Hz发送不代表视觉真实100FPS，只是周期上报最新结果。
```

语音播报：

```text
接收ESP32-S3的AUDIO_CMD。
播放预录制WAV。
回传audio_state。
```

禁止：

```text
不要无线传图。
不要控制电机。
不要承担总状态机。
不要识别到score后无条件自主播报。
```

---

## 8. yaw串级闭环

ESP32-S3负责x轴yaw闭环。

控制层级：

```text
视觉外环：
  dx -> yaw_angle_ref
  频率：50Hz-100Hz

角度环：
  yaw_angle_ref - yaw_encoder
  频率：200Hz-500Hz

角速度环：
  BMI088 gyro_z / yaw_rate
  频率：500Hz-1000Hz

电机执行：
  CAN输出yaw电机命令
  频率：按电机协议能力，目标500Hz左右
```

必要条件：

```text
ESP32-S3必须能通过CAN获得yaw电机反馈或编码器反馈。
ESP32-S3到yaw电机必须是有线CAN。
MaixCam2到S3的Wi-Fi只作为视觉外环输入。
```

视觉数据使用规则：

```text
age < 50ms：正常使用
50ms <= age < 100ms：降低视觉外环增益
100ms <= age < 150ms：保持或短时预测
age >= 150ms：丢弃视觉目标，yaw保持或回中
```

---

## 9. pitch控制方案

pitch由ESP32-S3控制。

```text
dy_ref -> pitch_target += Ky * dy_ref -> PWM控制舵机
```

限制：

```text
pitch_target限制在机械安全角内。
舵机动作限速，避免过冲。
valid=0时保持或回中。
```

摄像头与激光安装：

```text
摄像头和激光在x轴尽量保持一致。
允许存在y轴偏差，但必须标定dy_offset。
比赛主流程不识别激光点，使用固定标定映射补偿摄像头-激光偏差。
最终闭环目标是让激光等效落点与靶心重合。
```

---

## 10. 激光控制方案

当前激光由ESP32-S3控制是可行的。

推荐连接：

```text
ESP32-S3 GPIO -> MOSFET/三极管驱动 -> 激光笔电源
```

不要让ESP32-S3 GPIO直接给激光供电。

控制逻辑：

```text
ESP32-S3判断：
  vision_valid = 1
  yaw/pitch误差满足阈值
  yaw未超限
  pitch状态正常
  通信未超时
  锁定时间达到要求

满足条件后：
  laser_on = 1
```

如果需要DJI C板参与安全授权：

```text
DJI C板 -> ESP32-S3：
  laser_enable_permission

ESP32-S3最终输出：
  laser_on = local_lock_ok && laser_enable_permission
```

推荐使用“双条件授权”：

```text
DJI C板负责底盘安全许可。
ESP32-S3负责云台和视觉锁定许可。
两者都满足才开激光。
```

安全规则：

```text
任一关键通信超时：关闭激光。
vision_valid=0：关闭激光。
yaw超限：关闭激光。
pitch故障：关闭激光。
底盘不稳定：关闭激光。
```

---

## 11. 通信协议

### 11.1 MaixCam2 -> ESP32-S3

Wi-Fi UDP坐标包，建议50Hz-100Hz：

```text
msg_id
seq
timestamp_ms
valid
dx
dy
score
target_x, target_y
laser_est_x, laser_est_y
quality
vision_fps
audio_state
crc16
```

要求：

```text
只传坐标，不传图像。
ESP32-S3只使用最新seq。
旧包、乱序包直接丢弃。
```

### 11.2 DJI C板 -> ESP32-S3

低频无线位姿包，建议10Hz-30Hz：

```text
msg_id
seq
timestamp_ms
chassis_x
chassis_y
chassis_yaw
chassis_v
line_state
mark_detected
balance_ok
battery_voltage
laser_enable_permission
fault_code
crc16
```

### 11.3 ESP32-S3 -> DJI C板

低频无线云台状态包，建议10Hz-30Hz：

```text
msg_id
seq
timestamp_ms
gimbal_yaw
gimbal_pitch
target_valid
dx_ref
dy_ref
score
lock_state
laser_state
fault_code
crc16
```

### 11.4 ESP32-S3 -> yaw电机

有线CAN：

```text
yaw_motor_cmd
yaw_target
yaw_rate_target
current_or_torque_cmd
```

### 11.5 yaw电机 -> ESP32-S3

有线CAN：

```text
yaw_encoder
yaw_speed
yaw_current
yaw_motor_fault
```

---

## 12. 三维空间结构

低频无线用于统一空间坐标。

坐标系：

```text
W：世界/场地坐标系
C：底盘坐标系，由DJI C板估计
G：yaw云台坐标系，由ESP32-S3估计
P：pitch/相机坐标系，由ESP32-S3 + MaixCam2估计
T：靶面坐标系，由MaixCam2视觉识别
```

数据来源：

```text
DJI C板：
  提供 chassis_x、chassis_y、chassis_yaw

ESP32-S3：
  提供 gimbal_yaw、gimbal_pitch

MaixCam2：
  提供 target_x/y、laser_est_x/y、dx/dy
```

使用原则：

```text
三维空间结构用于状态估计、显示、辅助决策。
不作为yaw高频内环主反馈。
超过200ms未收到DJI C板位姿，冻结底盘位姿，只用云台局部控制。
```

---

## 13. 任务状态机

总状态机建议仍由DJI C板和ESP32-S3分层协同：

```text
DJI C板：
  负责底盘任务状态。

ESP32-S3：
  负责云台打靶状态。
```

状态流程：

```text
INIT
BALANCE_READY
LINE_FOLLOW
LINE_LOST_STRAIGHT
MARK_DETECTED
STOP_AT_MARK
GIMBAL_CENTER
AIM_SEARCH
AIM_TRACK
LOCK_3S
REPORT_SCORE
WAIT_TARGET_MOVE
RELOCK
CONTINUE_TO_B
STOP_AT_B
```

状态同步：

```text
DJI C板停车并发送MARK_DETECTED/STOP_AT_MARK。
ESP32-S3进入AIM_SEARCH/AIM_TRACK。
ESP32-S3完成LOCK_3S后触发MaixCam2播报。
ESP32-S3回传lock_state和score。
DJI C板收到完成状态后继续巡线。
```

---

## 14. 风险与处理

| 风险 | 后果 | 处理 |
|---|---|---|
| MaixCam2 Wi-Fi坐标包抖动 | yaw视觉外环滞后 | timestamp、seq、age判断，超过150ms丢弃 |
| 视觉100Hz发送但真实帧率低 | 重复旧数据 | 包内保留vision_seq和vision_timestamp |
| ESP32-S3 CAN反馈不足 | yaw闭环不完整 | 必须读取yaw编码器/速度反馈 |
| S3控制激光无硬件驱动 | GPIO损坏 | 使用MOSFET/三极管驱动 |
| 底盘和云台无线状态不同步 | 三维坐标误差 | 低频同步只做辅助，超时冻结 |
| USB线被yaw卷绕 | 摄像头掉线 | yaw软件±120°，机械±130° |
| Wi-Fi占用S3资源 | 实时任务抖动 | 控制任务独立核心，Wi-Fi任务低优先级 |
| 激光误开 | 安全风险 | 双条件授权，任一故障立即关闭 |
| LQR未站稳就打开积分/ESO补偿 | 车体抢控制、恢复时猛冲 | 先Ki=0，ESO只观察，确认z2可信后再限幅补偿 |
| 电机输出饱和但积分继续累加 | 脱离饱和后突然冲车 | 输出饱和时冻结积分或使用Kaw抗饱和 |
| ESO扰动估计过激 | 高频抖动、轮端电流震荡 | z2必须限幅，wo从小到大调 |

---

## 15. 调试顺序

```text
1. 当前阶段先完成BMI088 SPI 1000Hz读取、Kalman姿态、VOFA/蓝牙串口可视化。
2. 保持驱动轮电机电流输出关闭，只验证pitch、yaw和编码器反馈。
3. 完成CAN收发和左右轮反馈读取，但仍不下发驱动电流。
4. 开始底盘闭环前，确认机械直立零点、pitch极性、左右轮方向和急停逻辑。
5. 先关ESO补偿，Ki=0，只调LQR，让车能站住。
6. 加速度外环PID，让车能慢速前后移动。
7. 加转向/巡线yaw_cmd。
8. 开ESO，只观察z1、z2，不参与补偿。
9. z2波形可信后，逐步加入限幅扰动补偿。
10. 最后开启Ki_eff，必须带平衡风险削弱和输出饱和保护。
11. DJI C板完成巡线和标识点停车。
12. ESP32-S3完成CAN收发器和yaw电机通信。
13. ESP32-S3完成yaw编码器/电机反馈读取。
14. ESP32-S3完成上层BMI088 1000Hz读取。
15. ESP32-S3完成yaw角速度环。
16. ESP32-S3完成yaw角度环。
17. ESP32-S3完成pitch舵机控制。
18. MaixCam2完成靶子ROI、角点、透视变换和靶心识别。
19. MaixCam2完成Wi-Fi坐标包发送。
20. ESP32-S3完成视觉坐标接收、滤波、age判断。
21. 接入视觉外环控制yaw。
22. ESP32-S3完成激光MOSFET控制。
23. 接入锁定3秒逻辑。
24. MaixCam2完成语音播报。
25. DJI C板和ESP32-S3完成低频无线位姿同步。
26. 联调完整任务流程。
```

---

## 16. 最终数据归属

```text
DJI C板拥有：
  底盘原始数据
  平衡控制权
  巡线控制权
  底盘位姿估计

ESP32-S3拥有：
  上层BMI088原始数据
  yaw电机控制权
  pitch控制权
  激光控制权
  视觉数据整形权
  云台局部状态机

MaixCam2拥有：
  图像原始数据
  靶子ROI识别
  靶心计算
  摄像头-激光固定映射补偿
  score计算
  语音播报执行
```

最终禁止跨界：

```text
MaixCam2不控制电机。
DJI C板不处理原始图像。
DJI C板和S3的低频无线位姿同步不进入yaw高频内环。
MaixCam2到S3只传坐标，不传图像。
```
