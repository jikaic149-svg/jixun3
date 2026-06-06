# 三端分工任务文档

## 0. 分工总览

本项目拆成三个并行任务：

```text
视觉端：
  MaixCam2 + USB摄像头 + 视觉识别 + 坐标输出 + 语音播报

硬件端：
  机械结构 + 电源分配 + 线束 + CAN收发器 + 激光驱动 + 安装标定

电控端：
  DJI C板底盘控制 + ESP32-S3云台控制 + 通信协议 + 状态机
```

三端边界：

```text
视觉端不控制电机。
硬件端不决定控制算法。
电控端不处理原始图像。
```

最终交付关系：

```text
视觉端输出 dx/dy/score/valid。
硬件端保证传感器、电机、电源、线束可靠。
电控端使用视觉坐标和传感器数据完成平衡、巡线、打靶。
```

---

## 1. 视觉端任务

### 1.1 目标

视觉端负责：

```text
1. USB摄像头采图。
2. 靶心识别。
3. 基于固定标定关系补偿摄像头-激光安装偏差。
4. 计算激光等效落点相对靶心的dx、dy。
5. 计算score。
6. 通过Wi-Fi向ESP32-S3发送坐标包。
7. 根据ESP32-S3命令播放WAV语音。
```

### 1.2 硬件范围

```text
MaixCam2
USB摄像头
摄像头-激光固定标定参数
板载或外接音频输出
```

### 1.3 输入数据

```text
USB摄像头图像帧
ESP32-S3发送的AUDIO_CMD
ESP32-S3发送的视觉模式命令
```

### 1.4 输出数据

MaixCam2通过Wi-Fi UDP发送：

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

### 1.5 处理流程

```text
USB摄像头采图
  -> 图像预处理
  -> 靶心识别
  -> 根据固定标定补偿得到激光等效落点
  -> 计算dx/dy
  -> 计算score
  -> 发送坐标包
  -> 接收播报命令
  -> 播放WAV
```

### 1.6 频率指标

```text
图像处理真实帧率：以实测为准，目标60FPS-80FPS
坐标包发送频率：50Hz-100Hz
语音播报：按任务触发
```

注意：

```text
100Hz发送不等于视觉真实100FPS。
如果图像帧未更新，必须保持原始vision_timestamp，不能伪装成新视觉帧。
```

### 1.7 验收标准

```text
1. 能稳定识别靶心。
2. 能完成摄像头-激光固定偏差标定。
3. 能输出dx/dy/score。
4. 坐标包带seq、timestamp、CRC。
5. Wi-Fi丢包后ESP32-S3可识别旧包。
6. 播放WAV时视觉帧率不能明显掉帧。
```

### 1.8 禁止事项

```text
不要无线传图。
不要控制电机。
不要无条件自主播报分数。
不要在视觉主循环里阻塞播放音频。
```

---

## 2. 硬件端任务

### 2.1 目标

硬件端负责：

```text
1. 平衡车底盘机械结构。
2. yaw/pitch二轴云台结构。
3. USB摄像头、激光笔、MaixCam2、ESP32-S3安装。
4. 电池和分电板供电。
5. CAN收发器和通信线束。
6. 激光MOSFET/三极管驱动。
7. yaw有限旋转走线。
8. 摄像头和激光标定结构。
```

### 2.2 机械范围

```text
两轮平衡车底盘
yaw无刷云台
pitch舵机支架
USB摄像头安装座
激光笔安装座
底层配重块安装位
ESP32-S3上层固定板
MaixCam2下层固定板
线束应力释放结构
机械限位结构
```

### 2.3 电气范围

```text
电池
分电板
DJI C板供电
ESP32-S3稳压
MaixCam2稳压
pitch舵机稳压
激光供电
CAN收发器
无线串口/无线模块
共地或隔离方案
```

### 2.4 关键连接

```text
ESP32-S3 GPIO -> MOSFET/三极管 -> 激光笔
ESP32-S3 TWAI_TX/RX -> CAN收发器 -> yaw电机CAN
DJI C板 -> 两轮无刷CAN
USB摄像头 -> MaixCam2
pitch舵机 -> ESP32-S3 PWM
上层BMI088 -> ESP32-S3 SPI
```

### 2.5 yaw走线要求

```text
yaw软件限位：±120°
yaw机械限位：约±130°
USB线不走滑环
线束沿yaw轴附近走
上下两端做应力释放
留U形柔性余量
```

### 2.6 激光安装要求

```text
摄像头和激光在x轴尽量保持一致。
允许存在y轴偏差。
激光安装必须固定可靠，比赛主流程不依赖MaixCam2识别激光点。
标定阶段可临时观察激光落点，用于确认laser_est映射关系。
激光电源由MOSFET/三极管驱动，不直接接S3 GPIO。
```

### 2.7 验收标准

```text
1. 底盘能稳定站立结构不松动。
2. 底层可按调试结果添加配重块，保证重心居中且偏低。
3. yaw在±120°内线束不拉扯。
4. pitch舵机全行程不碰撞。
5. USB摄像头不掉线。
6. ESP32-S3能稳定读BMI088。
7. ESP32-S3能通过CAN控制yaw电机。
8. 激光可由ESP32-S3开关控制。
9. MaixCam2能稳定接收USB摄像头画面。
```

### 2.8 禁止事项

```text
不要让USB线穿过滑环。
不要用S3 GPIO直接给激光供电。
不要让舵机电源从S3板载5V直接取。
不要让电机电源噪声污染视觉和IMU供电。
```

---

## 3. 电控端任务

### 3.1 目标

电控端负责：

```text
1. DJI C板底盘平衡控制。
2. 灰度巡线和标识点停车。
3. ESP32-S3 yaw串级闭环。
4. ESP32-S3 pitch舵机闭环。
5. ESP32-S3激光安全控制。
6. MaixCam2、ESP32-S3、DJI C板通信协议。
7. 任务状态机。
8. 无线遥测和调试。
```

### 3.2 DJI C板任务

```text
BMI088采样：1000Hz
姿态估计：500Hz-1000Hz
LQR平衡环：500Hz-1000Hz
轮速/ADRC执行层：200Hz-500Hz
灰度巡线PID：100Hz-200Hz
底盘位姿同步：10Hz-30Hz
```

底盘控制：

```text
外层PID：
  line_error、目标速度、断线航向 -> v_ref、yaw_diff_ref

中层LQR：
  pitch、pitch_rate、车速估计 -> 左右轮目标修正

内层ADRC：
  left_ref/right_ref -> CAN电机控制量
```

### 3.3 ESP32-S3任务

```text
Wi-Fi接收MaixCam2坐标：50Hz-100Hz
SPI读取上层BMI088：1000Hz
yaw控制任务：500Hz-1000Hz
VisionFilterTask：100Hz
pitch舵机控制：50Hz-100Hz
DJI C低频无线同步：10Hz-30Hz
SafetyTask：50Hz-100Hz
```

yaw控制：

```text
视觉外环：
  dx -> yaw_angle_ref

角度环：
  yaw_angle_ref - yaw_encoder

角速度环：
  BMI088 gyro_z / yaw_rate

执行：
  CAN输出yaw电机命令
```

pitch控制：

```text
dy_ref -> pitch_target -> PWM舵机
```

激光控制：

```text
laser_on = local_lock_ok && chassis_permission
```

### 3.4 通信任务

MaixCam2 -> ESP32-S3：

```text
Wi-Fi UDP
dx/dy/score/valid/timestamp/CRC
```

DJI C板 -> ESP32-S3：

```text
低频无线
chassis_x/y/yaw/balance_ok/mark_detected/permission
```

ESP32-S3 -> DJI C板：

```text
低频无线
gimbal_yaw/pitch/lock_state/score/laser_state/fault
```

ESP32-S3 -> yaw电机：

```text
有线CAN
yaw_motor_cmd
```

### 3.5 状态机

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

### 3.6 验收标准

```text
1. DJI C板能稳定完成底盘直立。
2. 底盘能完成巡线和断线直走。
3. ESP32-S3能稳定接收MaixCam2坐标。
4. ESP32-S3能通过CAN完成yaw角度/速度闭环。
5. ESP32-S3能完成pitch舵机闭环。
6. ESP32-S3能安全控制激光。
7. 三节点能完成状态同步。
8. 完整流程能从A巡线到标识点、打靶、播报、继续到B。
```

### 3.7 禁止事项

```text
不要让无线进入底盘平衡闭环。
不要让无线进入yaw角速度内环。
不要把过期视觉包当新数据。
不要在高频控制任务里打印日志。
不要在通信超时时保持激光开启。
```

---

## 4. 三端接口汇总

### 4.1 视觉端 -> 电控端

```text
MaixCam2 -> ESP32-S3
协议：Wi-Fi UDP
内容：dx、dy、score、valid、timestamp、target_x/y、laser_est_x/y、quality、audio_state
频率：50Hz-100Hz
```

### 4.2 电控端 -> 视觉端

```text
ESP32-S3 -> MaixCam2
协议：Wi-Fi UDP
内容：AUDIO_CMD、mode、start_aim、stop_aim
频率：按需
```

### 4.3 硬件端 -> 电控端

```text
提供：
  CAN收发器
  激光驱动电路
  稳压供电
  yaw/pitch机械限位
  可靠线束
```

### 4.4 电控端 -> 硬件端

```text
需要硬件保证：
  ESP32-S3能读上层BMI088
  ESP32-S3能通过CAN控制yaw
  ESP32-S3能控制激光
  MaixCam2能稳定采图
  yaw线束不干涉
```

---

## 5. 集成顺序

```text
1. 硬件端完成供电、安装、线束、CAN、激光驱动。
2. 电控端完成DJI C板底盘直立。
3. 电控端完成ESP32-S3 yaw CAN控制。
4. 视觉端完成靶心识别。
5. 视觉端完成摄像头-激光固定偏差标定。
6. 视觉端完成Wi-Fi坐标包。
7. 电控端接入视觉坐标，完成yaw/pitch闭环。
8. 电控端接入激光锁定。
9. 视觉端接入语音播报。
10. 三端联调完整任务。
```
