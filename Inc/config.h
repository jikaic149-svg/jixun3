#ifndef __CONFIG_H__
#define __CONFIG_H__

/*
 * ========================================================
 * 平衡车底盘 QDrive 左右轮参数
 * ========================================================
 *
 * 左轮：can.id = 000
 * 右轮：can.id = 001
 */

#define LEFT_WHEEL_MOTOR_ID     0x00U
#define RIGHT_WHEEL_MOTOR_ID    0x01U

#define DRIVE_CAN_BAUD_RATE     1000000U
#define DRIVE_UART_BAUD_RATE    115200U

#define MOTOR_RATED_VOLTAGE_MIN  7     // 额定电压最小值 (V)
#define MOTOR_RATED_VOLTAGE_MAX  26    // 额定电压最大值 (V)
#define MOTOR_RATED_CURRENT      1.0f  // 额定电流 (A)
#define MOTOR_PEAK_CURRENT       2.0f  // 峰值电流 (A)
#define MOTOR_RATED_TORQUE       0.2f  // 额定扭矩 (Nm)
#define MOTOR_PEAK_TORQUE        0.3f  // 峰值扭矩 (Nm)
#define MOTOR_RATED_SPEED        500   // 额定转速 (rpm)
#define MOTOR_PEAK_SPEED         800   // 峰值转速 (rpm)
#define MOTOR_POLE_PAIRS         14    // 极对数
#define MOTOR_ENCODER_BITS       15    // 编码器位数 (bit)
#define MOTOR_CONTROL_ACCURACY   0.0109f // 控制精度 (°)

/* 左轮 QDrive 内部参数 */
#define LEFT_WHEEL_PID_SPEED_KP      0.00300f
#define LEFT_WHEEL_PID_SPEED_KI      0.000078f
#define LEFT_WHEEL_PID_SPEED_KD      0.000f
#define LEFT_WHEEL_PID_ANGLE_KP      1200.0f
#define LEFT_WHEEL_PID_ANGLE_KI      0.000f
#define LEFT_WHEEL_PID_ANGLE_KD      0.000f
#define LEFT_WHEEL_LIMIT_SPEED_RPM   500.0f
#define LEFT_WHEEL_LIMIT_CURRENT_A   1.65f

/* 右轮 QDrive 内部参数 */
#define RIGHT_WHEEL_PID_SPEED_KP     0.00300f
#define RIGHT_WHEEL_PID_SPEED_KI     0.000390f
#define RIGHT_WHEEL_PID_SPEED_KD     0.000f
#define RIGHT_WHEEL_PID_ANGLE_KP     1200.0f
#define RIGHT_WHEEL_PID_ANGLE_KI     0.000f
#define RIGHT_WHEEL_PID_ANGLE_KD     0.000f
#define RIGHT_WHEEL_LIMIT_SPEED_RPM  500.0f
#define RIGHT_WHEEL_LIMIT_CURRENT_A  1.65f

/*
 * ========================================================
 * 平衡车 IMU 与 Kalman 姿态配置
 * ========================================================
 *
 * 当前 BMI088 为水平安装。
 * 默认假设：传感器 X 轴朝车体前方，Z 轴垂直地面。
 * 前后平衡角使用 Pitch 加速度角，对应 Y 轴陀螺仪角速度。
 * yaw 只作为航向/差速外环参考，不进入直立平衡内环。
 */

#define BALANCE_IMU_SAMPLE_HZ        1000.0f
#define BALANCE_IMU_DT_SEC           (1.0f / BALANCE_IMU_SAMPLE_HZ)

#define BALANCE_AXIS_X               0
#define BALANCE_AXIS_Y               1
#define BALANCE_AXIS_Z               2

#define BALANCE_ACCEL_ANGLE_ROLL     0
#define BALANCE_ACCEL_ANGLE_PITCH    1

/* 水平安装默认使用 Pitch 加速度角 + Y 轴陀螺仪角速度。 */
#define BALANCE_ACCEL_ANGLE_MODE     BALANCE_ACCEL_ANGLE_PITCH
#define BALANCE_ACCEL_ANGLE_SIGN     1.0f
#define BALANCE_GYRO_AXIS            BALANCE_AXIS_Y
#define BALANCE_GYRO_SIGN            -1.0f

/* yaw 只用于底盘航向估计，现场如果方向反了只改符号。 */
#define CHASSIS_YAW_GYRO_AXIS        BALANCE_AXIS_Z
#define CHASSIS_YAW_GYRO_SIGN        1.0f
#define CHASSIS_YAW_RATE_BIAS_DPS    0.085f
/* Yaw rate scalar Kalman: bigger Q or smaller R = faster response, less smoothing. */
#define CHASSIS_YAW_RATE_KALMAN_Q    0.10f
#define CHASSIS_YAW_RATE_KALMAN_R    0.50f

/* 机械安装固定偏置；开机零点会在此基础上再归零。 */
#define BALANCE_ANGLE_OFFSET_DEG     0.0f
/* Balance car must not treat power-on pose as zero. Use fixed mechanical offset only. */
#define BALANCE_BOOT_ZERO_ENABLE     0

#define BALANCE_KALMAN_Q_ANGLE       0.012f
#define BALANCE_KALMAN_Q_BIAS        0.003f
#define BALANCE_KALMAN_R_MEASURE     0.110f

#define BALANCE_ACCEL_GATE_ENABLE    0
#define BALANCE_ACCEL_GRAVITY_MS2    9.80665f
#define BALANCE_ACCEL_NORM_MIN_G     0.75f
#define BALANCE_ACCEL_NORM_MAX_G     1.25f
#define BALANCE_ACCEL_ANGLE_LPF_ALPHA 0.05f

/* DRDY/SPI DMA未触发时允许轮询降级，先保证VOFA+有数据用于排查链路。 */
#define IMU_POLL_FALLBACK_ENABLE     1
#define IMU_POLL_FALLBACK_DIVIDER    1U
#define IMU_POLL_FALLBACK_DT_SEC     0.010f

/* VOFA CSV final attitude output:
 * ch0 pitch Kalman angle, deg
 * ch1 yaw compensated angle, deg
 */
#define VOFA_OUTPUT_ENABLE           1
#define VOFA_OUTPUT_DIVIDER          5U
#define VOFA_OUTPUT_CHANNEL_COUNT    2U
#define VOFA_UART_PORT_USART1        1
#define VOFA_UART_PORT_USART6        6
#define VOFA_UART_PORT               VOFA_UART_PORT_USART6

/* 旧通用宏只用于兼容现有代码，新底盘逻辑应使用 LEFT_/RIGHT_ 前缀。 */
#define MOTOR_PID_SPEED_KP       RIGHT_WHEEL_PID_SPEED_KP
#define MOTOR_PID_SPEED_KI       RIGHT_WHEEL_PID_SPEED_KI
#define MOTOR_PID_SPEED_KD       RIGHT_WHEEL_PID_SPEED_KD

#define MOTOR_PID_ANGLE_KP       RIGHT_WHEEL_PID_ANGLE_KP
#define MOTOR_PID_ANGLE_KI       RIGHT_WHEEL_PID_ANGLE_KI
#define MOTOR_PID_ANGLE_KD       RIGHT_WHEEL_PID_ANGLE_KD

#define MOTOR_LIMIT_SPEED        RIGHT_WHEEL_LIMIT_SPEED_RPM
#define MOTOR_LIMIT_CURRENT      RIGHT_WHEEL_LIMIT_CURRENT_A

/*
 * --------------------------------------------------------
 * 旧云台兼容参数
 * --------------------------------------------------------
 * 当前主任务是平衡车底盘，这一段不作为底盘控制配置。
 * 现有工程仍有旧 gimbal_task 引用这些宏，先保留以避免编译断裂。
 */
#define MOTOR_PITCH_MECH_ZERO    0.0f     // Pitch 轴机械零点 (单位: 度)
#define MOTOR_YAW_MECH_ZERO      0.0f     // Yaw 轴机械零点 (单位: 度)

/*
 * --------------------------------------------------------
 * 云台外环 (1000Hz) PID 增稳控制参数 (极度保守)
 * --------------------------------------------------------
 */
// 极性宏：极其重要！如果发生正反馈震荡（一碰就加速甩头），请将其改为 -1.0f
#define PITCH_POLARITY 1.0f
#define YAW_POLARITY   -1.0f

// 针对云台惯量调整：加大 YAW 力度，减小 PITCH 力度
// 注意：一旦方向正确，系统仍然会画大圆发散的原因是缺少阻尼（D环）！
// 现在重新引入并增强 D 环，将其稳死。
#define GIMBAL_PID_PITCH_KP 0.025f
#define GIMBAL_PID_PITCH_KI 0.0001f
#define GIMBAL_PID_PITCH_KD 0.0020f
#define GIMBAL_PID_YAW_KP   0.050f
#define GIMBAL_PID_YAW_KI   0.0002f
#define GIMBAL_PID_YAW_KD   0.0010f

// 编码器硬锁模式 PID (只用于上电初期的极静校准)
#define ENC_LOCK_KP 0.080f
#define ENC_LOCK_KD 0.002f

// 控制死区 (度)，在此误差范围内视为已到达目标，不进行 P 环修正，消除微抖动
#define GIMBAL_PITCH_DEADBAND 0.5f
#define GIMBAL_YAW_DEADBAND   0.2f

// === 核心保护：误差限幅 (极度重要，防画龙防疯转) ===
// 无论外面怎么扰动，系统认为的“最大误差”不超过此值。防止放手瞬间积累极大动能。
#define GIMBAL_MAX_ERR_PITCH  20.0f
#define GIMBAL_MAX_ERR_YAW    20.0f

// 积分项限幅 (防积分饱和)
#define GIMBAL_MAX_I_PITCH    0.3f  // 积分最大只允许提供 0.3A 电流，只用于消除微小稳态误差
#define GIMBAL_MAX_I_YAW      0.3f

// 安全限幅保护 (最大输出指令电流 A，用手能轻松捏住，防烧毁防击穿)
#define GIMBAL_MAX_OUTPUT_CURRENT 1.0f

#endif /* __CONFIG_H__ */
