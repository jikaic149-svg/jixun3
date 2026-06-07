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

/*
 * QD4310 无刷电机规格
 */
#define QD4310_RATED_VOLTAGE_MIN_V       7.0f
#define QD4310_RATED_VOLTAGE_MAX_V       26.0f
#define QD4310_RATED_CURRENT_A           1.0f
#define QD4310_PEAK_CURRENT_A            2.0f
#define QD4310_RATED_TORQUE_NM           0.2f
#define QD4310_PEAK_TORQUE_NM            0.3f
#define QD4310_RATED_SPEED_RPM           500.0f
#define QD4310_PEAK_SPEED_RPM            800.0f
#define QD4310_POLE_PAIRS                14U
#define QD4310_ENCODER_BITS              15U
#define QD4310_CONTROL_ACCURACY_DEG      0.0109f
#define QD4310_WEIGHT_KG                 0.127f

#define QD4310_MODE_CURRENT              1U
#define QD4310_MODE_SPEED                1U
#define QD4310_MODE_ANGLE                1U
#define QD4310_MODE_LOW_SPEED_TORQUE     1U
#define QD4310_MODE_ANGLE_STEP           1U

#define QD4310_COMM_CAN                  1U
#define QD4310_COMM_USB                  1U
#define QD4310_COMM_UART                 1U
#define QD4310_CAN_BAUD_RATE             1000000U
#define QD4310_UART_BAUD_RATE_DEFAULT    115200U

/* 旧通用电机规格宏，保留给已有代码兼容。 */
#define MOTOR_RATED_VOLTAGE_MIN          QD4310_RATED_VOLTAGE_MIN_V
#define MOTOR_RATED_VOLTAGE_MAX          QD4310_RATED_VOLTAGE_MAX_V
#define MOTOR_RATED_CURRENT              QD4310_RATED_CURRENT_A
#define MOTOR_PEAK_CURRENT               QD4310_PEAK_CURRENT_A
#define MOTOR_RATED_TORQUE               QD4310_RATED_TORQUE_NM
#define MOTOR_PEAK_TORQUE                QD4310_PEAK_TORQUE_NM
#define MOTOR_RATED_SPEED                QD4310_RATED_SPEED_RPM
#define MOTOR_PEAK_SPEED                 QD4310_PEAK_SPEED_RPM
#define MOTOR_POLE_PAIRS                 QD4310_POLE_PAIRS
#define MOTOR_ENCODER_BITS               QD4310_ENCODER_BITS
#define MOTOR_CONTROL_ACCURACY           QD4310_CONTROL_ACCURACY_DEG
#define MOTOR_WEIGHT_KG                  QD4310_WEIGHT_KG

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

/* 轮端方向归一化：正方向定义为车体前进。右轮实测极性相反。 */
#define LEFT_WHEEL_FEEDBACK_SIGN      (-1.0f)
#define RIGHT_WHEEL_FEEDBACK_SIGN     1.0f
#define LEFT_WHEEL_CURRENT_SIGN       (-1.0f)
#define RIGHT_WHEEL_CURRENT_SIGN      1.0f

/* 无刷启动流程：CAN启动后先enable左右轮，任务内发送0A保活并检测反馈。 */
#define WHEEL_FEEDBACK_TIMEOUT_MS     100U
#define WHEEL_ZERO_CURRENT_KEEPALIVE  1
#define WHEEL_SAFE_TASK_PERIOD_MS     1U

/* 离地左轮 ESO+PI 轮速测试：上电默认急停，按键授权后才输出非零电流。 */
#define WHEEL_SPEED_TEST_ENABLE             1U
#define WHEEL_SPEED_TEST_REF_RPM            100.0f
#define WHEEL_SPEED_TEST_REF_MIN_ABS_RPM    20.0f
#define WHEEL_SPEED_TEST_REF_MAX_ABS_RPM    500.0f
#define WHEEL_SPEED_TEST_CURRENT_LIMIT_A    0.60f
#define WHEEL_SPEED_TEST_BRAKE_CURRENT_LIMIT_A 0.010f
#define WHEEL_SPEED_TEST_DT_SEC             0.001f
#define WHEEL_ENCODER_SPEED_WINDOW_MS       20U
#define WHEEL_ENCODER_SPEED_LPF_ALPHA       0.40f
#define WHEEL_SPEED_TEST_UNIDIRECTIONAL     0U

/* jixun2风格轮速PID：u = (Kp*err + Ki*integral - Kd*speed) * polarity。 */
#define WHEEL_SPEED_PID_KP_A_PER_RPM        0.00150f
#define WHEEL_SPEED_PID_KI_A_PER_RPM_S      0.001000f
#define WHEEL_SPEED_PID_KD_A_PER_RPM        0.000f
#define WHEEL_SPEED_PID_POLARITY            1.0f
#define LEFT_WHEEL_SPEED_PID_POLARITY       WHEEL_SPEED_PID_POLARITY
#define RIGHT_WHEEL_SPEED_PID_POLARITY      WHEEL_SPEED_PID_POLARITY
#define WHEEL_SPEED_PID_INTEGRAL_LIMIT      120.0f
#define WHEEL_SPEED_FF_STATIC_A             0.010f
#define WHEEL_SPEED_FF_KV_A_PER_RPM         0.00020f
#define LEFT_WHEEL_SPEED_FEEDBACK_LPF_ALPHA 0.25f
#define RIGHT_WHEEL_SPEED_FEEDBACK_LPF_ALPHA 0.10f
#define WHEEL_SPEED_TARGET_RAMP_RPM_PER_SEC 3000.0f
#define WHEEL_SPEED_CURRENT_SLEW_A_PER_SEC  8.0f

/* 三个主调参数：Kp/Ki 产生期望轮速加速度，wo 是 ESO 带宽。 */
#define WHEEL_SPEED_ESO_PI_KP               0.5f
#define WHEEL_SPEED_ESO_PI_KI               0.0f
#define WHEEL_SPEED_ESO_WO                  20.0f

/* 固定估计参数。 */
#define WHEEL_SPEED_ESO_B0_RPM_S_PER_A      800.0f
#define WHEEL_SPEED_ESO_Z2_LIMIT_RPM_S      60.0f
#define WHEEL_SPEED_ESO_KAW                 0.0f

#define KEY_DEBOUNCE_MS                     30U

/* First balance stage: fixed-gain LQR only, no speed/line/yaw outer loop yet. */
#define BALANCE_LQR_ENABLE                  1U
#define BALANCE_LQR_OUTPUT_MODE_CURRENT     0U
#define BALANCE_LQR_OUTPUT_MODE_SPEED       1U
#define BALANCE_LQR_OUTPUT_MODE             BALANCE_LQR_OUTPUT_MODE_CURRENT
#define CHASSIS_FAULT_NONE                  0U
#define CHASSIS_FAULT_FEEDBACK              1U
#define CHASSIS_FAULT_ENABLE                2U
#define CHASSIS_FAULT_LQR_ANGLE             3U
#define CHASSIS_FAULT_LQR_RATE              4U
#define BALANCE_LQR_CURRENT_LIMIT_A         0.60f
#define BALANCE_LQR_CURRENT_SLEW_A_PER_SEC  50.0f
#define BALANCE_LQR_SAFE_ANGLE_DEG          45.0f
#define BALANCE_LQR_SAFE_RATE_DPS           900.0f
#define BALANCE_LQR_POSITION_LIMIT_M        0.30f
#define BALANCE_LQR_VELOCITY_LPF_ALPHA      0.15f
#define BALANCE_LQR_USE_WHEEL_STATE         1U
#define BALANCE_LQR_VELOCITY_LIMIT_MPS      1.5f
#define BALANCE_LQR_SPEED_SOURCE_CAN        0U
#define BALANCE_LQR_SPEED_SOURCE_ENCODER    1U
#define BALANCE_LQR_SPEED_SOURCE            BALANCE_LQR_SPEED_SOURCE_CAN
#define BALANCE_LQR_SPEED_REF_LIMIT_RPM     300.0f
#define BALANCE_LQR_FORCE_TO_SPEED_RPM_PER_N 45.0f
#define BALANCE_LQR_K_X_SCALE               0.30f
#define BALANCE_LQR_K_V_SCALE               0.30f
#define BALANCE_LQR_K_THETA_SCALE           0.35f
#define BALANCE_LQR_K_THETA_DOT_SCALE       0.05f
#define BALANCE_LQR_THETA_REF_DEG           0.0f
#define BALANCE_LQR_POSITION_SIGN           1.0f
#define BALANCE_LQR_VELOCITY_SIGN           1.0f
#define BALANCE_LQR_DT_SEC                  (0.001f * (float)WHEEL_SAFE_TASK_PERIOD_MS)
#define BALANCE_DEG_TO_RAD                  0.017453292519943295f
#define BALANCE_RPM_TO_MPS                  \
    (BALANCE_WHEEL_RADIUS_M * 6.283185307179586f / 60.0f)

/*
 * ========================================================
 * 平衡车 LQR 物理模型与初始增益
 * ========================================================
 *
 * 状态定义：
 * X = [x, v, theta, theta_dot]^T
 *
 * x         车体前后位移，m
 * v         车体前后速度，m/s
 * theta     车体俯仰角，直立为0，rad
 * theta_dot 俯仰角速度，rad/s
 *
 * 控制输入：
 * F = 左右轮共同产生的等效水平驱动力，N
 */

#define BALANCE_LQR_STATE_COUNT              4U
#define BALANCE_LQR_STATE_X                  0U
#define BALANCE_LQR_STATE_V                  1U
#define BALANCE_LQR_STATE_THETA              2U
#define BALANCE_LQR_STATE_THETA_DOT          3U

#define BALANCE_TOTAL_MASS_KG                0.606f
#define BALANCE_WHEEL_MOTOR_SINGLE_MASS_KG   0.1535f
#define BALANCE_WHEEL_MOTOR_PAIR_MASS_KG     0.307f
#define BALANCE_BASE_MASS_KG                 0.307f
#define BALANCE_BODY_MASS_KG                 0.299f

#define BALANCE_BATTERY_MASS_KG              0.1789f
#define BALANCE_C_BOARD_MASS_KG              0.038f
#define BALANCE_PLA_FRAME_MASS_KG            0.083f

#define BALANCE_WHEEL_RADIUS_M               0.031f
#define BALANCE_HALF_TRACK_M                 0.083f
#define BALANCE_TRACK_M                      0.166f
#define BALANCE_BODY_COM_LENGTH_M            0.03522f
#define BALANCE_BODY_PITCH_INERTIA_KGM2      0.00013f

/* 电池95mm长边沿左右方向，即沿pitch轴；重心约在零件1中心下方15mm。 */
#define BALANCE_BATTERY_LENGTH_M             0.095f
#define BALANCE_BATTERY_WIDTH_M              0.034f
#define BALANCE_BATTERY_HEIGHT_M             0.027f
#define BALANCE_BATTERY_COM_Z_OFFSET_M       (-0.015f)

/* LQR权重：Q = diag([0.5, 2.0, 500.0, 30.0]), R = 0.20 */
#define BALANCE_LQR_Q_X                      0.5f
#define BALANCE_LQR_Q_V                      2.0f
#define BALANCE_LQR_Q_THETA                  500.0f
#define BALANCE_LQR_Q_THETA_DOT              30.0f
#define BALANCE_LQR_R_FORCE                  0.20f

/*
 * 离散LQR增益：
 * K = [-1.1349, -3.9377, -50.6923, -9.0807]
 * F = -K * X
 */
#define BALANCE_LQR_K_X                      (-1.1349f)
#define BALANCE_LQR_K_V                      (-3.9377f)
#define BALANCE_LQR_K_THETA                  (-50.6923f)
#define BALANCE_LQR_K_THETA_DOT              (-9.0807f)

/* 展开后的等效正系数，仅用于人工核对方向。 */
#define BALANCE_LQR_F_X_COEF                 1.1349f
#define BALANCE_LQR_F_V_COEF                 3.9377f
#define BALANCE_LQR_F_THETA_COEF             50.6923f
#define BALANCE_LQR_F_THETA_DOT_COEF         9.0807f

/*
 * 方向验证：
 * 车前倾时轮子必须向前救车。
 * 如果方向反，只改整体符号，不要先改K。
 */
#define BALANCE_LQR_FORCE_SIGN               (-1.0f)

/*
 * 电流换算：
 * tau_total = F * r
 * tau_each  = tau_total / 2
 * I_each    = F * r / (2 * Kt)
 *
 * 暂用 Kt = 0.05 N*m/A：
 * I_each = 0.31 * F
 */
#define BALANCE_MOTOR_KT_NM_PER_A            0.05f
#define BALANCE_FORCE_TO_EACH_CURRENT_A_PER_N \
    (BALANCE_WHEEL_RADIUS_M / (2.0f * BALANCE_MOTOR_KT_NM_PER_A))
#define BALANCE_LQR_FORCE_LIMIT_N \
    (BALANCE_LQR_CURRENT_LIMIT_A / BALANCE_FORCE_TO_EACH_CURRENT_A_PER_N)

/* 转向差动：I_L = I_each - I_yaw, I_R = I_each + I_yaw */
#define BALANCE_LQR_LEFT_YAW_SIGN            (-1.0f)
#define BALANCE_LQR_RIGHT_YAW_SIGN           1.0f

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

/* VOFA CSV output mode. 控制/采样高频跑，VOFA只做降采样可视化。 */
#define VOFA_OUTPUT_MODE_ATTITUDE    0
#define VOFA_OUTPUT_MODE_ENCODER     1
#define VOFA_OUTPUT_MODE_CAN_DIAG    2
#define VOFA_OUTPUT_MODE             VOFA_OUTPUT_MODE_ENCODER

/* ENCODER模式通道顺序：
 * I0 左轮目标rpm
 * I1 左轮CAN反馈速度rpm
 * I2 左轮命令电流A
 * I3 右轮目标rpm
 * I4 右轮CAN反馈速度rpm
 * I5 右轮命令电流A
 *
 * CAN_DIAG模式通道顺序：
 * I0 左轮enabled
 * I1 右轮enabled
 * I2 左轮反馈计数
 * I3 右轮反馈计数
 * I4 最后CAN StdId
 * I5 CAN接收总计数
 */
#define VOFA_OUTPUT_ENABLE           1
#define VOFA_OUTPUT_DIVIDER          5U
#define VOFA_OUTPUT_PERIOD_MS        20U
#if VOFA_OUTPUT_MODE == VOFA_OUTPUT_MODE_ENCODER
#if BALANCE_LQR_ENABLE
#define VOFA_OUTPUT_CHANNEL_COUNT    8U
#else
#define VOFA_OUTPUT_CHANNEL_COUNT    6U
#endif
#elif VOFA_OUTPUT_MODE == VOFA_OUTPUT_MODE_CAN_DIAG
#define VOFA_OUTPUT_CHANNEL_COUNT    6U
#else
#define VOFA_OUTPUT_CHANNEL_COUNT    2U
#endif
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
