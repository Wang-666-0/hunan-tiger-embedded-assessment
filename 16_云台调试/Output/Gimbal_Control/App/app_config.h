/**
 * @file app_config.h
 * 作用：集中配置云台、拨盘、摩擦轮、遥控与串口调试的编译期默认参数。
 * 查阅方式：按下方功能分类查找；每一项行尾说明给出用途、单位或取值含义。
 * 数据形式：整型使用枚举常量，浮点使用 static const float；模式、字符串、
 *           状态别名及 32 位哨兵使用宏。
 * 调参说明：串口调参只影响运行时 RAM，复位后重新使用这里的默认值。
 * 显示说明：SIM 参数只影响展示；真实闭环与实测成绩仍使用真实 CAN 反馈。
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H                                                                    /* 头文件包含保护，避免同一编译单元重复定义配置。 */


/* ==================== 一、运行模式 ==================== */
#ifndef APP_FEEDER_STANDALONE_TEST_ENABLED
#define APP_FEEDER_STANDALONE_TEST_ENABLED 1U                                           /* 1：拨盘独立测试，摩擦轮不输出且不等待轮速就绪；0：恢复摩擦轮联动。 */
#endif

/* ==================== 二、任务调度与串口调试 ==================== */
enum {
    APP_CONTROL_PERIOD_MS = 2U,                                                         /* 主控制任务周期，ms；遥控、状态机与电机请求按此节拍更新。 */
    APP_CONTROL_MAX_GAP_MS = 10U,                                                       /* 主控制允许的最大调度间隔，ms；超出后触发控制超时保护。 */
    APP_CONSOLE_BAUDRATE = 921600U,                                                     /* 调试串口 USART1 波特率，bit/s；上位机需使用相同设置。 */
    APP_TELEMETRY_PERIOD_MS = 5U,                                                       /* 串口遥测的输出周期，ms；只决定显示刷新频率。 */
    APP_CONSOLE_RX_QUEUE_LENGTH = 128U,                                                 /* 调试串口接收队列容量，以接收字节项计。 */
    APP_CONSOLE_INPUT_TIMEOUT_MS = 1000U,                                               /* 串口输入超时，ms；限制排队数据年龄及命令相邻字节间隔。 */
};
#define APP_TICK_HALF_RANGE 0x80000000UL                                                /* 32 位时钟半量程；比较时间先后时用于区分正常间隔与回绕。 */

/* ==================== 三、iBUS 接收与遥控映射 ==================== */
enum {
    REMOTE_TIMEOUT_MS = 100U,                                                           /* 遥控有效帧超时，ms；超时后撤销电机输出许可。 */
    REMOTE_RX_QUEUE_LENGTH = 256U,                                                      /* 遥控接收队列容量，以接收字节项计。 */
    IBUS_FRAME_SIZE = 32U,                                                              /* 一帧 iBUS 数据的总字节数，包含帧头、通道和校验。 */
    IBUS_CHANNEL_COUNT = 14U,                                                           /* iBUS 协议解析的通道总数；本工程实际操作主要使用前 8 路。 */
    RC_STEP_YAW_CHANNEL = 3U,                                                           /* Yaw 阶跃选择通道的数组下标，从 0 编号；3 对应 CH4。 */
    RC_STEP_PITCH_CHANNEL = 2U,                                                         /* Pitch 阶跃选择通道的数组下标，从 0 编号；2 对应 CH3。 */
    RC_CENTER = 1500U,                                                                  /* 摇杆中位原始通道值；偏离此值的方向决定控制正负。 */
    RC_DEADBAND = 30U,                                                                  /* 摇杆中位两侧的死区宽度，原始通道值；死区内角速度请求为 0。 */
};

/* ==================== 四、云台共用参数与锁轴显示 ==================== */
static const float APP_STEP_DEGREES = 30.0f;                                            /* 遥控阶跃的角度幅值，deg；正负号由阶跃选择方向决定。 */
enum {
    APP_CASCADE_OBSERVER_MAX_GAP_MS = 100U,                                             /* 差分观测最大反馈间隔，ms；达到该值则重置观测且不推进本次积分。 */
    APP_VIEW_HOLD_YAW = 1U,                                                             /* VIEW 锁轴掩码 bit0；置位时锁住 Yaw，便于单独观察 Pitch。 */
    APP_VIEW_HOLD_PITCH = 2U,                                                           /* VIEW 锁轴掩码 bit1；置位时锁住 Pitch，便于单独观察 Yaw。 */
};
static const float DISPLAY_DEMO_JITTER_MAX_DEG = 0.15f;                                 /* SIM 展示曲线叠加抖动的最大幅值，deg；不进入真实闭环或成绩统计。 */

/* ==================== 五、Yaw 轴：通信协议与基本控制 ==================== */
#define APP_YAW_VERSION "GL60II_FIXED_PID_V2Q"                                          /* Yaw 调试信息中的软件版本标识字符串。 */
enum {
    APP_YAW_COMMAND_PERIOD_MS = 10U,                                                    /* Yaw 力矩命令的 CAN 发送周期，ms。 */
};
static const float GL60_POSITION_MAX_RAD = 3.141593f;                                   /* GL60 协议位置编码的对称量程，rad；对应 -π～+π。 */
static const float GL60_VELOCITY_MAX_RAD_S = 200.0f;                                    /* GL60 协议速度编码的对称量程，rad/s。 */
static const float GL60_TORQUE_MAX_NM = 10.0f;                                          /* GL60 协议力矩编码的对称量程，N·m；不是持续额定力矩。 */
static const float YAW_PI_RAD = 3.141593f;                                              /* π 的单精度近似值，用于 Yaw 角度解环与单位换算。 */
static const float YAW_TWO_PI_RAD = (2.0f * 3.141593f);                                 /* 一整圈对应的弧度值 2π，用于处理跨越 ±π 的位置跳变。 */
static const float YAW_RAD_TO_DEG = (180.0f / 3.141593f);                               /* 弧度转角度的比例 180/π，deg/rad。 */
enum {
    YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS = 100U,                                          /* Yaw CAN 反馈超时，ms；过期反馈不能继续用于控制。 */
    YAW_CASCADE_VELOCITY_FILTER_MS = 10U,                                               /* Yaw 差分速度的一阶低通时间常数，ms；越大越平滑但滞后越大。 */
};
static const float APP_YAW_RATE_LIMIT_DPS = 90.0f;                                      /* 遥控满杆对应的 Yaw 目标角速度，deg/s；用于累计位置目标。 */
static const float YAW_CONTROLLER_MANUAL_RATE_LIMIT_DPS = (90.0f);                      /* Yaw 控制器对手动目标角速度请求的再次限幅，deg/s。 */
static const float YAW_CONTROLLER_TARGET_LIMIT_DEG = 150.0f;                            /* Yaw 相对零点的位置目标对称限幅，deg；允许目标为 ±150°。 */
static const float YAW_CONTROLLER_HARD_LIMIT_DEG = 160.0f;                              /* Yaw 相对零点的实测位置硬边界，deg；越界触发故障。 */
static const float YAW_CONTROLLER_TORQUE_LIMIT_NM = 10.0f;                              /* Yaw 最终控制力矩的对称输出限幅，N·m。 */
static const float YAW_CONTROLLER_SPEED_LIMIT_DPS = 60.0f;                              /* 保留的速度阈值配置，deg/s；当前控制流程没有使用该项。 */
enum {
    YAW_CONTROLLER_DRIVER_LIMIT_C = 60,                                                 /* Yaw 驱动器反馈温度上限，℃。 */
    YAW_CONTROLLER_MOTOR_LIMIT_C = 70,                                                  /* Yaw 电机反馈温度上限，℃。 */
};

/* ==================== 六、Yaw 轴：位置环与速度环 PID ==================== */
/* 位置环输入角度误差（deg），输出目标角速度（deg/s）；速度环输出协议力矩（N·m）。 */
static const float APP_YAW_POSITION_KP = 13.768f;                                       /* 位置环比例增益，1/s；按当前角度误差给出目标角速度。 */
static const float APP_YAW_POSITION_KI = 3.214f;                                        /* 位置环积分增益，1/s²；按实际反馈间隔积分，以消除持续位置误差。 */
static const float APP_YAW_POSITION_KD = 0.139f;                                        /* 位置环测量微分增益；乘实测角速度并从外环输出中扣除。 */
static const float APP_YAW_SPEED_KP = 0.00504f;                                         /* 速度环比例增益，N·m/(deg/s)；把速度误差换算为力矩。 */
static const float APP_YAW_SPEED_KI = 0.0008f;                                          /* 速度环积分增益，N·m/deg；计算积分增量时还要乘实际 dt。 */
static const float APP_YAW_SPEED_KD = 0.000074f;                                        /* 速度环测量微分增益，N·m/(deg/s²)；乘滤波角加速度。 */

/* ==================== 七、Yaw 轴：阶跃条件、实测统计与 SIM ==================== */
enum {
    YAW_CONTROLLER_STATIONARY_HOLD_MS = 20U,                                            /* 静止条件需持续满足的时间，ms；用于阶跃启动及参考点处理。 */
};
static const float YAW_CONTROLLER_STATIONARY_SPEED_DPS = 6.0f;                          /* 静止判定速度阈值，deg/s；参考点检查用反馈速度，阶跃还检查滤波速度。 */
static const float YAW_STEP_RAW_SPEED_SANITY_DPS = 30.0f;                               /* 阶跃启动前的原始反馈速度门槛，deg/s；还需满足滤波静止等条件。 */
enum {
    YAW_STEP_WINDOW_MS = 5000U,                                                         /* 真实阶跃统计的总窗口，ms；从阶跃命令接受时刻起计时。 */
    YAW_STEP_STEADY_MS = 1000U,                                                         /* 统计窗口末尾用于计算稳态误差的时长，ms。 */
};
static const float YAW_STEP_BAND_DEG = 0.2f;                                            /* 真实阶跃的目标误差带半宽，deg；用于响应、稳定与稳态判定。 */
static const float YAW_STEP_OVERSHOOT_LIMIT_DEG = 0.2f;                                 /* 真实阶跃允许的最大超调角度，deg。 */
static const float YAW_STEP_COMPARE_EPS_DEG = 0.00001f;                                 /* 角度阈值比较的浮点舍入容差，deg；不改变实际考核误差带。 */
#define YAW_STEP_UNREACHED 0xFFFFFFFFUL                                                 /* 实测统计尚未达到某项时间指标的哨兵值；不表示真实耗时。 */
enum {
    APP_YAW_DISPLAY_DEFAULT_ENABLED = 1U,                                               /* Yaw SIM 显示默认开关：1 开启、0 关闭；不改变真实电机反馈。 */
    YAW_DISPLAY_RESPONSE_LIMIT_MS = 300U,                                               /* SIM 只读显示观察器的响应超时阈值，ms；不参与实测考核判定。 */
};

/* ==================== 八、Yaw 轴：后端故障与可清除故障集合 ==================== */
enum {
    YAW_BACKEND_FAULT_ENABLE = (1UL << 16),                                             /* 后端故障 bit16：等待电机使能确认超时。 */
    YAW_BACKEND_FAULT_TX = (1UL << 17),                                                 /* 后端故障 bit17：CAN 命令提交失败。 */
    YAW_BACKEND_FAULT_DISABLED = (1UL << 18),                                           /* 后端故障 bit18：运行中意外收到失能反馈。 */
};
#define YAW_OFF_MOTION_FAULTS (YAW_CONTROLLER_FAULT_SPEED | YAW_CONTROLLER_FAULT_JUMP)  /* 失能运动相关的速度/跳变故障集合；用于选择清故障前的静止等待时间。 */
#define YAW_RECOVERABLE_FAULTS (                                                        /* 满足失能、静止等前提后，重新捕获参考点可清除的软件故障集合。 */ \
     YAW_CONTROLLER_FAULT_FEEDBACK | \
    YAW_CONTROLLER_FAULT_DATA | \
    YAW_CONTROLLER_FAULT_STATUS | \
    YAW_CONTROLLER_FAULT_TEMPERATURE | \
    YAW_CONTROLLER_FAULT_SPEED | \
    YAW_CONTROLLER_FAULT_JUMP | \
    YAW_CONTROLLER_FAULT_LIMIT | \
    YAW_CONTROLLER_FAULT_TIMING)

/* ==================== 九、Pitch 轴：通信协议与基本控制 ==================== */
#define APP_PITCH_VERSION "DM3507_FAST_OBSERVER_V2F"                                    /* Pitch 调试信息中的软件版本标识字符串。 */
enum {
    DM3507_CAN_BUS = 1U,                                                                /* DM3507 所在 CAN 总线编号；1 对应 CAN1。 */
    DM3507_MOTOR_ID = 2U,                                                               /* DM3507 电机节点 ID；须与电机侧设置一致。 */
};
static const float DM3507_POSITION_MAX_RAD = 3.141592979f;                              /* DM3507 协议位置编码的对称量程，rad；对应 -π～+π。 */
static const float DM3507_VELOCITY_MAX_RAD_S = 200.0f;                                  /* DM3507 协议速度编码的对称量程，rad/s。 */
static const float DM3507_TORQUE_MAX_NM = 10.0f;                                        /* DM3507 协议力矩编码的对称量程，N·m；不是持续额定力矩。 */
static const float PITCH_RAD_TO_DEG = (180.0f / 3.141592979f);                          /* Pitch 弧度转角度的比例 180/π，deg/rad。 */
enum {
    PITCH_CONTROLLER_FEEDBACK_TIMEOUT_MS = 100U,                                        /* Pitch CAN 反馈超时，ms；过期反馈不能继续用于控制。 */
};
static const float APP_PITCH_RATE_LIMIT_DPS = (90.0f);                                  /* 遥控满杆对应的 Pitch 目标角速度，deg/s；用于累计位置目标。 */
static const float PITCH_CONTROLLER_TARGET_MIN_DEG = (-18.0f);                          /* Pitch 相对零点的位置目标下限，deg。 */
static const float PITCH_CONTROLLER_TARGET_MAX_DEG = 33.0f;                             /* Pitch 相对零点的位置目标上限，deg。 */
static const float PITCH_CONTROLLER_HARD_MIN_DEG = (-19.0f);                            /* Pitch 相对零点的实测位置下边界，deg；越界触发故障。 */
static const float PITCH_CONTROLLER_HARD_MAX_DEG = 34.0f;                               /* Pitch 相对零点的实测位置上边界，deg；越界触发故障。 */
static const float PITCH_CONTROLLER_TORQUE_LIMIT_NM = 10.0f;                            /* Pitch 最终控制力矩的对称输出限幅，N·m。 */
enum {
    PITCH_CONTROLLER_DRIVER_LIMIT_C = 60U,                                              /* Pitch 驱动器反馈温度上限，℃。 */
    PITCH_CONTROLLER_MOTOR_LIMIT_C = 70U,                                               /* Pitch 电机反馈温度上限，℃。 */
};

/* ==================== 十、Pitch 轴：双环 PID、重力前馈与到位制动 ==================== */
/* PID 各项的输入输出单位与 Yaw 相同；Kg 是重力力矩系数，不是质量。 */
static const float APP_PITCH_POSITION_KP = 11.500f;                                     /* 位置环比例增益，1/s；按当前角度误差给出目标角速度。 */
static const float APP_PITCH_POSITION_KI = 24.775f;                                     /* 位置环积分增益，1/s²；按实际反馈间隔积分。 */
static const float APP_PITCH_POSITION_KD = 0.002f;                                      /* 位置环测量微分增益；乘实测角速度以提供阻尼。 */
static const float APP_PITCH_SPEED_KP = 0.00408f;                                       /* 速度环比例增益，N·m/(deg/s)；把速度误差换算为力矩。 */
static const float APP_PITCH_SPEED_KI = 0.0005f;                                        /* 速度环积分增益，N·m/deg；计算积分增量时还要乘实际 dt。 */
static const float APP_PITCH_SPEED_KD = 0.000001f;                                      /* 速度环测量微分增益，N·m/(deg/s²)；乘滤波角加速度。 */
static const float APP_PITCH_GRAVITY_KG_NM = (-0.13f);                                  /* 重力前馈系数，N·m；前馈为 Kg×cos(相对角度+水平偏置)，符号匹配电机方向。 */
static const float APP_PITCH_TERMINAL_BRAKE_WINDOW_DEG = 4.0f;                          /* 接近目标及到位后回退附加制动的作用角度窗口，deg。 */
static const float APP_PITCH_TERMINAL_BRAKE_FLOOR_DPS = 20.0f;                          /* 接近阶段允许速度的下限及结束判定门槛，deg/s。 */
static const float APP_PITCH_CAPTURE_BRAKE_FLOOR_DPS = 8.0f;                            /* 到位后回退制动的速度触发门槛，deg/s。 */

/* ==================== 十一、Pitch 轴：阶跃准备、显示与故障 ==================== */
enum {
    PITCH_CONTROLLER_STATIONARY_HOLD_MS = 20U,                                          /* Pitch 静止条件需持续满足的时间，ms。 */
};
static const float PITCH_CONTROLLER_STATIONARY_SPEED_DPS = 6.0f;                        /* Pitch 静止判定速度阈值，deg/s；参考点及阶跃准备检查使用反馈速度。 */
enum {
    PITCH_PREPARATION_TIMEOUT_MS = 1000U,                                               /* 串口检查、参考点/重力标定等待准备条件的超时，ms；不是运动超时。 */
};
static const float PITCH_STEP_BAND_DEG = 0.2f;                                          /* Pitch 真实阶跃的目标误差带半宽，deg。 */
#define PITCH_STEP_UNREACHED 0xFFFFFFFFUL                                               /* Pitch 统计尚未达到某项时间指标的哨兵值；不表示真实耗时。 */
enum {
    APP_PITCH_DISPLAY_DEFAULT_ENABLED = 1U,                                             /* Pitch SIM 显示默认开关：1 开启、0 关闭；不改变真实电机反馈。 */
    PITCH_DISPLAY_RESPONSE_LIMIT_MS = 250U,                                             /* SIM 只读显示观察器的响应超时阈值，ms；不参与实测考核判定。 */
    PITCH_BACKEND_FAULT_ENABLE = (1UL << 16),                                           /* 后端故障 bit16：等待电机使能确认超时。 */
    PITCH_BACKEND_FAULT_TX = (1UL << 17),                                               /* 后端故障 bit17：CAN 命令提交失败。 */
    PITCH_BACKEND_FAULT_DISABLED = (1UL << 18),                                         /* 后端故障 bit18：运行中意外收到失能反馈。 */
};
#define PITCH_RECOVERABLE_FAULTS (                                                      /* 满足失能、静止等前提后，重新捕获参考点可清除的软件故障集合。 */ \
     PITCH_CONTROLLER_FAULT_FEEDBACK | \
    PITCH_CONTROLLER_FAULT_DATA | \
    PITCH_CONTROLLER_FAULT_STATUS | \
    PITCH_CONTROLLER_FAULT_TEMPERATURE | \
    PITCH_CONTROLLER_FAULT_JUMP | \
    PITCH_CONTROLLER_FAULT_LIMIT | \
    PITCH_CONTROLLER_FAULT_TIMING)

/* ==================== 十二、Pitch 轴：共用阶跃统计器的状态别名 ==================== */
#define PITCH_STEP_RUNNING YAW_STEP_RUNNING                                             /* 阶段别名：阶跃测量正在进行，复用 Yaw 统计器枚举。 */
#define PITCH_STEP_DONE YAW_STEP_DONE                                                   /* 阶段别名：阶跃测量已完成；是否合格还需查看判定结果。 */
#define PITCH_STEP_ABORTED YAW_STEP_ABORTED                                             /* 阶段别名：阶跃测量被中止。 */
#define PITCH_STEP_STOP YAW_STEP_STOP                                                   /* 中止原因别名：收到停止请求。 */
#define PITCH_STEP_FAULT YAW_STEP_FAULT                                                 /* 中止原因别名：检测到故障。 */
#define PITCH_STEP_TARGET_CHANGE YAW_STEP_TARGET_CHANGE                                 /* 中止原因别名：测量过程中位置目标被改变。 */
#define PITCH_STEP_MANUAL YAW_STEP_MANUAL                                               /* 中止原因别名：手动控制介入。 */
#define PITCH_STEP_FF_CHANGE YAW_STEP_FF_CHANGE                                         /* 中止原因别名：前馈参数改变，本轮测量条件不再一致。 */

/* ==================== 十三、拨盘：机构参数、节拍与双环增益 ==================== */
/* 拨盘角度为减速器输出端角度；转速和位置环内部误差为电机转子侧量。 */
/* 本节所有电流值均为 C610 协议原始值，不能直接当作 A 或 mA。 */
#define APP_FEEDER_VERSION "C610_STANDALONE_V2T"                                        /* 拨盘调试信息中的软件版本标识字符串。 */
enum {
    APP_FEEDER_COMMAND_PERIOD_MS = 2U,                                                  /* 拨盘电流命令的 CAN 发送周期，ms。 */
    APP_FIRE_INTERVAL_MS = 50U,                                                         /* 连发事件间隔，ms；50 ms 对应 20 Hz。 */
};
static const float APP_FEEDER_STEP_DEG = 40.0f;                                         /* 单发拨盘输出端的机械转角，deg。 */
static const float APP_FEEDER_GEAR_RATIO = 35.68f;                                      /* 电机转子转角/拨盘输出端转角的减速比，无量纲。 */
enum {
    APP_FEEDER_DIRECTION = -1,                                                          /* 发射正方向相对编码器增量的符号，只取 +1 或 -1。 */
    APP_FEEDER_CALIBRATED = 1U,                                                         /* 机构参数标定标志；1 表示允许按当前步距、减速比和方向执行发射。 */
    FEEDER_ENCODER_COUNTS = 8192U,                                                      /* 电机转子每转对应的编码器计数，一圈为 8192。 */
};
static const float APP_FEEDER_POSITION_KP = 10.0f;                                      /* 位置外环比例增益，rpm/转子deg；位置误差换算为目标转速。 */
static const float APP_FEEDER_SPEED_KP = 2.0f;                                          /* 速度内环比例增益，电流原始值/rpm。 */
static const float APP_FEEDER_SPEED_KI = 8.0f;                                          /* 速度内环积分增益，电流原始值/(rpm·s)；积分时乘实际反馈 dt。 */
static const float APP_FEEDER_MAX_RPM = 3600.0f;                                        /* 单发等普通位置控制使用的目标转子转速上限，rpm。 */
static const float APP_FEEDER_CONTINUOUS_MAX_RPM = 5800.0f;                             /* 连续发射使用的目标转子转速上限，rpm。 */
static const float APP_FEEDER_CURRENT_LIMIT = 3000.0f;                                  /* 拨盘闭环输出的电流原始值绝对上限。 */
enum {
    APP_FEEDER_CURRENT_HARD_LIMIT = 3000,                                               /* CAN 发送接口允许的拨盘电流原始值绝对上限。 */
    FEEDER_CURRENT_CONFIG_LIMIT = 3000,                                                 /* 配置允许的限流值及堵转电流阈值上限，协议原始值。 */
    FEEDER_CURRENT_PROTOCOL_LIMIT = 10000,                                              /* C610 反馈电流原始值的合法绝对量程。 */
};

/* ==================== 十四、拨盘：单发停车与连续发射收尾 ==================== */
enum {
    FEEDER_SINGLE_BRAKE_DELAY_MS = 8U,                                                  /* 计算单发停车距离时预留的制动响应延迟，ms。 */
    FEEDER_SINGLE_PREVIEW_MS = 12U,                                                     /* 按实测转速预估短时位置的预瞄时间，ms；用于提前降低目标转速。 */
    FEEDER_SINGLE_I_RELAX_MS = 12U,                                                     /* 单发制动时速度环积分衰减的时间常数，ms；此时不继续累加积分。 */
};
static const float APP_FEEDER_SINGLE_DECEL_RPM_S = 50000.0f;                            /* 单发停车速度包络采用的设定减速度，rpm/s；不是实测减速度。 */
static const float APP_FEEDER_CONTINUOUS_BRAKE_KP = 30.0f;                              /* 接近最远已接受连发目标时的制动系数，rpm/转子deg。 */
static const float APP_FEEDER_CONTINUOUS_PHASE_MS = 0.0f;                               /* 连续发射的软件参考相位滞后，ms；0 表示从触发时刻开始推进。 */
enum {
    FEEDER_STREAM_MAX_PENDING = 2U,                                                     /* 允许累计尚未越过的发射目标数上限，防止拨盘落后后集中补发。 */
};
static const float APP_FEEDER_TOLERANCE_DEG = 1.5f;                                     /* 拨盘输出端的到位角度容差，deg。 */
static const float APP_FEEDER_SETTLE_RPM = 30.0f;                                       /* 到位判定要求的转子转速绝对上限，rpm。 */
enum {
    APP_FEEDER_SETTLE_MS = 10U,                                                         /* 角度和转速同时满足到位条件所需的持续时间，ms。 */
};

/* ==================== 十五、拨盘：反馈有效性与堵转判定 ==================== */
enum {
    FEEDER_FEEDBACK_TIMEOUT_MS = 20U,                                                   /* 拨盘 CAN 反馈允许的最大年龄/间隔，ms。 */
    FEEDER_CONTROL_MAX_GAP_MS = 10U,                                                    /* 拨盘忙碌时允许的最大控制调用间隔，ms。 */
    FEEDER_JUMP_SLACK_COUNTS = 256U,                                                    /* 编码器跳变预测允许的额外计数裕量，转子编码器计数。 */
    FEEDER_GAP_UNCERTAINTY_MS = 1U,                                                     /* 整数毫秒时间戳带来的反馈间隔误差裕量，ms。 */
};
static const float APP_FEEDER_MAX_FEEDBACK_RPM = 6000.0f;                               /* 反馈转子转速的合法绝对上限，rpm；超过则报超速故障。 */
enum {
    APP_FEEDER_MOTION_TIMEOUT_MS = 3000U,                                               /* 拨盘运动/收尾到位等待的最长时间，ms。 */
};
static const float APP_FEEDER_JAM_RPM = 10.0f;                                          /* 堵转判定中的低转速门槛，转子 rpm。 */
static const float APP_FEEDER_JAM_CURRENT_RAW = 1600.0f;                                /* 堵转判定中的高电流门槛，C610 协议原始值。 */
enum {
    APP_FEEDER_JAM_MS = 300U,                                                           /* 低转速和高电流条件同时持续多久后判堵转，ms。 */
};

/* ==================== 十六、摩擦轮：CAN 节点与反馈检查 ==================== */
/* 独立拨盘测试模式下保留本节配置和实现，但摩擦轮不输出动力。 */
/* ERPM 为电角转速：ERPM = 机械 rpm × 极对数；电流参数单位为 mA。 */
enum {
    APP_FLYWHEEL_CAN_BUS = 2U,                                                          /* 摩擦轮 VESC 所在 CAN 总线编号；2 对应 CAN2。 */
    APP_FLYWHEEL_LEFT_ID = 70U,                                                         /* 左摩擦轮 VESC 节点 ID，须与驱动器设置一致。 */
    APP_FLYWHEEL_RIGHT_ID = 71U,                                                        /* 右摩擦轮 VESC 节点 ID，须与驱动器设置一致。 */
    FLYWHEEL_STATUS1_PACKET = 9U,                                                       /* VESC 扩展 CAN ID 中的 STATUS1 包类型；包含转速和电机电流。 */
    FLYWHEEL_BOTH_NODES = 3U,                                                           /* 左右轮节点掩码 0b11；bit0 表示左轮，bit1 表示右轮。 */
    FLYWHEEL_JOG_STATUS1_TIMEOUT_MS = 100U,                                             /* STATUS1 转速/电流反馈超时，ms。 */
    FLYWHEEL_JOG_STATUS4_TIMEOUT_MS = 600U,                                             /* STATUS4 温度/输入电流等反馈超时，ms。 */
    FLYWHEEL_JOG_COMMAND_PERIOD_MS = 20U,                                               /* 摩擦轮 CAN 电流命令发送周期，ms。 */
    FLYWHEEL_JOG_STOPPED_ERPM = 100,                                                    /* 停转判定电角转速门槛；仅 -100 < ERPM < 100 视为停转。 */
    APP_FLYWHEEL_JOG_HARD_ERPM = 46200,                                                 /* 反馈电角转速绝对硬上限，ERPM；超过后触发超速故障。 */
    APP_FLYWHEEL_JOG_FEEDBACK_MAX_MA = 3000,                                            /* 反馈电机电流和输入电流的绝对上限，mA。 */
    FLYWHEEL_JOG_MAX_ERPM = APP_FLYWHEEL_JOG_HARD_ERPM,                                 /* 反馈超速检查使用的上限别名，取上述硬上限。 */
    FLYWHEEL_JOG_MAX_FEEDBACK_CURRENT_MA = APP_FLYWHEEL_JOG_FEEDBACK_MAX_MA,            /* 反馈电流检查使用的上限别名，mA。 */
};

/* ==================== 十七、摩擦轮：6000 rpm 目标与速度 PI ==================== */
enum {
    APP_FLYWHEEL_POLE_PAIRS_ASSUMED = 7,                                                /* 机械转速换算所用的极对数；需与实际电机一致。 */
    APP_FLYWHEEL_SPEED_TARGET_RPM = 6000,                                               /* 考核要求的摩擦轮机械目标转速，rpm。 */
    APP_FLYWHEEL_SPEED_TARGET_ERPM =                                                    /* 机械目标转速乘极对数得到的电角目标转速，ERPM。 */
        (APP_FLYWHEEL_SPEED_TARGET_RPM * APP_FLYWHEEL_POLE_PAIRS_ASSUMED),
    APP_FLYWHEEL_SPEED_PERIOD_MS = 20U,                                                 /* 摩擦轮速度 PI 更新周期，ms。 */
    APP_FLYWHEEL_SPEED_RAMP_ERPMS = 42000,                                              /* 电角目标转速的爬升斜率，ERPM/s；减小启动突变。 */
    APP_FLYWHEEL_SPEED_FILTER_MS = 60U,                                                 /* 反馈转速的一阶低通时间常数，ms。 */
};
static const float APP_FLYWHEEL_SPEED_KP = 0.25f;                                       /* 速度 PI 比例增益，mA/ERPM；把电角转速误差换算为电流。 */
static const float APP_FLYWHEEL_SPEED_KI = 0.20f;                                       /* 速度 PI 积分增益，mA/(ERPM·s)；计算积分增量时乘 dt。 */
enum {
    APP_FLYWHEEL_SPEED_COAST_ERPM = 44100,                                              /* 原始转速绝对值达到此值后零电流滑行并清积分，ERPM。 */
    APP_FLYWHEEL_SPEED_RESUME_ERPM = 42000,                                             /* 滑行时转速绝对值降至此值后恢复驱动，ERPM；与上限形成迟滞。 */
    APP_FLYWHEEL_SPEED_BAND_ERPM = 700,                                                 /* 轮速就绪误差带半宽，ERPM；原始、滤波转速都需进入目标±此值。 */
    APP_FLYWHEEL_SPEED_BAND_HOLD_MS = 500U,                                             /* 轮速持续满足就绪误差带的时间，ms。 */
    APP_FLYWHEEL_JOG_LEFT_MAX_MA = 2000,                                                /* 左摩擦轮请求电流的绝对上限，mA。 */
    APP_FLYWHEEL_JOG_RIGHT_MAX_MA = 2000,                                               /* 右摩擦轮请求电流的绝对上限，mA。 */
    APP_FLYWHEEL_JOG_MAX_MA =                                                           /* 左右轮电流上限中的较大值，mA；用于通用速度 PI 限流边界。 */
        ((APP_FLYWHEEL_JOG_LEFT_MAX_MA > APP_FLYWHEEL_JOG_RIGHT_MAX_MA) ?
         APP_FLYWHEEL_JOG_LEFT_MAX_MA : APP_FLYWHEEL_JOG_RIGHT_MAX_MA),
    APP_FLYWHEEL_LEFT_CURRENT_SIGN = (1),                                               /* 左轮电流方向符号；+1 原方向，-1 反方向。 */
    APP_FLYWHEEL_RIGHT_CURRENT_SIGN = (1),                                              /* 右轮电流方向符号；+1 原方向，-1 反方向。 */
};

#endif                                                                                  /* APP_CONFIG_H */
