# FOLLOW 模式前馈优化方案

## 当前代码架构概要

### 数据流

```
遥控器/鼠标 ──→ robot_cmd (RemoteControlSet/MouseKeySet)
                   │
                   ├─→ gimbal_cmd → gimbal → yaw 电机转动
                   │                              │
                   │                              ├─→ IMU (Gyro[2] 作速度反馈)
                   │                              └─→ 电机编码器 (angle_single_round)
                   │
                   ├─→ gimbal_feed ← gimbal 回传
                   │       └─→ CalcOffsetAngle() → chassis_raw_offset_angle
                   │
                   └─→ chassis_cmd (含 offset_angle) → chassis FOLLOW 模式 → wz → 麦轮
```

### 关键约束

1. **IMU 在云台上**（ONE_BOARD 模式），云台用 IMU 稳定后，底盘转动时 `Gyro[2]` 理论上 ≈ 0
2. **底盘电机全部 OPEN_LOOP**，`LimitChassisOutput()` 自行计算 speed_PID → current_PID，不走 `DJIMotorControl()` 的串级 PID
3. **`Chassis_Ctrl_Cmd_s`** 是 robot_cmd → chassis 的唯一数据通道
4. robot_cmd 中 `add_yaw` 是 gimbal 目标角速度的天然来源

### 关键代码位置

| 文件 | 行号 | 作用 |
|------|------|------|
| [chassis.c:333-341](../application/chassis/chassis.c#L333-L341) | 追随模式 wz 计算 | 当前为死区 + 二次函数 |
| [robot_cmd.c:116-143](../application/cmd/robot_cmd.c#L116-L143) | `CalcOffsetAngle()` | 计算 yaw 电机相对零位的角度 |
| [robot_cmd.c:154-171](../application/cmd/robot_cmd.c#L154-L171) | `UpdateChassisFollowAlignment()` | 进入 FOLLOW 时归零 offset |
| [robot_cmd.c:248](../application/cmd/robot_cmd.c#L248) | `RemoteControlSet()` | `add_yaw` = 遥控器 yaw 增量 |
| [robot_cmd.c:302](../application/cmd/robot_cmd.c#L302) | `MouseKeySet()` | `add_yaw` = 鼠标 yaw 增量 |
| [robot_def.h:148-161](../application/robot_def.h#L148-L161) | `Chassis_Ctrl_Cmd_s` | robot_cmd → chassis 数据结构 |
| [robot_def.h:210-217](../application/robot_def.h#L210-L217) | `Gimbal_Upload_Data_s` | gimbal → robot_cmd 反馈数据 |
| [gimbal.c:153](../application/gimbal/gimbal.c#L153) | `GimbalTask()` | 发布 gimbal 反馈（含 IMU 数据） |
| [chassis.c:185-277](../application/chassis/chassis.c#L185-L277) | `LimitChassisOutput()` | 底盘功率限幅 + PID 计算 |
| [ins_task.h:28-37](../modules/imu/ins_task.h#L28-L37) | `attitude_t` | Gyro[3], Accel[3], Pitch, Roll, Yaw, YawTotalAngle |

---

## 最终控制结构

```
wz_target = wz_feedback + Kff * gimbal_yaw_rate
                │              │
                │              └── 前馈：云台一动底盘就跟
                └── 反馈：消除累积角度偏差
```

- **偏角反馈**：负责让底盘最终回到云台指向
- **角速度前馈**：负责在云台刚开始转动时底盘立即启动
- **限幅和斜坡**：防止突然猛转

---

## 一、数据结构改动

### 1.1 扩展 `Chassis_Ctrl_Cmd_s`

**文件**：[robot_def.h](../application/robot_def.h)

```c
typedef struct
{
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    float power_limit;  // 功率限制
    float gimbal_yaw_rate; // 新增：云台 yaw 角速度，供 chassis 前馈用 (°/s)
    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
} Chassis_Ctrl_Cmd_s;
```

### 1.2 调试数据结构

**文件**：[chassis.c](../application/chassis/chassis.c)（或单独的 debug 头文件）

```c
typedef struct
{
    float yaw_relative;       // 云台相对底盘角度 (°)
    float yaw_error;          // 跟随控制角度误差 (°)
    float gimbal_yaw_rate;    // 前馈角速度 (°/s)
    float wz_feedback;        // 偏角反馈输出 (°/s)
    float wz_feedforward;     // 前馈输出 (°/s)
    float wz_target;          // 最终底盘目标角速度 (°/s)
    float chassis_wz_actual;  // 底盘实际角速度 (°/s)
    float motor_current_max;  // 四轮最大电流
} FollowDebugData_t;
```

---

## 二、前馈信号来源

### 候选对比

| 候选 | 来源位置 | 优点 | 缺点 |
|------|----------|------|------|
| `add_yaw / dt` | `RemoteControlSet()` / `MouseKeySet()` | 不含底盘运动、噪声小、真正前馈 | 不反映云台 PID 实际响应 |
| `gimbal_imu_data.Gyro[2]` | `gimbal_fetch_data` | 反映云台真实运动 | 可能含底盘转动分量 |

### 建议

**首选 `add_yaw / dt`**。`robot_cmd` 中已计算好云台目标增量，除以周期（0.005s）即得目标角速度。此值天然不含底盘运动。

备选用 IMU Gyro[2] 作为对比通道，通过 VOFA+ 曲线对比两者的差异。

### 实现

在 `RemoteControlSet()` 和 `MouseKeySet()` 末尾（`VAL_LIMIT` 之后）填入：

```c
// add_yaw 是每帧增量（5ms），除以 dt 得到角速度 (°/s)
chassis_cmd_send.gimbal_yaw_rate = add_yaw / 0.005f;
```

---

## 三、VOFA+ 串口配置

### 3.1 硬件

| 项目 | 说明 |
|------|------|
| UART | 选一个空闲的（如 USART2），CubeMX 设为 Asynchronous |
| 波特率 | **921600**（最低 460800） |
| 接线 | USB-TTL RX → C 板 UART TX；GND → GND |
| 发送模式 | DMA（`HAL_UART_Transmit_DMA`），不阻塞控制任务 |

### 3.2 协议

使用 VOFA+ **JustFloat** 协议：每帧 = N 个 float（小端序）+ 4 字节尾帧 `{0x00, 0x00, 0x80, 0x7F}`。

### 3.3 发送函数

```c
#define VOFA_CHANNEL_COUNT 8

static const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7F};

static void VOFA_Send(float *data, uint8_t count)
{
    static uint8_t buf[VOFA_CHANNEL_COUNT * 4 + 4];
    memcpy(buf, data, count * 4);
    memcpy(buf + count * 4, vofa_tail, 4);
    HAL_UART_Transmit_DMA(&huart2, buf, count * 4 + 4);
}
```

> 注：`huart2` 替换为实际选用的 UART handle。

### 3.4 调用位置

在 `ChassisTask()` 中，跟随模式处理完成后：

```c
static uint8_t vofa_div = 0;
vofa_div++;

if (vofa_div >= 2)  // 200Hz → 100Hz 发送
{
    vofa_div = 0;
    static float vofa_data[VOFA_CHANNEL_COUNT];
    vofa_data[0] = chassis_cmd_recv.offset_angle;
    vofa_data[1] = yaw_error;
    vofa_data[2] = chassis_cmd_recv.gimbal_yaw_rate;
    vofa_data[3] = wz_feedback;
    vofa_data[4] = wz_feedforward;
    vofa_data[5] = wz_target;
    vofa_data[6] = chassis_wz_actual;
    vofa_data[7] = motor_current_max;
    VOFA_Send(vofa_data, VOFA_CHANNEL_COUNT);
}
```

---

## 四、反馈部分重写

### 4.1 平滑死区

```c
/**
 * @brief 平滑死区：deadband 内返回 0，之外减去边界
 *        避免 float_deadband 的硬切换导致输出跳变
 */
static float ApplySoftDeadband(float input, float deadband)
{
    if (input > deadband)
        return input - deadband;
    if (input < -deadband)
        return input + deadband;
    return 0.0f;
}
```

与现有 `float_deadband`（[user_lib.c:88-95](../modules/algorithm/user_lib.c#L88-L95)）的区别：

| 函数 | offset=1.6°(死区1.5°) | 效果 |
|------|----------------------|------|
| `float_deadband` | 返回 1.6 | 硬切换，刚过死区就有输出 |
| `ApplySoftDeadband` | 返回 0.1 | 平滑过渡，输出从 0 逐渐增加 |

### 4.2 分段反馈

```c
#define FOLLOW_YAW_DEADBAND_DEG  0.5f
#define FOLLOW_SMALL_END_DEG     4.0f
#define FOLLOW_KP_SMALL          15.0f
#define FOLLOW_KP_LARGE          7.0f
#define FOLLOW_WZ_MAX            120.0f

/**
 * @brief 分段线性跟随反馈
 *        小角度（≤4°）：较高线性增益，解决迟钝
 *        大角度（>4°）：降低增益增长率，避免猛冲
 */
static float FollowYawFeedback(float yaw_error_deg)
{
    float e = ApplySoftDeadband(yaw_error_deg, FOLLOW_YAW_DEADBAND_DEG);
    float abs_e = fabsf(e);
    float wz_abs;

    if (abs_e <= FOLLOW_SMALL_END_DEG)
        wz_abs = FOLLOW_KP_SMALL * abs_e;
    else
        wz_abs = FOLLOW_KP_SMALL * FOLLOW_SMALL_END_DEG
               + FOLLOW_KP_LARGE * (abs_e - FOLLOW_SMALL_END_DEG);

    if (wz_abs > FOLLOW_WZ_MAX)
        wz_abs = FOLLOW_WZ_MAX;

    return (e > 0.0f) ? -wz_abs : wz_abs;  // error>0 → wz 为负（底盘顺时针转）
}
```

对应输出曲线：

| 原始误差 | 有效误差 | 反馈 wz (°/s) |
|----------|----------|---------------|
| 0.3° | 0 | 0 |
| 0.8° | 0.3° | 4.5 |
| 1.0° | 0.5° | 7.5 |
| 1.5° | 1.0° | 15 |
| 2.5° | 2.0° | 30 |
| 4.5° | 4.0° | 60 |
| 10° | 9.5° | 98.5 |

> 对比原方案（`wz = 3×e²`）：10° 误差时 wz = 300°/s，远超当前 `WZ_MAX=120`。新方案更可控。

### 4.3 斜坡限幅

```c
#define FOLLOW_ACCEL_MAX  600.0f   // 最大角加速度 (°/s²)
#define FOLLOW_DT         0.005f   // 控制周期 (s)

/**
 * @brief 限制目标角速度的变化速率，避免突然猛转
 */
static float SlewLimit(float target, float prev, float max_accel, float dt)
{
    float delta = target - prev;
    float max_delta = max_accel * dt;  // 每周期允许的最大变化量

    if (delta > max_delta)
        return prev + max_delta;
    if (delta < -max_delta)
        return prev - max_delta;
    return target;
}
```

### 4.4 前馈通道

```c
#define FOLLOW_KFF            0.0f   // 前馈增益（第一阶段先关闭）
#define GYRO_RATE_DEADBAND    0.8f   // 角速度死区 (°/s)
#define GIMBAL_GYRO_LPF_ALPHA 0.25f  // 角速度低通系数（如果使用 IMU 数据）

static float gimbal_rate_filtered = 0.0f;

/**
 * @brief 角速度死区，消除静止噪声
 */
static float ApplyRateDeadband(float rate_deg_s)
{
    if (rate_deg_s > GYRO_RATE_DEADBAND)
        return rate_deg_s - GYRO_RATE_DEADBAND;
    if (rate_deg_s < -GYRO_RATE_DEADBAND)
        return rate_deg_s + GYRO_RATE_DEADBAND;
    return 0.0f;
}
```

### 4.5 主控制函数

```c
static float wz_target_last = 0.0f;

/**
 * @brief FOLLOW 模式云台跟随控制
 * @param offset_angle     云台相对底盘的偏角 (°)
 * @param gimbal_yaw_rate  云台 yaw 角速度 (°/s)
 * @param[out] dbg         调试数据
 * @return 底盘目标角速度 wz (°/s)
 */
static float ChassisFollowControl(float offset_angle, float gimbal_yaw_rate,
                                  FollowDebugData_t *dbg)
{
    // 1. 偏角反馈
    float wz_fb = FollowYawFeedback(offset_angle);

    // 2. 前馈（第一阶段 KFF=0）
    float rate_ff = ApplyRateDeadband(gimbal_yaw_rate);
    float wz_ff = FOLLOW_KFF * rate_ff;

    // 3. 反馈 + 前馈
    float wz_raw = wz_fb + wz_ff;

    // 4. 总角速度限幅
    if (wz_raw > FOLLOW_WZ_MAX)  wz_raw = FOLLOW_WZ_MAX;
    if (wz_raw < -FOLLOW_WZ_MAX) wz_raw = -FOLLOW_WZ_MAX;

    // 5. 斜坡限幅
    float wz_target = SlewLimit(wz_raw, wz_target_last, FOLLOW_ACCEL_MAX, FOLLOW_DT);
    wz_target_last = wz_target;

    // 6. 填入调试数据
    dbg->yaw_relative    = offset_angle;
    dbg->yaw_error       = offset_angle;  // 目标 = 0°
    dbg->gimbal_yaw_rate = gimbal_yaw_rate;
    dbg->wz_feedback     = wz_fb;
    dbg->wz_feedforward  = wz_ff;
    dbg->wz_target       = wz_target;

    return wz_target;
}
```

### 4.6 替换原有代码

在 [chassis.c:333-341](../application/chassis/chassis.c#L333-L341) 的 `case CHASSIS_FOLLOW_GIMBAL_YAW:` 中，将死区 + 二次函数替换为：

```c
case CHASSIS_FOLLOW_GIMBAL_YAW:
{
    static FollowDebugData_t follow_dbg;
    chassis_cmd_recv.wz = ChassisFollowControl(
        chassis_cmd_recv.offset_angle,
        chassis_cmd_recv.gimbal_yaw_rate,
        &follow_dbg
    );

    // VOFA+ 发送（100Hz）
    static uint8_t vofa_div = 0;
    if (++vofa_div >= 2)
    {
        vofa_div = 0;
        follow_dbg.chassis_wz_actual = chassis_feedback_data.real_wz;
        follow_dbg.motor_current_max = GetChassisMaxMotorCurrent();  // 需要实现
        VOFA_Send((float*)&follow_dbg, VOFA_CHANNEL_COUNT);
    }
    break;
}
```

---

## 五、调参顺序

### 第一阶段：前馈关闭，只调反馈

```
FOLLOW_KFF = 0.0f
死区 = 0.5°
KP_SMALL = 10 → 12 → 15 → 18 → 20
WZ_MAX  = 120
ACCEL_MAX = 600
```

目标：
- 云台偏开约 1° 时，底盘能明显启动
- 松手后不在零点来回摆动
- 大角度时不突然猛冲

VOFA+ 重点看：`yaw_error`、`wz_feedback`、`chassis_wz_actual`

### 第二阶段：加入前馈

```
FOLLOW_KFF: 0.20 → 0.35 → 0.50 → 0.65 → 0.80
```

每档都做同一组快速云台动作，记录 `yaw_error` 峰值的变化趋势。

### 第三阶段：细调限幅

根据曲线微调：
- `WZ_MAX`：大角度跟随速度
- `ACCEL_MAX`：启动/停止的急缓

### 常见现象及处理

| 现象 | 曲线表现 | 处理 |
|------|----------|------|
| 启动慢 | `gimbal_yaw_rate` 已上升，`wz_target` 很小 | 增加 KFF 或 KP_SMALL |
| 停止过冲 | `yaw_error` 穿过 0 变反向 | 降低 KFF 或 ACCEL_MAX |
| 静止抖动 | `gimbal_yaw_rate` 在 0 附近正负跳 | 重新校零偏、增大角速度死区 |
| 方向反了 | 云台左转底盘右转 | 在 `FollowYawFeedback` 返回前加负号 |
| 大角度猛冲 | `wz_feedback` 快速到上限 | 降低 KP_LARGE 或 WZ_MAX |

---

## 六、四组串口测试

### 测试1：全部静止 10 秒

保持云台和底盘不动。

**重点观察**：
- `gimbal_yaw_rate` 接近 0（波动 < ±0.5~1.0°/s 为正常）
- `chassis_wz_actual` 接近 0
- `yaw_error` 不持续漂移
- `wz_feedback` 不频繁正负跳变

> 如果静止时 `gimbal_yaw_rate` 持续偏在 +3 或 -3°/s，说明陀螺仪有零偏，需要校准。有零偏时不能直接加入前馈，否则底盘会自行缓慢旋转。

### 测试2：慢速转动云台

云台以约 20~30°/s 转动 ±10~15°。

**重点观察**：
1. `gimbal_yaw_rate` 的变化 → `yaw_error` 的增长 → `wz_feedback` 的启动
2. 三者之间的时序关系

**预期**（原方案）：死区 1.5° + 二次曲线 → 底盘明显滞后。这组曲线用来量化改进前的基准。

### 测试3：快速转动云台

快速转动 10~20° 然后立即停止。

**记录**：
- yaw_error 最大峰值
- 底盘实际角速度启动时间
- 云台停止后底盘过冲量和收敛时间

加前馈后用同一组动作对比。

### 测试4：只转底盘，云台不动

保持遥控器/鼠标云台指令不变，只让底盘旋转。

**重点观察**：`gimbal_yaw_rate`

- **情况 A**：底盘转，`gimbal_yaw_rate` ≈ 0 → 云台稳定好，IMU Gyro[2] 可用作备选前馈源
- **情况 B**：底盘转，`gimbal_yaw_rate` 也跟着变 → IMU 数据含底盘自转分量，不能直接用作前馈

---

## 七、参数汇总

### 初始值（第一轮架空调试）

```c
#define FOLLOW_YAW_DEADBAND_DEG  0.5f
#define FOLLOW_SMALL_END_DEG     4.0f
#define FOLLOW_KP_SMALL          15.0f
#define FOLLOW_KP_LARGE          7.0f
#define FOLLOW_KFF               0.20f
#define FOLLOW_WZ_MAX            120.0f
#define FOLLOW_ACCEL_MAX         600.0f
#define FOLLOW_DT                0.005f
#define GIMBAL_GYRO_LPF_ALPHA    0.25f
#define GYRO_RATE_DEADBAND       0.8f
```

### 第二轮

```c
#define FOLLOW_KP_SMALL          18.0f
#define FOLLOW_KFF               0.40f
#define FOLLOW_WZ_MAX            150.0f
#define FOLLOW_ACCEL_MAX         800.0f
```

### 第三轮（根据曲线决定）

```c
#define FOLLOW_KFF               0.60f ~ 0.80f
```

---

## 八、修改文件清单

| 文件 | 改动 | 说明 |
|------|------|------|
| [robot_def.h](../application/robot_def.h) | `Chassis_Ctrl_Cmd_s` 加 `gimbal_yaw_rate` | robot_cmd → chassis 数据通道扩展 |
| [robot_cmd.c](../application/cmd/robot_cmd.c) | `RemoteControlSet()` / `MouseKeySet()` 中填 `gimbal_yaw_rate` | 提供前馈源数据 |
| [chassis.c](../application/chassis/chassis.c) | 重写 `CHASSIS_FOLLOW_GIMBAL_YAW` 分支 | 核心控制逻辑 |
| [chassis.c](../application/chassis/chassis.c) | 新增 `FollowYawFeedback()`、`ApplySoftDeadband()`、`SlewLimit()`、`ChassisFollowControl()` | 控制函数 |
| [chassis.c](../application/chassis/chassis.c) | 新增 `VOFA_Send()`、`FollowDebugData_t` | 调试输出 |
| [chassis.c](../application/chassis/chassis.c) | 恢复 `EstimateSpeed()` | 获取底盘实际 wz |
| 新增或 CubeMX | 空闲 UART 配置 | USART2 / UART4 / UART5 |

---

## 九、安全检查清单

- [ ] 底盘架空（四个轮子离地）
- [ ] 遥控器左拨杆下位 = `CHASSIS_ZERO_FORCE` 可用
- [ ] 第一阶段 `FOLLOW_KFF = 0.0`，确认反馈正确后再开前馈
- [ ] 串口先不发，确认控制逻辑正确后再打开调试输出
- [ ] `wz_target` 方向与实际底盘旋转方向一致

---

## 附录：VisionSend 频率修复

**问题**：`VisionSend()` 在 `RobotCMDTask()` 中以 200Hz 调用，USB CDC 连续发送 43 字节/帧，上位机串口读取未做 `SP` 帧头同步时，USB 驱动攒包导致读帧错位，CRC 持续失败。

**修复**：[robot_cmd.c:631](../application/cmd/robot_cmd.c) 加 4 分频 → 50Hz。上位机侧需做 `SP` 帧头搜索 + CRC 校验。
