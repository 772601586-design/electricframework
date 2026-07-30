# 云台 1 kHz 二阶轨迹重建与速度前馈修复报告

## 1. 修复目标

修复视觉协议虽然提供 `yaw/pitch` 的位置、速度、加速度，但云台只在 200 Hz 使用位置目标的问题。修复后由 1 kHz 电机任务根据最新视觉帧进行恒加速度二阶传播，并将传播速度作为速度环前馈。

控制关系为：

```text
theta_ref(t) = theta0 + omega0 * dt + 0.5 * alpha0 * dt^2
omega_ref(t) = omega0 + alpha0 * dt

speed_ref = angle_pid_output + omega_ref
current_ref = speed_pid(speed_ref - omega_measure)
```

## 2. 修复前的问题

1. `RobotCMDTask()` 约 200 Hz，只使用视觉位置字段，忽略 `yaw_vel/yaw_acc/pitch_vel/pitch_acc`。
2. `DJIMotorControl()` 约 1 kHz，但同一个 `pid_ref` 会被保持约 5 个周期，目标呈阶梯变化。
3. 视觉快照没有接收时间戳和代次，控制任务无法判断是否收到新帧。
4. 200 Hz `GimbalTask()` 和拟新增的 1 kHz 传播任务可能同时写电机参考值。
5. 前馈启用后会直接解引用指针，配置遗漏会导致 HardFault。
6. 位置环输出限幅发生在速度前馈叠加之前，前馈可能绕过总速度参考限幅。
7. pitch 配置使用 `MOTOR_DIRECTION_REVERSE`，速度前馈需要与被反向后的位置参考保持一致。

## 3. 已完成的修改

### 3.1 视觉接收元数据

`Vision_Rx_Snapshot_s` 新增：

- `rx_dwt_count`：CRC 通过时记录的 DWT 周期计数。
- `generation`：每收到一帧有效数据递增。

新增 `VisionGetRxSnapshotWithMeta()`，在关中断的短临界区内原子复制数据、时间和代次。

中断时间戳使用只读的 `DWT->CYCCNT`，没有在 USB 中断中调用会更新共享状态的 `DWT_GetTimeline_us()`。新增的周期差换算函数支持一次 32 位自然回绕。

### 3.2 单一参考值写入者

`GimbalTask()` 保留以下职责：

- 接收 `gimbal_cmd`。
- 处理无力/陀螺仪模式切换。
- 使能、停止电机并切换反馈源。
- 生成供快速任务读取的原子命令快照。

`GimbalFastTask()` 成为 yaw、pitch 电机参考值和速度前馈的唯一写入者。它在 `MotorControlTask()` 前执行：

```c
GimbalFastTask();
MotorControlTask();
```

手动模式仍使用原有角度目标，速度前馈固定为零。

### 3.3 二阶传播和超时策略

视觉模式下，每个新代次保存一组传播锚点：

```text
theta0, omega0, alpha0, rx_dwt_count
```

1 kHz 周期始终从锚点直接计算当前目标，不进行逐周期积分，因此不会累计积分和任务周期误差。

当前安全参数：

| 参数 | 数值 | 行为 |
|---|---:|---|
| 最大传播时间 | 4 ms | 超过后冻结位置并将速度前馈清零 |
| 指令失效时间 | 10 ms | 停止使用该视觉轨迹，保持最后位置 |

pitch 传播位置会重新经过机器人配置中的机械角度限幅。触发位置限幅时对应速度前馈清零。

超时或非法帧的 `generation` 会被锁存为拒绝代次。同一代次即使遇到 DWT 32 位计数整圈回绕也不会重新启用，只有收到新一帧、代次递增后才恢复传播。

### 3.4 速度前馈连接

yaw、pitch 分别使用具有静态生命周期的变量：

```c
static float yaw_speed_ff_rad_s;
static float pitch_speed_ff_rad_s;
```

指针和 `SPEED_FEEDFORWARD` 标志在 `DJIMotorInit()` / `XMMotorInit()` 之前配置。

位置参考仍按现有角度环接口转换为 degree；速度前馈保持 rad/s，与 `Gyro[]` 反馈单位一致。pitch 速度前馈根据 `motor_reverse_flag` 使用与有效位置参考相同的方向。

### 3.5 电机层防护

DJI 和小米电机模块均增加：

- 前馈启用但指针为空时记录错误并关闭对应前馈位。
- 补齐小米电机初始化时遗漏的前馈指针复制。
- 速度前馈叠加后，使用位置 PID 的 `MaxOut` 再次限制总速度参考。

## 4. 调试观测

可通过 `GimbalGetTrajectoryDebug()` 观察：

| 字段 | 单位 | 含义 |
|---|---|---|
| `vision_generation` | - | 当前使用的有效视觉帧代次 |
| `command_age_us` | us | 当前控制时刻距接收时刻的时间 |
| `vision_active` | bool | 当前是否由视觉轨迹接管 |
| `command_stale` | bool | 指令无效或超时 |
| `yaw_ref_deg` | degree | 传播后的 yaw 位置参考 |
| `yaw_speed_ff_rad_s` | rad/s | yaw 速度前馈 |
| `pitch_ref_deg` | degree | 限幅后的 pitch 位置参考 |
| `pitch_speed_ff_rad_s` | rad/s | pitch 速度前馈 |

还应同时观察：

```text
yaw_motor->motor_controller.angle_PID.Output
yaw_motor->motor_controller.speed_PID.Ref
yaw_motor->motor_controller.speed_PID.Output
pitch_motor->motor_controller.angle_PID.Output
pitch_motor->motor_controller.speed_PID.Ref
pitch_motor->motor_controller.speed_PID.Output
gimba_IMU_data->YawTotalAngle / Pitch / Gyro[]
```

## 5. 编译验证

执行：

```text
cmake --build build-validation --parallel 4
```

结果：ARM 固件编译和链接成功，生成 `basic_framework.elf/.hex/.bin`。本次修改没有引入新的编译错误。输出中仍有工程原有的未使用变量、枚举比较和 newlib syscall 警告，与本次轨迹修复无关。

## 6. 实机验收步骤

1. 架空云台，先令视觉发送静止目标：`omega=0, alpha=0`，确认速度前馈为零且位置不漂移。
2. yaw 发送小幅正速度目标，确认 `yaw_speed_ff_rad_s`、`Gyro[2]` 和角度变化方向一致。
3. pitch 发送小幅正速度目标，重点确认反向配置后的前馈没有与位置环对抗。
4. 断开视觉数据，确认 4 ms 后速度前馈归零，10 ms 后 `command_stale=1`。
5. 以目标频率发送数据，确认 `generation` 连续增长，`command_age_us` 通常小于一个视觉周期。
6. 对比修复前后的 `theta_ref/theta_measure/error` 曲线和速度环饱和比例。

## 7. 尚未解决或需要确认的事项

1. 协议没有上位机采样时间戳，目前传播锚点是 MCU 接收时刻，不能补偿视觉计算和 USB 传输延迟。
2. 当前角度环使用 degree、速度环使用 rad/s，保留该混合单位是为了不直接改变既有 PID 数值；后续若统一为 SI 单位必须重新换算和验证 PID 参数。
3. 当前仅加入速度前馈，`J/B/C/mgr/Kt` 对应的电流前馈尚未实现。
4. yaw 是否为连续多圈角度需要和视觉协议确认；若视觉在 `[-pi, pi]` 回绕，需要增加基于当前参考的解包处理。
5. 编译验证不能替代实机方向、限幅和稳定性测试，首次上电必须小幅、低速验证。
