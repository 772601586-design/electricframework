#include "gimbal.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "xmmotor.h"

#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "master_process.h"
#include "bsp_dwt.h"

#include <math.h>

#define VISION_TRAJECTORY_MAX_PROPAGATION_US 4000ULL
#define VISION_TRAJECTORY_STALE_US 10000ULL

typedef struct
{
    float theta0_rad;
    float omega0_rad_s;
    float alpha0_rad_s2;
    uint32_t anchor_dwt_count;
} Gimbal_Axis_Trajectory_s;

static Gimbal_Config_s *gimbal_config;

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor, *mini_yaw_motor, *pitch_motor;
static XMMotorInstance *pitch_xmmotor;
static gimbal_mode_e last_mode = GIMBAL_ZERO_FORCE;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static Gimbal_Ctrl_Cmd_s gimbal_fast_cmd;         // 供1kHz任务读取的原子快照

static float yaw_speed_ff_rad_s;
static float pitch_speed_ff_rad_s;
static Gimbal_Axis_Trajectory_s yaw_trajectory;
static Gimbal_Axis_Trajectory_s pitch_trajectory;
static uint32_t trajectory_generation;
static uint32_t rejected_trajectory_generation;
static uint8_t trajectory_active;
static volatile Gimbal_Trajectory_Debug_s gimbal_trajectory_debug;

static BMI088Instance *bmi088; // 云台IMU

static uint8_t VisionTrajectoryDataValid(const Vision_Recv_s *vision)
{
    return isfinite(vision->yaw) && isfinite(vision->yaw_vel) && isfinite(vision->yaw_acc) &&
           isfinite(vision->pitch) && isfinite(vision->pitch_vel) && isfinite(vision->pitch_acc);
}

static void AxisTrajectoryAccept(Gimbal_Axis_Trajectory_s *trajectory,
                                 float theta_rad, float omega_rad_s,
                                 float alpha_rad_s2, uint32_t anchor_dwt_count)
{
    trajectory->theta0_rad = theta_rad;
    trajectory->omega0_rad_s = omega_rad_s;
    trajectory->alpha0_rad_s2 = alpha_rad_s2;
    trajectory->anchor_dwt_count = anchor_dwt_count;
}

static float VisionYawToNearestTotalRad(float vision_yaw_rad)
{
    float current_total_rad = gimba_IMU_data->YawTotalAngle * DEGREE_2_RAD;
    return vision_yaw_rad + PI2 * roundf((current_total_rad - vision_yaw_rad) / PI2);
}

static void AxisTrajectoryPropagate(const Gimbal_Axis_Trajectory_s *trajectory,
                                    uint32_t now_dwt_count, float *theta_ref_rad,
                                    float *omega_ref_rad_s)
{
    uint32_t age_us = DWT_CycleDeltaToUs(trajectory->anchor_dwt_count, now_dwt_count);
    uint32_t propagation_us = age_us;

    if (propagation_us > VISION_TRAJECTORY_MAX_PROPAGATION_US)
        propagation_us = VISION_TRAJECTORY_MAX_PROPAGATION_US;

    float dt = (float)propagation_us * 1e-6f;
    *theta_ref_rad = trajectory->theta0_rad + trajectory->omega0_rad_s * dt +
                     0.5f * trajectory->alpha0_rad_s2 * dt * dt;
    *omega_ref_rad_s = trajectory->omega0_rad_s + trajectory->alpha0_rad_s2 * dt;

    if (age_us > VISION_TRAJECTORY_MAX_PROPAGATION_US)
        *omega_ref_rad_s = 0.0f;
}

static float MotorReferenceSign(const Motor_Init_Config_s *config)
{
    return config->controller_setting_init_config.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -1.0f : 1.0f;
}

static void UpdateFastCommandSnapshot(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    gimbal_fast_cmd = gimbal_cmd_recv;
    __set_PRIMASK(primask);
}

static Gimbal_Ctrl_Cmd_s GetFastCommandSnapshot(void)
{
    Gimbal_Ctrl_Cmd_s snapshot;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    snapshot = gimbal_fast_cmd;
    __set_PRIMASK(primask);
    return snapshot;
}

void GimbalInit()
{   
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源

    gimbal_config = GimbalConfigFeed();
  
    // yaw轴初始化
    gimbal_config->yaw_motor_config.controller_param_init_config.other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle;
    gimbal_config->yaw_motor_config.controller_param_init_config.other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2];
    gimbal_config->yaw_motor_config.controller_param_init_config.speed_feedforward_ptr = &yaw_speed_ff_rad_s;
    gimbal_config->yaw_motor_config.controller_setting_init_config.feedforward_flag |= SPEED_FEEDFORWARD;
    yaw_motor = DJIMotorInit(&gimbal_config->yaw_motor_config);
    
    /*
     * 小 yaw 电机当前未启用。不要在这里保留一个没有函数体的 if：
     * 被注释掉的语句会让下面的 pitch 角度反馈初始化意外成为 if 的
     * 单条语句，导致 SINGLE_GIMBAL 的反馈指针保持为 NULL。
     */
    mini_yaw_motor = NULL;
    
#ifdef _IS_IMU_ROLL
    // PITCH
    gimbal_config->pitch_motor_config.controller_param_init_config.other_angle_feedback_ptr = &gimba_IMU_data->Roll;
    gimbal_config->pitch_motor_config.controller_param_init_config.other_speed_feedback_ptr = &gimba_IMU_data->Gyro[1];
#else
    // PITCH
    gimbal_config->pitch_motor_config.controller_param_init_config.other_angle_feedback_ptr = &gimba_IMU_data->Pitch;
    gimbal_config->pitch_motor_config.controller_param_init_config.other_speed_feedback_ptr = &gimba_IMU_data->Gyro[0];
#endif

    gimbal_config->pitch_motor_config.controller_param_init_config.speed_feedforward_ptr = &pitch_speed_ff_rad_s;
    gimbal_config->pitch_motor_config.controller_setting_init_config.feedforward_flag |= SPEED_FEEDFORWARD;

    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    switch (gimbal_config->pitch_motor_config.motor_type)
    {
    case GM6020:
        pitch_motor = DJIMotorInit(&gimbal_config->pitch_motor_config);
        break;
    case XMCY:
        pitch_xmmotor = XMMotorInit(&gimbal_config->pitch_motor_config);
        break;
    default:
        break;
    }

    UpdateFastCommandSnapshot();

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    if (last_mode == GIMBAL_ZERO_FORCE && gimbal_cmd_recv.gimbal_mode == GIMBAL_GYRO_MODE)
    {
        gimbal_cmd_recv.pitch = gimba_IMU_data->Pitch;
        gimbal_cmd_recv.yaw = gimba_IMU_data->YawTotalAngle;
    }
    last_mode = gimbal_cmd_recv.gimbal_mode;

      // 只要是从无力模式切回控制模式，强制同步目标值
    // if (gimbal_cmd_recv.last_mode == GIMBAL_ZERO_FORCE && gimbal_cmd_recv.gimbal_mode == GIMBAL_GYRO_MODE) 
    // {
    //     gimbal_cmd_recv.pitch = gimba_IMU_data->Pitch; // 目标 = 当前实际位置
    // }

    // gimbal_cmd_recv.last_mode = gimbal_cmd_recv.gimbal_mode;
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);

        if (gimbal_config->gimbal_type == MINI_GIMBAL && mini_yaw_motor != NULL)
            DJIMotorStop(mini_yaw_motor);

        switch (gimbal_config->pitch_motor_config.motor_type)
        {
        case GM6020:
            DJIMotorStop(pitch_motor);
            break;
        case XMCY:
            XMMotorStop(pitch_xmmotor);
            break;
        default:
            break;
        }
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: 
        DJIMotorEnable(yaw_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorOuterLoop(yaw_motor, ANGLE_LOOP);
        DJIMotorCloseLoop(yaw_motor, ANGLE_LOOP | SPEED_LOOP);

        if (gimbal_config->gimbal_type == MINI_GIMBAL && mini_yaw_motor != NULL)
        {
            DJIMotorEnable(mini_yaw_motor);
        }
    
        switch (gimbal_config->pitch_motor_config.motor_type)
        {
        case GM6020:
            DJIMotorEnable(pitch_motor);
            DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
            DJIMotorOuterLoop(pitch_motor, ANGLE_LOOP);
            DJIMotorCloseLoop(pitch_motor, ANGLE_LOOP | SPEED_LOOP);
            break;
        case XMCY:
            XMMotorEnable(pitch_xmmotor);
            XMMotorChangeFeed(pitch_xmmotor, ANGLE_LOOP, OTHER_FEED);
            XMMotorChangeFeed(pitch_xmmotor, SPEED_LOOP, OTHER_FEED);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }

    UpdateFastCommandSnapshot();
    
    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd,还有云台电机状态
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    //根据电机状态反馈是否离线
    DJIMotorIsOnline(yaw_motor);
    DJIMotorIsOnline(pitch_motor);
    // 反馈电机离线数量
    if (yaw_motor->online_flag == MOTOR_OFFLINE || pitch_motor->online_flag == MOTOR_OFFLINE)
    {
        gimbal_feedback_data.motor_offline_count = 1;
        gimbal_feedback_data.motor_state = GIMBAL_MOTOR_OFFLINE;
    }
    else if(yaw_motor->online_flag == MOTOR_OFFLINE && pitch_motor->online_flag == MOTOR_OFFLINE)
    {
        gimbal_feedback_data.motor_offline_count = 2;
        gimbal_feedback_data.motor_state = GIMBAL_MOTOR_OFFLINE;
    }
    else 
    {
        gimbal_feedback_data.motor_offline_count = 0;
        gimbal_feedback_data.motor_state = GIMBAL_MOTOR_ONLINE;
    }

    //推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}

void GimbalFastTask(void)
{
    Vision_Rx_Snapshot_s vision_snapshot = {0};
    Gimbal_Ctrl_Cmd_s command = GetFastCommandSnapshot();
    uint32_t now_dwt_count = DWT_GetCycleCount();

    if (command.gimbal_mode != GIMBAL_GYRO_MODE)
    {
        trajectory_active = 0;
        yaw_speed_ff_rad_s = 0.0f;
        pitch_speed_ff_rad_s = 0.0f;
        gimbal_trajectory_debug.vision_active = 0;
        return;
    }

    if (!command.vision_control)
    {
        trajectory_active = 0;
        yaw_speed_ff_rad_s = 0.0f;
        pitch_speed_ff_rad_s = 0.0f;
        DJIMotorSetRef(yaw_motor, command.yaw);

        if (gimbal_config->gimbal_type == MINI_GIMBAL && mini_yaw_motor != NULL)
            DJIMotorSetRef(mini_yaw_motor, command.mini_yaw);

        switch (gimbal_config->pitch_motor_config.motor_type)
        {
        case GM6020:
            DJIMotorSetRef(pitch_motor, command.pitch);
            break;
        case XMCY:
            XMMotorSetRef(pitch_xmmotor, command.pitch);
            break;
        default:
            break;
        }

        gimbal_trajectory_debug.vision_active = 0;
        gimbal_trajectory_debug.command_stale = 0;
        return;
    }

    if (!VisionGetRxSnapshotWithMeta(&vision_snapshot) ||
        vision_snapshot.generation == 0 ||
        (vision_snapshot.data.mode != 1 && vision_snapshot.data.mode != 2) ||
        !VisionTrajectoryDataValid(&vision_snapshot.data))
    {
        if (vision_snapshot.generation != 0)
            rejected_trajectory_generation = vision_snapshot.generation;
        trajectory_active = 0;
        yaw_speed_ff_rad_s = 0.0f;
        pitch_speed_ff_rad_s = 0.0f;
        gimbal_trajectory_debug.vision_active = 0;
        gimbal_trajectory_debug.command_stale = 1;
        return;
    }

    uint32_t command_age_us = DWT_CycleDeltaToUs(vision_snapshot.rx_dwt_count, now_dwt_count);
    if (command_age_us > VISION_TRAJECTORY_STALE_US)
    {
        rejected_trajectory_generation = vision_snapshot.generation;
        trajectory_active = 0;
        yaw_speed_ff_rad_s = 0.0f;
        pitch_speed_ff_rad_s = 0.0f;
        gimbal_trajectory_debug.vision_active = 0;
        gimbal_trajectory_debug.command_stale = 1;
        gimbal_trajectory_debug.command_age_us = (uint32_t)command_age_us;
        return;
    }

    if (vision_snapshot.generation == rejected_trajectory_generation)
    {
        trajectory_active = 0;
        yaw_speed_ff_rad_s = 0.0f;
        pitch_speed_ff_rad_s = 0.0f;
        gimbal_trajectory_debug.vision_active = 0;
        gimbal_trajectory_debug.command_stale = 1;
        return;
    }

    if (vision_snapshot.generation != trajectory_generation)
    {
        AxisTrajectoryAccept(&yaw_trajectory,
                             VisionYawToNearestTotalRad(vision_snapshot.data.yaw),
                             vision_snapshot.data.yaw_vel,
                             vision_snapshot.data.yaw_acc,
                             vision_snapshot.rx_dwt_count);
        AxisTrajectoryAccept(&pitch_trajectory,
                             -vision_snapshot.data.pitch + VISION_PITCH_ZERO_OFFSET_DEG * DEGREE_2_RAD,
                             -vision_snapshot.data.pitch_vel,
                             -vision_snapshot.data.pitch_acc,
                             vision_snapshot.rx_dwt_count);
        trajectory_generation = vision_snapshot.generation;
    }
    trajectory_active = 1;

    float yaw_ref_rad;
    float pitch_ref_rad;
    float yaw_omega_ref_rad_s;
    float pitch_omega_ref_rad_s;
    AxisTrajectoryPropagate(&yaw_trajectory, now_dwt_count, &yaw_ref_rad, &yaw_omega_ref_rad_s);
    AxisTrajectoryPropagate(&pitch_trajectory, now_dwt_count, &pitch_ref_rad, &pitch_omega_ref_rad_s);

    float yaw_ref_deg = yaw_ref_rad * RAD_2_DEGREE;
    float pitch_ref_deg = pitch_ref_rad * RAD_2_DEGREE;
    float pitch_ref_unlimited_deg = pitch_ref_deg;
    LIMIT_MIN_MAX(pitch_ref_deg,
                  gimbal_config->gimbal_offset.pitch_min_angle,
                  gimbal_config->gimbal_offset.pitch_max_angle);
    if (pitch_ref_deg != pitch_ref_unlimited_deg)
        pitch_omega_ref_rad_s = 0.0f;

    yaw_speed_ff_rad_s = MotorReferenceSign(&gimbal_config->yaw_motor_config) * yaw_omega_ref_rad_s;
    pitch_speed_ff_rad_s = MotorReferenceSign(&gimbal_config->pitch_motor_config) * pitch_omega_ref_rad_s;

    DJIMotorSetRef(yaw_motor, yaw_ref_deg);
    switch (gimbal_config->pitch_motor_config.motor_type)
    {
    case GM6020:
        DJIMotorSetRef(pitch_motor, pitch_ref_deg);
        break;
    case XMCY:
        XMMotorSetRef(pitch_xmmotor, pitch_ref_deg);
        break;
    default:
        break;
    }

    gimbal_trajectory_debug.vision_generation = trajectory_generation;
    gimbal_trajectory_debug.command_age_us = (uint32_t)command_age_us;
    gimbal_trajectory_debug.vision_active = 1;
    gimbal_trajectory_debug.command_stale = 0;
    gimbal_trajectory_debug.yaw_ref_deg = yaw_ref_deg;
    gimbal_trajectory_debug.yaw_speed_ff_rad_s = yaw_speed_ff_rad_s;
    gimbal_trajectory_debug.pitch_ref_deg = pitch_ref_deg;
    gimbal_trajectory_debug.pitch_speed_ff_rad_s = pitch_speed_ff_rad_s;
}

const volatile Gimbal_Trajectory_Debug_s *GimbalGetTrajectoryDebug(void)
{
    return &gimbal_trajectory_debug;
}
