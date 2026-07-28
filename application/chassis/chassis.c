/**
 * @file chassis.c
 * @author Zero 1433026627@qq.com
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2024-11-18
 *
 * @copyright Copyright (c) 2022 Team JiaoLong-SJTU
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "power_manager.h"
#include "message_center.h"
#include "user_lib.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include "main.h"

static Chassis_Config_s *chassis_config;

#define FOLLOW_YAW_DEADBAND_DEG  0.5f
#define FOLLOW_SMALL_END_DEG     4.0f
#define FOLLOW_KP_SMALL          15.0f
#define FOLLOW_KP_LARGE          7.0f
#define FOLLOW_KFF               0.0f    // 前馈增益,第一阶段先关
#define FOLLOW_WZ_MAX            120.0f  // 最大角速度 (°/s)
#define FOLLOW_ACCEL_MAX         600.0f  // 最大角加速度 (°/s²)
#define FOLLOW_DT                0.005f  // 控制周期 (s)
#define GIMBAL_GYRO_LPF_ALPHA    0.25f   // 角速度低通系数
#define GYRO_RATE_DEADBAND       0.8f    // 角速度死区 (°/s)

#define ROTATE_WZ_MAX 8800.0f      // 自旋模式最大旋转速度(电机RPM单位,M3508极限附近)
#define ROTATE_WZ_DEFAULT 7500.0f  // 自旋模式默认旋转速度(电机RPM单位)

// VOFA+ JustFloat 调试
#define VOFA_CHANNEL_COUNT 8

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
static PowerManagerInstance *power_manager;                         // 功率管理

/* VOFA+ 调试数据结构 */
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

// VOFA+ JustFloat 尾帧
static const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7F};

/* 私有变量用于底盘旋转的机械参数常量 */
static float lf_center, rf_center, lb_center, rb_center; // 左右前后轮子中心
static float rpm_2_wheel_vector;
/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;                 // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb;             // 底盘速度解算后的输出
static float rl_vt_lf, rl_vt_rf, rl_vt_lb, rl_vt_rb; // 底盘真实速度 m/s

static float last_offset_angle;  // 上一周期offset_angle,用于检测底盘旋转增量

/* 自旋模式坐标变换用 — 累积自旋相位保证平动方向稳定 */
static float spin_phase_accum;   // 自旋累积角度(度)
static float last_spin_wz;       // 上一周期旋转速度,用于积分

/* 功率分配权重 — 自旋模式下平动优先 */
#define POWER_WEIGHT_TRANSLATION 1.3f  // 平动分量权重(>1优先保证)
#define POWER_WEIGHT_ROTATION    0.7f  // 转动分量权重(<1优先削减)

#ifndef GIMBAL_BOARD
void ChassisInit()
{
    static float half_wheel_base, half_track_width, perimeter_wheel;    // 半轴距,半轮距,轮子周长
    static float center_gimbal_offset_x, center_gimbal_offset_y;        // 中心云台偏移量
    static float reduction_ratio_wheel;                                 // 轮子减速比

    chassis_config = ChassisConfigFeed();

    half_wheel_base = (chassis_config->wheel_measure.wheel_base / 2.0f);
    half_track_width = (chassis_config->wheel_measure.track_width / 2.0f);
    perimeter_wheel = (chassis_config->wheel_measure.radius_wheel * 2 * PI);
    center_gimbal_offset_x = chassis_config->wheel_measure.center_gimbal_offset_x;
    center_gimbal_offset_y = chassis_config->wheel_measure.center_gimbal_offset_y;
    reduction_ratio_wheel = chassis_config->wheel_measure.reduction_ratio_wheel;

    lf_center = ((half_track_width + center_gimbal_offset_x + half_wheel_base - center_gimbal_offset_y) * DEGREE_2_RAD);
    rf_center = ((half_track_width - center_gimbal_offset_x + half_wheel_base - center_gimbal_offset_y) * DEGREE_2_RAD);
    lb_center = ((half_track_width + center_gimbal_offset_x + half_wheel_base + center_gimbal_offset_y) * DEGREE_2_RAD);
    rb_center = ((half_track_width - center_gimbal_offset_x + half_wheel_base + center_gimbal_offset_y) * DEGREE_2_RAD);
    rpm_2_wheel_vector = (ANGLE_2_RPM_PER_MIN * (perimeter_wheel * MM_2_M) / (reduction_ratio_wheel * MIN_2_SEC));

    // 四个轮子的参数一样,改tx_id和反转标志位即可
    chassis_config->chassis_motor_config.can_init_config.tx_id = chassis_config->chassis_motor_id[MOTOR_LF];
    chassis_config->chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf = DJIMotorInit(&chassis_config->chassis_motor_config);     

    chassis_config->chassis_motor_config.can_init_config.tx_id = chassis_config->chassis_motor_id[MOTOR_RF];
    chassis_config->chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_config->chassis_motor_config);

    chassis_config->chassis_motor_config.can_init_config.tx_id = chassis_config->chassis_motor_id[MOTOR_LB]; 
    chassis_config->chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb = DJIMotorInit(&chassis_config->chassis_motor_config);

    chassis_config->chassis_motor_config.can_init_config.tx_id = chassis_config->chassis_motor_id[MOTOR_RB];     
    chassis_config->chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DJIMotorInit(&chassis_config->chassis_motor_config);

    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x302, // 超级电容默认接收id
            .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }};
    cap = SuperCapInit(&cap_conf); // 超级电容初始化

    PowerManager_Init_Config_s power_manager_conf = {
        .k1 = 0.013,    // 越大功率限制权重越大  
        .k2 = 5.23,     // 5.23
        .k3 = 0.82,     // 0.82
        };
    power_manager = PowerControlInit(&power_manager_conf); // 功率管理初始化 

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}

/* ==================== FOLLOW 模式控制函数 ==================== */

/**
 * @brief 平滑死区,deadband内返回0,之外减去边界
 */
static float ApplySoftDeadband(float input, float deadband)
{
    if (input > deadband)
        return input - deadband;
    if (input < -deadband)
        return input + deadband;
    return 0.0f;
}

/**
 * @brief 分段线性跟随反馈
 *        小角度(≤4°):高线性增益,解决迟钝
 *        大角度(>4°):降低增益增长率,避免猛冲
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

    return (e > 0.0f) ? -wz_abs : wz_abs; // error>0 → wz为负(底盘顺时针转)
}

/**
 * @brief 角速度微小死区,消除静止噪声
 */
static float ApplyRateDeadband(float rate_deg_s)
{
    if (rate_deg_s > GYRO_RATE_DEADBAND)
        return rate_deg_s - GYRO_RATE_DEADBAND;
    if (rate_deg_s < -GYRO_RATE_DEADBAND)
        return rate_deg_s + GYRO_RATE_DEADBAND;
    return 0.0f;
}

/**
 * @brief 斜坡限幅,限制目标角速度变化速率
 */
static float SlewLimit(float target, float prev, float max_accel, float dt)
{
    float delta = target - prev;
    float max_delta = max_accel * dt;

    if (delta > max_delta)
        return prev + max_delta;
    if (delta < -max_delta)
        return prev - max_delta;
    return target;
}

/**
 * @brief 获取四轮最大电流绝对值
 */
static float GetChassisMaxMotorCurrent(void)
{
    float max_current = 0.0f;
    float abs_current;
    abs_current = (float)abs(motor_lf->measure.real_current);
    if (abs_current > max_current) max_current = abs_current;
    abs_current = (float)abs(motor_rf->measure.real_current);
    if (abs_current > max_current) max_current = abs_current;
    abs_current = (float)abs(motor_lb->measure.real_current);
    if (abs_current > max_current) max_current = abs_current;
    abs_current = (float)abs(motor_rb->measure.real_current);
    if (abs_current > max_current) max_current = abs_current;
    return max_current;
}

static float wz_target_last = 0.0f;

/**
 * @brief FOLLOW 模式云台跟随控制
 * @param offset_angle     云台相对底盘偏角 (°)
 * @param gimbal_yaw_rate  云台yaw角速度 (°/s)
 * @param dbg              调试数据输出
 * @return 底盘目标角速度 wz (°/s)
 */
static float ChassisFollowControl(float offset_angle, float gimbal_yaw_rate,
                                  FollowDebugData_t *dbg)
{
    // 1. 偏角反馈
    float wz_fb = FollowYawFeedback(offset_angle);

    // 2. 前馈(第一阶段 KFF=0)
    float rate_ff = ApplyRateDeadband(gimbal_yaw_rate);
    float wz_ff = FOLLOW_KFF * rate_ff;

    // 3. 反馈+前馈
    float wz_raw = wz_fb + wz_ff;

    // 4. 总角速度硬限幅
    if (wz_raw > FOLLOW_WZ_MAX)  wz_raw = FOLLOW_WZ_MAX;
    if (wz_raw < -FOLLOW_WZ_MAX) wz_raw = -FOLLOW_WZ_MAX;

    // 5. 斜坡限幅
    float wz_target = SlewLimit(wz_raw, wz_target_last, FOLLOW_ACCEL_MAX, FOLLOW_DT);
    wz_target_last = wz_target;

    // 6. 填入调试数据
    if (dbg)
    {
        dbg->yaw_relative    = offset_angle;
        dbg->yaw_error       = offset_angle; // 目标=0°
        dbg->gimbal_yaw_rate = gimbal_yaw_rate;
        dbg->wz_feedback     = wz_fb;
        dbg->wz_feedforward  = wz_ff;
        dbg->wz_target       = wz_target;
    }

    return wz_target;
}

/**
 * @brief VOFA+ JustFloat 协议发送(通过DMA,不阻塞)
 * @note  使用前需在CubeMX中配置对应UART并开启TX DMA
 *        将下面宏改为实际使用的UART Handle
 */
UART_HandleTypeDef huart2;  // VOFA调试用, 需在CubeMX中配置UART2 init并启用TX DMA
#define VOFA_HUART huart2

static void VOFA_Send(float *data, uint8_t count)
{
    static uint8_t buf[VOFA_CHANNEL_COUNT * 4 + 4];
    memcpy(buf, data, count * 4);
    memcpy(buf + count * 4, (void *)vofa_tail, 4);
    HAL_UART_Transmit_DMA(&VOFA_HUART, buf, count * 4 + 4);
}

/* ==================== FOLLOW 模式控制函数结束 ==================== */

/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void ChassisCalculate()
{
    switch (chassis_config->wheel_type)
    {
    case MECANUM_WHEEL:
        // 麦轮解算逆运动学模型
        vt_lf = -chassis_vx - chassis_vy - chassis_cmd_recv.wz * lf_center;
        vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * rf_center;
        vt_lb = chassis_vx - chassis_vy - chassis_cmd_recv.wz * lb_center;
        vt_rb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * rb_center;
        break;
    case OMNI_WHEEL:
        // 全向轮解算逆运动学模型
        vt_lf = -sqrtf(2) / 2.0f * chassis_vx + sqrtf(2) / 2.0f * chassis_vy - chassis_cmd_recv.wz * lf_center;
        vt_rf = sqrtf(2) / 2.0f * chassis_vx + sqrtf(2) / 2.0f * chassis_vy - chassis_cmd_recv.wz * rf_center;
        vt_lb = -sqrtf(2) / 2.0f * chassis_vx - sqrtf(2) / 2.0f * chassis_vy - chassis_cmd_recv.wz * lb_center;
        vt_rb = sqrtf(2) / 2.0f * chassis_vx - sqrtf(2) / 2.0f * chassis_vy - chassis_cmd_recv.wz * rb_center;

        // vt_lf = -sqrtf(2) / 2.0f * chassis_cmd_recv.vx - sqrtf(2) / 2.0f * chassis_cmd_recv.vy - chassis_cmd_recv.wz * lf_center;
        // vt_rf = -sqrtf(2) / 2.0f * chassis_cmd_recv.vx + sqrtf(2) / 2.0f * chassis_cmd_recv.vy - chassis_cmd_recv.wz * rf_center;
        // vt_lb =  sqrtf(2) / 2.0f * chassis_cmd_recv.vx + sqrtf(2) / 2.0f * chassis_cmd_recv.vy - chassis_cmd_recv.wz * lb_center;
        // vt_rb =  sqrtf(2) / 2.0f * chassis_cmd_recv.vx - sqrtf(2) / 2.0f * chassis_cmd_recv.vy - chassis_cmd_recv.wz * rb_center;
        break;
    default:
        break;
    }
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
    // 底盘电机顺序和限制速度电机顺序一致，通过指针访问减少内存浪费
    static Motor_Controller_s *motor_controller;   // 电机控制器指针
    static DJI_Motor_Measure_s *measure;           // 电机测量值指针    
    DJIMotorInstance *motor[4] = {motor_lf, motor_rf, motor_lb, motor_rb};
    float limit_vt[4] = {vt_lf, vt_rf, vt_lb, vt_rb};

    float currentPower[4];
    float error[4];
    float allocatablePower, sumPowerRequired, sumCurrentPower, sumError;
    float errorConfidence, powerWeight_Error, powerWeight_Prop, powerWeight, delta;

    // 裁判系统获得的功率限制值
    allocatablePower = chassis_cmd_recv.power_limit;
    for (int i = 0; i < 4; i++)
    {   
        // 不用速度闭环，开启开环控制方便直接输出电流值
        DJIMotorOuterLoop(motor[i], OPEN_LOOP);
        DJIMotorCloseLoop(motor[i], OPEN_LOOP);        

        measure = &motor[i]->measure;
        motor_controller = &motor[i]->motor_controller;   

        if (motor[i]->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            limit_vt[i] *= -1;

        // 实时计算速度环PID输出
        limit_vt[i] = PIDCalculate(&motor_controller->speed_PID, measure->speed_aps, limit_vt[i]);
        limit_vt[i] = PIDCalculate(&motor_controller->current_PID, measure->real_current, limit_vt[i]);   

        currentPower[i] = motor_controller->current_PID.Output * TOQUE_COEFFICIENT_3508 * measure->speed_aps * DEGREE_2_RAD + power_manager->k1 * fabs(measure->speed_aps) * DEGREE_2_RAD + \
                        power_manager->k2 * motor_controller->current_PID.Output * TOQUE_COEFFICIENT_3508 * motor_controller->current_PID.Output * TOQUE_COEFFICIENT_3508 + power_manager->k3;
        error[i] = fabs(motor_controller->speed_PID.Ref - measure->speed_aps); 
        sumCurrentPower += currentPower[i]; 


        if (floatEqual(currentPower[i], 0.0f) || currentPower[i] < 0.0f) 
        {
            allocatablePower += -currentPower[i];
        }
        else
        {
            sumPowerRequired += currentPower[i];
            sumError += error[i];
        }                
    }  

    // 当前功率大于最大功率时进行功率分配
    if (sumCurrentPower > chassis_cmd_recv.power_limit)
    {
        // 等比缩放保证每个轮子输出限制功率下最大功率
        if (sumError > ERROR_POWERDISTRIBUTE)
            errorConfidence = 1.0f;
        else if (sumError > PROP_POWERDISTRIBUTE)
            errorConfidence = float_constrain((sumError - PROP_POWERDISTRIBUTE) / (ERROR_POWERDISTRIBUTE - PROP_POWERDISTRIBUTE), 0.0f, 1.0f);
        else
            errorConfidence = 0.0f;
            
        for (int i = 0; i < 4; i++)
        { 
            measure = &motor[i]->measure;
            motor_controller = &motor[i]->motor_controller;              

            if (floatEqual(currentPower[i], 0.0f) || currentPower[i] < 0.0f)
                continue;

            // 功率分配避免起步时无法走直线，同时云台跟随也可以避免此问题
            powerWeight_Error = fabs(motor_controller->speed_PID.Ref - measure->speed_aps) / sumError;
            powerWeight_Prop  = currentPower[i] / sumPowerRequired;
            powerWeight       = errorConfidence * powerWeight_Error + (1.0f - errorConfidence) * powerWeight_Prop;
            // 自旋模式下,适当提高功率占比权重,保证旋转速度
            if (chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE)
                powerWeight = 0.3f * powerWeight_Error + 0.7f * powerWeight_Prop;
            delta             = measure->speed_aps * DEGREE_2_RAD * measure->speed_aps * DEGREE_2_RAD - 
                        4.0f * power_manager->k2 * (power_manager->k1 * fabs(measure->speed_aps) * DEGREE_2_RAD - powerWeight * allocatablePower + power_manager->k3);
            // 求解出力矩并且转换成电流值
            if (floatEqual(delta, 0.0f))  
                limit_vt[i] = (((-measure->speed_aps * DEGREE_2_RAD)) / (2.0f * power_manager->k2)) / TOQUE_COEFFICIENT_3508;
            else if (delta > 0.0f)  
                limit_vt[i] = motor_controller->current_PID.Output > 0.0f ? ((-measure->speed_aps * DEGREE_2_RAD + sqrtf(delta)) / (2.0f * power_manager->k2)) / TOQUE_COEFFICIENT_3508 \
                                : (((-measure->speed_aps * DEGREE_2_RAD - sqrtf(delta))) / (2.0f * power_manager->k2)) / TOQUE_COEFFICIENT_3508;
            else  
                limit_vt[i] = (((-measure->speed_aps * DEGREE_2_RAD)) / (2.0f * power_manager->k2)) / TOQUE_COEFFICIENT_3508;
        }                
    }  

    // 输出功率限制后电流值
    DJIMotorSetRef(motor_lf, limit_vt[0]);
    DJIMotorSetRef(motor_rf, limit_vt[1]);
    DJIMotorSetRef(motor_lb, limit_vt[2]);
    DJIMotorSetRef(motor_rb, limit_vt[3]);           
}

#define RPM_2_VECTOR (ANGLE_2_RPM_PER_MIN * RPM_2_WHEEL_VECTOR)
/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
/**
 * @brief 根据四轮速度反馈,逆运动学估算底盘实际速度
 * @note  麦轮正运动学(A^T*A)^−1*A^T 伪逆解得:
 *        vx = (-lf - rf + lb + rb) / 4
 *        vy = (-lf + rf - lb + rb) / 4
 *        wz = -(lf + rf + lb + rb) / (4*C)  (C为轮中心距,单位rad)
 *        结果存储到 chassis_feedback_data
 */
static void EstimateSpeed()
{
    float half_wb = chassis_config->wheel_measure.wheel_base / 2.0f;   // 半轴距 mm
    float half_tw = chassis_config->wheel_measure.track_width / 2.0f;  // 半轮距 mm
    float reduction = chassis_config->wheel_measure.reduction_ratio_wheel;
    float radius = chassis_config->wheel_measure.radius_wheel * MM_2_M; // 轮半径 m
    float center_dist_m = (half_wb + half_tw) * MM_2_M;                 // 轮中心距 m

    // 获取四轮转速(°/s at motor, 考虑全部REVERSE后取反)
    float vt_lf = -motor_lf->measure.speed_aps;
    float vt_rf = -motor_rf->measure.speed_aps;
    float vt_lb = -motor_lb->measure.speed_aps;
    float vt_rb = -motor_rb->measure.speed_aps;

    // 轮系线速度 (m/s)
    float wheel_lf = vt_lf / reduction * DEGREE_2_RAD * radius;
    float wheel_rf = vt_rf / reduction * DEGREE_2_RAD * radius;
    float wheel_lb = vt_lb / reduction * DEGREE_2_RAD * radius;
    float wheel_rb = vt_rb / reduction * DEGREE_2_RAD * radius;

    // 正运动学伪逆解
    chassis_feedback_data.real_vx = (-wheel_lf - wheel_rf + wheel_lb + wheel_rb) / 4.0f;
    chassis_feedback_data.real_vy = (-wheel_lf + wheel_rf - wheel_lb + wheel_rb) / 4.0f;

    // wz(rad/s) = -sum(tangential) / (4 * center_dist)
    // 转换为 °/s
    float wz_rad = -(wheel_lf + wheel_rf + wheel_lb + wheel_rb) / (4.0f * center_dist_m);
    chassis_feedback_data.real_wz = wz_rad * RAD_2_DEGREE;
}
/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }
    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
        chassis_cmd_recv.wz = chassis_cmd_recv.wz / rpm_2_wheel_vector;
        break;     
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,平滑死区+分段反馈+前馈
    {
        static FollowDebugData_t follow_dbg;
        chassis_cmd_recv.wz = ChassisFollowControl(
            chassis_cmd_recv.offset_angle,
            chassis_cmd_recv.gimbal_yaw_rate,
            &follow_dbg
        );

        // VOFA+ 调试输出(100Hz,每2个周期发一次)
        static uint8_t vofa_div = 0;
        if (++vofa_div >= 2)
        {
            vofa_div = 0;
            follow_dbg.chassis_wz_actual = chassis_feedback_data.real_wz;
            follow_dbg.motor_current_max = GetChassisMaxMotorCurrent();
            VOFA_Send((float *)&follow_dbg, VOFA_CHANNEL_COUNT);
        }
        break;
    }
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;使用遥控器wz输入动态调节旋转速度
    {
        // 将遥控器wz(约±2.5 rad/s量程)映射到电机旋转速度单位
        // 与FOLLOW模式一致,直接使用电机RPM单位,不经过rpm_2_wheel_vector缩放
        // 映射关系: 摇杆满量程→ROTATE_WZ_MAX, 中位死区→ROTATE_WZ_DEFAULT
        static float rc_wz_ratio;
        rc_wz_ratio = chassis_cmd_recv.wz / 2.5f; // 归一化到±1 (2.5为遥控器wz满量程)
        if (fabs(rc_wz_ratio) < 0.15f)
            chassis_cmd_recv.wz = (chassis_cmd_recv.wz >= 0.0f ? 1.0f : -1.0f) * ROTATE_WZ_DEFAULT;
        else
            chassis_cmd_recv.wz = rc_wz_ratio * ROTATE_WZ_MAX;
        VAL_LIMIT(chassis_cmd_recv.wz, -ROTATE_WZ_MAX, ROTATE_WZ_MAX);

        // 累积自旋相位: 通过offset_angle的变化量检测实际底盘旋转
        // offset_angle = gimbal_yaw - chassis_yaw, gimbal被IMU稳定在场地系
        // 故 delta(offset_angle) ≈ -chassis_rotation, 取反即得底盘旋转增量
        static float delta_offset;
        delta_offset = chassis_cmd_recv.offset_angle - last_offset_angle;
        if (delta_offset > 180.0f)  delta_offset -= 360.0f;
        if (delta_offset < -180.0f) delta_offset += 360.0f;
        spin_phase_accum -= delta_offset; // 累加底盘旋转角度
        while (spin_phase_accum > 180.0f)  spin_phase_accum -= 360.0f;
        while (spin_phase_accum < -180.0f) spin_phase_accum += 360.0f;

        last_spin_wz = chassis_cmd_recv.wz;
        break;
    }
    default:
        break;
    }

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    // 自旋模式: offset_angle因底盘旋转而剧烈变化,叠加自旋累积相位抵消底盘转动,
    //           使得vx/vy在场地坐标系中保持稳定方向,避免平动分量被旋转"甩散"
    static float sin_theta, cos_theta, transform_angle;
    transform_angle = chassis_cmd_recv.offset_angle;
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE)
        transform_angle += spin_phase_accum;
    cos_theta = arm_cos_f32(transform_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(transform_angle * DEGREE_2_RAD);
    chassis_vx = (chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta) / rpm_2_wheel_vector;
    chassis_vy = (chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta) / rpm_2_wheel_vector;
   
    // 根据控制模式进行正运动学解算,计算底盘输出
    ChassisCalculate();

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    EstimateSpeed();

    // 保存本周期offset_angle,用于自旋模式的底盘旋转相位检测
    last_offset_angle = chassis_cmd_recv.offset_angle;

// #ifdef ONE_BOARD
//     // 根据电机状态反馈是否离线
//     DJIMotorIsOnline(motor_lf);
//     DJIMotorIsOnline(motor_rf);
//     DJIMotorIsOnline(motor_lb);
//     DJIMotorIsOnline(motor_rb);
//     // 反馈电机离线数量
//     if (motor_lf->online_flag || motor_rf->online_flag || motor_lb->online_flag || motor_rb->online_flag)
//     {
//         chassis_feedback_data.motor_offline_count = motor_lf->online_flag + motor_rf->online_flag + motor_lb->online_flag + motor_rb->online_flag;  
//         chassis_feedback_data.motor_state = CHASSIS_MOTOR_OFFLINE;
//     }
//     else
//     {
//         chassis_feedback_data.motor_offline_count = 0;
//         chassis_feedback_data.motor_state = CHASSIS_MOTOR_ONLINE;
//     }
// #endif

#ifdef CHASSIS_BOARD
    ;
#endif // CHASSIS_BOARD

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}
#endif 
