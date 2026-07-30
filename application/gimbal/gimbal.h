#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

typedef struct
{
    uint32_t vision_generation;
    uint32_t command_age_us;
    uint8_t vision_active;
    uint8_t command_stale;
    float yaw_ref_deg;
    float yaw_speed_ff_rad_s;
    float pitch_ref_deg;
    float pitch_speed_ff_rad_s;
} Gimbal_Trajectory_Debug_s;

/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台任务
 * 
 */
void GimbalTask();

/**
 * @brief 在电机PID计算前以1kHz重建云台目标轨迹
 */
void GimbalFastTask(void);

const volatile Gimbal_Trajectory_Debug_s *GimbalGetTrajectoryDebug(void);

#endif // GIMBAL_H
