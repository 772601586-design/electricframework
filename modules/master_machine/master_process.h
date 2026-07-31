#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include <stdint.h>

#define VISION_RECV_SIZE 64u
#define VISION_SEND_SIZE 64u

#pragma pack(push, 1)

/* 上位机 -> 下位机，和 sp_vision_25 的 VisionToGimbal 对齐 */
typedef struct
{
    uint8_t head[2];   // 'S', 'P'
    uint8_t mode;      // 0: no target, 1: track, 2: track with fire advice
    float yaw;         // rad, left positive, wrapped to [-pi, pi]
    float yaw_vel;     // rad/s
    float yaw_acc;     // rad/s^2
    float pitch;       // rad, down positive
    float pitch_vel;   // rad/s
    float pitch_acc;   // rad/s^2
    uint16_t crc16;
} Vision_Recv_s;

/* 下位机 -> 上位机，和 sp_vision_25 的 GimbalToVision 对齐 */
typedef struct
{
    uint8_t head[2];   // 'S', 'P'
    uint8_t mode;      // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
    float q[4];        // w, x, y, z in the vision frame: x forward, y left, z up
    float yaw;         // rad, left positive, wrapped to [-pi, pi]
    float yaw_vel;     // rad/s
    float pitch;       // rad, down positive
    float pitch_vel;   // rad/s
    float bullet_speed;
    uint16_t bullet_count;
    uint16_t crc16;
} Vision_Send_s;

#pragma pack(pop)

typedef struct
{
    uint32_t tx_ok_count;
    uint32_t tx_queued_count;
    uint32_t tx_pending_replace_count;
    uint32_t tx_complete_count;
    uint32_t tx_busy_drop_count;
    uint32_t tx_fail_count;
    uint32_t rx_ok_count;
    uint32_t rx_crc_error_count;
    uint32_t usb_reset_count;
} Vision_Comm_Stats_s;

typedef struct
{
    Vision_Recv_s data;
    uint32_t rx_dwt_count;
    uint32_t generation;
} Vision_Rx_Snapshot_s;

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);
uint8_t VisionGetRxSnapshot(Vision_Recv_s *snapshot);
uint8_t VisionGetRxSnapshotWithMeta(Vision_Rx_Snapshot_s *snapshot);
uint8_t VisionSend(void);
const volatile Vision_Comm_Stats_s *VisionGetCommStats(void);

/* 每个控制周期把下位机状态塞给上位机 */
void VisionUpdateTx(uint8_t mode,
                    float q0, float q1, float q2, float q3,
                    float yaw, float yaw_vel,
                    float pitch, float pitch_vel,
                    float bullet_speed, uint16_t bullet_count);

#endif
