#include "master_process.h"

#include "daemon.h"
#include "bsp_log.h"
#include "bsp_usb.h"
#include "bsp_dwt.h"

#include <string.h>

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;
static uint8_t *vis_recv_buff;
static uint8_t vision_rx_frame[sizeof(Vision_Recv_s)];
static uint16_t vision_rx_frame_len;
static volatile Vision_Comm_Stats_s vision_comm_stats;
static volatile uint8_t vision_offline_reported;
static volatile uint32_t vision_rx_dwt_count;
static volatile uint32_t vision_rx_generation;

_Static_assert(sizeof(Vision_Recv_s) == 29u, "Vision_Recv_s protocol size changed");
_Static_assert(sizeof(Vision_Send_s) == 43u, "Vision_Send_s protocol size changed");

/* 和 sp_vision_25 一致的 CRC16/X25 */
static uint16_t VisionCRC16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;

    while (len--)
    {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0x8408;
            else
                crc >>= 1;
        }
    }

    return crc;
}

static uint8_t VisionCheckCRC16(const uint8_t *data, uint32_t len)
{
    uint16_t rx_crc = (uint16_t)data[len - 2] | ((uint16_t)data[len - 1] << 8);
    uint16_t calc_crc = VisionCRC16(data, len - 2);
    return (rx_crc == calc_crc);
}

static void VisionOfflineCallback(void *id)
{
    uint32_t primask;
    uint8_t report_offline = 0;

    (void)id;
    primask = __get_PRIMASK();
    __disable_irq();

    /*
     * A valid frame may arrive after DaemonTask decides to invoke this
     * callback. Re-check the daemon while USB interrupts are masked so a
     * freshly received command is not overwritten with mode 0.
     */
    if (vision_daemon_instance != NULL &&
        !DaemonIsOnline(vision_daemon_instance))
    {
        recv_data.mode = 0;
        if (!vision_offline_reported)
        {
            vision_offline_reported = 1;
            report_offline = 1;
        }
    }
    __set_PRIMASK(primask);

    if (report_offline)
        LOGWARNING("[vision] vision offline.");
}

void VisionUpdateTx(uint8_t mode,
                    float q0, float q1, float q2, float q3,
                    float yaw, float yaw_vel,
                    float pitch, float pitch_vel,
                    float bullet_speed, uint16_t bullet_count)
{
    send_data.head[0] = 'S';
    send_data.head[1] = 'P';
    send_data.mode = mode;

    send_data.q[0] = q0;
    send_data.q[1] = q1;
    send_data.q[2] = q2;
    send_data.q[3] = q3;

    send_data.yaw = yaw;
    send_data.yaw_vel = yaw_vel;
    send_data.pitch = pitch;
    send_data.pitch_vel = pitch_vel;
    send_data.bullet_speed = bullet_speed;
    send_data.bullet_count = bullet_count;
}

static void VisionRxResync(void)
{
    for (uint16_t i = 1; i + 1 < sizeof(vision_rx_frame); i++)
    {
        if (vision_rx_frame[i] == 'S' && vision_rx_frame[i + 1] == 'P')
        {
            vision_rx_frame_len = sizeof(vision_rx_frame) - i;
            memmove(vision_rx_frame, vision_rx_frame + i, vision_rx_frame_len);
            return;
        }
    }

    if (vision_rx_frame[sizeof(vision_rx_frame) - 1] == 'S')
    {
        vision_rx_frame[0] = 'S';
        vision_rx_frame_len = 1;
    }
    else
    {
        vision_rx_frame_len = 0;
    }
}

static void DecodeVision(uint16_t recv_len)
{
    for (uint16_t i = 0; i < recv_len; i++)
    {
        uint8_t byte = vis_recv_buff[i];

        if (vision_rx_frame_len == 0)
        {
            if (byte == 'S')
            {
                vision_rx_frame[0] = byte;
                vision_rx_frame_len = 1;
            }
            continue;
        }

        if (vision_rx_frame_len == 1)
        {
            if (byte == 'P')
            {
                vision_rx_frame[1] = byte;
                vision_rx_frame_len = 2;
            }
            else if (byte != 'S')
            {
                vision_rx_frame_len = 0;
            }
            continue;
        }

        vision_rx_frame[vision_rx_frame_len++] = byte;

        if (vision_rx_frame_len == sizeof(vision_rx_frame))
        {
            if (VisionCheckCRC16(vision_rx_frame, sizeof(vision_rx_frame)))
            {
                memcpy(&recv_data, vision_rx_frame, sizeof(recv_data));
                vision_rx_dwt_count = DWT_GetCycleCount();
                vision_rx_generation++;
                vision_comm_stats.rx_ok_count++;
                vision_offline_reported = 0;
                DaemonReload(vision_daemon_instance);
                vision_rx_frame_len = 0;
            }
            else
            {
                vision_comm_stats.rx_crc_error_count++;
                VisionRxResync();
            }
        }
    }
}

/**
 * @brief USB reset 回调。
 *        只清理接收状态和控制模式,不再维护 TX 双缓冲状态。
 */
static void VisionUsbResetCallback(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    vision_rx_frame_len = 0;
    recv_data.mode = 0;
    vision_comm_stats.usb_reset_count++;
    __set_PRIMASK(primask);
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    (void)_handle;

    memset(&recv_data, 0, sizeof(recv_data));
    memset(&send_data, 0, sizeof(send_data));
    memset(vision_rx_frame, 0, sizeof(vision_rx_frame));
    memset((void *)&vision_comm_stats, 0, sizeof(vision_comm_stats));
    vision_rx_frame_len = 0;
    vision_offline_reported = 0;
    vision_rx_dwt_count = 0;
    vision_rx_generation = 0;

    USB_Init_Config_s conf = {
        .tx_cbk = NULL,  // 不注册 TX complete,不在回调中续发
        .rx_cbk = DecodeVision,
        .reset_cbk = VisionUsbResetCallback,
    };

    vis_recv_buff = USBInit(conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback,
        .owner_id = NULL,
        .reload_count = 5,
    };

    vision_daemon_instance = DaemonRegister(&daemon_conf);
    return &recv_data;
}

uint8_t VisionGetRxSnapshot(Vision_Recv_s *snapshot)
{
    uint32_t primask;

    if (snapshot == NULL)
        return 0;

    /*
     * recv_data is replaced in the USB OUT interrupt. Copying one complete
     * 29-byte frame with interrupts masked prevents the control task from
     * observing fields from two different frames.
     */
    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(snapshot, &recv_data, sizeof(*snapshot));
    __set_PRIMASK(primask);

    return 1;
}

uint8_t VisionGetRxSnapshotWithMeta(Vision_Rx_Snapshot_s *snapshot)
{
    uint32_t primask;

    if (snapshot == NULL)
        return 0;

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(&snapshot->data, &recv_data, sizeof(snapshot->data));
    snapshot->rx_dwt_count = vision_rx_dwt_count;
    snapshot->generation = vision_rx_generation;
    __set_PRIMASK(primask);

    return 1;
}

/**
 * @brief CDC 忙就丢帧,不做 pending 排队。
 *        和 Restar2027Sentinel 行为一致:
 *        build packet → USBTransmit → USBD_BUSY 就丢弃本帧。
 */
uint8_t VisionSend(void)
{
    Vision_Send_s tx_pkt = send_data;
    uint8_t status;

    tx_pkt.head[0] = 'S';
    tx_pkt.head[1] = 'P';
    tx_pkt.crc16 = VisionCRC16((const uint8_t *)&tx_pkt, sizeof(Vision_Send_s) - 2);

    status = USBTransmit((const uint8_t *)&tx_pkt, sizeof(Vision_Send_s));

    if (status == USBD_OK)
        vision_comm_stats.tx_ok_count++;
    else if (status == USBD_BUSY)
        vision_comm_stats.tx_busy_drop_count++;
    else
        vision_comm_stats.tx_fail_count++;

    return status;
}

const volatile Vision_Comm_Stats_s *VisionGetCommStats(void)
{
    return &vision_comm_stats;
}
