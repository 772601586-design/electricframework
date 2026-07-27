#include "bsp_can.h"
#include "main.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

/* can instance ptrs storage, used for recv callback */
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx;

/* ----------------two static function called by CANRegister()-------------------- */

static void CANAddFilter(CANInstance *_instance)
{
    CAN_FilterTypeDef can_filter_conf = {0};
    static uint8_t can1_filter_idx = 0, can2_filter_idx = 14;
    uint32_t ext_id = _instance->ext_id;

    can_filter_conf.FilterFIFOAssignment = (_instance->rx_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
    can_filter_conf.SlaveStartFilterBank = 14;
    can_filter_conf.FilterBank = _instance->can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++);
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;

    if (_instance->txconf.IDE == CAN_ID_EXT)
    {
        can_filter_conf.FilterMode = CAN_FILTERMODE_IDMASK;
        can_filter_conf.FilterScale = CAN_FILTERSCALE_32BIT;
        can_filter_conf.FilterIdHigh = (uint16_t)((ext_id >> 13) & 0xFFFF);
        can_filter_conf.FilterIdLow = (uint16_t)(((ext_id << 3) & 0xFFF8) | CAN_ID_EXT);
        can_filter_conf.FilterMaskIdHigh = 0xFFFF;
        can_filter_conf.FilterMaskIdLow = 0xFFFC;
    }
    else
    {
        // Standard IDs use exact-match filtering to keep unrelated traffic out of the RX FIFO.
        can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;
        can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;
        can_filter_conf.FilterIdHigh = (uint16_t)(_instance->rx_id << 5);
        can_filter_conf.FilterIdLow = (uint16_t)(_instance->rx_id << 5);
        can_filter_conf.FilterMaskIdHigh = 0x0000;
        can_filter_conf.FilterMaskIdLow = 0x0000;
    }

    HAL_CAN_ConfigFilter(_instance->can_handle, &can_filter_conf);
}

static void CANServiceInit()
{
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/* ----------------------- two extern callable function -----------------------*/

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (!idx)
    {
        CANServiceInit();
        LOGINFO("[bsp_can] CAN Service Init");
    }
    if (idx >= CAN_MX_REGISTER_CNT)
    {
        while (1)
            LOGERROR("[bsp_can] CAN instance exceeded MAX num, consider balance the load of CAN bus");
    }
    for (size_t i = 0; i < idx; i++)
    {
        if (can_instance[i]->tx_id == config->tx_id && can_instance[i]->rx_id == config->rx_id &&
            can_instance[i]->can_handle == config->can_handle)
        {
            while (1)
                LOGERROR("[}bsp_can] CAN id crash ,tx [%d] or rx [%d] already registered", &config->tx_id, &config->rx_id);
        }
    }

    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance));
    memset(instance, 0, sizeof(CANInstance));
    instance->txconf.StdId = config->tx_id;
    instance->txconf.IDE = config->mode;
    instance->txconf.RTR = CAN_RTR_DATA;
    instance->txconf.DLC = 0x08;

    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->ext_id = config->ext_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;

    CANAddFilter(instance);
    can_instance[idx++] = instance;
    return instance;
}

uint8_t CANTransmit(CANInstance *_instance, float timeout)
{
    uint32_t TxMailbox;
    static uint32_t busy_count;
    static volatile float wait_time __attribute__((unused));
    float dwt_start = DWT_GetTimeline_ms();
    while (HAL_CAN_GetTxMailboxesFreeLevel(_instance->can_handle) == 0)
    {
        if (DWT_GetTimeline_ms() - dwt_start > timeout)
        {
            LOGWARNING("[bsp_can] CAN MAILbox full! failed to add msg to mailbox. Cnt [%d]", busy_count);
            busy_count++;
            return 0;
        }
    }
    wait_time = DWT_GetTimeline_ms() - dwt_start;
    if (HAL_CAN_AddTxMessage(_instance->can_handle, &_instance->txconf, _instance->tx_buff, &TxMailbox))
    {
        LOGWARNING("[bsp_can] CAN bus BUS! cnt:%d", busy_count);
        busy_count++;
        return 0;
    }
    return 1;
}

void CANSetDLC(CANInstance *_instance, uint8_t length)
{
    if (length > 8 || length == 0)
        while (1)
            LOGERROR("[bsp_can] CAN DLC error! check your code or wild pointer");
    _instance->txconf.DLC = length;
}

/* -----------------------belows are callback definitions--------------------------*/

static void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
{
    static CAN_RxHeaderTypeDef rxconf;
    uint8_t can_rx_buff[8];
    uint8_t is_target_std;
    uint8_t is_target_ext;

    while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox))
    {
        HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff);
        for (size_t i = 0; i < idx; ++i)
        {
            is_target_std = _hcan == can_instance[i]->can_handle &&
                            rxconf.IDE == CAN_ID_STD &&
                            rxconf.StdId == can_instance[i]->rx_id;
            is_target_ext = _hcan == can_instance[i]->can_handle &&
                            rxconf.IDE == CAN_ID_EXT &&
                            rxconf.ExtId == can_instance[i]->ext_id;

            if (!(is_target_std || is_target_ext))
                continue;

            if (can_instance[i]->can_module_callback != NULL)
            {
                can_instance[i]->rx_len = rxconf.DLC;
                memcpy(can_instance[i]->rx_buff, can_rx_buff, rxconf.DLC);
                can_instance[i]->can_module_callback(can_instance[i]);
            }

            rxconf.ExtId = 0;
            rxconf.StdId = 0;
            return;
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO1);
}
