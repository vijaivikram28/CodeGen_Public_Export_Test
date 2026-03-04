
#include "bms_addr_assign.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* Local definitions and types */
#ifndef BMS_AA_LOG
/* Define macro for minimal logging if required; can be overridden */
#define BMS_AA_LOG(msg)    (void)(msg)
#endif

/* CAN BSW API prototypes (implemented here as simple wrappers using HAL).
   These are kept static to this module and provide the required
   primitives for transmission / reception of extended-ID CAN frames. */

/* External CAN peripheral handle provided by application (declare in cfg) */
extern CAN_HandleTypeDef hcan_bms_aa; /* must be provided by system */

/* Transmit an extended CAN frame. Returns 0 on success, non-zero on error. */
static int32_t BMS_AA_CAN_TransmitExt(uint32_t ext_id, const uint8_t * data, uint8_t len);

/* Receive an extended CAN frame with timeout (ms). Returns 0 on success, non-zero on timeout/error.
   Populates ext_id, data and len_out. */
static int32_t BMS_AA_CAN_ReceiveExt(uint32_t * ext_id, uint8_t * data, uint8_t * len_out, uint32_t timeout_ms);

/* Helper to configure RX filter to accept all extended frames (simple implementation). */
static int32_t BMS_AA_CAN_ConfigFilterAll(void);

/* Internal helper functions */
static void BMS_AA_BuildAssignToken(uint8_t candidate_addr, uint8_t token_buffer[8u]);
static void BMS_AA_BuildAssignConfirm(uint8_t assigned_addr, uint8_t confirm_buffer[8u]);

/* Internal state storage */
static BMS_AA_Assigned_t g_assigned_list[MAX_RACKS];
static uint8_t g_assigned_count = 0u;

/* Public API implementation */

BMS_AA_Result_t BMS_AA_Init(void)
{
    uint32_t i;

    /* Validate CAN handle pointer */
    if (&hcan_bms_aa == NULL)
    {
        return BMS_AA_ERROR;
    }

    /* Initialize assigned list */
    for (i = 0u; i < (uint32_t)MAX_RACKS; ++i)
    {
        g_assigned_list[i].address = DEFAULT_ADDRESS;
        (void)memset(g_assigned_list[i].hw_id, 0u, sizeof(g_assigned_list[i].hw_id));
    }
    g_assigned_count = 0u;

    /* Configure CAN filter to accept all extended frames for discovery */
    if (BMS_AA_CAN_ConfigFilterAll() != 0)
    {
        return BMS_AA_ERROR;
    }

    return BMS_AA_OK;
}

BMS_AA_Result_t BMS_AA_AssignAddresses(uint8_t expected_racks,
                                       BMS_AA_Assigned_t assigned_out[MAX_RACKS],
                                       uint8_t * out_count)
{
    uint8_t candidate;
    uint8_t assigned_local = 0u;
    uint8_t retry;
    uint8_t resp_buffer[8u];
    uint8_t resp_len;
    uint32_t resp_ext_id;
    uint32_t start_time_ms;
    uint32_t now_ms;
    int32_t tx_res;
    int32_t rx_res;
    uint8_t responses_collected;
    uint8_t resp_hw_ids[8u][8u]; /* up to 8 simultaneous for collision tracking (bounded) */
    uint8_t resp_hw_count;
    uint8_t i;
    uint8_t j;

    if (assigned_out == NULL)
    {
        return BMS_AA_INVALID_PARAM;
    }
    if (out_count == NULL)
    {
        return BMS_AA_INVALID_PARAM;
    }
    if ((expected_racks == 0u) || (expected_racks > MAX_RACKS))
    {
        return BMS_AA_INVALID_PARAM;
    }

    /* Reset local state and internal list */
    (void)BMS_AA_Init();

    /* Token-based assignment:
       For candidate addresses from 1..MAX_RACKS assign tokens and collect responses.
       If exactly one unassigned rack responds, mark assigned. If multiple respond, attempt
       collision resolution with retries (simple randomized backoff by varying token content).
    */

    for (candidate = 1u; (candidate <= (uint8_t)MAX_RACKS) && (assigned_local < expected_racks); ++candidate)
    {
        /* For each candidate address, attempt up to MAX_RETRY_ATTEMPTS */
        for (retry = 0u; retry < MAX_RETRY_ATTEMPTS; ++retry)
        {
            uint8_t token[8u];
            /* Build token - includes candidate address and retry counter for jitter */
            BMS_AA_BuildAssignToken(candidate, token);

            /* Transmit token as extended CAN frame with ASSIGN_TOKEN_EXT_ID */
            tx_res = BMS_AA_CAN_TransmitExt(ASSIGN_TOKEN_EXT_ID, token, 8u);
            if (tx_res != 0)
            {
                /* TX failed, try again or bail if fatal */
                BMS_AA_LOG("CAN TX failed for assign token");
                continue;
            }

            /* Collect responses for TIMEOUT_FOR_RACK_MS */
            responses_collected = 0u;
            resp_hw_count = 0u;
            start_time_ms = HAL_GetTick();

            for (;;)
            {
                now_ms = HAL_GetTick();
                if ((now_ms - start_time_ms) >= TIMEOUT_FOR_RACK_MS)
                {
                    /* timeout window expired */
                    break;
                }

                rx_res = BMS_AA_CAN_ReceiveExt(&resp_ext_id, resp_buffer, &resp_len, 10u);
                if (rx_res != 0)
                {
                    /* No frame received within small chunk timeout, continue waiting until overall timeout */
                    continue;
                }

                /* Only process replies with expected response ID */
                if (resp_ext_id != ASSIGN_RESPONSE_EXT_ID)
                {
                    /* ignore unrelated frames */
                    continue;
                }

                /* Basic validation: expect at least 1 byte (address 0x00) and up to 8 bytes hw id */
                if (resp_len == 0u)
                {
                    continue;
                }

                /* The responding rack should send its current address (should be DEFAULT_ADDRESS) followed by hw id */
                /* Check first byte equals DEFAULT_ADDRESS to avoid already-assigned racks responding */
                if (resp_buffer[0u] != DEFAULT_ADDRESS)
                {
                    /* ignore unexpected response */
                    continue;
                }

                /* Store hw id (up to 7 bytes following) */
                (void)memcpy(resp_hw_ids[resp_hw_count], &resp_buffer[1u], ((resp_len - 1u) <= 8u) ? (resp_len - 1u) : 8u);
                ++resp_hw_count;
                ++responses_collected;

                /* If more than one response collected, we have a collision */
                if (responses_collected > 1u)
                {
                    break;
                }
            } /* end per-token response collection */

            if (responses_collected == 1u)
            {
                /* Single rack responded -> assign candidate address */
                /* Send confirm frame */
                uint8_t confirm[8u];
                BMS_AA_BuildAssignConfirm(candidate, confirm);
                (void)BMS_AA_CAN_TransmitExt(ASSIGN_CONFIRM_EXT_ID, confirm, 8u);

                /* Record assignment */
                g_assigned_list[assigned_local].address = candidate;
                (void)memcpy(g_assigned_list[assigned_local].hw_id, resp_hw_ids[0u], sizeof(g_assigned_list[assigned_local].hw_id));
                ++assigned_local;

                BMS_AA_LOG("Assigned address to single responder");
                /* Candidate assigned, break retry loop and move to next candidate */
                break;
            }
            else if (responses_collected > 1u)
            {
                /* Collision detected: perform simple collision resolution by retrying token with variation */
                BMS_AA_LOG("Collision detected, retrying token");
                /* Small deterministic delay to allow devices to desynchronize (could be replaced by jitter) */
                HAL_Delay(5u + (retry * 2u));
                /* Continue retry loop */
            }
            else
            {
                /* No response: no unassigned rack accepted this candidate. Move to next candidate. */
                BMS_AA_LOG("No response for candidate, moving on");
                break;
            }
        } /* retry loop */
    } /* candidate loop */

    /* Prepare output */
    for (i = 0u; i < assigned_local; ++i)
    {
        assigned_out[i] = g_assigned_list[i];
    }
    *out_count = assigned_local;

    return BMS_AA_OK;
}

uint8_t BMS_AA_ComputeSuccessRate(uint8_t expected_racks, uint8_t assigned_count)
{
    uint32_t rate;

    if (expected_racks == 0u)
    {
        return 0u;
    }

    rate = ((uint32_t)assigned_count * 100u) / (uint32_t)expected_racks;

    if (rate > 100u)
    {
        rate = 100u;
    }

    return (uint8_t)rate;
}

/* ===== Internal helper implementations ===== */

static void BMS_AA_BuildAssignToken(uint8_t candidate_addr, uint8_t token_buffer[8u])
{
    uint8_t i;
    /* token layout:
       [0] = 0xAA (command marker)
       [1] = candidate_addr
       [2] = retry/jitter (LSB of tick)
       [3..7] = reserved / zero
    */
    token_buffer[0u] = 0xAAu;
    token_buffer[1u] = candidate_addr;
    token_buffer[2u] = (uint8_t)(HAL_GetTick() & 0xFFu);
    for (i = 3u; i < 8u; ++i)
    {
        token_buffer[i] = 0u;
    }
}

static void BMS_AA_BuildAssignConfirm(uint8_t assigned_addr, uint8_t confirm_buffer[8u])
{
    uint8_t i;
    /* confirm layout:
       [0] = 0xCC (confirm marker)
       [1] = assigned_addr
       [2..7] = reserved
    */
    confirm_buffer[0u] = 0xCCu;
    confirm_buffer[1u] = assigned_addr;
    for (i = 2u; i < 8u; ++i)
    {
        confirm_buffer[i] = 0u;
    }
}

/* CAN wrapper implementations using STM32H7 HAL */

/* Configure filter to accept all extended frames for discovery use-case */
static int32_t BMS_AA_CAN_ConfigFilterAll(void)
{
    CAN_FilterTypeDef filter;
    int32_t res;

    /* Basic filter: accept all IDs (both ext and std) on FIFO0 */
    /* Note: This is a simplified implementation. In production, refine filters. */
    filter.FilterBank = 0u;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000u;
    filter.FilterIdLow = 0x0000u;
    filter.FilterMaskIdHigh = 0x0000u;
    filter.FilterMaskIdLow = 0x0000u;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14u;

    if (HAL_CAN_ConfigFilter(&hcan_bms_aa, &filter) != HAL_OK)
    {
        res = -1;
    }
    else
    {
        /* Start CAN if not already started */
        if (HAL_CAN_Start(&hcan_bms_aa) != HAL_OK)
        {
            res = -1;
        }
        else
        {
            /* Activate notification for RX FIFO0 message pending if needed by other modules */
            (void)HAL_CAN_ActivateNotification(&hcan_bms_aa, CAN_IT_RX_FIFO0_MSG_PENDING);
            res = 0;
        }
    }

    return res;
}

static int32_t BMS_AA_CAN_TransmitExt(uint32_t ext_id, const uint8_t * data, uint8_t len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t mailbox;
    HAL_StatusTypeDef hal_res;

    if (data == NULL)
    {
        return -1;
    }
    if (len > 8u)
    {
        return -1;
    }

    tx_header.DLC = len;
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.ExtId = ext_id;
    tx_header.TransmitGlobalTime = DISABLE;

    hal_res = HAL_CAN_AddTxMessage(&hcan_bms_aa, &tx_header, (uint8_t *)data, &mailbox);
    if (hal_res != HAL_OK)
    {
        return -1;
    }

    /* Wait for transmission completion with modest timeout */
    const uint32_t start = HAL_GetTick();
    while (HAL_CAN_IsTxMessagePending(&hcan_bms_aa, mailbox) != 0)
    {
        if ((HAL_GetTick() - start) > 50u)
        {
            /* Transmission not completed within 50 ms: consider error */
            return -1;
        }
    }

    return 0;
}

static int32_t BMS_AA_CAN_ReceiveExt(uint32_t * ext_id, uint8_t * data, uint8_t * len_out, uint32_t timeout_ms)
{
    CAN_RxHeaderTypeDef rx_header;
    HAL_StatusTypeDef hal_res;
    uint32_t start;
    (void)rx_header;

    if ((ext_id == NULL) || (data == NULL) || (len_out == NULL))
    {
        return -1;
    }

    start = HAL_GetTick();

    for (;;)
    {
        /* Check if there is a pending message in FIFO0 via HAL macro */
        if (__HAL_CAN_GET_FLAG(&hcan_bms_aa, CAN_FLAG_FMP0) != RESET)
        {
            /* Get message */
            hal_res = HAL_CAN_GetRxMessage(&hcan_bms_aa, CAN_RX_FIFO0, &rx_header, data);
            if (hal_res != HAL_OK)
            {
                return -1;
            }

            /* Only accept extended frames */
            if (rx_header.IDE != CAN_ID_EXT)
            {
                /* skip non-extended */
                continue;
            }

            *ext_id = rx_header.ExtId;
            *len_out = rx_header.DLC;
            return 0;
        }

        /* Timeout check */
        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            return -1; /* timeout */
        }
    }
}


