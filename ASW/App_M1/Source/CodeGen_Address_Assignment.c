/**
 * @file CodeGen_Address_Assignment.c
 * @brief Dynamic Address Assignment Implementation for BMS CAN Communication
 */

#include "CodeGen_Address_Assignment.h"
#include "CodeGen_CAN_operations.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* Global Address Management Variables */
BMS_AddressManager g_bms_address_manager = {0};
BMS_AddressControl g_bms_address_control = {0};

/* Internal Function Prototypes */
static HAL_StatusTypeDef prv_send_addressing_request(uint8_t proposed_address);
static HAL_StatusTypeDef prv_process_addressing_response(uint32_t can_id, uint8_t* payload);
static void prv_update_address_bitmap(uint8_t address);

bool is_init=0;

/**
 * @brief Initialize Address Assignment Process
 * @return HAL Status of initialization
 */
HAL_StatusTypeDef BMS_InitAddressAssignment(void) {
    /* Reset Address Management Structures */
    if (is_init==0)
    {
      memset(&g_bms_address_manager, 0, sizeof(BMS_AddressManager));
      is_init=1;
    }
    memset(&g_bms_address_control, 0, sizeof(BMS_AddressControl));

    /* Initialize State Machine */
    g_bms_address_control.state = BMS_ADDR_STATE_INIT;
    g_bms_address_control.retry_count = 0;

    /* Perform Initial Dynamic Addressing */
    return BMS_PerformDynamicAddressing();
}

/**
 * @brief Perform Dynamic Address Assignment
 * @return HAL Status of addressing process
 */
HAL_StatusTypeDef BMS_PerformDynamicAddressing(void) {
    HAL_StatusTypeDef status = HAL_ERROR;

    /* Validate Current State */
    if (g_bms_address_control.state != BMS_ADDR_STATE_INIT) {
        return HAL_ERROR;
    }

    /* Attempt Address Assignment for Range */
    for (uint8_t proposed_address = BMS_ADDRESS_RANGE_START;
         proposed_address < BMS_ADDRESS_RANGE_END;
         proposed_address++) {

        /* Check Address Availability */
        if (!BMS_IsAddressAvailable(proposed_address)) {
            continue;
        }

           /* Check for Duplicate Address */
        if (g_bms_address_control.proposed_address == proposed_address) {
            /* Return Duplicate Error */
            return HAL_ERROR;
        }

        /* Generate Addressing Token */
        g_bms_address_control.addressing_token = BMS_GenerateAddressingToken();


        /* Update Control State */
        g_bms_address_control.proposed_address = proposed_address;
        g_bms_address_control.state = BMS_ADDR_STATE_REQUESTING;

        /* Send Addressing Request */
        status = prv_send_addressing_request(proposed_address);
        if (status != HAL_OK) {
            break;
        }

    }

    return status;
}

/**
 * @brief Send Addressing Request Message
 * @param proposed_address Proposed rack address
 * @return HAL Status of message transmission
 */
static HAL_StatusTypeDef prv_send_addressing_request(uint8_t proposed_address) {
    uint8_t payload[BMS_ADDRESSING_MSG_LENGTH] = {0};

    /* Populate Addressing Request Payload */
    payload[BMS_ADDRESSING_ADDR_INDEX] = proposed_address;

    /* Compute CRC for Validation */
    uint16_t crc = BMS_ComputeAddressCRC(payload, BMS_ADDRESSING_MSG_LENGTH);
    g_bms_address_control.address_crc = crc;
    memcpy(&payload[BMS_ADDRESSING_CRC_INDEX], &crc, sizeof(crc));

    payload[BMS_ADDRESSING_TOKEN_INDEX] = g_bms_address_control.addressing_token;

    /* Construct CAN Message ID */
    uint32_t can_id = BMS_ADDRESSING_REQUEST_BASE_ID | proposed_address;

    /* Transmit Addressing Request */
    BMS_CAN_Transmit(&bms_can_config, can_id, payload, BMS_ADDRESSING_MSG_LENGTH);
    HAL_Delay(10);

    /* Receive Addressing Response */
    uint32_t rx_length = BMS_ADDRESSING_MSG_LENGTH;
    can_id= BMS_ADDRESSING_RESPONSE_BASE_ID | proposed_address;
    if (BMS_CAN_Receive(&bms_can_config, &can_id, payload, &rx_length) == HAL_OK)
    {
        /* Process Addressing Response */
        BMS_HandleAddressingResponse(can_id, payload);
    }
    return HAL_ERROR;
}

/**
 * @brief Handle Addressing Response
 * @param can_id Received CAN Message ID
 * @param payload Received payload data
 * @return HAL Status of response processing
 */
HAL_StatusTypeDef BMS_HandleAddressingResponse(uint32_t can_id, uint8_t* payload) {
    HAL_StatusTypeDef status = HAL_ERROR;

    /* Validate Input Parameters */
    if (payload == NULL) {
        return HAL_ERROR;
    }

    /* Process Addressing Response */
    status = prv_process_addressing_response(can_id, payload);

    return status;
}

/**
 * @brief Internal Process for Addressing Response
 * @param can_id Received CAN Message ID
 * @param payload Received payload data
 * @return HAL Status of response processing
 */
static HAL_StatusTypeDef prv_process_addressing_response(uint32_t can_id, uint8_t* payload) {
    uint8_t received_address = can_id & 0x7F;  // Extract address from CAN ID
    uint8_t response_token = payload[BMS_ADDRESSING_TOKEN_INDEX];
    uint8_t response_crc = payload[BMS_ADDRESSING_CRC_INDEX];

    /* Validate Response Token */
    if (response_token != g_bms_address_control.addressing_token) {
        return HAL_ERROR;
    }

    if (g_bms_address_control.addressing_token == 0 || g_bms_address_control.address_crc != response_crc) {
        return HAL_ERROR;
    }
 
     
     /* Check for Address Collision */
     if (received_address != g_bms_address_control.proposed_address ) {
       return HAL_ERROR;
   }

    /* Check for Address Collision */
    if (!BMS_IsAddressAvailable(received_address)) {
        g_bms_address_control.state = BMS_ADDR_STATE_COLLISION;
        return BMS_ResolveAddressCollision(received_address);
    }

    /* Update Address Bitmap */
    prv_update_address_bitmap(received_address);

    /* Update Address Control State */
    g_bms_address_control.proposed_address = received_address;
    g_bms_address_control.state = BMS_ADDR_STATE_ASSIGNED;

    return HAL_OK;
}

/**
 * @brief Update Address Bitmap
 * @param address Assigned address
 */
static void prv_update_address_bitmap(uint8_t address) {
    uint8_t bitmap_index = address / 8;
    uint8_t bitmap_bit = address % 8;

    g_bms_address_manager.rack_bitmap[bitmap_index] |= (1 << bitmap_bit);
    g_bms_address_manager.assigned_addresses[address] = address;
}

/**
 * @brief Resolve Address Collision
 * @param conflicting_address Address with collision
 * @return HAL Status of collision resolution
 */
HAL_StatusTypeDef BMS_ResolveAddressCollision(uint8_t conflicting_address) {
    /* Implement Exponential Backoff */
    uint32_t backoff_delay = (conflicting_address * 10) * (1 << g_bms_address_control.retry_count);

    /* Increment Retry Count */
    g_bms_address_control.retry_count++;

    /* Check Maximum Retry Limit */
    if (g_bms_address_control.retry_count >= BMS_ADDRESSING_RETRY_ATTEMPTS) {
        g_bms_address_control.state = BMS_ADDR_STATE_ERROR;
        return HAL_ERROR;
    }

    /* Delay and Retry Addressing */
    HAL_Delay(backoff_delay);
    return BMS_PerformDynamicAddressing();
}

/**
 * @brief Check Address Availability
 * @param address Address to check
 * @return Boolean indicating address availability
 */
bool BMS_IsAddressAvailable(uint8_t address) {
    uint8_t bitmap_index = address / 8;
    uint8_t bitmap_bit = address % 8;

    return !(g_bms_address_manager.rack_bitmap[bitmap_index] & (1 << bitmap_bit));
}

/**
 * @brief Generate Addressing Token
 * @return Generated token value
 */
uint8_t BMS_GenerateAddressingToken(void) {
    /* Use Hardware Tick as Entropy Source */
    return (uint8_t)((HAL_GetTick() & 0x3F)+1);
}

/**
 * @brief Compute CRC for Address Validation
 * @param data Input data buffer
 * @param length Length of data
 * @return Computed CRC value
 */
uint16_t BMS_ComputeAddressCRC(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++) {
        crc ^= (data[i] << 8);
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
    }

    return crc;
}
