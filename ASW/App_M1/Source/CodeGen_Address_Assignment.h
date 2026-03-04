/**
 * @file CodeGen_BMS_Addressing.h
 * @brief Dynamic Address Assignment Header for BMS CAN Communication
 */

#ifndef BMS_ADDRESSING_H
#define BMS_ADDRESSING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "CodeGen_BMS_config.h"
#include <stdbool.h>

/* Address Assignment CAN Message Identifiers */
#define BMS_ADDRESSING_BROADCAST_ID     (0x18FFF000UL)
#define BMS_ADDRESSING_REQUEST_BASE_ID  (0x18FFF000UL)
#define BMS_ADDRESSING_RESPONSE_BASE_ID (0x18FFF800UL)

/* Address Assignment Message Formats */
#define BMS_ADDRESSING_MSG_LENGTH       (8U)
#define BMS_ADDRESSING_TOKEN_INDEX      (0U)
#define BMS_ADDRESSING_ADDR_INDEX       (1U)
#define BMS_ADDRESSING_CRC_INDEX        (2U)

/* Address Assignment States */
typedef enum {
    BMS_ADDR_STATE_INIT = 0,
    BMS_ADDR_STATE_REQUESTING,
    BMS_ADDR_STATE_ASSIGNED,
    BMS_ADDR_STATE_COLLISION,
    BMS_ADDR_STATE_ERROR
} BMS_AddressState;

/* Address Assignment Control Structure */
typedef struct {
    BMS_AddressState   state;
    uint8_t            proposed_address;
    uint32_t           addressing_token;
    uint8_t            retry_count;
    uint16_t           address_crc;
} BMS_AddressControl;

/* Address Management Configuration */
typedef struct {
    uint8_t rack_bitmap[(BMS_MAX_RACK_COUNT + 7) / 8];
    uint8_t assigned_addresses[BMS_MAX_RACK_COUNT];
    uint8_t unassigned_rack_count;
    uint8_t current_retry_count;
} BMS_AddressManager;

/* Function Prototypes for Address Assignment */
HAL_StatusTypeDef BMS_InitAddressAssignment(void);
HAL_StatusTypeDef BMS_PerformDynamicAddressing(void);
HAL_StatusTypeDef BMS_HandleAddressingResponse(uint32_t can_id, uint8_t* payload);
HAL_StatusTypeDef BMS_ResolveAddressCollision(uint8_t conflicting_address);

/* Utility Function Prototypes */
bool BMS_IsAddressAvailable(uint8_t address);
uint16_t BMS_ComputeAddressCRC(const uint8_t* data, size_t length);
uint8_t BMS_GenerateAddressingToken(void);

/* External Global Variables */
extern BMS_AddressManager g_bms_address_manager;
extern BMS_AddressControl g_bms_address_control;

#ifdef __cplusplus
}
#endif

#endif /* BMS_ADDRESSING_H */
