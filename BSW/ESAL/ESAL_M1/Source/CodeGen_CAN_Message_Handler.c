/**
 * @file CodeGen_CAN_Message_Handler.h
 * @brief CAN Message Processing Header for Battery Management System
 */

#ifndef BMS_CAN_MESSAGE_PROCESSING_H
#define BMS_CAN_MESSAGE_PROCESSING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "CodeGen_BMS_config.h"

/* CAN Message Configuration Constants */
#define BMS_CAN_RACK_ID_MASK        (0x000000FFU)
#define BMS_CAN_RACK_ID_SHIFT       (0U)
#define BMS_MAX_CAN_FIELDS          (6U)
#define BMS_MAX_CAN_MESSAGES        (3U)

/* Enumeration for CAN Message Types */
typedef enum {
    BMS_MSG_RACK_STATUS = 0x18FF01U,
    BMS_MSG_RACK_ERROR  = 0x18FF02U,
    BMS_MSG_BIN_DATA    = 0x18FF03U
} BMS_CANMessageType;

/* Rack Status Message Structure */
typedef struct {
    float   voltage;         // Voltage in V
    float   current;         // Current in A
    float   dcir;            // DCIR in Ohm
    uint8_t soc;             // State of Charge %
    uint8_t soh;             // State of Health %
    uint8_t rack_status;     // Rack Status Bit
} BMS_RackStatusMessage;

/* Rack Error Message Structure */
typedef struct {
    uint8_t  error_code;     // Error Code
    uint16_t rack_ot;        // Over Temperature Flag
    uint16_t rack_ov;        // Over Voltage Flag
} BMS_RackErrorMessage;

/* Rack BIN Data Message Structure */
typedef struct {
    uint64_t bin_id;         // BIN Identifier
} BMS_BinDataMessage;

/* Union for CAN Message Parsing */
typedef union {
    BMS_RackStatusMessage status;
    BMS_RackErrorMessage  error;
    BMS_BinDataMessage    bin_data;
} BMS_CANMessageData;

/* CAN Message Processing Configuration */
typedef struct {
    uint8_t                rack_id;
    BMS_CANMessageType     msg_type;
    BMS_CANMessageData     msg_data;
    uint32_t               timestamp;
} BMS_CANMessageContext;

/* Function Prototypes for CAN Message Processing */
HAL_StatusTypeDef BMS_CAN_DecodeMessage(
    const uint32_t msg_id, 
    const uint8_t* payload, 
    BMS_CANMessageContext* message_ctx
);

HAL_StatusTypeDef BMS_CAN_ValidateMessage(
    const BMS_CANMessageContext* message_ctx
);

uint8_t BMS_CAN_ExtractRackID(
    const uint32_t msg_id
);

void BMS_CAN_InitMessageContext(
    BMS_CANMessageContext* message_ctx
);

HAL_StatusTypeDef BMS_CAN_ProcessMessage(
    BMS_CANMessageContext* message_ctx
);

/* External Global Variables */
extern BMS_CANMessageContext g_bms_message_contexts[BMS_MAX_RACK_COUNT];

#ifdef __cplusplus
}
#endif

#endif /* BMS_CAN_MESSAGE_PROCESSING_H */