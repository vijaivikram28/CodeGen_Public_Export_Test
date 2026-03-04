/**
 * @file CodeGen_BMS_Dynamic_Address_Assignment.h
 * @brief Header file for Dynamic Address Assignment in BMS using FDCAN1
 */

#ifndef BMS_DYNAMIC_ADDRESS_ASSIGNMENT_H
#define BMS_DYNAMIC_ADDRESS_ASSIGNMENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "CodeGen_Address_Assignment.h"
#include "CodeGen_BMS_config.h"
#include "CodeGen_logging.h"

/* User Button Pin Definition */
#define BMS_USER_BUTTON_PIN GPIO_PIN_13
#define BMS_USER_BUTTON_GPIO_PORT GPIOC

/* Function Prototypes */
void BMS_InitHardware(void);

/* Function Prototypes */
void BMS_StartDynamicAddressAssignment(void);
void BMS_CheckUserButton(void);
void BMS_LogAddressAssignmentRatio(uint8_t assignedCount, uint8_t totalCount);

#ifdef __cplusplus
}
#endif

#endif /* BMS_DYNAMIC_ADDRESS_ASSIGNMENT_H */
