/**
 * @file CodeGen_BMS_Dynamic_Address_Assignment.c
 * @brief Implementation of Dynamic Address Assignment in BMS
 */

#include "CodeGen_Dynamic_address_assignment.h"
#include "CodeGen_Address_Assignment.h"
#include "CodeGen_logging.h"

///**
// * @brief Main function for executing dynamic address assignment
// */
//int main(void) {
//    HAL_Init();  // Initialize HAL Library
//    BMS_InitHardware();  // Initialize Hardware
//    BMS_LogInitialize();  // Initialize Logging
//
//    while (1) {
//        BMS_CheckUserButton();  // Check User Button and Execute Address Assignment
//    }
//}

/**
 * @brief Initialize Hardware components
 */
void BMS_InitHardware(void) {
    // Initialize User Button GPIO
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIOC clock
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Configure User Button Pin
    GPIO_InitStruct.Pin = BMS_USER_BUTTON_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BMS_USER_BUTTON_GPIO_PORT, &GPIO_InitStruct);

    // Initialize CAN peripheral and other necessary hardware (not shown, but assumed to be implemented)
    // Example: BMS_CAN_Init();
}

/**
 * @brief Check User Button State
 */
void BMS_CheckUserButton(void) {
    if (HAL_GPIO_ReadPin(BMS_USER_BUTTON_GPIO_PORT, BMS_USER_BUTTON_PIN) == GPIO_PIN_SET) {
        // Start dynamic address assignment when the button is pressed
        BMS_StartDynamicAddressAssignment();
    }
}

/**
 * @brief Start Dynamic Address Assignment
 */
void BMS_StartDynamicAddressAssignment(void) {
    HAL_StatusTypeDef status = BMS_InitAddressAssignment();

    if (status != HAL_OK) {
        // Handle initialization error
        return;
    }

    uint8_t totalRackCount = BMS_MAX_RACK_COUNT;
    uint8_t assignedCount = 0;

    for (uint32_t i = 0; i < sizeof(g_bms_address_manager.rack_bitmap); i++) {
    	assignedCount += __builtin_popcount(g_bms_address_manager.rack_bitmap[i]);
        }

    // Log the ratio of addresses assigned to total racks
    BMS_LogAddressAssignmentRatio(assignedCount, totalRackCount);
}

/**
 * @brief Log Address Assignment Ratio
 * @param assignedCount Number of addresses assigned
 * @param totalCount Total number of racks
 */
void BMS_LogAddressAssignmentRatio(uint8_t assignedCount, uint8_t totalCount) {
    // Log the ratio as an event (assuming the ratio is defined as an integer percentage)
    uint32_t ratio = (assignedCount * 100) / totalCount;
    BMS_LogEventSafe(BMS_EVENT_SYSTEM_INIT, BMS_LOG_LEVEL_INFO, ratio);
}
