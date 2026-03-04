
#ifndef BMS_ADDR_ASSIGN_CFG_H
#define BMS_ADDR_ASSIGN_CFG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Standard headers required */
#include "stm32h7xx_hal.h"
#include <stdint.h>

/* Configuration macros (modifiable per project) */
#define MAX_RACKS                   64u
#define TIMEOUT_FOR_RACK_MS         100u    /* ms - timeout for rack responses during initialization */
#define MAX_RETRY_ATTEMPTS          5u      /* Maximum retry attempts */
#define DEFAULT_ADDRESS             0x00u   /* Default unassigned address value */

/* CAN Extended IDs used for address assignment protocol (application-defined) */
#define ASSIGN_TOKEN_EXT_ID         0x18FF1000u  /* Master -> broadcast candidate token */
#define ASSIGN_RESPONSE_EXT_ID      0x18FF2000u  /* Rack -> response to token (data includes current addr and hw id) */
#define ASSIGN_CONFIRM_EXT_ID       0x18FF3000u  /* Master -> confirm assigned address */

/* External CAN handle to be provided by top-level application linking this module.
   Example in application: CAN_HandleTypeDef hcan_bms_aa;
   The symbol name must match exactly. */
extern CAN_HandleTypeDef hcan_bms_aa;

#ifdef __cplusplus
}
#endif

#endif /* BMS_ADDR_ASSIGN_CFG_H */