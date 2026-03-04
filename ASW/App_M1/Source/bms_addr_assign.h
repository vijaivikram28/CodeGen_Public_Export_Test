
#ifndef BMS_ADDR_ASSIGN_H
#define BMS_ADDR_ASSIGN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Standard headers */
#include <stdint.h>
#include <stdbool.h>

/* Configuration header */
#include "bms_addr_assign_cfg.h"

/* Public return codes */
typedef enum
{
    BMS_AA_OK = 0u,
    BMS_AA_ERROR = 1u,
    BMS_AA_TIMEOUT = 2u,
    BMS_AA_INVALID_PARAM = 3u
} BMS_AA_Result_t;

/* Assigned address record */
typedef struct
{
    uint8_t address;       /* Assigned address (1..MAX_RACKS), 0x00 = unassigned */
    uint8_t hw_id[8u];     /* Hardware ID reported by rack (optional, fixed-length) */
} BMS_AA_Assigned_t;

/* Public API
   - expected_racks: number of racks expected to be discovered (<= MAX_RACKS)
   - assigned_out: pointer to caller-provided array of BMS_AA_Assigned_t sized MAX_RACKS
   - out_count: pointer to uint8_t set to number of successfully assigned racks
   Returns BMS_AA_OK on success (may still have partial assignments), BMS_AA_ERROR on fatal error */
BMS_AA_Result_t BMS_AA_Init(void);
BMS_AA_Result_t BMS_AA_AssignAddresses(uint8_t expected_racks,
                                       BMS_AA_Assigned_t assigned_out[MAX_RACKS],
                                       uint8_t * out_count);

/* Utility: compute success rate (0..100) based on expected and actual */
uint8_t BMS_AA_ComputeSuccessRate(uint8_t expected_racks, uint8_t assigned_count);

#ifdef __cplusplus
}
#endif

#endif /* BMS_ADDR_ASSIGN_H */


