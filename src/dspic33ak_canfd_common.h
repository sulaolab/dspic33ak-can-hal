/**
 * @file    dspic33ak_canfd_common.h
 * @brief   dsPIC33AK CAN FD HAL - shared primitives used by the role layer:
 *          validation, register lookup, bit-timing computation and per-instance
 *          mode state.
 */
#ifndef DSPIC33AK_CANFD_COMMON_H
#define DSPIC33AK_CANFD_COMMON_H

#include "dspic33ak_canfd.h"
#include "dspic33ak_canfd_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/** True if inst is a valid enum value (does not check presence). */
bool dspic33ak_canfd_inst_is_valid(dspic33ak_canfd_instance_t inst);

/** Resolve an instance to its register table; ERR_NOT_PRESENT if absent. */
dspic33ak_canfd_status_t dspic33ak_canfd_get_regs(dspic33ak_canfd_instance_t inst,
                                                  const dspic33ak_canfd_regs_t **regs);

/**
 * Compute nominal + data bit-timing register words and the TDC word for a
 * given CAN clock, target rates and sample point.
 *
 * @param fcan_hz       CAN module clock (FCAN), e.g. 20 MHz.
 * @param nominal_bps   arbitration phase bit rate, e.g. 500000.
 * @param data_bps      data phase bit rate, e.g. 2000000.
 * @param sample_pct    sample point in percent, e.g. 80.
 * @param nbtcfg/dbtcfg/tdc  filled with the CxNBTCFG / CxDBTCFG / CxTDC words.
 * @return OK or ERR_INVALID_ARG if no valid configuration was found.
 */
dspic33ak_canfd_status_t dspic33ak_canfd_calc_bit_timing(uint32_t fcan_hz,
                                                         uint32_t nominal_bps,
                                                         uint32_t data_bps,
                                                         uint8_t  sample_pct,
                                                         uint32_t *nbtcfg,
                                                         uint32_t *dbtcfg,
                                                         uint32_t *tdc);

/* Per-instance mode bookkeeping (drives is_initialized). */
void                   dspic33ak_canfd_set_mode(dspic33ak_canfd_instance_t inst,
                                                dspic33ak_canfd_mode_t mode);
dspic33ak_canfd_mode_t dspic33ak_canfd_get_mode(dspic33ak_canfd_instance_t inst);

#ifdef __cplusplus
}
#endif

#endif /* DSPIC33AK_CANFD_COMMON_H */
