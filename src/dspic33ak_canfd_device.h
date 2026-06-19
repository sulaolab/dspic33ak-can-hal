/**
 * @file    dspic33ak_canfd_device.h
 * @brief   dsPIC33AK CAN FD HAL - device/instance map.
 *
 * Maps an instance enum to its register pointer table. This is the only
 * boundary between the instance-agnostic HAL logic and the concrete MCU
 * register set; only dspic33ak_canfd_device.c names raw C1.../C2... symbols.
 */
#ifndef DSPIC33AK_CANFD_DEVICE_H
#define DSPIC33AK_CANFD_DEVICE_H

#include "dspic33ak_canfd.h"
#include "dspic33ak_canfd_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool present;
    dspic33ak_canfd_regs_t regs;
} dspic33ak_canfd_device_t;

/** Return the device entry for an instance, or NULL if absent/out of range. */
const dspic33ak_canfd_device_t *dspic33ak_canfd_get_device(dspic33ak_canfd_instance_t inst);

/** Convenience: true if the instance is present on this device. */
bool dspic33ak_canfd_instance_is_present(dspic33ak_canfd_instance_t inst);

#ifdef __cplusplus
}
#endif

#endif /* DSPIC33AK_CANFD_DEVICE_H */
