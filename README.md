# dspic33ak-can-hal

Small, readable CAN FD HAL for Microchip dsPIC33AK devices.

This project is intended as a compact alternative to large generated driver code.
The goal is not to hide everything behind a framework, but to provide a simple
CAN FD layer that is easy to read, test, modify, and adapt.

## Status

Current validation target:

* Device: dsPIC33AK512MPS512
* Compiler: XC-DSC v3.31.01
* DFP: Microchip dsPIC33AK-MP DFP 1.3.185 (or compatible)

The instance table is built from `#if defined(C1CON)` / `#if defined(C2CON)`
presence tests, so it adapts to whichever CAN FD instances the selected device
header defines, without device-name conditionals.

This HAL is used in a larger board project; on the dsPIC33AK512MPS512 target it
has been validated for:

* Internal-loopback frame round-trip (no transceiver)
* External-loopback through an external CAN transceiver
* Two-board CAN FD bus communication on CAN1 (CAN2 is supported by the device
  map but has not been hardware-validated yet)
* Interrupt-driven receive (the RX-available event path) via the optional event layer
* Runtime bit-rate change

Confirmed operations on the validation target:

* CAN FD frames (FDF + BRS) and classic frames, 11-bit and 29-bit identifiers
* Nominal 500 kbit/s + data 2 Mbit/s at FCAN = 20 MHz, 80% sample point
* Blocking transmit (TX queue) and blocking / polled receive (RX FIFO)
* Optional interrupt/event layer: the RX-available and RX-overflow event path is
  hardware-validated; bus status (error counters / state / bus-off) is queried
  synchronously via `dspic33ak_canfd_get_status()`. TX-complete (`tx_start`) is
  present in the API but **experimental** — its TX interrupt path is not yet
  validated on this silicon — and bus-error / invalid-message are not delivered
  as interrupts in this phase.
* Bus-off detection and re-init recovery

## Design policy

This HAL is intentionally small.

* The API is plain; one instance enum identifies a controller.
* No XC-DSC / DFP bitfield structures (`C1CONbits` / ...) are exposed in the
  public API.
* Device-specific register symbols are isolated in a small per-instance pointer
  table (`dspic33ak_canfd_device.c`), built from `#if defined(C1CON)` presence
  tests, so it adapts to the device without device-name conditionals.
* This HAL owns only the CAN FD module registers. It does NOT touch power
  (`PMDx`), the CAN clock, or pins / PPS — the board layer does that before
  `dspic33ak_canfd_init()` (including the RX PPS input, which must be mapped even
  for internal loopback).
* The application owns the message RAM region and passes it in; the HAL never
  statically allocates it.
* The core (`_node`) is blocking and ISR-free. The optional interrupt/event
  layer (`_isr`) is additive and opt-in; the HAL does not define interrupt
  vectors — the application forwards the CAN vectors to a HAL handler.

## Scope

In scope:

* CAN FD bit timing (nominal + data phase, BRS), `dspic33ak_canfd_set_bitrate()`
* Modes: Normal-FD, Normal-Classic, internal loopback, external loopback,
  listen-only
* Blocking transmit / receive via TX queue + RX FIFO with an accept-all filter
* Optional event layer: callback, enable/disable, async TX, bus-status decode,
  the `dspic33ak_canfd_irq_handler()` entry point
* Both CAN FD instances present on the device (C1 / C2)

Out of scope (not handled here):

* Power / clock / PPS bring-up — board layer
* HAL-owned interrupt vectors (the application forwards them)
* Configurable identifier filters beyond the accept-all RX filter
* Transmit-event FIFO (TEF), timestamps, multi-FIFO object models
* RTOS locking / cross-context mutual exclusion

## Files

```text
src/
  dspic33ak_canfd.h          (instance / status / mode types, lifecycle)
  dspic33ak_canfd_reg.h      (register pointer table + bit definitions)
  dspic33ak_canfd_device.c   dspic33ak_canfd_device.h   (instance -> register map)
  dspic33ak_canfd_common.c   dspic33ak_canfd_common.h   (bit-timing calc, mode bookkeeping)
  dspic33ak_canfd_node.c     dspic33ak_canfd_node.h     (init / transmit / receive / set_bitrate)
  dspic33ak_canfd_isr.c      dspic33ak_canfd_isr.h      (optional interrupt/event layer)
docs/
  canfd_hal_design.md
```

`dspic33ak_canfd_reg.h` is the only place that touches the raw CAN SFRs as
32-bit pointers and bit masks; the driver body drives any instance through a
per-instance pointer table. Consumer projects compile `dspic33ak_canfd_isr.c`
only when interrupt/event support is needed.

## Basic usage

The board layer enables the module, starts the CAN clock and maps the pins
(TX out, RX in, transceiver standby), then the HAL is brought up:

```c
#include "dspic33ak_canfd_node.h"

static uint32_t can1_msg_ram[DSPIC33AK_CANFD_MSG_RAM_WORDS] __attribute__((aligned(4)));

dspic33ak_canfd_config_t cfg = {
    .can_clk_hz   = 20000000u,   /* FCAN provided by the board clock setup       */
    .nominal_bps  = 500000u,
    .data_bps     = 2000000u,
    .sample_pct   = 80u,
    .brs          = true,
    .mode         = DSPIC33AK_CANFD_MODE_NORMAL_FD,
    .timeout_ms   = 10u,
    .get_ms       = board_get_ms,   /* NULL => spin, no timeout */
    .msg_ram      = can1_msg_ram,
    .msg_ram_size = (uint16_t)sizeof(can1_msg_ram),
};
dspic33ak_canfd_init(DSPIC33AK_CANFD_INST_1, &cfg);

dspic33ak_canfd_frame_t tx = { .id = 0x123u, .fd = true, .brs = true, .len = 8u };
/* ... fill tx.data ... */
dspic33ak_canfd_transmit(DSPIC33AK_CANFD_INST_1, &tx);

dspic33ak_canfd_frame_t rx;
if (dspic33ak_canfd_rx_available(DSPIC33AK_CANFD_INST_1)) {
    dspic33ak_canfd_receive(DSPIC33AK_CANFD_INST_1, &rx);
}
```

Interrupt-driven receive keeps the vectors in the application:

```c
#include "dspic33ak_canfd_isr.h"

static void on_can_event(dspic33ak_canfd_instance_t inst, uint32_t events, void *u)
{
    if (events & DSPIC33AK_CANFD_EVENT_RX_AVAILABLE) {
        /* drain with dspic33ak_canfd_receive() here */
    }
}

dspic33ak_canfd_isr_set_callback(DSPIC33AK_CANFD_INST_1, on_can_event, NULL);
dspic33ak_canfd_isr_enable(DSPIC33AK_CANFD_INST_1, 4u);  /* priority 1..7 */

/* dsPIC33AK CAN raises separate RX / TX / general vectors; forward all three: */
void __attribute__((interrupt, no_auto_psv)) _C1RXInterrupt(void) { dspic33ak_canfd_irq_handler(DSPIC33AK_CANFD_INST_1); }
void __attribute__((interrupt, no_auto_psv)) _C1TXInterrupt(void) { dspic33ak_canfd_irq_handler(DSPIC33AK_CANFD_INST_1); }
void __attribute__((interrupt, no_auto_psv)) _C1Interrupt  (void) { dspic33ak_canfd_irq_handler(DSPIC33AK_CANFD_INST_1); }
```

## API summary

Lifecycle / config (`dspic33ak_canfd_node.h`, `dspic33ak_canfd.h`):

* `dspic33ak_canfd_init()`        — bring up an instance in a given mode
* `dspic33ak_canfd_set_bitrate()` — re-apply nominal + data bit timing at runtime
* `dspic33ak_canfd_deinit()`
* `dspic33ak_canfd_is_present()` / `dspic33ak_canfd_is_initialized()`
* `dspic33ak_canfd_msg_ram_size()`

Transfer:

* `dspic33ak_canfd_transmit()`     — blocking, into the TX queue
* `dspic33ak_canfd_rx_available()`
* `dspic33ak_canfd_receive()`      — blocking / polled, from RX FIFO 1

Optional event layer (`dspic33ak_canfd_isr.h`):

* `dspic33ak_canfd_isr_set_callback()` / `dspic33ak_canfd_isr_enable()` / `_disable()`
* `dspic33ak_canfd_tx_start()` / `dspic33ak_canfd_tx_is_busy()` / `_tx_abort()` (TX-complete experimental)
* `dspic33ak_canfd_irq_handler()`  — call from the app-owned `_CxRXInterrupt`, `_CxTXInterrupt` and `_CxInterrupt` vectors
* `dspic33ak_canfd_get_status()`   — decoded error counters / bus state

## Notes

* The CAN SFRs and message RAM are 32-bit / word-access only on the dsPIC33A
  core; the register layer uses 32-bit pointers, and message-RAM payloads are
  packed/unpacked as 32-bit words (byte access faults with an address-error
  trap).
* This HAL is the CAN layer only; power/clock/PPS belong to the board layer.
* CMSIS-Driver CAN wrappers are intentionally kept in a separate repository
  (`dspic33ak-can-cmsis-driver`).
* This repository does not include Microchip DFP header files.

## License

MIT No Attribution License (MIT-0). See [LICENSE](LICENSE).

Attribution is appreciated but not required.
