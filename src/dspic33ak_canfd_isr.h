/**
 * @file    dspic33ak_canfd_isr.h
 * @brief   dsPIC33AK CAN FD HAL - OPTIONAL interrupt / event layer.
 *
 * This file is entirely additive and OPTIONAL. The basic CAN HAL
 * (dspic33ak_canfd_node.h: blocking init / transmit / receive) works on its own
 * and is unchanged by this layer - a user who only needs simple blocking CAN
 * never has to include this header or compile dspic33ak_canfd_isr.c.
 *
 * This layer adds what an event-driven driver needs (and what a CMSIS-Driver CAN
 * wrapper maps onto): a user event callback, interrupt enable/disable, an async
 * "fire and forget" transmit with a TX-complete event, an ISR entry point the
 * application forwards its CAN vector to, and a synchronous bus-health query.
 * No ARM_CAN_* types appear here; those stay in the (future) CMSIS wrapper.
 *
 * Opt-in sequence:
 *   1. dspic33ak_canfd_init(...)              // basic init (node layer)
 *   2. dspic33ak_canfd_isr_set_callback(...)  // your event callback
 *   3. dspic33ak_canfd_isr_enable(inst, prio) // arm RX + TX + error interrupts
 *   4. forward the CAN vectors to dspic33ak_canfd_irq_handler(inst)
 *
 * NOTE (dsPIC33AK): the CAN FD module raises SEPARATE CPU interrupts for
 * receive (_CxRXInterrupt), transmit (_CxTXInterrupt) and general/error
 * (_CxInterrupt). The application must forward ALL of them to the single
 * dspic33ak_canfd_irq_handler() - it reads CxINT and dispatches everything.
 * Forwarding only _CxInterrupt misses RX-FIFO interrupts (RX then only gets
 * serviced on overflow).
 */
#ifndef DSPIC33AK_CANFD_ISR_H
#define DSPIC33AK_CANFD_ISR_H

#include "dspic33ak_canfd.h"
#include "dspic33ak_canfd_node.h"   /* dspic33ak_canfd_frame_t for tx_start */

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------- */
/* Event bit-flags. The callback receives an OR of these per ISR.         */
/* (Style mirrors hal_uart EVENT_*; maps cleanly to ARM_CAN_EVENT_* later.)*/
/* ---------------------------------------------------------------------- */
#define DSPIC33AK_CANFD_EVENT_TX_COMPLETE   (1u << 0)  /* all queued TX frames sent */
#define DSPIC33AK_CANFD_EVENT_RX_AVAILABLE  (1u << 1)  /* >=1 frame waiting in RX FIFO */
#define DSPIC33AK_CANFD_EVENT_BUS_ERROR     (1u << 2)  /* error state changed (CERRIF) */
#define DSPIC33AK_CANFD_EVENT_BUS_OFF       (1u << 3)  /* controller went bus-off */
#define DSPIC33AK_CANFD_EVENT_RX_OVERFLOW   (1u << 4)  /* RX FIFO overflow */
#define DSPIC33AK_CANFD_EVENT_INVALID_MSG   (1u << 5)  /* invalid message (IVMIF) */

/**
 * User event callback. Invoked from dspic33ak_canfd_irq_handler() context with
 * an OR of the DSPIC33AK_CANFD_EVENT_* bits that occurred.
 *
 * On DSPIC33AK_CANFD_EVENT_RX_AVAILABLE the handler does NOT drain the FIFO;
 * the callback (or app) must call dspic33ak_canfd_receive() to consume the
 * frame(s). Draining inside the callback is fine - data is already present so
 * receive() returns without blocking, and it keeps the RX interrupt from
 * re-asserting immediately (the CMSIS "read in the RECEIVE event" pattern).
 */
typedef void (*dspic33ak_canfd_event_callback_t)(dspic33ak_canfd_instance_t inst,
                                                 uint32_t events,
                                                 void *user_data);

/** Decoded CxTREC error state (synchronous snapshot). */
typedef struct {
    uint8_t tx_err_count;   /* TERRCNT */
    uint8_t rx_err_count;   /* RERRCNT */
    bool    error_warning;  /* EWARN  - TEC or REC >= 96 */
    bool    error_passive;  /* TXBP or RXBP */
    bool    bus_off;        /* TXBO   */
} dspic33ak_canfd_bus_status_t;

/* ---------------------------------------------------------------------- */
/* Setup                                                                  */
/* ---------------------------------------------------------------------- */

/** Register (or clear, with NULL) the event callback for an instance. */
dspic33ak_canfd_status_t dspic33ak_canfd_isr_set_callback(
    dspic33ak_canfd_instance_t inst,
    dspic33ak_canfd_event_callback_t callback,
    void *user_data);

/**
 * Enable interrupts for an instance: the RX-not-empty and RX-overflow sources,
 * plus the top-level CPU interrupt at @p priority (1..7). Call after
 * dspic33ak_canfd_init(). TX-complete is armed per-transmit by
 * dspic33ak_canfd_tx_start(), not here.
 *
 * Bus-error (CERR) / invalid-message (IVM) sources are deliberately NOT armed:
 * on dsPIC33AK they are delivered on separate CPU vectors this layer does not
 * forward, so enabling them would trap. Bus health is queried synchronously via
 * dspic33ak_canfd_get_status(), and BUS_OFF is derived from CxTREC in the handler.
 *
 * The HAL does NOT define the _CxInterrupt vector; the application owns it and
 * forwards to dspic33ak_canfd_irq_handler(inst).
 */
dspic33ak_canfd_status_t dspic33ak_canfd_isr_enable(dspic33ak_canfd_instance_t inst,
                                                    uint8_t priority);

/** Disable the top-level CPU interrupt and the module interrupt sources. */
dspic33ak_canfd_status_t dspic33ak_canfd_isr_disable(dspic33ak_canfd_instance_t inst);

/* ---------------------------------------------------------------------- */
/* Async transmit                                                         */
/* ---------------------------------------------------------------------- */

/**
 * Queue @p frame into the TX queue (same path as dspic33ak_canfd_transmit) and
 * arm the TXQ-empty interrupt so DSPIC33AK_CANFD_EVENT_TX_COMPLETE fires once
 * all queued frames have actually been transmitted on the bus.
 *
 * This is NOT fully asynchronous: it goes through dspic33ak_canfd_transmit(), so
 * if the TX queue is full it first blocks for queue space (honoring the
 * configured timeout). Once the frame is queued it returns without waiting for
 * the frame to leave the bus.
 *
 * NOTE: the RX event path (isr_enable + RX_AVAILABLE) is HW-validated. The
 * TX-complete interrupt arming here is NOT yet validated on dsPIC33AK512MPS512 -
 * enabling it currently triggers an unhandled-interrupt trap, under
 * investigation. Prefer the blocking dspic33ak_canfd_transmit() for TX and use
 * the RX events; consumers that need a TX-complete signal (e.g. a CMSIS-Driver
 * SEND_COMPLETE) should treat this as unavailable until it is validated.
 */
dspic33ak_canfd_status_t dspic33ak_canfd_tx_start(dspic33ak_canfd_instance_t inst,
                                                  const dspic33ak_canfd_frame_t *frame);

/** True between tx_start() and the TX_COMPLETE event. */
bool dspic33ak_canfd_tx_is_busy(dspic33ak_canfd_instance_t inst);

/** Disarm the TX-complete interrupt and clear the busy flag (does not abort an
 *  in-flight frame on the wire). */
dspic33ak_canfd_status_t dspic33ak_canfd_tx_abort(dspic33ak_canfd_instance_t inst);

/* ---------------------------------------------------------------------- */
/* ISR entry + status                                                     */
/* ---------------------------------------------------------------------- */

/**
 * Service the module interrupt: read/clear the module flags, build the event
 * word and invoke the callback. Ordinary function, NOT an ISR vector - forward
 * ALL of the instance's CAN vectors to it:
 *
 *   void __attribute__((interrupt, no_auto_psv)) _C1RXInterrupt(void)
 *   { dspic33ak_canfd_irq_handler(DSPIC33AK_CANFD_INST_1); }
 *   void __attribute__((interrupt, no_auto_psv)) _C1TXInterrupt(void)
 *   { dspic33ak_canfd_irq_handler(DSPIC33AK_CANFD_INST_1); }
 *   void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void)
 *   { dspic33ak_canfd_irq_handler(DSPIC33AK_CANFD_INST_1); }
 */
void dspic33ak_canfd_irq_handler(dspic33ak_canfd_instance_t inst);

/** Read and decode CxTREC. Works with or without interrupts enabled. */
dspic33ak_canfd_status_t dspic33ak_canfd_get_status(dspic33ak_canfd_instance_t inst,
                                                    dspic33ak_canfd_bus_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* DSPIC33AK_CANFD_ISR_H */
