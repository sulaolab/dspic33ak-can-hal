/**
 * @file    dspic33ak_canfd_isr.c
 * @brief   dsPIC33AK CAN FD HAL - OPTIONAL interrupt / event layer.
 *
 * Additive only: this compilation unit touches extra interrupt-enable bits in
 * registers the device map already exposes (CxINT, CxFIFOCON1, CxTXQCON) and
 * reads CxTREC. It does NOT modify the blocking node layer or the message-RAM
 * layout. A project that never references this file gets the exact same
 * blocking behaviour as before.
 *
 * The HAL owns only the module interrupt sources and the top-level priority /
 * enable / flag bits (_CxIF/_CxIE/_CxIP). It does NOT define the _CxInterrupt
 * vector - the application owns that and forwards to dspic33ak_canfd_irq_handler().
 */
#include "dspic33ak_canfd_isr.h"
#include "dspic33ak_canfd_common.h"
#include "dspic33ak_canfd_reg.h"

#include <xc.h>
#include <stddef.h>

/* ISR-layer state only; the node layer's g_node[] is never touched here. */
static dspic33ak_canfd_event_callback_t g_cb[DSPIC33AK_CANFD_INST_COUNT];
static void                            *g_ud[DSPIC33AK_CANFD_INST_COUNT];
static volatile bool                    g_tx_busy[DSPIC33AK_CANFD_INST_COUNT];

/* ---------------------------------------------------------------------- */
/* Top-level CPU interrupt line helpers (per instance, guarded by symbol).  */
/*                                                                          */
/* IMPORTANT: on dsPIC33AK the CAN FD module has SEPARATE CPU interrupt      */
/* vectors/IFS-IEC bits: CxRX (receive FIFO), CxTX (transmit FIFO) and Cx    */
/* (general/error). The RX-FIFO-not-empty interrupt is delivered on CxRX,    */
/* NOT on the general Cx line - so all of them must be enabled and cleared,  */
/* and the application must forward _CxRXInterrupt / _CxTXInterrupt /        */
/* _CxInterrupt to dspic33ak_canfd_irq_handler().                           */
/* ---------------------------------------------------------------------- */
static void irq_line_enable(dspic33ak_canfd_instance_t inst, uint8_t priority)
{
    switch (inst) {
    case DSPIC33AK_CANFD_INST_1:
#if defined(_C1RXIF)
        _C1RXIF = 0; _C1RXIP = priority; _C1RXIE = 1;
#endif
#if defined(_C1TXIF)
        _C1TXIF = 0; _C1TXIP = priority; _C1TXIE = 1;
#endif
#if defined(_C1IF)
        _C1IF = 0; _C1IP = priority; _C1IE = 1;
#endif
        break;
    case DSPIC33AK_CANFD_INST_2:
#if defined(_C2RXIF)
        _C2RXIF = 0; _C2RXIP = priority; _C2RXIE = 1;
#endif
#if defined(_C2TXIF)
        _C2TXIF = 0; _C2TXIP = priority; _C2TXIE = 1;
#endif
#if defined(_C2IF)
        _C2IF = 0; _C2IP = priority; _C2IE = 1;
#endif
        break;
    default:
        break;
    }
}

static void irq_line_disable(dspic33ak_canfd_instance_t inst)
{
    switch (inst) {
    case DSPIC33AK_CANFD_INST_1:
#if defined(_C1RXIF)
        _C1RXIE = 0; _C1RXIF = 0;
#endif
#if defined(_C1TXIF)
        _C1TXIE = 0; _C1TXIF = 0;
#endif
#if defined(_C1IF)
        _C1IE = 0; _C1IF = 0;
#endif
        break;
    case DSPIC33AK_CANFD_INST_2:
#if defined(_C2RXIF)
        _C2RXIE = 0; _C2RXIF = 0;
#endif
#if defined(_C2TXIF)
        _C2TXIE = 0; _C2TXIF = 0;
#endif
#if defined(_C2IF)
        _C2IE = 0; _C2IF = 0;
#endif
        break;
    default:
        break;
    }
}

static void irq_line_clear(dspic33ak_canfd_instance_t inst)
{
    switch (inst) {
    case DSPIC33AK_CANFD_INST_1:
#if defined(_C1RXIF)
        _C1RXIF = 0;
#endif
#if defined(_C1TXIF)
        _C1TXIF = 0;
#endif
#if defined(_C1IF)
        _C1IF = 0;
#endif
        break;
    case DSPIC33AK_CANFD_INST_2:
#if defined(_C2RXIF)
        _C2RXIF = 0;
#endif
#if defined(_C2TXIF)
        _C2TXIF = 0;
#endif
#if defined(_C2IF)
        _C2IF = 0;
#endif
        break;
    default:
        break;
    }
}

/* ---------------------------------------------------------------------- */
/* Setup                                                                  */
/* ---------------------------------------------------------------------- */
dspic33ak_canfd_status_t dspic33ak_canfd_isr_set_callback(
    dspic33ak_canfd_instance_t inst,
    dspic33ak_canfd_event_callback_t callback,
    void *user_data)
{
    if (!dspic33ak_canfd_inst_is_valid(inst)) {
        return DSPIC33AK_CANFD_ERR_INVALID_ARG;
    }
    g_cb[inst] = callback;
    g_ud[inst] = user_data;
    return DSPIC33AK_CANFD_OK;
}

dspic33ak_canfd_status_t dspic33ak_canfd_isr_enable(dspic33ak_canfd_instance_t inst,
                                                    uint8_t priority)
{
    const dspic33ak_canfd_regs_t *regs;
    dspic33ak_canfd_status_t st;

    if (priority < 1u || priority > 7u) {
        return DSPIC33AK_CANFD_ERR_INVALID_ARG;
    }
    st = dspic33ak_canfd_get_regs(inst, &regs);
    if (st != DSPIC33AK_CANFD_OK) {
        return st;
    }
    if (!dspic33ak_canfd_is_initialized(inst)) {
        return DSPIC33AK_CANFD_ERR_NOT_INITIALIZED;
    }

    /* RX FIFO 1: interrupt when not-empty and on overflow. */
    dspic33ak_canfd_reg_set(regs->FIFOCON1,
                            DSPIC33AK_CANFD_FIFOCON_TFNRFNIE | DSPIC33AK_CANFD_FIFOCON_RXOVIE);
    /* Module roll-up enables: RX + RX-overflow only. NOTE: on dsPIC33AK the
     * bus-error (CERR) / invalid-message (IVM) module interrupts are delivered
     * on SEPARATE CPU vectors (CxWARN/CxMON/...), not the RX/TX/general lines
     * this layer forwards - enabling them here would trap to _DefaultInterrupt
     * unless those vectors are also wired. Bus health is instead surfaced
     * synchronously via dspic33ak_canfd_get_status() (CxTREC), and BUS_OFF is
     * derived from TREC inside the handler. */
    dspic33ak_canfd_reg_set(regs->INT,
                            DSPIC33AK_CANFD_INT_RXIE | DSPIC33AK_CANFD_INT_RXOVIE);

    g_tx_busy[inst] = false;
    irq_line_enable(inst, priority);
    return DSPIC33AK_CANFD_OK;
}

dspic33ak_canfd_status_t dspic33ak_canfd_isr_disable(dspic33ak_canfd_instance_t inst)
{
    const dspic33ak_canfd_regs_t *regs;
    dspic33ak_canfd_status_t st;

    st = dspic33ak_canfd_get_regs(inst, &regs);
    if (st != DSPIC33AK_CANFD_OK) {
        return st;
    }

    irq_line_disable(inst);
    /* Drop all module interrupt enables (TX, RX, RXOV, CERR, IVM). */
    dspic33ak_canfd_reg_clear(regs->INT,
                              DSPIC33AK_CANFD_INT_TXIE | DSPIC33AK_CANFD_INT_RXIE |
                              DSPIC33AK_CANFD_INT_RXOVIE | DSPIC33AK_CANFD_INT_CERRIE |
                              DSPIC33AK_CANFD_INT_IVMIE);
    dspic33ak_canfd_reg_clear(regs->FIFOCON1,
                              DSPIC33AK_CANFD_FIFOCON_TFNRFNIE | DSPIC33AK_CANFD_FIFOCON_RXOVIE);
    dspic33ak_canfd_reg_clear(regs->TXQCON, DSPIC33AK_CANFD_TXQCON_TXQEIE);
    g_tx_busy[inst] = false;
    return DSPIC33AK_CANFD_OK;
}

/* ---------------------------------------------------------------------- */
/* Async transmit                                                         */
/* ---------------------------------------------------------------------- */
dspic33ak_canfd_status_t dspic33ak_canfd_tx_start(dspic33ak_canfd_instance_t inst,
                                                  const dspic33ak_canfd_frame_t *frame)
{
    const dspic33ak_canfd_regs_t *regs;
    dspic33ak_canfd_status_t st;

    st = dspic33ak_canfd_get_regs(inst, &regs);
    if (st != DSPIC33AK_CANFD_OK) {
        return st;
    }

    /* Queue the frame using the existing (non-waiting after TXREQ) node path. */
    st = dspic33ak_canfd_transmit(inst, frame);
    if (st != DSPIC33AK_CANFD_OK) {
        return st;
    }

    g_tx_busy[inst] = true;
    /* Arm "TXQ empty" so TX_COMPLETE fires once everything queued is on the bus. */
    dspic33ak_canfd_reg_set(regs->TXQCON, DSPIC33AK_CANFD_TXQCON_TXQEIE);
    dspic33ak_canfd_reg_set(regs->INT, DSPIC33AK_CANFD_INT_TXIE);
    return DSPIC33AK_CANFD_OK;
}

bool dspic33ak_canfd_tx_is_busy(dspic33ak_canfd_instance_t inst)
{
    if (!dspic33ak_canfd_inst_is_valid(inst)) {
        return false;
    }
    return g_tx_busy[inst];
}

dspic33ak_canfd_status_t dspic33ak_canfd_tx_abort(dspic33ak_canfd_instance_t inst)
{
    const dspic33ak_canfd_regs_t *regs;
    dspic33ak_canfd_status_t st;

    st = dspic33ak_canfd_get_regs(inst, &regs);
    if (st != DSPIC33AK_CANFD_OK) {
        return st;
    }
    dspic33ak_canfd_reg_clear(regs->TXQCON, DSPIC33AK_CANFD_TXQCON_TXQEIE);
    dspic33ak_canfd_reg_clear(regs->INT, DSPIC33AK_CANFD_INT_TXIE);
    g_tx_busy[inst] = false;
    return DSPIC33AK_CANFD_OK;
}

/* ---------------------------------------------------------------------- */
/* ISR entry + status                                                     */
/* ---------------------------------------------------------------------- */
void dspic33ak_canfd_irq_handler(dspic33ak_canfd_instance_t inst)
{
    const dspic33ak_canfd_regs_t *regs;
    uint32_t intf;
    uint32_t events = 0u;

    if (dspic33ak_canfd_get_regs(inst, &regs) != DSPIC33AK_CANFD_OK) {
        irq_line_clear(inst);
        return;
    }

    intf = *regs->INT;

    if ((intf & DSPIC33AK_CANFD_INT_RXIF) != 0u) {
        /* Handler does not drain; the callback consumes via receive(). */
        events |= DSPIC33AK_CANFD_EVENT_RX_AVAILABLE;
    }
    if ((intf & DSPIC33AK_CANFD_INT_RXOVIF) != 0u) {
        events |= DSPIC33AK_CANFD_EVENT_RX_OVERFLOW;
        dspic33ak_canfd_reg_clear(regs->FIFOSTA1, DSPIC33AK_CANFD_FIFOSTA_RXOVIF);
    }
    if (((intf & DSPIC33AK_CANFD_INT_TXIF) != 0u) &&
        ((*regs->TXQSTA & DSPIC33AK_CANFD_TXQSTA_TXQEIF) != 0u)) {
        events |= DSPIC33AK_CANFD_EVENT_TX_COMPLETE;
        /* One-shot: disarm so the level-sensitive TXQ-empty flag stops re-firing. */
        dspic33ak_canfd_reg_clear(regs->TXQCON, DSPIC33AK_CANFD_TXQCON_TXQEIE);
        dspic33ak_canfd_reg_clear(regs->INT, DSPIC33AK_CANFD_INT_TXIE);
        g_tx_busy[inst] = false;
    }
    if ((intf & DSPIC33AK_CANFD_INT_CERRIF) != 0u) {
        events |= DSPIC33AK_CANFD_EVENT_BUS_ERROR;
    }
    if ((intf & DSPIC33AK_CANFD_INT_IVMIF) != 0u) {
        events |= DSPIC33AK_CANFD_EVENT_INVALID_MSG;
    }

    /* Clear the writable module flags (TXIF/RXIF/RXOVIF are read-only roll-ups). */
    dspic33ak_canfd_reg_clear(regs->INT, DSPIC33AK_CANFD_INT_CLR_MASK);

    if ((*regs->TREC & DSPIC33AK_CANFD_TREC_TXBO) != 0u) {
        events |= DSPIC33AK_CANFD_EVENT_BUS_OFF;
    }

    /* Callback first (drains RX so RXIF de-asserts), then clear the CPU flag. */
    if (events != 0u && g_cb[inst] != NULL) {
        g_cb[inst](inst, events, g_ud[inst]);
    }
    irq_line_clear(inst);
}

dspic33ak_canfd_status_t dspic33ak_canfd_get_status(dspic33ak_canfd_instance_t inst,
                                                    dspic33ak_canfd_bus_status_t *status)
{
    const dspic33ak_canfd_regs_t *regs;
    dspic33ak_canfd_status_t st;
    uint32_t t;

    if (status == NULL) {
        return DSPIC33AK_CANFD_ERR_INVALID_ARG;
    }
    st = dspic33ak_canfd_get_regs(inst, &regs);
    if (st != DSPIC33AK_CANFD_OK) {
        return st;
    }

    t = *regs->TREC;
    status->rx_err_count  = (uint8_t)((t & DSPIC33AK_CANFD_TREC_RERRCNT_MASK) >> DSPIC33AK_CANFD_TREC_RERRCNT_POS);
    status->tx_err_count  = (uint8_t)((t & DSPIC33AK_CANFD_TREC_TERRCNT_MASK) >> DSPIC33AK_CANFD_TREC_TERRCNT_POS);
    status->error_warning = (t & DSPIC33AK_CANFD_TREC_EWARN) != 0u;
    status->error_passive = (t & (DSPIC33AK_CANFD_TREC_TXBP | DSPIC33AK_CANFD_TREC_RXBP)) != 0u;
    status->bus_off       = (t & DSPIC33AK_CANFD_TREC_TXBO) != 0u;
    return DSPIC33AK_CANFD_OK;
}
