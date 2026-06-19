/**
 * @file    dspic33ak_canfd_node.c
 * @brief   dsPIC33AK CAN FD HAL - node (transmit/receive) role.
 *
 * Fixed phase-1 geometry: TX queue with TXQ_DEPTH objects + RX FIFO 1 with
 * RX_DEPTH objects, 64-byte payload, accept-all filter 0 -> FIFO 1, no TEF, no
 * RX timestamp. Blocking TX/RX by polling the FIFO status flags.
 */
#include "dspic33ak_canfd_node.h"
#include "dspic33ak_canfd_common.h"

#define TXQ_DEPTH        4u
#define RX_DEPTH         4u
#define PAYLOAD_BYTES    64u
#define PLSIZE_CODE      DSPIC33AK_CANFD_PLSIZE_64
#define OBJ_BYTES        (DSPIC33AK_CANFD_OBJ_HEADER_BYTES + PAYLOAD_BYTES) /* 72 */
#define MSG_RAM_BYTES    ((TXQ_DEPTH + RX_DEPTH) * OBJ_BYTES)              /* 576 */

/* Keep the public compile-time constant in lock-step with the actual geometry. */
_Static_assert(DSPIC33AK_CANFD_MSG_RAM_BYTES == MSG_RAM_BYTES,
               "DSPIC33AK_CANFD_MSG_RAM_BYTES must match the TXQ/RX FIFO geometry");

#define DEFAULT_SAMPLE_PCT  80u

typedef struct {
    bool                      initialized;
    dspic33ak_canfd_get_ms_fn get_ms;
    uint32_t                  timeout_ms;
} node_state_t;

static node_state_t g_node[DSPIC33AK_CANFD_INST_COUNT];

/* ------------------------------------------------------------------ */
/* helpers                                                            */
/* ------------------------------------------------------------------ */

static uint8_t mode_to_reqop(dspic33ak_canfd_mode_t mode)
{
    switch (mode) {
    case DSPIC33AK_CANFD_MODE_NORMAL_FD:         return DSPIC33AK_CANFD_OPMODE_NORMAL_FD;
    case DSPIC33AK_CANFD_MODE_NORMAL_CLASSIC:    return DSPIC33AK_CANFD_OPMODE_NORMAL_CLASSIC;
    case DSPIC33AK_CANFD_MODE_INTERNAL_LOOPBACK: return DSPIC33AK_CANFD_OPMODE_INTERNAL_LOOP;
    case DSPIC33AK_CANFD_MODE_EXTERNAL_LOOPBACK: return DSPIC33AK_CANFD_OPMODE_EXTERNAL_LOOP;
    case DSPIC33AK_CANFD_MODE_LISTEN_ONLY:       return DSPIC33AK_CANFD_OPMODE_LISTEN_ONLY;
    default:                                     return DSPIC33AK_CANFD_OPMODE_CONFIG;
    }
}

static uint8_t opmod_get(const dspic33ak_canfd_regs_t *regs)
{
    return (uint8_t)((*regs->CON & DSPIC33AK_CANFD_CON_OPMOD_MASK)
                     >> DSPIC33AK_CANFD_CON_OPMOD_POS);
}

/* Request an operating mode and wait until OPMOD reflects it. */
static dspic33ak_canfd_status_t request_mode(dspic33ak_canfd_instance_t inst,
                                             const dspic33ak_canfd_regs_t *regs,
                                             uint8_t reqop)
{
    node_state_t *st = &g_node[inst];
    uint32_t start = (st->get_ms != NULL) ? st->get_ms() : 0u;

    dspic33ak_canfd_reg_write_field(regs->CON, DSPIC33AK_CANFD_CON_REQOP_MASK,
                                    (uint32_t)reqop << DSPIC33AK_CANFD_CON_REQOP_POS);
    while (opmod_get(regs) != reqop) {
        if (st->get_ms != NULL && st->timeout_ms != 0u) {
            if ((st->get_ms() - start) >= st->timeout_ms) {
                return DSPIC33AK_CANFD_ERR_TIMEOUT;
            }
        }
    }
    return DSPIC33AK_CANFD_OK;
}

/* Wait until (*reg & mask) becomes non-zero, honoring the configured timeout. */
static dspic33ak_canfd_status_t wait_flag(dspic33ak_canfd_instance_t inst,
                                          volatile uint32_t *reg, uint32_t mask)
{
    node_state_t *st = &g_node[inst];
    uint32_t start = (st->get_ms != NULL) ? st->get_ms() : 0u;

    while ((*reg & mask) == 0u) {
        if (st->get_ms != NULL && st->timeout_ms != 0u) {
            if ((st->get_ms() - start) >= st->timeout_ms) {
                return DSPIC33AK_CANFD_ERR_TIMEOUT;
            }
        }
    }
    return DSPIC33AK_CANFD_OK;
}

/* Build the object ID word (T0/R0) from a frame. */
static uint32_t build_id_word(const dspic33ak_canfd_frame_t *f)
{
    if (f->extended) {
        uint32_t sid = (f->id >> 18) & DSPIC33AK_CANFD_OBJID_SID_MASK;
        uint32_t eid = f->id & DSPIC33AK_CANFD_OBJID_EID_MASK;
        return (sid << DSPIC33AK_CANFD_OBJID_SID_POS)
             | (eid << DSPIC33AK_CANFD_OBJID_EID_POS);
    }
    return (f->id & DSPIC33AK_CANFD_OBJID_SID_MASK) << DSPIC33AK_CANFD_OBJID_SID_POS;
}

/* Parse the object ID word back into the frame's id/extended fields. */
static void parse_id_word(uint32_t w, bool extended, dspic33ak_canfd_frame_t *f)
{
    uint32_t sid = (w >> DSPIC33AK_CANFD_OBJID_SID_POS) & DSPIC33AK_CANFD_OBJID_SID_MASK;
    uint32_t eid = (w >> DSPIC33AK_CANFD_OBJID_EID_POS) & DSPIC33AK_CANFD_OBJID_EID_MASK;

    f->extended = extended;
    f->id = extended ? ((sid << 18) | eid) : sid;
}

/* ------------------------------------------------------------------ */
/* public API                                                         */
/* ------------------------------------------------------------------ */

uint16_t dspic33ak_canfd_msg_ram_size(void)
{
    return (uint16_t)MSG_RAM_BYTES;
}

dspic33ak_canfd_status_t dspic33ak_canfd_init(dspic33ak_canfd_instance_t inst,
                                              const dspic33ak_canfd_config_t *config)
{
    const dspic33ak_canfd_regs_t *regs;
    dspic33ak_canfd_status_t st;
    uint32_t nbtcfg = 0u, dbtcfg = 0u, tdc = 0u;
    uint8_t sample_pct;

    if (config == NULL || config->msg_ram == NULL) {
        return DSPIC33AK_CANFD_ERR_INVALID_ARG;
    }
    /* Message RAM is word-access only on this device; a misaligned base would
     * fault with an address-error trap on the first object access. */
    if (((uintptr_t)config->msg_ram & 3u) != 0u) {
        return DSPIC33AK_CANFD_ERR_INVALID_ARG;
    }
    if (config->msg_ram_size < MSG_RAM_BYTES) {
        return DSPIC33AK_CANFD_ERR_INVALID_ARG;
    }
    if (config->mode == DSPIC33AK_CANFD_MODE_NONE) {
        return DSPIC33AK_CANFD_ERR_INVALID_ARG;
    }
    st = dspic33ak_canfd_get_regs(inst, &regs);
    if (st != DSPIC33AK_CANFD_OK) {
        return st;
    }

    sample_pct = (config->sample_pct != 0u) ? config->sample_pct : DEFAULT_SAMPLE_PCT;
    st = dspic33ak_canfd_calc_bit_timing(config->can_clk_hz, config->nominal_bps,
                                         config->data_bps, sample_pct,
                                         &nbtcfg, &dbtcfg, &tdc);
    if (st != DSPIC33AK_CANFD_OK) {
        return st;
    }

    g_node[inst].get_ms = config->get_ms;
    g_node[inst].timeout_ms = config->timeout_ms;
    g_node[inst].initialized = false;
    /* Invalidate any prior public state up-front, so a re-init that fails part
     * way through cannot leave dspic33ak_canfd_is_initialized() reporting true
     * while the node state says false. Restored to config->mode on success. */
    dspic33ak_canfd_set_mode(inst, DSPIC33AK_CANFD_MODE_NONE);

    /* Enable module, enter configuration mode. */
    dspic33ak_canfd_reg_set(regs->CON, DSPIC33AK_CANFD_CON_ON);
    st = request_mode(inst, regs, DSPIC33AK_CANFD_OPMODE_CONFIG);
    if (st != DSPIC33AK_CANFD_OK) {
        /* Leave the module as we found it (off) on a failed bring-up. */
        dspic33ak_canfd_reg_clear(regs->CON, DSPIC33AK_CANFD_CON_ON);
        return st;
    }

    /* Clock source = CAN clock generator (CLKGEN10); ISO CRC on. */
    dspic33ak_canfd_reg_clear(regs->CON, DSPIC33AK_CANFD_CON_CLKSEL);
    dspic33ak_canfd_reg_set(regs->CON, DSPIC33AK_CANFD_CON_ISOCRCEN);

    /* Bit-rate switching: BRSDIS=1 disables it. */
    if (config->brs) {
        dspic33ak_canfd_reg_clear(regs->CON, DSPIC33AK_CANFD_CON_BRSDIS);
    } else {
        dspic33ak_canfd_reg_set(regs->CON, DSPIC33AK_CANFD_CON_BRSDIS);
    }

    /* No transmit event FIFO; enable the TX queue. */
    dspic33ak_canfd_reg_clear(regs->CON, DSPIC33AK_CANFD_CON_STEF);
    dspic33ak_canfd_reg_set(regs->CON, DSPIC33AK_CANFD_CON_TXQEN);

    /* Bit timing. */
    *regs->NBTCFG = nbtcfg;
    *regs->DBTCFG = dbtcfg;
    *regs->TDC    = tdc;

    /* Message RAM base. */
    *regs->FIFOBA = (uint32_t)(uintptr_t)config->msg_ram;

    /* TX queue: TXQ_DEPTH objects, 64-byte payload. */
    *regs->TXQCON = ((uint32_t)PLSIZE_CODE     << DSPIC33AK_CANFD_FIFOCON_PLSIZE_POS)
                  | ((uint32_t)(TXQ_DEPTH - 1u) << DSPIC33AK_CANFD_FIFOCON_FSIZE_POS);

    /* FIFO 1 as receive: RX_DEPTH objects, 64-byte payload, no timestamp. */
    *regs->FIFOCON1 = ((uint32_t)PLSIZE_CODE    << DSPIC33AK_CANFD_FIFOCON_PLSIZE_POS)
                    | ((uint32_t)(RX_DEPTH - 1u) << DSPIC33AK_CANFD_FIFOCON_FSIZE_POS);

    /* Filter 0: accept all, route to FIFO 1. */
    *regs->FLTOBJ0 = 0u;
    *regs->MASK0   = 0u;                              /* all id bits don't-care */
    *regs->FLTCON0 = ((uint32_t)1u << DSPIC33AK_CANFD_FLTCON0_F0BP_POS) /* dest FIFO 1 */
                   | DSPIC33AK_CANFD_FLTCON0_FLTEN0;

    /* Enter the requested operating mode. */
    st = request_mode(inst, regs, mode_to_reqop(config->mode));
    if (st != DSPIC33AK_CANFD_OK) {
        /* Public mode is already NONE (cleared above); power the module back down. */
        dspic33ak_canfd_reg_clear(regs->CON, DSPIC33AK_CANFD_CON_ON);
        return st;
    }

    dspic33ak_canfd_set_mode(inst, config->mode);
    g_node[inst].initialized = true;
    return DSPIC33AK_CANFD_OK;
}

dspic33ak_canfd_status_t dspic33ak_canfd_set_bitrate(dspic33ak_canfd_instance_t inst,
                                                     uint32_t can_clk_hz,
                                                     uint32_t nominal_bps,
                                                     uint32_t data_bps,
                                                     uint8_t  sample_pct)
{
    const dspic33ak_canfd_regs_t *regs;
    dspic33ak_canfd_status_t st;
    uint32_t nbtcfg = 0u, dbtcfg = 0u, tdc = 0u;
    dspic33ak_canfd_mode_t prev_mode;
    uint8_t sp;

    if (can_clk_hz == 0u || nominal_bps == 0u || data_bps == 0u) {
        return DSPIC33AK_CANFD_ERR_INVALID_ARG;
    }
    st = dspic33ak_canfd_get_regs(inst, &regs);
    if (st != DSPIC33AK_CANFD_OK) {
        return st;
    }
    if (!g_node[inst].initialized) {
        return DSPIC33AK_CANFD_ERR_NOT_INITIALIZED;
    }

    sp = (sample_pct != 0u) ? sample_pct : DEFAULT_SAMPLE_PCT;
    st = dspic33ak_canfd_calc_bit_timing(can_clk_hz, nominal_bps, data_bps, sp,
                                         &nbtcfg, &dbtcfg, &tdc);
    if (st != DSPIC33AK_CANFD_OK) {
        return st;
    }

    /* NBTCFG/DBTCFG/TDC are writable only in Configuration mode: drop to Config,
     * apply, then restore the operating mode the instance was in. */
    prev_mode = dspic33ak_canfd_get_mode(inst);
    st = request_mode(inst, regs, DSPIC33AK_CANFD_OPMODE_CONFIG);
    if (st != DSPIC33AK_CANFD_OK) {
        return st;
    }

    *regs->NBTCFG = nbtcfg;
    *regs->DBTCFG = dbtcfg;
    *regs->TDC    = tdc;

    st = request_mode(inst, regs, mode_to_reqop(prev_mode));
    if (st != DSPIC33AK_CANFD_OK) {
        return st;
    }
    return DSPIC33AK_CANFD_OK;
}

dspic33ak_canfd_status_t dspic33ak_canfd_transmit(dspic33ak_canfd_instance_t inst,
                                                  const dspic33ak_canfd_frame_t *frame)
{
    const dspic33ak_canfd_regs_t *regs;
    dspic33ak_canfd_status_t st;
    volatile uint32_t *obj;
    uint8_t dlc, actual_len, w, nwords;
    uint32_t ctrl;

    if (frame == NULL || frame->len > PAYLOAD_BYTES) {
        return DSPIC33AK_CANFD_ERR_INVALID_ARG;
    }
    /* Classic CAN frames carry at most 8 data bytes; only CAN FD reaches 64. */
    if (!frame->fd && frame->len > 8u) {
        return DSPIC33AK_CANFD_ERR_INVALID_ARG;
    }
    if (!dspic33ak_canfd_inst_is_valid(inst) || !g_node[inst].initialized) {
        return DSPIC33AK_CANFD_ERR_NOT_INITIALIZED;
    }
    (void)dspic33ak_canfd_get_regs(inst, &regs);

    /* Wait for room in the TX queue (TXQNIF == not full). */
    st = wait_flag(inst, regs->TXQSTA, DSPIC33AK_CANFD_FIFOSTA_TFNRFNIF);
    if (st != DSPIC33AK_CANFD_OK) {
        return st;
    }

    dlc = dspic33ak_canfd_len_to_dlc(frame->len);
    actual_len = dspic33ak_canfd_dlc_to_len(dlc);

    ctrl = (uint32_t)(dlc & DSPIC33AK_CANFD_OBJCTRL_DLC_MASK);
    if (frame->extended) ctrl |= DSPIC33AK_CANFD_OBJCTRL_IDE;
    if (frame->fd)       ctrl |= DSPIC33AK_CANFD_OBJCTRL_FDF;
    if (frame->fd && frame->brs) ctrl |= DSPIC33AK_CANFD_OBJCTRL_BRS;

    /* The TX queue user address points at the next free object in message RAM. */
    obj = (volatile uint32_t *)(uintptr_t)(*regs->TXQUA);
    obj[0] = build_id_word(frame);
    obj[1] = ctrl;

    /* Payload: pack into 32-bit words. CAN message RAM only allows word access on
     * this device - byte access faults with an address-error trap. */
    nwords = (uint8_t)((actual_len + 3u) / 4u);
    for (w = 0u; w < nwords; w++) {
        uint8_t b = (uint8_t)(w * 4u);
        uint32_t word = 0u;
        if ((b + 0u) < frame->len) word |= (uint32_t)frame->data[b + 0u];
        if ((b + 1u) < frame->len) word |= (uint32_t)frame->data[b + 1u] << 8;
        if ((b + 2u) < frame->len) word |= (uint32_t)frame->data[b + 2u] << 16;
        if ((b + 3u) < frame->len) word |= (uint32_t)frame->data[b + 3u] << 24;
        obj[2u + w] = word;
    }

    /* Commit: advance the queue head, then request transmission. */
    dspic33ak_canfd_reg_set(regs->TXQCON, DSPIC33AK_CANFD_FIFOCON_UINC);
    dspic33ak_canfd_reg_set(regs->TXQCON, DSPIC33AK_CANFD_FIFOCON_TXREQ);
    return DSPIC33AK_CANFD_OK;
}

bool dspic33ak_canfd_rx_available(dspic33ak_canfd_instance_t inst)
{
    const dspic33ak_canfd_regs_t *regs;

    if (!dspic33ak_canfd_inst_is_valid(inst) || !g_node[inst].initialized) {
        return false;
    }
    (void)dspic33ak_canfd_get_regs(inst, &regs);
    return (*regs->FIFOSTA1 & DSPIC33AK_CANFD_FIFOSTA_TFNRFNIF) != 0u; /* not empty */
}

dspic33ak_canfd_status_t dspic33ak_canfd_receive(dspic33ak_canfd_instance_t inst,
                                                 dspic33ak_canfd_frame_t *frame)
{
    const dspic33ak_canfd_regs_t *regs;
    dspic33ak_canfd_status_t st;
    volatile uint32_t *obj;
    uint32_t id_word, ctrl;
    uint8_t dlc, w, nwords;

    if (frame == NULL) {
        return DSPIC33AK_CANFD_ERR_INVALID_ARG;
    }
    if (!dspic33ak_canfd_inst_is_valid(inst) || !g_node[inst].initialized) {
        return DSPIC33AK_CANFD_ERR_NOT_INITIALIZED;
    }
    (void)dspic33ak_canfd_get_regs(inst, &regs);

    /* Wait for a received frame (RFNIF == not empty). */
    st = wait_flag(inst, regs->FIFOSTA1, DSPIC33AK_CANFD_FIFOSTA_TFNRFNIF);
    if (st != DSPIC33AK_CANFD_OK) {
        return DSPIC33AK_CANFD_ERR_NO_MESSAGE;
    }

    obj = (volatile uint32_t *)(uintptr_t)(*regs->FIFOUA1);
    id_word = obj[0];
    ctrl    = obj[1];

    parse_id_word(id_word, (ctrl & DSPIC33AK_CANFD_OBJCTRL_IDE) != 0u, frame);
    frame->fd  = (ctrl & DSPIC33AK_CANFD_OBJCTRL_FDF) != 0u;
    frame->brs = (ctrl & DSPIC33AK_CANFD_OBJCTRL_BRS) != 0u;
    dlc = (uint8_t)(ctrl & DSPIC33AK_CANFD_OBJCTRL_DLC_MASK);
    frame->len = dspic33ak_canfd_dlc_to_len(dlc);
    if (frame->len > PAYLOAD_BYTES) {
        frame->len = PAYLOAD_BYTES;
    }

    /* Payload: read 32-bit words from message RAM (byte access faults with an
     * address-error trap on this device) and unpack into the byte buffer. */
    nwords = (uint8_t)((frame->len + 3u) / 4u);
    for (w = 0u; w < nwords; w++) {
        uint32_t word = obj[2u + w];          /* data words follow the 2 header words */
        uint8_t b = (uint8_t)(w * 4u);
        if ((b + 0u) < frame->len) frame->data[b + 0u] = (uint8_t)(word & 0xFFu);
        if ((b + 1u) < frame->len) frame->data[b + 1u] = (uint8_t)((word >> 8) & 0xFFu);
        if ((b + 2u) < frame->len) frame->data[b + 2u] = (uint8_t)((word >> 16) & 0xFFu);
        if ((b + 3u) < frame->len) frame->data[b + 3u] = (uint8_t)((word >> 24) & 0xFFu);
    }

    /* Release the object back to the FIFO. */
    dspic33ak_canfd_reg_set(regs->FIFOCON1, DSPIC33AK_CANFD_FIFOCON_UINC);
    return DSPIC33AK_CANFD_OK;
}

dspic33ak_canfd_status_t dspic33ak_canfd_deinit(dspic33ak_canfd_instance_t inst)
{
    const dspic33ak_canfd_regs_t *regs;
    dspic33ak_canfd_status_t st;

    st = dspic33ak_canfd_get_regs(inst, &regs);
    if (st != DSPIC33AK_CANFD_OK) {
        return st;
    }
    /* Idempotent: if the instance was never initialized there is nothing to tear
     * down, and the module clock may not be running — calling request_mode() here
     * could spin forever when no timeout source is configured. */
    if (!dspic33ak_canfd_is_initialized(inst)) {
        g_node[inst].initialized = false;
        return DSPIC33AK_CANFD_OK;
    }
    /* Request configuration mode then turn the module off. */
    (void)request_mode(inst, regs, DSPIC33AK_CANFD_OPMODE_CONFIG);
    dspic33ak_canfd_reg_clear(regs->CON, DSPIC33AK_CANFD_CON_ON);

    g_node[inst].initialized = false;
    dspic33ak_canfd_set_mode(inst, DSPIC33AK_CANFD_MODE_NONE);
    return DSPIC33AK_CANFD_OK;
}
