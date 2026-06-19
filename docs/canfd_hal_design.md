# dsPIC33AK CAN FD HAL — design notes

## Layering

```
dspic33ak_canfd.h        public types: instance / status / mode enums, get_ms typedef, lifecycle
dspic33ak_canfd_reg.h    register pointer struct + bit masks + msg-object layout (internal only)
dspic33ak_canfd_device.* instance -> concrete SFR map (the only place naming C1.../C2... ), #if defined(C1CON)
dspic33ak_canfd_common.* inst validation, get_regs, calc_bit_timing(), mode bookkeeping
dspic33ak_canfd_node.*   blocking core: init / transmit / receive / set_bitrate
dspic33ak_canfd_isr.*    OPTIONAL interrupt/event layer (additive, opt-in)
```

`_reg.h` is never included by a public header. Only `_device.c` and the internal
`_common.h` see the raw SFR names, so the public API stays free of XC-DSC / DFP
bitfield types — the same policy as the other dsPIC33AK HALs.

## Board responsibilities (NOT in the HAL)

The HAL never touches power, clock or pins. Before `dspic33ak_canfd_init()` the
board/application must:

1. Enable the module: `PMD3bits.CxMD = 0`.
2. Provide the CAN module clock (FCAN). 20 MHz is the validated value
   (`config.can_clk_hz`); the bit-timing solver works for other clocks too.
3. Map PPS: TX out, **RX in**, and drive the transceiver standby pin. The CAN
   **RX PPS input must be assigned even for internal loopback** — otherwise the
   module never integrates (11 recessive bits) and `init` returns `ERR_TIMEOUT`
   stuck in Configuration mode.

The application also owns the **message RAM** region (4-byte aligned,
`>= dspic33ak_canfd_msg_ram_size()` bytes) and passes it via the config.

## Hardware notes that shaped the code

* **Message RAM is word-access only.** Byte access to CAN message RAM faults
  with an address-error trap; payloads are packed/unpacked as 32-bit words
  (header words T0/R0, T1/R1, then data little-endian).
* **Bit timing** is computed by `calc_bit_timing(fcan, nominal, data, sample%)`;
  register fields are `value - 1`. At FCAN 20 MHz, 500 k / 2 M, 80% SP the
  validated values are NBTCFG=0x001E0707, DBTCFG=0x00060101, TDC=0x00020700.
* **set_bitrate** re-applies timing without a full re-init: it drops the
  instance to Configuration mode, writes NBTCFG/DBTCFG/TDC, then restores the
  previous operating mode (timing registers are config-locked).

## Interrupt/event layer (`_isr`)

Additive and opt-in; the blocking core works without it. It provides an event
callback (RX-available / TX-complete / bus-off / RX-overflow / invalid-message),
`isr_enable/disable`, an async `tx_start`, and a synchronous `get_status`
(CxTREC decode: TEC/REC, error-warning, error-passive, bus-off).

**Separate CPU vectors.** On dsPIC33AK the CAN FD module raises *separate* CPU
interrupt vectors: `CxRX` (receive FIFO), `CxTX` (transmit) and `Cx`
(general/error). The RX-FIFO-not-empty interrupt arrives on **`_CxRXInterrupt`**,
not the general `_CxInterrupt`. `isr_enable` arms, and `irq_handler` clears, all
three lines; the application must forward **all three** vectors to
`dspic33ak_canfd_irq_handler()`. Forwarding only `_CxInterrupt` services RX only
on FIFO overflow (frames then arrive in bursts of the FIFO depth).

**Signal-only RX.** `irq_handler` signals `RX_AVAILABLE`; it does not drain the
FIFO. The consumer drains with `dspic33ak_canfd_receive()`. If the read is
deferred (e.g. a CMSIS-Driver-style "read in the RECEIVE event" model), drain the
FIFO inside the callback into a software queue so the level-sensitive RX
interrupt does not re-assert and starve the main loop.

**Keep the callback short.** It runs in CAN ISR context; do not printf or call
blocking APIs from it, and do not call the blocking `transmit()` from inside the
ISR (its timeout uses the ms tick, which a lower-priority timer ISR cannot
advance while the CAN ISR holds the CPU).

## Instance map

`dspic33ak_canfd_device.c` is the only file naming `C1...` / `C2...`. Each row is
emitted only when the device header defines that instance's SFRs
(`#if defined(C1CON)` / `#if defined(C2CON)`), so the table tracks the silicon
and the rest of the HAL stays device-neutral.
