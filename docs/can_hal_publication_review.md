# dsPIC33AK CAN HAL Publication Review

Review target:

- Repository: `dspic33ak-can-hal`
- Local path: `C:\00_storage\_Git_Work\vscode-home\dspic33ak-can-hal`
- Reviewed state: `main` at `409ff33`
- Scope: publication-readiness review for the standalone CAN FD HAL repository
- Review mode: source/documentation review only

No source changes were made during the review.

## Overall Assessment

The repository is close to publication-ready as a small, readable standalone CAN
FD HAL.

The split from the starter project looks clean:

- The repository contains CAN HAL files only under `src/`.
- Board-specific clock, PPS, pin, PMD and transceiver standby setup are not
  mixed into the HAL.
- The HAL keeps interrupt vector ownership in the application.
- The public API avoids exposing XC-DSC / DFP bitfield types.
- The device map isolates raw `C1...` / `C2...` SFR symbols in
  `src/dspic33ak_canfd_device.c`.
- The documentation captures important hardware lessons such as RX PPS
  requirement, separate CAN CPU vectors, and word-only CAN message RAM access.

The main remaining publication risks are wording/contract mismatches around the
optional ISR/event layer, especially TX-complete.

## Positive Findings

### Repository Shape

- `src/` contains only CAN HAL source/header files.
- `docs/canfd_hal_design.md` documents design intent and hardware-specific
  pitfalls.
- `README.md` describes scope, validation target, usage, API summary, and
  out-of-scope responsibilities.
- `LICENSE` is MIT-0 style and suitable for public release.

### HAL Boundary

The HAL boundary is consistent with the other small dsPIC33AK HALs:

- The HAL does not configure PPS.
- The HAL does not enable PMD/module power.
- The HAL does not set up the CAN module clock.
- The HAL does not define `_CxRXInterrupt`, `_CxTXInterrupt`, or `_CxInterrupt`.
- The application owns message RAM and passes it via `dspic33ak_canfd_config_t`.

This is an important strength for later CMSIS-Driver CAN wrapper work.

### Message RAM Handling

The previous starter-side hard-coded `576` byte message RAM size has been
improved:

- `DSPIC33AK_CANFD_MSG_RAM_BYTES` and `DSPIC33AK_CANFD_MSG_RAM_WORDS` are
  exposed in `src/dspic33ak_canfd_node.h`.
- `src/dspic33ak_canfd_node.c` uses a `_Static_assert` to keep the public macro
  synchronized with the internal TXQ/RX FIFO geometry.

This is a good publication-readiness improvement.

### Interrupt Ownership

The HAL does not define interrupt vectors. The application is expected to forward
all relevant vectors to `dspic33ak_canfd_irq_handler()`.

The design document correctly explains that dsPIC33AK CAN uses separate CPU
vectors:

- `_CxRXInterrupt`
- `_CxTXInterrupt`
- `_CxInterrupt`

This avoids the earlier failure mode where RX events arrived in bursts because
only the general vector was handled.

## Recommended Fixes Before Publication

### 1. Clarify TX-complete Validation Status

Priority: high  
Type: documentation/API contract

`README.md` currently reads as if the optional event layer, including
TX-complete, is validated:

- `README.md` says the HAL has been validated for interrupt-driven receive via
  the optional event layer.
- `README.md` also lists `RX-available / TX-complete / bus-off / overflow`
  events in the confirmed operations section.

However, `src/dspic33ak_canfd_isr.h` says:

- RX event path is hardware-validated.
- TX-complete interrupt arming is not yet validated on dsPIC33AK512MPS512.
- Enabling it currently triggers an unhandled-interrupt trap.
- Consumers needing TX-complete should treat it as unavailable until validated.

Recommended action:

- Update `README.md` to say that the hardware-validated ISR path is
  RX-available plus status polling.
- Mark `dspic33ak_canfd_tx_start()` / TX-complete as experimental or
  not-yet-validated.
- Avoid listing TX-complete under fully confirmed operations until it is
  actually validated.

Suggested wording:

```text
Optional interrupt/event layer: RX-available event path is hardware validated.
TX-complete event support is present in the API but remains experimental until
the TX interrupt path is validated on hardware.
```

### 2. Align `dspic33ak_canfd_isr_enable()` Comment With Implementation

Priority: high  
Type: header/API documentation

`src/dspic33ak_canfd_isr.h` says `dspic33ak_canfd_isr_enable()` enables:

- RX-not-empty
- RX-overflow
- bus-error
- invalid-message

But `src/dspic33ak_canfd_isr.c` intentionally enables only:

- RX
- RX-overflow

The implementation includes a valuable note explaining that CERR/IVM interrupts
are delivered on other CPU vectors and are not enabled to avoid
`_DefaultInterrupt` traps. Bus health is instead exposed by
`dspic33ak_canfd_get_status()`.

Recommended action:

- Update the header comment to match the implementation.
- Say that `isr_enable()` enables RX-not-empty and RX-overflow only.
- Say that bus health should be queried synchronously with
  `dspic33ak_canfd_get_status()`.
- Avoid implying CERR/IVM interrupt delivery is currently enabled.

### 3. Fix README IRQ Summary Wording

Priority: medium  
Type: documentation

The README interrupt snippet correctly forwards all three vectors, but the API
summary says:

```text
dspic33ak_canfd_irq_handler() — call from the app-owned _CxInterrupt
```

This is too narrow and can mislead users into forwarding only `_CxInterrupt`.

Recommended action:

- Change the summary to say:

```text
dspic33ak_canfd_irq_handler() — call from app-owned _CxRXInterrupt,
_CxTXInterrupt and _CxInterrupt vectors
```

### 4. Document `dspic33ak_canfd_tx_start()` Blocking Behavior

Priority: medium  
Type: documentation/API expectation

`dspic33ak_canfd_tx_start()` sounds fully asynchronous, but internally it calls
`dspic33ak_canfd_transmit()`.

That means it can still wait for TX queue space if the queue is full, because
`transmit()` uses the blocking/waiting TXQ path.

Recommended action:

- Document this clearly while TX-complete remains experimental.
- Suggested wording:

```text
dspic33ak_canfd_tx_start() queues through the same TX queue path as
dspic33ak_canfd_transmit(); it may wait for TX queue space according to the
configured timeout. The TX-complete interrupt path remains experimental.
```

### 5. Consider Runtime Alignment Check for Message RAM

Priority: low to medium  
Type: optional robustness

Documentation says message RAM must be 4-byte aligned. The current
`dspic33ak_canfd_init()` checks:

- `config != NULL`
- `config->msg_ram != NULL`
- `msg_ram_size >= required size`
- mode is not `NONE`

It does not currently check pointer alignment.

Recommended action:

- Either add a small runtime alignment check:

```c
if (((uintptr_t)config->msg_ram & 0x3u) != 0u) {
    return DSPIC33AK_CANFD_ERR_INVALID_ARG;
}
```

- Or explicitly document that alignment is caller responsibility and not checked
  in Phase 1.

This is not a blocker if the validation apps use the provided
`DSPIC33AK_CANFD_MSG_RAM_WORDS` pattern with `__attribute__((aligned(4)))`.

## Additional Notes

### README Validation Claims

`README.md` says two-board CAN FD bus communication is validated on CAN1 and
CAN2. If this has been hardware-validated outside the reviewed repo, the claim
is fine. If only the starter project validated CAN1, consider softening this to
avoid overclaiming before public release.

### Error Events

The API defines:

- `DSPIC33AK_CANFD_EVENT_BUS_ERROR`
- `DSPIC33AK_CANFD_EVENT_INVALID_MSG`
- `DSPIC33AK_CANFD_EVENT_BUS_OFF`

Given the current implementation, BUS_OFF can be derived from `CxTREC` inside
the handler, but CERR/IVM interrupt sources are not enabled by
`dspic33ak_canfd_isr_enable()`. This is acceptable if documented, but the README
and header should not imply all error events are currently interrupt-driven.

### Public Header Consistency

The public headers are generally readable and wrapper-friendly. The most
important cleanup is making sure comments in `dspic33ak_canfd_isr.h` describe
the current Phase 1 behavior exactly.

## Suggested Minimal Publication Patch

For a safe first public release, a minimal patch could be documentation-only:

1. Update `README.md` confirmed operations to say:
   - RX-available event path is validated.
   - TX-complete event API exists but remains experimental.
2. Update `README.md` API summary for `dspic33ak_canfd_irq_handler()` to mention
   all three vectors.
3. Update `src/dspic33ak_canfd_isr.h` comments to say `isr_enable()` enables
   RX/RX-overflow only, while bus status is queried synchronously.
4. Optionally update `docs/canfd_hal_design.md` to mirror the TX-complete
   experimental status.

No large source rewrite appears necessary before publication.

## Final Recommendation

Recommendation: proceed toward publication after a small wording cleanup.

The current architecture is sound for an experimental, readable,
FAE/evaluation-oriented HAL. The only high-priority issue is preventing users
from misunderstanding the current interrupt/event support level, especially
around TX-complete and error-event delivery.
