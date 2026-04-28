# PS/2 Mouse Init Fix — Release Build (USB_HID=1)

## Symptom

On USB_HID=1 release builds, a real PS/2 mouse fails to initialize.
BAT completes (`0xFA 0xAA 0x00`), then the first SET_SAMPLE_RATE
parameter byte returns `0xFC` (BAT failed / error) instead of `0xFA`
(ACK), and subsequent commands cascade into the same failure. USB_HID=0
works fine with the same mouse on the same hardware. A USB→PS/2
converter also works fine on the release build — only real mice fail.

## Fix

The fix is layered. Each defense is individually necessary — disabling
any one of them causes the mouse to fail to initialize on HID=1.

Applied in `drivers/ps2/ps2.c`:

1. **Skip IntelliMouse magic knock + post-BAT config writes.**
   `mouse_reset_and_init()` now goes straight from `RESET` to
   `ENABLE_STREAM`, skipping the multi-byte sequences that trigger the
   failure. The mouse runs in its post-BAT default — 100 Hz sample
   rate, 4 counts/mm, 3-byte packets (no scroll wheel). Fully usable
   for SNES Mouse emulation.

2. **NVIC mask during the host-to-device frame.** `mouse_send_byte`
   wraps the inhibit-through-ACK sequence in
   `save_and_disable_interrupts()`. Prevents HDMI DMA IRQ (priority 0,
   ~31 kHz on Core 0) from preempting the bit-bang long enough to
   violate the device's setup time.

3. **Inter-byte gap (50 ms).** A static timestamp records the end of
   each successful (or failed) TX. The next `mouse_send_byte` call
   waits until at least 50 ms have elapsed. Works around a
   decoder-state quirk on cheap PS/2 mice that reject back-to-back
   bytes issued too close together.

4. **Bus-idle wait + atomic SIO→PIO handoff.** After ACK, spin up to
   100 µs waiting for both CLK and DATA high. Then re-init PIO pins
   and restart the SM under NVIC mask, so no IRQ or spurious edge can
   slip a partial frame into the PIO RX.

## Known side effect — HDMI resync at welcome screen

With this fix applied and a real PS/2 mouse plugged in, the HDMI signal
briefly resyncs when the welcome screen first appears. The signal
recovers on its own within ~1 second and there is no functional impact
after that point.

This is a **pre-existing electrical or timing issue**, not caused by
this fix. Evidence:

- Disabling the mouse PIO IRQ does not eliminate the resync.
- Unplugging the mouse eliminates the resync.
- On HEAD (without this fix) on USB_HID=1 the resync never appears
  because the real PS/2 mouse never successfully reaches streaming
  mode — so the symptom was latent.

Root cause is likely electrical coupling between the PS/2 lines (GPIO
0/1) and the HDMI TMDS pairs (GPIO 12-19) on the M2 board, but this
has not been confirmed with a scope. Investigating and fixing the
HDMI resync is out of scope for the mouse init fix.

## Why the original sequence fails (unconfirmed)

Evidence across 10 experimental builds:

- Failure is deterministic: second byte of a two-byte command → `0xFC`.
- Inter-byte gap (2 ms → 10 ms → 50 ms) moves the failure point but
  alone doesn't fix it.
- NVIC mask alone flips the failure byte (`0xFE` → `0xFC`) but doesn't
  fix it.
- Reordering USB init before PS/2 init has no effect.
- Full PIO SM restart (`pio_sm_restart`) actually makes it slightly
  worse.
- Frank-wolf uses byte-identical wire-level PS/2 code on the same
  hardware with the same mouse and works fine.
- Test #14 from the original prompt: merely *linking* TinyUSB host
  (without calling `tuh_init()`) is sufficient to reproduce the
  symptom.

Best-guess root cause: binary-layout-sensitive timing. Linking TinyUSB
shifts `mouse_send_byte` in flash, changes XIP cache behavior, and
nudges the exact microsecond timing of the GPIO SIO→PIO handoff.
Some specific layout allows a spurious edge or partial frame to be
captured during the handoff, producing a 22-bit sample that decodes
with valid parity as `0xFC`. Only the layered workaround (skip
multi-byte commands + NVIC mask + gap + atomic handoff) completely
eliminated the failure in our testing. Confirming the root cause
would require a logic analyzer — we don't have one.

## Tradeoffs

- **No scroll wheel detection.** SNES Mouse games don't use a wheel,
  so irrelevant for the target use case.
- **Lower sample rate** (100 Hz vs preferred 200 Hz). SNES Mouse
  protocol reports at 60 Hz, so 100 Hz input is already oversampling.
- **Lower resolution** (4 vs 8 counts/mm). Not user-visible.
- **Slightly slower init** from the 50 ms inter-byte gap, but only
  `ENABLE_STREAM` runs via the multi-byte path now, so actual added
  time is ~50 ms total.

## Files changed

- `drivers/ps2/ps2.c`:
  - Added `mouse_last_tx_us` static + `MOUSE_INTER_BYTE_GAP_US` macro.
  - `mouse_send_byte`: inter-byte-gap wait at entry; NVIC mask around
    the timing-sensitive TX section; bus-idle wait + atomic SIO→PIO
    handoff; timestamp stamp on success and failure.
  - `mouse_reset_and_init`: skip `mouse_enable_intellimouse()` +
    `SET_SAMPLE_RATE` + `SET_RESOLUTION` + `SET_SCALING_1_1`.
