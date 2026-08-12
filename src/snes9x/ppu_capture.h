/*
 * frank-snes — PPU command stream capture (C2 PPU offload)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * The master streams every PPU-visible write to the C2 slave, which replays
 * them and renders the frame independently. Proven offline against the
 * research core: 7 games, 6,900 frames, every frame pixel-identical to a
 * normal run, with the 65816 never executing during replay.
 *
 * Measured on MK3: ~1,200 writes + 224 scanline markers per frame,
 * ~4 KB/frame = 0.2 MB/s against a 50.4 MB/s link.
 *
 * Three things this format gets right, each of which was a real bug first:
 *
 *   - The end of a frame is an EXPLICIT record, emitted where
 *     S9xEndScreenRefresh is actually called. Inferring it from the scanline
 *     counter wrapping puts the VBlank OAM DMA (~544 writes) on the wrong
 *     side of the boundary and corrupts the previous frame's sprites.
 *
 *   - DMA does NOT go through S9xSetPPU. It calls the REGISTER_21xx helpers
 *     directly from dma.c, so those call sites must be hooked too; hooking
 *     only S9xSetPPU sees 42 VRAM writes/frame instead of ~955.
 *
 *   - Only $2100..$213F is PPU state. S9xSetPPU also handles $2140-3 (APU
 *     ports) and $2180-3 (WRAM), which must not be replayed as PPU writes.
 *     The mask is 0xffc0 — 0xff40 still lets $2181-3 through.
 */
#ifndef PPU_CAPTURE_H
#define PPU_CAPTURE_H

#include <stdint.h>
#include <stdbool.h>

#define PPUCAP_WRITE 0x01   /* reg8 (low byte of $21xx), value8 */
#define PPUCAP_LINE  0x05   /* scanline8 — slave runs RenderLine() */
#define PPUCAP_ENDF  0x06   /* end of frame — slave flushes and starts anew */

#ifdef FRANK_SNES_PPU_CAPTURE

bool ppucap_init(void);                       /* allocates the PSRAM store */
const uint8_t *ppucap_take(uint32_t *len);    /* this frame's stream */
void ppucap_write(uint16_t address, uint8_t value);
void ppucap_line(uint8_t line);
void ppucap_endframe(void);

/* Bytes captured per frame, and dropped-record count. The capture's COST is
   measured by alternating it on and off (see ppu_capture.c), not by timing
   each write - that would cost more than the thing measured. */
extern volatile uint32_t frank_cap_bytes;
extern volatile uint32_t frank_cap_overflow;
extern volatile uint32_t frank_cap_on;

#define PPUCAP_WRITE_HOOK(addr, val) \
   do { if (frank_cap_on && ((addr) & 0xffc0) == 0x2100) ppucap_write((addr), (val)); } while (0)
#define PPUCAP_PORT_HOOK(port, val)  do { if (frank_cap_on) ppucap_write((port), (val)); } while (0)
#define PPUCAP_LINE_HOOK(c)          do { if (frank_cap_on) ppucap_line((c)); } while (0)
#define PPUCAP_ENDF_HOOK()           ppucap_endframe()

#else

#define PPUCAP_WRITE_HOOK(addr, val) ((void)0)
#define PPUCAP_PORT_HOOK(port, val)  ((void)0)
#define PPUCAP_LINE_HOOK(c)          ((void)0)
#define PPUCAP_ENDF_HOOK()           ((void)0)

#endif /* FRANK_SNES_PPU_CAPTURE */

#endif /* PPU_CAPTURE_H */
