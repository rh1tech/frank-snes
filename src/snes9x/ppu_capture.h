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
/* page8, then PPUCAP_PAGE_BYTES of VRAM. The master owns VRAM; the slave is
 * given its CONTENT, not the commands that produced it.
 *
 * Replaying $2118/$2119 requires the slave to reproduce PPU.VMA exactly -
 * address, increment, the high-byte flag, the full-graphic remapping - and
 * to invalidate the same tile-cache entries in the same order. Every one of
 * those was a separate bug, and after each was fixed the mirror still
 * diverged on most frames. Sending the bytes removes the entire class: there
 * is nothing to reproduce, and a page that differs is simply resent. */
#define PPUCAP_VPAGE 0x08
/* Master-side diagnostics, carried to the slave so they can be READ.
 *
 * The master has no console and no USB - SWD is its only channel, and when
 * that drops (see the notes on picotool and on the probe) the chip becomes
 * completely unobservable while still running. The slave has a working USB
 * console, and the master already sends it a stream every frame, so the
 * cheapest reliable telemetry path is to put a record in that stream.
 *
 * Payload: PB:PC of the 65816, then the count of consecutive frames whose
 * captured stream was byte-identical - which is what a frozen GAME looks
 * like from outside, as opposed to a frozen CPU. */
#define PPUCAP_DBG   0x09   /* pc24, stall_frames32 */
#define PPUCAP_PAGE_BITS  9u
#define PPUCAP_PAGE_BYTES (1u << PPUCAP_PAGE_BITS)          /* 512 */
#define PPUCAP_PAGES      (0x10000u / PPUCAP_PAGE_BYTES)    /* 128 */

/* The SAME function on both chips, so the two sums are comparable. It lives
 * in the shared header for exactly that reason: two hand-written copies of a
 * checksum that must agree is a bug waiting to be mistaken for a transport
 * fault. FNV-1a, one pass, no table. */
static inline uint32_t ppucap_sum(const uint8_t *p, uint32_t n)
{
   uint32_t h = 2166136261u;
   for (uint32_t i = 0; i < n; i++) { h ^= p[i]; h *= 16777619u; }
   return h;
}

#ifdef FRANK_SNES_PPU_CAPTURE

bool ppucap_init(void);                       /* allocates the PSRAM store */
const uint8_t *ppucap_take(uint32_t *len);    /* this frame's stream */
void ppucap_write(uint16_t address, uint8_t value);
void ppucap_line(uint8_t line);
void ppucap_endframe(void);
/* Rebuild the slave's entire PPU state from this chip's, as ordinary write
   records spread over the next ~32 frames. Call whenever the link comes up:
   the slave has no way to recover writes it was not listening for. */
void ppucap_request_resync(void);
extern volatile uint32_t frank_cap_resyncs;

/* Bytes captured per frame, and dropped-record count. The capture's COST is
   measured by alternating it on and off (see ppu_capture.c), not by timing
   each write - that would cost more than the thing measured. */
extern volatile uint32_t frank_cap_bytes;
extern volatile uint32_t frank_cap_overflow;
/* How many consecutive frames the captured stream has been byte-identical.
   Zero while anything is changing. */
extern volatile uint32_t frank_cap_stall_frames;
extern volatile uint32_t frank_cap_on;
/* Checksum of the stream ppucap_take() last handed to the link, over exactly
   frank_cap_bytes. The slave returns the same sum over what it received; the
   pair says whether the wire or the capture is at fault. */
extern volatile uint32_t frank_cap_sum;
/* Reset by the reader; see ppu_capture.c. */
extern volatile uint32_t frank_cap_min, frank_cap_max, frank_cap_takes;
/* Captured writes by destination, cumulative. Compared against the slave's
   own counts for the same three registers. */
extern volatile uint32_t frank_cap_vram_w, frank_cap_cgram_w, frank_cap_oam_w;
/* VRAM is hashed in blocks so a divergence can be located, not just counted. */
#define PPUCAP_VRAM_BLOCKS 32u
extern volatile uint32_t frank_cap_vram_block[PPUCAP_VRAM_BLOCKS];

/* $2118/$2119 are NOT captured: VRAM travels as pages of content. Everything
   else still travels as a write, because those registers ARE the state. */
#define PPUCAP_IS_VRAM(a) (((a) & 0x3f) == 0x18 || ((a) & 0x3f) == 0x19)
#define PPUCAP_WRITE_HOOK(addr, val) \
   do { if (frank_cap_on && ((addr) & 0xffc0) == 0x2100 && !PPUCAP_IS_VRAM(addr)) \
           ppucap_write((addr), (val)); } while (0)
#define PPUCAP_PORT_HOOK(port, val)  \
   do { if (frank_cap_on && !PPUCAP_IS_VRAM(port)) ppucap_write((port), (val)); } while (0)
#define PPUCAP_LINE_HOOK(c)          do { if (frank_cap_on) ppucap_line((c)); } while (0)
#define PPUCAP_ENDF_HOOK()           ppucap_endframe()

#else

#define PPUCAP_WRITE_HOOK(addr, val) ((void)0)
#define PPUCAP_PORT_HOOK(port, val)  ((void)0)
#define PPUCAP_LINE_HOOK(c)          ((void)0)
#define PPUCAP_ENDF_HOOK()           ((void)0)

#endif /* FRANK_SNES_PPU_CAPTURE */

#endif /* PPU_CAPTURE_H */
