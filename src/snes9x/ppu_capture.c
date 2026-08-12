/*
 * frank-snes — PPU command stream capture (C2 PPU offload)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * See ppu_capture.h for the format and the three bugs it encodes fixes for.
 *
 * This stage only fills a buffer and measures the cost; the link transport is
 * separate, so the capture overhead can be measured against the master's
 * headroom before any wire code exists. Measured headroom with the renderer
 * offloaded: the master drops from 17,305 to 9,663 us/frame against a 20,000
 * us PAL budget, so there is ~10.3 ms to spend and this must stay far inside
 * it.
 */
#include "snes9x.h"
#include "ppu_capture.h"

#ifdef FRANK_SNES_PPU_CAPTURE

#include <string.h>
#ifdef PICO_ON_DEVICE
#include "pico/stdlib.h"
#endif
/* MEASUREMENT ONLY: a small WRAPPING buffer, not a frame store.
   A 16 KB frame-sized buffer here hung the board - the master has only
   ~22 KB of SRAM free and taking 16 KB of it starved the heap/stack. The
   real transport must put the frame buffer in PSRAM or push records to the
   link incrementally; it must not claim master SRAM. 2 KB is enough to make
   the memcpy cost representative. */
#define PPUCAP_BUF_BYTES 2048u

static uint8_t  ppucap_buf[PPUCAP_BUF_BYTES];
static uint32_t ppucap_len;

volatile uint32_t frank_cap_bytes;
volatile uint32_t frank_cap_on = 1;
volatile uint32_t frank_cap_overflow;


static uint32_t ppucap_total;      /* bytes the frame WOULD have produced */

static inline void ppucap_put(const uint8_t *b, uint32_t n)
{
   ppucap_total += n;
   if (ppucap_len + n > PPUCAP_BUF_BYTES)
   {
      ppucap_len = 0;            /* wrap: measuring cost, not keeping data */
      frank_cap_overflow++;
   }
   memcpy(&ppucap_buf[ppucap_len], b, n);
   ppucap_len += n;
}

/* Deliberately NOT timed per write. At ~1,200 writes/frame a time_us_32()
   pair per call would cost far more than the capture it claims to measure -
   the same mistake that made the profiling build's event timers useless. The
   cost is measured by alternating capture on and off and comparing frame
   time, exactly as FRANK_SNES_NO_RENDER measures the renderer. */
void ppucap_write(uint16_t address, uint8_t value)
{
   uint8_t r[3];
   r[0] = PPUCAP_WRITE;
   r[1] = (uint8_t)(address & 0x3f);
   r[2] = value;
   ppucap_put(r, 3);
}

void ppucap_line(uint8_t line)
{
   uint8_t r[2];
   r[0] = PPUCAP_LINE;
   r[1] = line;
   ppucap_put(r, 2);
}

void ppucap_endframe(void)
{
   uint8_t r[2];
   r[0] = PPUCAP_ENDF;
   r[1] = 0;
   ppucap_put(r, 2);

   /* Publish and reset. The link transport will take the buffer here. */
   frank_cap_bytes = ppucap_total;
   ppucap_len   = 0;
   ppucap_total = 0;

   /* Alternate capture on and off every 10 s so its cost is measured in the
      SAME scene, as FRANK_SNES_NO_RENDER does for the renderer. Comparing
      across sessions is worthless here - frame time swings 4% with scene. */
#ifdef PICO_ON_DEVICE
   frank_cap_on = ((time_us_32() / 10000000u) & 1u) ^ 1u;
#endif
}

#endif /* FRANK_SNES_PPU_CAPTURE */
