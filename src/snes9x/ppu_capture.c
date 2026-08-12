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
#include "psram_allocator.h"
#ifdef PICO_ON_DEVICE
#include "pico/stdlib.h"
#endif
/* A frame is ~4 KB, but a VRAM-upload frame can push 12,000 writes, so the
   store is 48 KB and it lives in PSRAM.
   It must NOT be in SRAM: a 16 KB static buffer here hung the board - master
   SRAM went from 22,032 to 7,128 bytes free and the heap starved. There is no
   room in SRAM for a frame store, and this is the frame store. */
/* 256 KB. 48 KB was not enough: MK3's ROM/attract upload frames push ~11,955
   writes to $2118 AND $2119 in a single frame, about 72 KB of records, and a
   dropped record corrupts the slave's VRAM mirror permanently - it is not a
   cosmetic loss. Measured overflow with a 48 KB store rose continuously in
   attract mode. PSRAM has 8 MB; there is no reason to be tight here. */
#define PPUCAP_BUF_BYTES (256u * 1024u)

static uint8_t *ppucap_buf;        /* PSRAM, allocated by ppucap_init() */
static uint32_t ppucap_len;

volatile uint32_t frank_cap_bytes;
volatile uint32_t frank_cap_on = 1;   /* never cleared; see endframe */
volatile uint32_t frank_cap_overflow;


static uint32_t ppucap_total;      /* bytes the frame WOULD have produced */

bool ppucap_init(void)
{
   ppucap_buf = (uint8_t *) psram_malloc(PPUCAP_BUF_BYTES);
   return ppucap_buf != NULL;
}

/* Hand this frame's stream to the link. Valid until the next endframe. */
const uint8_t *ppucap_take(uint32_t *len)
{
   *len = frank_cap_bytes;
   return ppucap_buf;
}

static inline void ppucap_put(const uint8_t *b, uint32_t n)
{
   ppucap_total += n;
   if (!ppucap_buf || ppucap_len + n > PPUCAP_BUF_BYTES)
   {
      /* Dropping records renders a WRONG frame, so this is counted and must
         be reported, not absorbed silently. */
      frank_cap_overflow++;
      return;
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
   frank_cap_bytes = ppucap_len;   /* what is actually IN the buffer */
   ppucap_len   = 0;
   ppucap_total = 0;

   /* Capture stays ON. It alternated once, to measure its own cost against
      the same scene - and that toggle survived into the offload, where every
      OFF phase sent the slave an empty stream, so it rendered nothing and the
      screen went black. The cost measurement is not worth a variable that can
      silently stop the picture. */
}

#endif /* FRANK_SNES_PPU_CAPTURE */
