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
#include "hardware/address_mapped.h"
#include "hardware/regs/addressmap.h"
#include "hardware/xip_cache.h"
#endif
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

/* TWO buffers, alternated per frame.
 *
 * With one buffer, ppucap_endframe() resets the write offset to 0, so the next
 * frame's records overwrite the very bytes the link's DMA may still be sending.
 * The slave then received a stream whose head was always valid and whose tail
 * was garbage: it hit an invalid tag about a third of the way in, took the
 * desync exit, drew a fraction of the frame and shipped an all-zero picture -
 * with every counter on both chips reporting success. Alternating buffers
 * means the frame in flight is never the frame being written. */
static uint8_t *ppucap_buf;        /* the one being WRITTEN */
static uint8_t *ppucap_bufs[2];
static uint32_t ppucap_which;
static uint32_t ppucap_len;

volatile uint32_t frank_cap_bytes;
volatile uint32_t frank_cap_on = 1;   /* never cleared; see endframe */
volatile uint32_t frank_cap_overflow;
volatile uint32_t frank_cap_sum;
/* Min/max captured length and a take count, over whatever window the reader
   chooses to reset them across. A single once-a-second sample of
   frank_cap_bytes cannot distinguish "every frame is this size" from "this
   one frame was", and that ambiguity has already cost a debugging round. */
volatile uint32_t frank_cap_min = 0xffffffffu;
volatile uint32_t frank_cap_max;
volatile uint32_t frank_cap_takes;
volatile uint32_t frank_cap_vram_w, frank_cap_cgram_w, frank_cap_oam_w;


static uint32_t ppucap_total;      /* bytes the frame WOULD have produced */

bool ppucap_init(void)
{
   uint8_t *p = (uint8_t *) psram_malloc(PPUCAP_BUF_BYTES);
   if (!p) return false;

   uint8_t *q = (uint8_t *) psram_malloc(PPUCAP_BUF_BYTES);
   if (!q) return false;

   ppucap_bufs[0] = p;
   ppucap_bufs[1] = q;
   ppucap_buf     = p;
   return true;
}

/* Hand this frame's stream to the link. Valid until the next endframe.
 *
 * The buffer is PSRAM and the CPU writes it THROUGH the XIP cache, so those
 * writes can still be sitting in cache lines when the link's DMA reads the
 * backing store - the slave then receives a correctly-sized buffer of stale
 * bytes, replays zero records and renders nothing, while the transfer itself
 * reports complete success. Clean the range so the DMA sees what we wrote.
 *
 * The uncached alias is NOT the answer: it works for these CPU writes but the
 * slave's mirror of this buffer is DMA-written, and pointing a DMA at the
 * uncached window took its link down. Clean here, invalidate there. */
const uint8_t *ppucap_take(uint32_t *len)
{
   /* The COMPLETED buffer, which is the one endframe just stopped writing -
      not ppucap_buf, which is already collecting the next frame. */
   const uint8_t *done = ppucap_bufs[ppucap_which ^ 1u];
   *len = frank_cap_bytes;
#ifdef PICO_ON_DEVICE
   if (done && frank_cap_bytes) {
      /* BOTH ends must be cache-line aligned, not just the length. psram_malloc
         returns a pointer past its own size header, so the start is 4-byte
         aligned and rarely line-aligned; an unaligned start left the first
         line uncleaned and the slave replayed a handful of records before
         hitting stale bytes. */
      /* Clean the WHOLE buffer, not just the written extent.
       * A range clean sized to frank_cap_bytes still left the slave replaying
       * a valid head and a corrupt tail - it bailed on an invalid tag about a
       * third of the way in and drew a fraction of the frame. Cleaning the
       * whole allocation removes any dependence on getting that extent exactly
       * right; it is a cache maintenance op over PSRAM, not a copy. */
      xip_cache_clean_all();
   }
#endif
   /* AFTER the clean, over exactly the bytes the link is about to send. This
      is the master's half of the truncation check - see link_ppu_stat_t. */
   frank_cap_sum = (done && frank_cap_bytes)
                 ? ppucap_sum(done, frank_cap_bytes) : 0u;
   frank_cap_takes++;
   if (frank_cap_bytes < frank_cap_min) frank_cap_min = frank_cap_bytes;
   if (frank_cap_bytes > frank_cap_max) frank_cap_max = frank_cap_bytes;
   return done;
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
   /* Counted by destination, cumulative. The slave renders a structurally
      correct but entirely empty frame, and the two explanations - "these
      writes are never captured" and "they are captured and lost on the way" -
      need completely different fixes. The slave counts the same three; the
      pair of numbers decides it. */
   switch (r[1]) {
   case 0x18: case 0x19: frank_cap_vram_w++;  break;
   case 0x22:            frank_cap_cgram_w++; break;
   case 0x04:            frank_cap_oam_w++;   break;
   default: break;
   }
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

   /* Swap: the next frame writes the other buffer, leaving this one intact
      for however long the link takes to ship it. */
   ppucap_which ^= 1u;
   ppucap_buf    = ppucap_bufs[ppucap_which];
   ppucap_len    = 0;
   ppucap_total = 0;

   /* Capture stays ON. It alternated once, to measure its own cost against
      the same scene - and that toggle survived into the offload, where every
      OFF phase sent the slave an empty stream, so it rendered nothing and the
      screen went black. The cost measurement is not worth a variable that can
      silently stop the picture. */
}

#endif /* FRANK_SNES_PPU_CAPTURE */
