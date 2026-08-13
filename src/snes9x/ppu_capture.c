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
/* The resync reads this chip's live PPU state - VRAM, CGRAM, OAM and the
   register mirror - and re-emits it as write records. */
#include "memmap.h"
#include "ppu.h"
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
volatile uint32_t frank_vram_writes;   /* see ppu.h - every real VRAM write */


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

/* ---- Full PPU state resync ------------------------------------------
 *
 * The stream only ever carries writes made SINCE the slave started
 * listening. Everything the game uploaded before link-up, and everything
 * lost while the link was down, is gone from the slave permanently: a
 * dropped VRAM write is not a cosmetic glitch, it leaves the mirror wrong
 * for as long as the game does not happen to rewrite that address.
 *
 * Measured, each chip counting its own VRAM identically: master 3,640
 * non-zero bytes of 8,192 sampled, slave 1,154, with 783,324 writes
 * replayed against 1,196,690 captured. About a third never arrived, and
 * what was missing included the tilemap - which is why the slave drew a
 * structurally perfect frame of nothing while its renderer was provably
 * able to draw (see g_ppu_selftest).
 *
 * The reconstruction is emitted as ORDINARY WRITE RECORDS: set the VRAM
 * address, then write the bytes; set the CGRAM address, then write the
 * palette. The slave replays them through the same S9xSetPPU as everything
 * else and needs no new record type, no new code and no new state machine.
 *
 * It is spread over frames rather than sent as one 200 KB burst, because
 * both halves land the stream in SRAM and neither buffer is that big - the
 * master bounces through 16 KB and the slave through 48 KB, and a stream
 * that overruns either falls back to a PSRAM DMA, which is what breaks
 * HDMI and what fails to receive.
 *
 * 512 bytes of VRAM per frame, not 2 KB. The cost of a resync chunk is not
 * the bytes on the wire, it is the RENDER: every $2118/$2119 write
 * invalidates the tile-cache entries it touches, so the slave re-converts
 * them. At 2 KB a frame the slave's render went from ~14 ms to 26 ms, which
 * is past the 20 ms frame, so core 0 blocked waiting for a staging slot, the
 * master's control frame found nobody listening ("frame header failed"), and
 * the recovery armed another resync - a feedback loop running at one link
 * failure per second. At 512 bytes the chunk is 1,539 bytes of records and a
 * full 64 KB sweep takes 128 frames, about 2.5 seconds, which only ever
 * happens on a link-up. */
#define RESYNC_VRAM_CHUNK 512u

static uint32_t resync_phase;   /* 0 idle, 1 registers+CGRAM, 2 VRAM, 3 OAM */
static uint32_t resync_off;     /* byte offset within the VRAM sweep        */
volatile uint32_t frank_cap_resyncs;

void ppucap_request_resync(void)
{
   resync_phase = 1;
   resync_off   = 0;
   frank_cap_resyncs++;
}

static void ppucap_emit(uint16_t addr, uint8_t val)
{
   uint8_t r[3];
   r[0] = PPUCAP_WRITE;
   r[1] = (uint8_t)(addr & 0x3f);
   r[2] = val;
   ppucap_put(r, 3);
}

/* Emitted at the HEAD of a frame's stream, so the reconstructed state is in
   place before that frame's own writes are applied on top of it. */
static void ppucap_emit_resync(void)
{
   if (!resync_phase || !Memory.VRAM || !Memory.FillRAM) return;

   if (resync_phase == 1) {
      /* Layout, windows and colour math. The write-twice scroll registers
         ($210D-$2114) are deliberately not reconstructed: one byte cannot
         restore a two-write latch, and games rewrite scroll every frame. */
      static const uint16_t regs[] = {
         0x2100, 0x2101, 0x2105, 0x2106, 0x2107, 0x2108, 0x2109, 0x210a,
         0x210b, 0x210c, 0x2123, 0x2124, 0x2125, 0x2126, 0x2127, 0x2128,
         0x2129, 0x212a, 0x212b, 0x212c, 0x212d, 0x212e, 0x212f, 0x2130,
         0x2131, 0x2133,
      };
      for (uint32_t i = 0; i < sizeof(regs) / sizeof(regs[0]); i++)
         ppucap_emit(regs[i], Memory.FillRAM[regs[i]]);

      /* CGRAM, all 256 entries, low byte then high. */
      ppucap_emit(0x2121, 0x00);
      for (uint32_t i = 0; i < 256u; i++) {
         ppucap_emit(0x2122, (uint8_t)(PPU.CGDATA[i] & 0xff));
         ppucap_emit(0x2122, (uint8_t)(PPU.CGDATA[i] >> 8));
      }
      resync_phase = 2;
      resync_off   = 0;
      return;
   }

   if (resync_phase == 2) {
      uint32_t n = RESYNC_VRAM_CHUNK;
      if (resync_off + n > VRAM_SIZE) n = VRAM_SIZE - resync_off;
      /* +1 word after $2119, which is what makes the pair of byte writes
         below advance one word at a time. */
      ppucap_emit(0x2115, 0x80);
      ppucap_emit(0x2116, (uint8_t)((resync_off >> 1) & 0xff));
      ppucap_emit(0x2117, (uint8_t)((resync_off >> 1) >> 8));
      for (uint32_t i = 0; i < n; i += 2u) {
         ppucap_emit(0x2118, Memory.VRAM[resync_off + i]);
         ppucap_emit(0x2119, Memory.VRAM[resync_off + i + 1u]);
      }
      resync_off += n;
      if (resync_off >= VRAM_SIZE) { resync_phase = 3; resync_off = 0; }
      return;
   }

   /* OAM, then hand the address registers back to whatever the game had
      them set to, so the resync cannot disturb a transfer in progress. */
   ppucap_emit(0x2102, 0x00);
   ppucap_emit(0x2103, 0x00);
   for (uint32_t i = 0; i < 544u; i++)
      ppucap_emit(0x2104, PPU.OAMData[i]);

   ppucap_emit(0x2115, Memory.FillRAM[0x2115]);
   ppucap_emit(0x2116, Memory.FillRAM[0x2116]);
   ppucap_emit(0x2117, Memory.FillRAM[0x2117]);
   ppucap_emit(0x2121, Memory.FillRAM[0x2121]);
   ppucap_emit(0x2102, Memory.FillRAM[0x2102]);
   ppucap_emit(0x2103, Memory.FillRAM[0x2103]);
   resync_phase = 0;
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

   /* At the head of the frame that is only now beginning, so the frame's own
      writes land on top of the reconstructed state rather than under it. */
   ppucap_emit_resync();

   /* Capture stays ON. It alternated once, to measure its own cost against
      the same scene - and that toggle survived into the offload, where every
      OFF phase sent the slave an empty stream, so it rendered nothing and the
      screen went black. The cost measurement is not worth a variable that can
      silently stop the picture. */
}

#endif /* FRANK_SNES_PPU_CAPTURE */
