/*
 * frank-snes — C2 slave: PPU command-stream replay
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * The master streams every PPU-visible write, one scanline marker per line,
 * and an explicit end-of-frame record. This replays them through the same
 * S9xSetPPU / RenderLine / S9xUpdateScreen the master used to run, and the
 * frame comes out identical — proven offline against the research core on
 * 7 games and 6,900 frames, with the 65816 never executing.
 *
 * Why this can work at all, where moving the SPC700 here could not: sound
 * needed synchronous round trips (8,700 port polls/frame against a 53-58 us
 * doorbell). Rendering needs none. The stream is one-way and the framebuffer
 * goes back one frame later.
 *
 * Measured prize on the master: dropping the renderer takes it from 17,305 to
 * 9,663 us/frame and 46 to 50 fps.
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "snes9x.h"
#include "memmap.h"
#include "ppu.h"
#include "gfx.h"
#include "ppu_capture.h"
#include "snes_alloc.h"
#include "psram_allocator.h"
#include "psram_init.h"
#include "hardware/regs/addressmap.h"
#include "hardware/xip_cache.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "settings.h"

/* RP2350A QFN-60: the slave's PSRAM chip select is GPIO0 (slave/CMakeLists). */
#define SLAVE_PSRAM_CS_PIN 0

/* The renderer reads four fields of g_settings (bg_enabled, sprites_enabled,
 * transparency_enabled, crt_overscan). The master's settings.c cannot come
 * with it - that file loads the settings from the SD card through FatFS, and
 * the slave has no SD. The master owns the user's settings and will send them
 * over the link with the stream; until then these are the shipping defaults.
 *
 * They must match the master's, or the two halves render differently. */
settings_t g_settings = {
   /* bg_enabled is a BITMASK (bit 0 = BG1 ... bit 3 = BG4), not an array of
      bools. Writing { true, true, true, true } into a scalar takes the first
      element, so this was 1 - BG1 only, every other layer suppressed. */
   .bg_enabled           = 0x0f,
   .sprites_enabled      = true,
   .transparency_enabled = true,
   .crt_overscan         = false,
   .hdma_enabled         = true,
};

/* The renderer calls into the master's HDMI palette API. The slave drives no
 * display: it renders to a paletted buffer and returns that buffer plus the
 * palette to the master, which owns the HDMI output. So these become a local
 * palette store that the link transport will ship alongside the frame. */
uint32_t slave_palette[256];
volatile bool slave_palette_dirty;

/* Diagnostics returned to the master each frame - see link_ppu_stat_t. */
volatile uint32_t slave_ppu_render_us;
volatile uint32_t slave_ppu_records;
volatile uint32_t slave_ppu_psram_ok;
volatile uint32_t slave_ppu_alloc_fail;
/* Largest power-of-two malloc the newlib heap will serve, in KB. GFX.ZERO
   needs 128; anything less and S9xInitGFX cannot succeed no matter what else
   is fixed. Zero means even 8 KB failed. */
volatile uint32_t slave_ppu_heap_max_kb;
/* bit0 = the 22,972 calloc succeeded, bit1 = the 128 KB malloc succeeded. */
volatile uint32_t slave_ppu_alloc_probe;
/* How far into a replay core 1 got before it stopped. The slave has no
   working SWD, so its PC cannot be read: this is the substitute. */
volatile uint32_t slave_ppu_stage;
volatile uint32_t slave_ppu_live_recs;
volatile uint32_t slave_ppu_last_tag;
volatile uint32_t slave_ppu_bad_lines;
/* Where and why the replay stopped, and a checksum of what it replayed from.
   See link_ppu_stat_t: these three numbers plus the master's frank_cap_sum
   separate "the wire corrupted it" from "the capture produced it" from "the
   master under-sent it" in a single run. */
volatile uint32_t slave_ppu_stream_len;
volatile uint32_t slave_ppu_stream_sum;
volatile uint32_t slave_ppu_exp_sum;    /* set by the link, from the payload */
volatile uint32_t slave_ppu_sum_ok;
volatile uint32_t slave_ppu_sum_bad;
volatile uint32_t slave_ppu_stop_off;
volatile uint32_t slave_ppu_stop_ctx;
volatile uint32_t slave_ppu_stop_why;   /* 1 bad tag, 2 truncated, 0 clean */
/* Does the slave hold any PPU state to draw FROM?
 *
 * A clean stream that draws nothing has two possible causes, and they need
 * different fixes: the renderer never runs (upd_calls stays 0), or it runs
 * against an empty machine - no VRAM uploaded, no palette, no layer enabled.
 * The stream only carries writes made SINCE the link came up, so a slave that
 * joined after the game programmed its PPU has nothing, and that is a design
 * gap rather than a bug in the transport. */
volatile uint32_t slave_ppu_upd_calls;  /* S9xUpdateScreen calls, cumulative */
volatile uint32_t slave_ppu_vram_w;     /* $2118/$2119 writes, cumulative    */
volatile uint32_t slave_ppu_cgram_w;    /* $2122 writes, cumulative          */
volatile uint32_t slave_ppu_r2100_w;    /* $2100 writes, cumulative          */
volatile uint32_t slave_ppu_r2100_last; /* the value of the last one         */
volatile uint32_t slave_ppu_r2100_seen; /* bitmap of high nibbles ever seen  */
/* Bisect: replay the register writes but draw nothing. Separates "the stream
   handling hangs" from "the renderer hangs". */
volatile uint32_t slave_ppu_skip_lines = 0;
uint8_t *slave_ppu_stream_buf;
uint8_t *slave_ppu_screen;   /* set per frame: where this frame is drawn */
volatile bool slave_psram_ready;

void graphics_set_palette(uint8_t i, uint32_t color888)
{
   slave_palette[i] = color888;
}

void graphics_request_palette_update(void)
{
   slave_palette_dirty = true;
}

/* The renderer's own state. On the master these are set up by S9xInitMemory
 * and S9xInitGFX, which drag in the ROM loader, the CPU and the APU. The
 * slave needs only the PPU-visible subset. */
/* Init stage trace.
 *
 * slave_ppu_init faults somewhere and takes USB-CDC down with it, so the only
 * evidence that survives is what reached the host BEFORE the fault. Each
 * marker is followed by a short sleep because CDC delivers from a timer
 * callback: without it the last line sits in the buffer and dies with the
 * device, which is precisely the line that names the culprit. Boot-time only. */
#define PSTAGE(...) do { printf("[ppu] " __VA_ARGS__); printf("\n"); \
                         sleep_ms(40); watchdog_update(); } while (0)

bool slave_ppu_init(void)
{
   /* The tile caches are 448 KB and do NOT fit in the slave's SRAM (356 KB
      free once the renderer is linked), so they go to the PSRAM on U5, which
      was unused until now. This is the same arrangement the master uses - its
      448 KB of caches are in PSRAM too, and it still renders a frame in
      7.6 ms, so a PSRAM-resident cache is known to be fast enough.
      Everything the renderer touches per PIXEL stays in SRAM. */
   PSTAGE("psram_init...");
   psram_init(SLAVE_PSRAM_CS_PIN);
   slave_psram_ready = true;   /* unblocks PSRAM routing in snes_alloc.h */
   PSTAGE("psram ok");

   IPPU.TileCache[TILE_2BIT]  = (uint8_t *) psram_malloc(MAX_2BIT_TILES * 64);
   IPPU.TileCache[TILE_4BIT]  = (uint8_t *) psram_malloc(MAX_4BIT_TILES * 64);
   IPPU.TileCache[TILE_8BIT]  = (uint8_t *) psram_malloc(MAX_8BIT_TILES * 64);
   IPPU.TileCached[TILE_2BIT] = (uint8_t *) snes_calloc(MAX_2BIT_TILES, 1);
   IPPU.TileCached[TILE_4BIT] = (uint8_t *) snes_calloc(MAX_4BIT_TILES, 1);
   IPPU.TileCached[TILE_8BIT] = (uint8_t *) snes_calloc(MAX_8BIT_TILES, 1);

   /* psram_malloc does not zero; ConvertTile fills entries on demand but the
      "is it cached" flags above must start clear, and they do (snes_calloc). */

   PSTAGE("tilecache %p %p %p cached %p",
          IPPU.TileCache[TILE_2BIT], IPPU.TileCache[TILE_4BIT],
          IPPU.TileCache[TILE_8BIT], IPPU.TileCached[TILE_2BIT]);
   slave_ppu_stream_buf = (uint8_t *) psram_malloc(256u * 1024u);
   PSTAGE("stream %p", slave_ppu_stream_buf);

   /* GFX must be set up before S9xInitGFX: it takes GFX.Pitch as an INPUT and
      copies it to RealPitch. Leaving it zero made slave_ppu_copy_frame
      compute a zero-byte picture, so the master received nothing and the
      screen stayed blank - with every other diagnostic reading healthy.
      Mirrors the master's S9xInitDisplay, except that the sub-screen and both
      Z buffers go to PSRAM: the slave has ~82 KB of SRAM free against 172 KB
      of buffers, and it renders a frame in ~200 us of a ~7,600 us budget, so
      it can afford the slower memory far more easily than the space.
      GFX.Screen is not allocated at all - the renderer draws straight into
      the framebuffer that goes on the wire. */
   GFX.Pitch  = SNES_WIDTH;
   GFX.ZPitch = SNES_WIDTH;
   GFX.SubScreen  = (uint8_t *) psram_malloc(SNES_WIDTH * SNES_HEIGHT);
   GFX.ZBuffer    = (uint8_t *) psram_malloc(SNES_WIDTH * SNES_HEIGHT);
   GFX.SubZBuffer = (uint8_t *) psram_malloc(SNES_WIDTH * SNES_HEIGHT);
   if (GFX.SubScreen) memset(GFX.SubScreen, 0, SNES_WIDTH * SNES_HEIGHT);

   /* These go to PSRAM, not the SRAM bump heap. S9xInitGFX allocates GFX.ZERO
      with a PLAIN malloc of 128 KB, which must come from the newlib heap
      between .bss and the stack - so that heap has to stay large. Holding
      VRAM (64 KB) and FillRAM (32 KB) in SRAM starved it, S9xInitGFX returned
      false, and slave_ppu_init bailed before setting RenderThisFrame. The
      slave renders in ~50-350 us of a ~7,600 us budget, so it can pay PSRAM
      latency far more easily than it can find the space. */
   /* VRAM in SRAM. It is read PER PIXEL by every tile fetch, so PSRAM
      latency lands in the renderer's innermost loop: with it in PSRAM a full
      frame replay cost 29,371 us against the master's 7,642 us for the same
      work, and the slave could not hold 50 fps. It only went to PSRAM when the
      SRAM bump heap was starved; that heap has since dropped from 136 KB to
      40 KB (it had been sized for an echo buffer the slave never allocates),
      so the 64 KB fits comfortably. */
   Memory.VRAM     = (uint8_t *) snes_malloc(VRAM_SIZE);
   if (!Memory.VRAM)   /* fall back rather than fail to render at all */
      Memory.VRAM  = (uint8_t *) psram_malloc(VRAM_SIZE);
   Memory.FillRAM  = (uint8_t *) psram_malloc(0x8000);
   IPPU.ScreenColors = (uint16_t *) psram_malloc(256 * 9 * sizeof(uint16_t));
   if (Memory.VRAM)      memset(Memory.VRAM, 0, VRAM_SIZE);
   if (Memory.FillRAM)   memset(Memory.FillRAM, 0, 0x8000);
   if (IPPU.ScreenColors) memset(IPPU.ScreenColors, 0, 256 * 9 * sizeof(uint16_t));
   IPPU.DirectColors = IPPU.ScreenColors + 256;
   PSTAGE("vram %p fill %p colors %p sub %p zb %p szb %p",
          Memory.VRAM, Memory.FillRAM, IPPU.ScreenColors,
          GFX.SubScreen, GFX.ZBuffer, GFX.SubZBuffer);

   /* Report WHICH allocation failed: a single psram_ok=0 said only that
      something did, and the something turned out to be an SRAM allocation,
      not PSRAM at all. */
   slave_ppu_alloc_fail =
        (!Memory.VRAM              ? 0x01u : 0u)
      | (!Memory.FillRAM           ? 0x02u : 0u)
      | (!IPPU.ScreenColors        ? 0x04u : 0u)
      | (!IPPU.TileCache[TILE_2BIT]? 0x08u : 0u)
      | (!IPPU.TileCache[TILE_4BIT]? 0x10u : 0u)
      | (!IPPU.TileCache[TILE_8BIT]? 0x20u : 0u)
      | (!IPPU.TileCached[TILE_2BIT]?0x40u : 0u);

   slave_ppu_alloc_fail |= (!slave_ppu_stream_buf ? 0x80u : 0u)
                        |  (!GFX.SubScreen        ? 0x100u : 0u)
                        |  (!GFX.ZBuffer          ? 0x200u : 0u);

   if (!GFX.SubScreen || !GFX.ZBuffer || !GFX.SubZBuffer ||
       !slave_ppu_stream_buf ||
       !Memory.VRAM || !Memory.FillRAM || !IPPU.ScreenColors
       || !IPPU.TileCache[TILE_2BIT] || !IPPU.TileCache[TILE_4BIT]
       || !IPPU.TileCache[TILE_8BIT] || !IPPU.TileCached[TILE_2BIT]
       || !IPPU.TileCached[TILE_4BIT] || !IPPU.TileCached[TILE_8BIT])
      return false;


   slave_ppu_psram_ok = (IPPU.TileCache[TILE_2BIT] != NULL) &&
                        (IPPU.TileCache[TILE_4BIT] != NULL) &&
                        (IPPU.TileCache[TILE_8BIT] != NULL);

   /* The newlib-heap probes that used to sit here are gone, and the reason is
      worth keeping: they PANICKED the chip. pico_malloc's __wrap_malloc calls
      panic() on failure instead of returning NULL (PICO_MALLOC_PANIC is on by
      default), so a probe written to ask "does 128 KB fit?" cannot answer no -
      it kills the board. It answered "yes" while the bump heap was 16 KB, then
      brought the firmware down once the heap grew to 136 KB and left ~95 KB of
      newlib heap behind.

      Nothing on this path allocates from newlib any more: LocalState goes to
      the SRAM bump heap and GFX.ZERO to PSRAM, both via snes_alloc.h. */
   if (!S9xInitGFX()) {
      slave_ppu_alloc_fail |= 0x400u;
      return false;
   }

   /* The slave has never had a PPU reset - the master does that inside
      S9xReset, which is CPU-side and does not come with the renderer. The
      consequence that mattered: IPPU.RenderThisFrame was zero, and BOTH
      RenderLine and S9xUpdateScreen are guarded by it, so the slave replayed
      every record and drew nothing. The symptom was a perfectly sized,
      perfectly delivered framebuffer of colour 0 with every counter healthy.

      Calling S9xResetPPU() outright is NOT the fix: it is a whole-machine
      reset that reaches into CPU-era state the slave does not have, and it
      took the link down at handshake. Everything else the renderer needs
      arrives in the stream - the master's own register writes set it. Only
      these two are never transmitted. */
   PSTAGE("S9xInitGFX ok, ZERO %p", (void *)GFX.ZERO);
   IPPU.RenderThisFrame = true;
   PPU.ScreenHeight     = SNES_HEIGHT;
   PSTAGE("init complete");
   return true;
}

/* Replay one frame's worth of records. Returns when the end-of-frame record
 * is consumed, with the finished picture in GFX.Screen.
 *
 * The end of a frame is an EXPLICIT record and must not be inferred from the
 * scanline counter wrapping: S9xEndScreenRefresh runs at HCOUNTER_MAX on the
 * last line, BEFORE the VBlank OAM DMA, so inferring puts ~544 OAM writes on
 * the wrong side of the boundary and corrupts the previous frame's sprites.
 * That cost 30 of 300 frames when the replay first ran offline. */
/* The link's DMA writes this buffer and the CPU reads it here. Reading it
 * through the XIP cache returns stale lines - the replay then hits an invalid
 * first record and stops, so a transfer that succeeded in every counter
 * produces zero records and a blank frame. Invalidate before reading.
 *
 * Pointing the DMA at the uncached alias instead does NOT work: it took the
 * link down entirely. The master cleans before its DMA reads; this end
 * invalidates after its DMA writes. */
void slave_ppu_replay(const uint8_t *rec, uint32_t len)
{
   uint32_t i = 0;
   slave_ppu_stage = 1;
   uint32_t t0 = time_us_32();
   slave_ppu_stage = 2;          /* time_us_32 returned */

   /* A PSRAM stream still needs its cache lines invalidated before the CPU
    * reads what the link's DMA just wrote; an SRAM stream needs nothing.
    *
    * It must be the whole-cache operation, NOT xip_cache_invalidate_range().
    * Cache maintenance BY ADDRESS is broken on RP2350 by erratum E11 - the
    * SDK works around it in the by-set/way functions and says so in
    * xip_cache.h - so the range call left an arbitrary subset of the lines
    * stale. The replay then read a patchwork of this frame's bytes and the
    * previous frame's, which parsed as valid records often enough to look
    * like a working transport: 225 records replayed, every counter healthy,
    * no picture. Proven by putting the master's first 32 stream bytes in the
    * control frame and printing them beside the slave's:
    *   sent: 05 00 05 01 05 02 05 03 05 04 ...
    *   got : 01 15 80 01 00 00 01 32 00 01 ...  (the frame before, 24 bytes
    *                                             of it, then the real stream)
    * The master's ppucap_take() already had to reach for the whole-cache form
    * for exactly this reason, and its comment describes the same symptom from
    * the other end.
    *
    * clean_all, not invalidate_all: the slave writes its PSRAM tile caches
    * THROUGH this cache, and invalidate_all discards pending write data.
    * clean_all writes those lines out, and - per the E11 workaround it
    * applies - leaves every line tagged such that the next access misses,
    * which is the invalidate this needs.
    *
    * Do NOT "simplify" this to the uncached alias at 0x14000000: that was
    * tried, and for a PSRAM buffer it lands at 0x15000000, which faults - the
    * slave hung on its first upload frame and the watchdog bounced it into
    * BOOTSEL three resets later. Cache maintenance is correct here; the alias
    * is not. */
   /* XIP is a RANGE, not a half-line. SRAM starts at 0x20000000, which is
      also >= XIP_BASE, so a `>= XIP_BASE` test called the whole-cache
      maintenance on every frame once the stream moved to an SRAM landing
      zone - invalidating the PSRAM tile caches, SubScreen and ZBuffer along
      with it, so the renderer re-fetched all of them from PSRAM every frame.
      An SRAM stream needs no cache maintenance at all. */
   if (len && (uintptr_t)rec >= XIP_BASE && (uintptr_t)rec < XIP_END)
      xip_cache_clean_all();

   slave_ppu_stage = 3;          /* cache guard passed */
   GFX.Screen = slave_ppu_screen;
   uint32_t n  = 0;
   slave_ppu_stage = 4;

   /* Sum what is about to be replayed, over exactly the bytes the master
      summed. Reading the whole stream once costs a few microseconds of a
      ~7,600 us budget and is the only way to tell a corrupt delivery from a
      corrupt capture. */
   slave_ppu_stream_len = len;
   slave_ppu_stream_sum = ppucap_sum(rec, len);
   if (slave_ppu_stream_sum == slave_ppu_exp_sum) slave_ppu_sum_ok++;
   else                                           slave_ppu_sum_bad++;
   slave_ppu_stop_why   = 0;
   slave_ppu_stop_off   = len;
   slave_ppu_stop_ctx   = 0;

   while (i < len)
   {
      slave_ppu_live_recs++;
      slave_ppu_last_tag = rec[i];
      switch (rec[i])
      {
      case PPUCAP_WRITE:
         if (i + 2 >= len) { slave_ppu_stop_why = 2; goto done; }
         slave_ppu_stage = 20;
         if ((rec[i + 1] & 0x3f) == 0x18 || (rec[i + 1] & 0x3f) == 0x19)
            slave_ppu_vram_w++;
         else if ((rec[i + 1] & 0x3f) == 0x22)
            slave_ppu_cgram_w++;
         else if ((rec[i + 1] & 0x3f) == 0x00) {
            /* $2100. The master reports INIDISP=0x0f and the slave's mirror
               reads 0x80, so the whole screen renders force-blanked. The
               store in S9xSetPPU is unconditional, so either this write never
               arrives or it arrives with a different value. */
            slave_ppu_r2100_w++;
            slave_ppu_r2100_last = rec[i + 2];
            slave_ppu_r2100_seen |= 1u << (rec[i + 2] >> 4);
         }
         S9xSetPPU(rec[i + 2], (uint16_t)(0x2100 | (rec[i + 1] & 0x3f)));
         slave_ppu_stage = 21;
         i += 3;
         break;

      case PPUCAP_LINE:
         if (i + 1 >= len) { slave_ppu_stop_why = 2; goto done; }
         /* A scanline number out of range walks the renderer off the end of
            GFX.Screen and corrupts whatever follows it, which does not fail
            where it happens. The stream should never carry one; if it does,
            skipping the line is survivable and a corrupted heap is not. */
         /* Bounded by the FRAMEBUFFER, not by the SNES maximum.
            GFX.Screen is 256x224 = 57,344 bytes because that is what
            LINK_PPU_MAX_BYTES carries. A stream that selects overscan asks
            for 239 lines - 61,184 bytes - and rendering it walks 3,840 bytes
            past the end of the buffer into whatever follows, which is how the
            slave died on the first upload frames after every link-up: not
            where the corruption happened, and with no message. SNES_HEIGHT_-
            EXTENDED here permits exactly that overrun, so it must be
            SNES_HEIGHT. */
         if (rec[i + 1] < SNES_HEIGHT && !slave_ppu_skip_lines) {
            slave_ppu_stage = 10;
            RenderLine(rec[i + 1]);
            slave_ppu_stage = 11;
         } else {
            slave_ppu_bad_lines++;
         }
         i += 2;
         break;

      case PPUCAP_ENDF:
         if (i + 1 >= len) { slave_ppu_stop_why = 2; goto done; }
         S9xEndScreenRefresh();
         /* Sampled HERE, between the two calls: S9xStartScreenRefresh below
            zeroes g_upd_screen_calls, so reading it anywhere else always
            returns 0 and looks like "the renderer never ran". */
         { extern uint32_t g_upd_screen_calls;
           slave_ppu_upd_calls += g_upd_screen_calls; }

         /* The VBlank OAM reload lives in the master's event ring, which is
            not part of the stream. The slave owns frame boundaries, so it
            performs the same reload — hardware behaviour, not a workaround. */
         PPU.ForcedBlanking = (Memory.FillRAM[0x2100] >> 7) & 1;
         if (!PPU.ForcedBlanking)
         {
            uint8_t tmp = 0;
            PPU.OAMAddr = PPU.SavedOAMAddr;
            if (PPU.OAMPriorityRotation)
               tmp = (PPU.OAMAddr & 0xFE) >> 1;
            if ((PPU.OAMFlip & 1) || PPU.FirstSprite != tmp)
            {
               PPU.FirstSprite = tmp;
               IPPU.OBJChanged = true;
            }
            PPU.OAMFlip = 0;
         }

         S9xStartScreenRefresh();
         i += 2;
         break;

      default:
         slave_ppu_stop_why = 1;
         goto done;              /* desynced: drop the rest of the frame */
      }
      n++;
   }
done:
   if (slave_ppu_stop_why) {
      slave_ppu_stop_off = i;
      /* The four bytes AT the stop, so the bad tag and its neighbours can be
         read rather than inferred. Byte-wise: rec may be unaligned. */
      uint32_t c = 0;
      for (uint32_t k = 0; k < 4u && i + k < len; k++)
         c |= (uint32_t)rec[i + k] << (8u * k);
      slave_ppu_stop_ctx = c;
   }
   slave_ppu_render_us = time_us_32() - t0;
   slave_ppu_records   = n;
}

/* Copy the finished picture out of GFX.Screen for transmission. The renderer
 * draws 8bpp paletted at GFX.Pitch bytes per line, which is what the master's
 * HDMI path already expects, so this is a straight copy of the visible lines
 * and no format conversion happens on either chip. */
uint32_t slave_ppu_copy_frame(uint8_t *dst, uint32_t max)
{
   uint32_t h = (uint32_t) PPU.ScreenHeight;
   uint32_t w = GFX.Pitch;
   uint32_t n = w * h;
   if (!dst || n > max) return 0;
   memcpy(dst, GFX.Screen, n);
   return n;
}

/* The renderer guards every draw on IPPU.RenderThisFrame. Nothing on the slave
   clears it, but arming it explicitly each frame keeps the intent visible -
   without it the slave replays the whole stream and draws nothing at all. */
void slave_ppu_arm_frame(void)
{
   IPPU.RenderThisFrame = true;

   /* Re-assert the height every frame. The replayed stream contains the
      register writes that select overscan ($2133 bit 2), so PPU.ScreenHeight
      does not stay where slave_ppu_init put it - and the framebuffer on the
      wire is a fixed 224 lines. Clamping here keeps the renderer inside the
      buffer no matter what the stream asks for. */
   PPU.ScreenHeight = SNES_HEIGHT;
}

/* The renderer replays every record and still draws nothing, so report the
   state the draw path actually gates on. */
uint32_t slave_ppu_dbg_pitch_h(void)
{
   return (uint32_t)GFX.Pitch | ((uint32_t)PPU.ScreenHeight << 16);
}

/* The four registers that decide whether anything reaches the screen at all:
   INIDISP (forced blank + brightness), BGMODE, and the main/sub screen layer
   enables. All zero means an unprogrammed PPU, not a broken renderer. */
uint32_t slave_ppu_dbg_regs(void)
{
   if (!Memory.FillRAM) return 0;
   return (uint32_t)Memory.FillRAM[0x2100]
        | ((uint32_t)Memory.FillRAM[0x2105] << 8)
        | ((uint32_t)Memory.FillRAM[0x212c] << 16)
        | ((uint32_t)Memory.FillRAM[0x212d] << 24);
}

/* Is there anything IN the machine to draw?
 *
 * A clean stream, open draw gates and an all-zero picture leave exactly two
 * possibilities, and counting the bytes separates them: VRAM and the palette
 * hold real data and the renderer is at fault, or they are empty and the
 * slave is simply missing the state the game established before it was
 * listening. Returns non-zero VRAM bytes, capped, in the low 16 bits and
 * non-zero palette entries in the high 16. */
uint32_t slave_ppu_dbg_content(void)
{
   uint32_t v = 0, c = 0;
   if (Memory.VRAM)
      for (uint32_t i = 0; i < VRAM_SIZE; i += 8)   /* sampled, not summed */
         if (Memory.VRAM[i]) v++;
   for (uint32_t i = 0; i < 256; i++)
      if (slave_palette[i]) c++;
   return (v & 0xffffu) | (c << 16);
}

uint32_t slave_ppu_dbg_flags(void)
{
   extern uint32_t g_upd_screen_calls;
   return (PPU.ForcedBlanking      ? 1u : 0u)
        | (IPPU.RenderThisFrame    ? 2u : 0u)
        | (GFX.SubScreen           ? 4u : 0u)
        | (GFX.Screen              ? 8u : 0u)
        | (g_upd_screen_calls << 8);
}

