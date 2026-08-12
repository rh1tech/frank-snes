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
   .bg_enabled           = { true, true, true, true },
   .sprites_enabled      = true,
   .transparency_enabled = true,
   .crt_overscan         = false,
};

/* The renderer calls into the master's HDMI palette API. The slave drives no
 * display: it renders to a paletted buffer and returns that buffer plus the
 * palette to the master, which owns the HDMI output. So these become a local
 * palette store that the link transport will ship alongside the frame. */
uint32_t slave_palette[256];
volatile bool slave_palette_dirty;

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
bool slave_ppu_init(void)
{
   /* The tile caches are 448 KB and do NOT fit in the slave's SRAM (356 KB
      free once the renderer is linked), so they go to the PSRAM on U5, which
      was unused until now. This is the same arrangement the master uses - its
      448 KB of caches are in PSRAM too, and it still renders a frame in
      7.6 ms, so a PSRAM-resident cache is known to be fast enough.
      Everything the renderer touches per PIXEL stays in SRAM. */
   psram_init(SLAVE_PSRAM_CS_PIN);

   IPPU.TileCache[TILE_2BIT]  = (uint8_t *) psram_malloc(MAX_2BIT_TILES * 64);
   IPPU.TileCache[TILE_4BIT]  = (uint8_t *) psram_malloc(MAX_4BIT_TILES * 64);
   IPPU.TileCache[TILE_8BIT]  = (uint8_t *) psram_malloc(MAX_8BIT_TILES * 64);
   IPPU.TileCached[TILE_2BIT] = (uint8_t *) snes_calloc(MAX_2BIT_TILES, 1);
   IPPU.TileCached[TILE_4BIT] = (uint8_t *) snes_calloc(MAX_4BIT_TILES, 1);
   IPPU.TileCached[TILE_8BIT] = (uint8_t *) snes_calloc(MAX_8BIT_TILES, 1);

   /* psram_malloc does not zero; ConvertTile fills entries on demand but the
      "is it cached" flags above must start clear, and they do (snes_calloc). */

   Memory.VRAM     = (uint8_t *) snes_calloc(VRAM_SIZE, 1);
   Memory.FillRAM  = (uint8_t *) snes_calloc(0x8000, 1);
   IPPU.ScreenColors = (uint16_t *) snes_calloc(256 * 9, sizeof(uint16_t));
   IPPU.DirectColors = IPPU.ScreenColors + 256;

   if (!Memory.VRAM || !Memory.FillRAM || !IPPU.ScreenColors
       || !IPPU.TileCache[TILE_2BIT] || !IPPU.TileCache[TILE_4BIT]
       || !IPPU.TileCache[TILE_8BIT] || !IPPU.TileCached[TILE_2BIT]
       || !IPPU.TileCached[TILE_4BIT] || !IPPU.TileCached[TILE_8BIT])
      return false;

   return S9xInitGFX();
}

/* Replay one frame's worth of records. Returns when the end-of-frame record
 * is consumed, with the finished picture in GFX.Screen.
 *
 * The end of a frame is an EXPLICIT record and must not be inferred from the
 * scanline counter wrapping: S9xEndScreenRefresh runs at HCOUNTER_MAX on the
 * last line, BEFORE the VBlank OAM DMA, so inferring puts ~544 OAM writes on
 * the wrong side of the boundary and corrupts the previous frame's sprites.
 * That cost 30 of 300 frames when the replay first ran offline. */
void slave_ppu_replay(const uint8_t *rec, uint32_t len)
{
   uint32_t i = 0;

   while (i < len)
   {
      switch (rec[i])
      {
      case PPUCAP_WRITE:
         if (i + 2 >= len) return;
         S9xSetPPU(rec[i + 2], (uint16_t)(0x2100 | (rec[i + 1] & 0x3f)));
         i += 3;
         break;

      case PPUCAP_LINE:
         if (i + 1 >= len) return;
         RenderLine(rec[i + 1]);
         i += 2;
         break;

      case PPUCAP_ENDF:
         if (i + 1 >= len) return;
         S9xEndScreenRefresh();

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
         return;                 /* desynced: drop the rest of the frame */
      }
   }
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
