/*
 * frank-snes — C2 slave firmware
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * slave_glue.c — the little that the slave needs beyond the DSP itself.
 *
 * This used to carry a dozen snes9x globals, because the legacy mixer
 * reached for Settings, SoundData, APU, IAPU and the rest. The accurate
 * DSP in src/snes9x/spc_dsp.c reaches for nothing: it owns its own
 * state and is handed a pointer to APU RAM. So all that is left here is
 * the allocator, and even that is only kept because the shared tree's
 * headers expect it to exist.
 */

#include <stdint.h>
#include <string.h>


/* ------------------------------------------------------------------ */
/* Allocator                                                          */
/* ------------------------------------------------------------------ */

/* The accurate DSP allocates nothing — it has no echo buffer of its own,
 * because the real hardware's echo lives inside APU RAM and so does
 * this one's. 8 KB is a courtesy for anything in the shared headers
 * that still calls snes_malloc(). */
/* 8 KB was right when the slave only ran the S-DSP. The renderer needs its
   PPU-visible state in SRAM - VRAM 64 KB, FillRAM 32 KB, ScreenColors ~4.6 KB,
   the tile-cached flags ~7 KB - and VRAM in particular is read per pixel, so
   it cannot go to PSRAM with the tile caches.
   Undersizing this does not fail loudly: snes_calloc returns NULL, the
   renderer initialises with no state, and the screen is simply black. */
#ifdef FRANK_SNES_PPU_SLAVE
#define SLAVE_HEAP_BYTES (120u * 1024u)
#else
#define SLAVE_HEAP_BYTES (8u * 1024u)
#endif

static uint8_t __attribute__((aligned(8))) slave_heap[SLAVE_HEAP_BYTES];
static size_t  slave_heap_used;

void *slave_alloc(size_t size)
{
    size = (size + 7u) & ~(size_t)7u;
    if (slave_heap_used + size > SLAVE_HEAP_BYTES)
        return NULL;
    void *p = slave_heap + slave_heap_used;
    slave_heap_used += size;
    return p;
}

size_t slave_heap_bytes_used(void) { return slave_heap_used; }
