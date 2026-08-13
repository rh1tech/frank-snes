/*
 * frank-snes — C2 slave firmware
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * snes_alloc.h — the slave's replacement for src/snes_alloc.h.
 *
 * This file shadows the master's copy: slave/src comes before src on the
 * include path, so soundux.c and apu.c pick this up instead without
 * either of them being modified.
 *
 * The master routes snes9x's allocations to PSRAM, which is right for a
 * chip holding a ROM image and frame buffers. The slave's only
 * allocation is soundux.c's 96 KB echo buffer, and that buffer is read
 * and written once per output sample — putting it in PSRAM would drop
 * the XIP read latency into the middle of the mixer's inner loop. There
 * is 520 KB of SRAM here against about 160 KB of demand, so it stays in
 * SRAM and the PSRAM on U5 goes unused.
 *
 * The allocator is a bump allocator with a no-op free, which is all the
 * sound code needs: it allocates once at init and holds it for the
 * lifetime of the firmware.
 */
#ifndef SNES_ALLOC_H
#define SNES_ALLOC_H

#include <stddef.h>
#include <string.h>

void  *slave_alloc(size_t size);
size_t slave_heap_bytes_used(void);

#ifdef FRANK_SNES_PPU_SLAVE

#include <stdbool.h>
#include "psram_allocator.h"

/* Set by slave_ppu_init the instant psram_init() returns. Allocations made
   before that - slave_sound_init runs first - must not be sent to a PSRAM
   that is not up yet, so they fall back to SRAM. They are all small. */
extern volatile bool slave_psram_ready;

/*
 * Big allocations go to PSRAM, small ones stay in SRAM.
 *
 * The renderer changed what this allocator is for. It was written for the
 * S-DSP alone, whose buffers are a few KB and are touched once per output
 * sample, so a 16 KB SRAM bump heap was exactly right. Then gfx.c arrived -
 * and gfx.c does `#define calloc snes_calloc`, so S9xInitGFX's two
 * allocations come HERE: LocalState at 22,972 bytes and GFX.ZERO at 131,072.
 * Neither fits in 16 KB, S9xInitGFX returned false, slave_ppu_init bailed
 * before setting IPPU.RenderThisFrame, and the slave replayed every record
 * and drew a blank frame with every other counter reading healthy.
 *
 * Growing the SRAM heap to 161 KB instead would put .data+.bss at ~477 KB of
 * 520 KB and leave ~43 KB for both stacks and the newlib heap. Not worth it:
 * the master holds these same tables in PSRAM (its snes_alloc.h routes every
 * snes_malloc there) and still renders in 7.6 ms, so PSRAM residency for them
 * is measured, not assumed.
 *
 * The threshold is set so that EXACTLY ONE allocation crosses it: GFX.ZERO.
 * That is deliberate. soundux.c's echo buffer is 96 KB and is read and written
 * once per output sample - it must stay in SRAM. It is also allocated by
 * slave_sound_init, which runs BEFORE psram_init, so a 32 KB threshold made
 * its destination depend on init ORDER: PSRAM if the ordering ever changed,
 * SRAM today, silently. A threshold above it removes the ordering dependency
 * instead of relying on it.
 *
 * Anything new and large added here needs a deliberate decision, not a
 * default. Hot per-pixel or per-sample data belongs in SRAM.
 */
#define SLAVE_PSRAM_ALLOC_THRESHOLD (128u * 1024u)

static inline void *snes_malloc(size_t size)
{
    if (size >= SLAVE_PSRAM_ALLOC_THRESHOLD && slave_psram_ready)
        return psram_malloc(size);
    return slave_alloc(size);
}

static inline void *snes_calloc(size_t nmemb, size_t size)
{
    size_t total = nmemb * size;
    void *ptr = snes_malloc(total);
    if (ptr) memset(ptr, 0, total);   /* psram_malloc does not zero */
    return ptr;
}

#else

static inline void *snes_malloc(size_t size)
{
    return slave_alloc(size);
}

static inline void *snes_calloc(size_t nmemb, size_t size)
{
    size_t total = nmemb * size;
    void *ptr = slave_alloc(total);
    if (ptr) memset(ptr, 0, total);
    return ptr;
}

#endif /* FRANK_SNES_PPU_SLAVE */

static inline void snes_free(void *ptr)
{
    (void)ptr;
}

static inline void *snes_realloc(void *ptr, size_t size)
{
    /* Never called by the sound code. Growing a bump allocation is not
     * something this allocator can do, so fail loudly rather than
     * pretend. */
    (void)ptr; (void)size;
    return NULL;
}

#endif /* SNES_ALLOC_H */
