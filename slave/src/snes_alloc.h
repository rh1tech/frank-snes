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
