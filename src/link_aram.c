/*
 * frank-snes — C2 inter-processor sound link
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * link_aram.c — which pages of APU RAM the slave still needs.
 *
 * APU RAM is 64 KB and travels as 256-byte pages, so the steady state is
 * nearly free: a game uploads its sample bank once and then writes
 * almost nothing, whereas a full image every frame would be 64 KB
 * against a budget of 16 ms.
 *
 * The dirty set is described as *runs* of consecutive pages rather than
 * as a bitmap, and each run is then sent straight out of IAPU.RAM. That
 * choice is about master SRAM, not about the wire: staging the marked
 * pages into one contiguous buffer would need a second 64 KB array, and
 * the master has nothing like that spare once the emulator, APU RAM and
 * the frame buffers are placed. Runs let the DMA read the live array.
 */

#include "link_aram.h"

#include <string.h>

static uint8_t        dirty_bits[LINK_ARAM_BITMAP_BYTES];
static link_aram_run_t scratch[LINK_ARAM_PAGES / 2 + 1];
static int            full_resync = 1;

/*
 * Pages 0x00 and 0x01 are marked every frame regardless.
 *
 * Page 0 is the zero page and the DSP/timer register file and page 1 is
 * the SPC700 stack. Between them they are written from a dozen places —
 * apu.c's timer and port handling, cpuexec.c's timer ticks, the
 * Push/PushW macros in spc700.c, the DirectPage store — none of which
 * route through the one general write path that calls link_aram_mark().
 * Re-sending 512 bytes a frame unconditionally (31 KB/s against a
 * 48 MB/s link) buys immunity from ever having missed one of those
 * sites, which is the kind of bug that surfaces as a single game with
 * quietly wrong samples. They are also adjacent, so they cost one run
 * rather than two.
 *
 * Page 0xFF is *not* in that set even though it holds the IPL ROM
 * window, because S9xAPUSetByte() covers it explicitly and
 * S9xSetAPUControl()'s ROM show/hide toggle marks it by hand.
 */
void link_aram_mark(uint32_t address)
{
    uint32_t page = (address & 0xFFFFu) >> LINK_ARAM_PAGE_BITS;
    dirty_bits[page >> 3] |= (uint8_t)(1u << (page & 7u));
}

void link_aram_force_resync(void)
{
    full_resync = 1;
}

static int page_dirty(uint32_t page)
{
    return (dirty_bits[page >> 3] & (1u << (page & 7u))) != 0;
}

/*
 * Clean pages the master must not re-send, and why.
 *
 * The echo delay line lives inside APU RAM, and on C2 it is the *slave*
 * that writes it: its DSP stores each sample at ESA and reads it back a
 * few tens of milliseconds later for feedback. The master's mirror never
 * sees those stores, so over the echo region the two copies are
 * legitimately different and the master's copy is the stale one.
 *
 * That only matters because coalescing re-sends clean pages. A run that
 * swallows a gap crossing the echo buffer drops the master's stale bytes
 * on top of a live delay line, and a delay line replaying old content is
 * heard as a sample repeating — intermittently, and only while echo is
 * enabled, which is the symptom that outlived every fix aimed at the
 * mixer itself.
 *
 * Pages the SPC700 genuinely wrote are still sent: a game is entitled to
 * place its echo buffer over memory it also uses, and its write is newer
 * than anything the DSP put there. Only clean pages are withheld.
 *
 * EDL = 0 is a 4-byte buffer, which still occupies a page. The region
 * wraps at 64 KB, as it does on hardware.
 */
int link_aram_page_is_echo(uint32_t page, uint8_t esa, uint8_t edl)
{
    uint32_t len = (edl & 0x0Fu) ? (edl & 0x0Fu) * (2048u / LINK_ARAM_PAGE_BYTES)
                                 : 1u;
    return ((page - esa) & (LINK_ARAM_PAGES - 1u)) < len;
}

/*
 * Turn the bitmap into at most LINK_ARAM_MAX_RUNS runs of consecutive
 * pages, then clear it.
 *
 * The first pass takes the natural runs. If there are too many, gaps are
 * swallowed one at a time until the count fits.
 */
uint32_t link_aram_collect(link_aram_run_t *out, uint8_t esa, uint8_t edl)
{
    if (full_resync) {
        full_resync = 0;
        memset(dirty_bits, 0, sizeof(dirty_bits));
        out[0].first_page = 0;
        out[0].page_count = LINK_ARAM_PAGES;
        return 1;
    }

    dirty_bits[0] |= 0x03u;   /* pages 0x00 and 0x01 */

    uint32_t n = 0;
    for (uint32_t page = 0; page < LINK_ARAM_PAGES; ) {
        if (!page_dirty(page)) { page++; continue; }
        uint32_t start = page;
        while (page < LINK_ARAM_PAGES && page_dirty(page)) page++;
        scratch[n].first_page = (uint16_t)start;
        scratch[n].page_count = (uint16_t)(page - start);
        n++;
    }

    memset(dirty_bits, 0, sizeof(dirty_bits));

    /*
     * Coalesce across the cheapest gap until the count fits.
     *
     * Cheapest, not smallest. Sending a clean page costs 256 bytes on a
     * wire that moves 48 MB/s, whereas sending an extra run costs a
     * doorbell round trip, which is far more expensive — so ordinarily
     * the smallest gap is the right one to swallow. A clean page inside
     * the echo region is the exception: it costs a piece of the slave's
     * live delay line, which no amount of wire time buys back. Pricing
     * those at 64 ordinary pages means every other gap is merged first,
     * and the delay line is crossed only when nothing else still fits
     * the run cap.
     */
    while (n > LINK_ARAM_MAX_RUNS) {
        uint32_t best = 0, best_cost = 0xFFFFFFFFu;
        for (uint32_t i = 0; i + 1 < n; i++) {
            uint32_t from = (uint32_t)scratch[i].first_page +
                            (uint32_t)scratch[i].page_count;
            uint32_t to   = (uint32_t)scratch[i + 1].first_page;
            uint32_t cost = to - from;
            for (uint32_t p = from; p < to; p++)
                if (link_aram_page_is_echo(p, esa, edl)) cost += 64u;
            if (cost < best_cost) { best_cost = cost; best = i; }
        }
        scratch[best].page_count = (uint16_t)
            ((uint32_t)scratch[best + 1].first_page +
             (uint32_t)scratch[best + 1].page_count -
             (uint32_t)scratch[best].first_page);
        memmove(&scratch[best + 1], &scratch[best + 2],
                (n - best - 2) * sizeof(scratch[0]));
        n--;
    }

    memcpy(out, scratch, n * sizeof(out[0]));
    return n;
}
