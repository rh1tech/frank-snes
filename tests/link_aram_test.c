/*
 * frank-snes — host test for src/link_aram.c
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * The APU RAM page tracker decides what the sound slave sees of the
 * game's sample data. Getting it wrong is silent — no counter moves, no
 * link error, the slave just mixes from bytes that are not what the
 * SPC700 wrote — so it is the part of the link worth testing on a host
 * where the failure can be made to print.
 *
 *   cc -O1 -Wall -Isrc -Ilink -o /tmp/link_aram_test \
 *       tests/link_aram_test.c src/link_aram.c && /tmp/link_aram_test
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "link_aram.h"

/* MK3's live echo configuration, read off the board over SWD:
 * ESA = 0xE4, EDL = 3 -> 6 KB at $E400..$FBFF, pages 0xE4..0xFB. */
#define ESA  0xE4
#define EDL  0x03

static link_aram_run_t runs[LINK_ARAM_MAX_RUNS];
static int dirty_ref[LINK_ARAM_PAGES];
static int fails;

#define CHECK(cond, ...) do { if (!(cond)) { \
    printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static void mark(uint32_t page)
{
    dirty_ref[page] = 1;
    link_aram_mark(page << LINK_ARAM_PAGE_BITS);
}

static void reset_case(void)
{
    memset(dirty_ref, 0, sizeof(dirty_ref));
    link_aram_force_resync();
    link_aram_collect(runs, ESA, EDL);   /* consume it, start clean */
    /* Pages 0 and 1 are re-sent unconditionally, so the reference set
     * has to include them or the "no clean page" checks misfire. */
    dirty_ref[0] = dirty_ref[1] = 1;
}

/* Every page the SPC700 dirtied must be covered, the runs must fit the
 * wire cap, and they must stay ordered and non-overlapping — the DMA
 * reads APU RAM in place from these, so an overlap sends the wrong
 * slice. */
static void verify_cover(const char *what, uint32_t n)
{
    CHECK(n <= LINK_ARAM_MAX_RUNS, "%s: %u runs exceeds cap", what, n);

    int covered[LINK_ARAM_PAGES];
    memset(covered, 0, sizeof(covered));
    for (uint32_t i = 0; i < n; i++)
        for (uint32_t p = 0; p < runs[i].page_count; p++)
            covered[runs[i].first_page + p] = 1;

    for (uint32_t p = 0; p < LINK_ARAM_PAGES; p++)
        CHECK(!dirty_ref[p] || covered[p], "%s: dirty page %02x not sent", what, p);

    for (uint32_t i = 0; i + 1 < n; i++)
        CHECK(runs[i].first_page + runs[i].page_count <= runs[i + 1].first_page,
              "%s: run %u overlaps run %u", what, i, i + 1);
}

static uint32_t clean_echo_pages_sent(uint32_t n)
{
    uint32_t bad = 0;
    for (uint32_t i = 0; i < n; i++)
        for (uint32_t p = 0; p < runs[i].page_count; p++) {
            uint32_t page = runs[i].first_page + p;
            if (!dirty_ref[page] && link_aram_page_is_echo(page, ESA, EDL))
                bad++;
        }
    return bad;
}

int main(void)
{
    uint32_t n, bad, n_echo = 0;

    for (uint32_t p = 0; p < LINK_ARAM_PAGES; p++)
        if (link_aram_page_is_echo(p, ESA, EDL)) n_echo++;
    CHECK(n_echo == 24, "echo region is %u pages, expected 24", n_echo);
    CHECK(link_aram_page_is_echo(0xE4, ESA, EDL) &&
          link_aram_page_is_echo(0xFB, ESA, EDL), "echo bounds wrong");
    CHECK(!link_aram_page_is_echo(0xE3, ESA, EDL) &&
          !link_aram_page_is_echo(0xFC, ESA, EDL), "echo bounds leak");
    printf("echo region: %u pages\n", n_echo);

    /* --- steady state: a few scattered writes, no coalescing --- */
    reset_case();
    mark(0x20); mark(0x21); mark(0x55); mark(0x90);
    n = link_aram_collect(runs, ESA, EDL);
    verify_cover("steady", n);
    printf("steady: %u runs, %u clean echo pages sent\n",
           n, clean_echo_pages_sent(n));
    CHECK(clean_echo_pages_sent(n) == 0, "steady: sent clean echo pages");

    /* --- the case the pricing exists for: the echo buffer sits in the
     *     *smallest* gap, so smallest-gap-first walks straight into it.
     *     Nine natural runs, one over the cap. Before the fix this sent
     *     24 pages of stale bytes over the slave's live delay line. --- */
    reset_case();
    mark(0x20); mark(0x40); mark(0x60); mark(0x80);
    mark(0xA0); mark(0xC0);
    mark(0xE3); mark(0xFC);          /* 24-page gap = the echo buffer */
    n = link_aram_collect(runs, ESA, EDL);
    verify_cover("echo-in-smallest-gap", n);
    bad = clean_echo_pages_sent(n);
    printf("echo-in-smallest-gap: %u runs, %u clean echo pages sent\n", n, bad);
    CHECK(bad == 0, "merged through the live delay line (%u pages)", bad);

    /* --- the game genuinely writes inside the echo region: those pages
     *     must still be sent, they are newer than the DSP's --- */
    reset_case();
    for (uint32_t p = 0x10; p <= 0x1F; p++) mark(p);
    mark(0x30); mark(0x40); mark(0x50); mark(0x60); mark(0x70); mark(0x80);
    mark(0xE8); mark(0xE9);
    n = link_aram_collect(runs, ESA, EDL);
    verify_cover("dirty-in-echo", n);
    printf("dirty-in-echo: %u runs\n", n);

    /* --- pathological: every other page dirty. Coalescing has to give
     *     up somewhere; it must still cover every dirty page and stay
     *     inside the cap. --- */
    reset_case();
    for (uint32_t p = 0; p < LINK_ARAM_PAGES; p += 2) mark(p);
    n = link_aram_collect(runs, ESA, EDL);
    verify_cover("comb", n);
    printf("comb: %u runs, %u clean echo pages sent (unavoidable)\n",
           n, clean_echo_pages_sent(n));

    /* --- a full resync still sends the whole image --- */
    memset(dirty_ref, 0, sizeof(dirty_ref));
    link_aram_force_resync();
    n = link_aram_collect(runs, ESA, EDL);
    CHECK(n == 1 && runs[0].first_page == 0 &&
          runs[0].page_count == LINK_ARAM_PAGES, "full resync wrong");
    printf("resync: %u run over %u pages\n", n, runs[0].page_count);

    /* --- an echo buffer that wraps past the top of APU RAM --- */
    CHECK(link_aram_page_is_echo(0xF0, 0xF0, 4) &&
          link_aram_page_is_echo(0xFF, 0xF0, 4) &&
          link_aram_page_is_echo(0x00, 0xF0, 4) &&
          link_aram_page_is_echo(0x0F, 0xF0, 4) &&
          !link_aram_page_is_echo(0x10, 0xF0, 4) &&
          !link_aram_page_is_echo(0xEF, 0xF0, 4), "wrapped echo region wrong");
    printf("wrap: ok\n");

    printf(fails ? "\n%d CHECK(s) FAILED\n" : "\nall checks passed\n", fails);
    return fails != 0;
}
