/*
 * frank-snes — C2 slave firmware (RP2350A, U6)
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * main.c — wait for the master, serve one opcode at a time.
 *
 * There is no scheduler and no interrupt work: the slave is entirely
 * reactive. Every exchange is initiated by the master's doorbell, and
 * between exchanges this loop simply blocks. That is what keeps the two
 * chips from needing to agree about absolute time — the slave can boot
 * seconds after the master, or be reset mid-game, and rejoin cleanly at
 * the next HELLO.
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/runtime_init.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/vreg.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/sysinfo.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"

#include "link_bus.h"
#include "link_pins.h"
#include "link_proto.h"
#include "snes9x.h"
#include "memmap.h"
#include "link_session.h"

#include "slave_sound.h"

/* From slave_glue.c, via the slave's snes_alloc.h. */
size_t slave_heap_bytes_used(void);

#ifndef CPU_CLOCK_MHZ
#define CPU_CLOCK_MHZ 252
#endif
#ifndef CPU_VOLTAGE
#define CPU_VOLTAGE VREG_VOLTAGE_1_50
#endif

#define SLAVE_FW_VERSION 0x0100u   /* 1.00 */

/* Flash is a W25Q128JVPIQ; keep QSPI inside its rating whatever the core
 * is clocked at. Without this the XIP window is driven far out of spec
 * at 504 MHz and instruction fetches come back corrupt — the core then
 * executes garbage and locks up before it reaches main().
 *
 * 88, the value frank-genesis uses on this same board. At 504 MHz that
 * picks a divisor of 6 (84 MHz on the wire); the 100 this originally
 * had picks 5, which is 100.8 MHz and past what the part will do. That
 * one number was the difference between a slave that boots and one that
 * locks up on power-on. */
#ifndef FLASH_MAX_FREQ_MHZ
#define FLASH_MAX_FREQ_MHZ 88
#endif

static void __no_inline_not_in_flash_func(set_flash_timings)(int cpu_mhz)
{
    const int clock_hz = cpu_mhz * 1000000;
    const int max_flash_freq = FLASH_MAX_FREQ_MHZ * 1000000;

    int divisor = (clock_hz + max_flash_freq - (max_flash_freq >> 4) - 1)
                / max_flash_freq;
    if (divisor == 1 && clock_hz >= 166000000)
        divisor = 2;

    int rxdelay = divisor;
    if (clock_hz / divisor > 100000000 && clock_hz >= 166000000)
        rxdelay += 1;

    qmi_hw->m[0].timing = 0x60007000 |
                          rxdelay << QMI_M0_TIMING_RXDELAY_LSB |
                          divisor << QMI_M0_TIMING_CLKDIV_LSB;
}

/* CP0 (the GPIO coprocessor) plus CP10/CP11 (VFP). */
#define CPACR_NEEDED 0x00F00000u

static inline void cpacr_ensure(void)
{
    volatile uint32_t *cpacr = (volatile uint32_t *)0xE000ED88u;
    if ((*cpacr & CPACR_NEEDED) != CPACR_NEEDED) {
        *cpacr |= CPACR_NEEDED;
        __asm volatile ("dsb; isb" ::: "memory");
    }
}

/* ------------------------------------------------------------------ */
/* Link state                                                         */
/* ------------------------------------------------------------------ */

static link_t         g_link;
static link_session_t g_sess;

static uint8_t __attribute__((aligned(4))) g_ctrl_tx[LINK_CTRL_BYTES];
static uint8_t __attribute__((aligned(4))) g_ctrl_rx[LINK_CTRL_BYTES];

/* DMA lands directly in these, so they are word-aligned and static. */
static link_event_t   __attribute__((aligned(4))) g_events[LINK_MAX_EVENTS];
static link_aram_run_t __attribute__((aligned(4))) g_runs[LINK_ARAM_MAX_RUNS];

/*
 * Where the mixer's output goes, double-buffered: one half is being sent
 * to the master while the other is being mixed into.
 *
 * That is what keeps the master off the critical path. Mixing and then
 * replying makes the master sit in link_m_recv_ctrl() for the whole mix
 * — measured at 683 us of master stall against 219 us for this ordering
 * — and it breaks outright once a frame asks for several chunks, since
 * the mix then outlasts the master's doorbell patience.
 *
 * The cost is one extra frame of audio latency, 16.7 ms. It is the same
 * one-frame-lag pipeline frank-genesis runs; it lives on this side of
 * the wire because the master's core 1 is the HDMI scanline pump.
 */
static int16_t __attribute__((aligned(4))) g_samples[2][LINK_MAX_SAMPLES];
static uint32_t g_mix_slot;            /* which half is mixed into next */

static link_frame_reply_t g_pending_reply;
static bool               g_have_pending;

/* Staging for the one-shot full APU RAM push. The bulk cannot land
 * straight in IAPU.RAM at an arbitrary offset because the DMA wants a
 * word-aligned destination and the chunk size is fixed; copying through
 * SRAM costs one memcpy on a path that runs once per ROM load. */
static uint8_t __attribute__((aligned(4))) g_chunk[LINK_ARAM_CHUNK_BYTES];

static uint32_t g_frames, g_bad_frames;

#if defined(FRANK_SNES_PPU_SLAVE) && defined(C2_NO_PIPELINE)
/* Not a build combination, and it must fail loudly rather than silently
   dropping the renderer. C2_NO_PIPELINE makes the slave mix BEFORE replying;
   with the renderer here that means rendering (~7.6 ms) before the reply,
   against a master doorbell patience of a few milliseconds. The link would
   drop every frame. The PPU offload requires the pipeline. */
#error "FRANK_SNES_PPU_SLAVE requires the reply pipeline: build with -DC2_NO_PIPELINE=OFF"
#endif

#ifdef FRANK_SNES_PPU_SLAVE
/* Double-buffered like the audio: one picture is on the wire while the next
   is being rendered. 8bpp paletted, the format the master's HDMI path already
   consumes, so the master needs no conversion. */
/* Must match the master's capture store (48 KB). A frame is ~2 KB in play,
   but a ROM-load frame pushes ~12,000 VRAM writes = ~36 KB. Sizing this
   smaller than the master's store desynchronises the WIRE, not just the
   frame: the master sends a bulk the slave decided to skip, and every
   exchange afterwards is misaligned. */
/* In PSRAM, and the same size as the master's store. A frame is ~2 KB in
   play but an upload frame is ~72 KB, and the slave has ~33 KB of SRAM free -
   this cannot live there. PSRAM is otherwise unused apart from the tile
   caches (448 KB of 8 MB). */
#define SLAVE_PPU_STREAM_BYTES (256u * 1024u)
static uint8_t *g_ppu_stream;

/* The SRAM landing zone is back, and this time there are numbers for it.
 *
 * The link's RX DMA cannot land a bulk of any size into the XIP window at the
 * wire rate. Measured with the stream going to PSRAM: 4 receives succeeded and
 * 1,083 failed, the four successes being the only 450-byte frames; every
 * 1,332-byte frame failed. The master meanwhile reported 48 sends a second and
 * spent 10,090 us of each frame in the stream phase against a ~26 us transfer.
 * The same failure is already recorded from the other direction in src/main.c:
 * the framebuffer bulk into PSRAM took the link offline 12 times in 16 seconds.
 * So the slave received essentially no PPU stream at all, which is why it held
 * no VRAM, no CGRAM and no palette and drew an empty frame - while the master
 * had captured 782,322 VRAM writes.
 *
 * It was removed once before as "redundant". It was not: it was masking the
 * cache bug in slave_ppu_replay, which is now fixed independently, and the
 * replay hang that got it reverted has to be re-tested against a picture
 * rather than against another blank screen.
 *
 * 48 KB covers every frame observed in play (450 B - 12 KB) and the 41,760 B
 * attract uploads. It was 64 KB until core 1 needed a 16 KB stack and RAM
 * overflowed by 3,180 bytes; 48 KB is what fits beside that stack while still
 * clearing the largest stream actually measured. A frame bigger than this
 * still falls back to PSRAM and will still mostly fail; g_ppu_oversize_sram
 * counts those so it cannot be mistaken for success. */
#define SLAVE_PPU_SRAM_BYTES LINK_PPU_STREAM_CHUNK
static uint8_t g_ppu_stream_sram[SLAVE_PPU_SRAM_BYTES] __attribute__((aligned(4)));
static uint32_t g_ppu_oversize_sram;
static uint8_t  g_ppu_fb[2][LINK_PPU_MAX_BYTES];
static uint32_t g_ppu_fb_bytes;
static uint32_t g_ppu_slot;
static bool     g_ppu_fb_valid;
volatile uint32_t g_ppu_oversize;
/* Core 0 -> core 1 render handoff. */
static volatile bool     g_render_req;
static volatile uint32_t g_render_len;
/* The buffer core 1 must replay, snapshotted at handoff.
   Core 1 must NOT read g_ppu_stream: core 0 rewrites that to the PSRAM buffer
   at the top of every frame, so core 1 raced it and ended up replaying - and
   cache-invalidating - the wrong buffer. */
static uint8_t * volatile g_render_buf;
static volatile uint32_t g_render_exp_hash;
/* Liveness for the core 0 -> core 1 handoff: how many renders were asked for
   and how many core 1 finished. Equal and climbing = healthy; kicks climbing
   with dones stuck = core 1 never ran or died on its first frame. */
static volatile uint32_t g_render_kicks, g_render_dones;
/* Core 1's stack, sized deliberately - see the launch site. */
static __attribute__((aligned(8))) uint32_t g_render_stack[16u * 1024u / 4u];
/* How often core 0 had to wait for core 1 to release the stream buffer.
   Non-zero means the render is overrunning the frame and the link is paying
   for it again; zero means the handoff is free. */
static volatile uint32_t g_render_waits, g_render_wait_us;
/* Which PSRAM staging buffer core 0 will fill next, and which one core 1 is
   replaying. Core 0 only has to wait when those are the same. */
static volatile uint32_t g_ppu_stage_slot, g_render_stage_slot = 0xffffffffu;
/* Staging copies that had to be repeated, and those that never took. */
static volatile uint32_t g_ppu_stage_retry, g_ppu_stage_lost;
extern uint8_t *slave_ppu_stage_buf[];
#define G_PPU_STAGE_SLOTS 4u
/* Diagnostic: receive the stream but skip the replay, to tell a bad RECEIVE
   from a bad REPLAY. Flipped over the wire is not possible here, so it is a
   build-time default that can be patched live via the debugger on the master
   side of the link only - set to 1 to isolate. */
volatile uint32_t g_render_disable = 0;
/* Draw a synthetic test pattern instead of replaying the stream - see the
   note at the render call. Isolates delivery from content. */
volatile uint32_t g_ppu_test_pattern = 0;
/* Render a self-built PPU state instead of the stream - the question is
   whether this renderer can draw at all on this chip. */
volatile uint32_t g_ppu_selftest = 0;
volatile uint32_t g_dbg_selftest_nz;
/* What the master said it sent, and what actually landed, for the SAME frame.
   Printed side by side when the checksums disagree. */
volatile uint8_t  g_ppu_exp_head[LINK_PPU_HEAD_BYTES];
volatile uint8_t  g_ppu_got_head[LINK_PPU_HEAD_BYTES];
volatile uint32_t g_ppu_head_valid;
volatile uint8_t  g_dbg_head[8];
volatile uint32_t g_dbg_len, g_dbg_sram;
volatile uint32_t g_dbg_nz_drawn, g_dbg_nz_sent;
/* Pixels the renderer actually WROTE, sentinel-based - see the render call. */
volatile uint32_t g_dbg_touched;
/* Did core 1 reach its loop, see a request, and start a render? Separates
   "core 1 never ran" from "core 1 ran and died inside the replay". */
static volatile uint32_t g_render_alive, g_render_spins, g_render_starts;
static uint32_t g_ppu_last_want;   /* frames too big for the buffer */
/* Did the stream bulk actually land? The master reports 48 sends a second of
   2,205 bytes each while the slave's replay counters sit frozen on a 540-byte
   frame, so the receive is the only step left between them. */
static uint32_t g_ppu_recv_ok, g_ppu_recv_fail, g_ppu_want_zero;
#endif

/* Boot progress marker.
 *
 * This chip has no console — its UART is not wired on this board — so
 * when it faults there is nothing to print. Each stage of bring-up
 * stamps this, and it can be read over SWD even after the core has
 * locked up, which is the only way to find out how far it got. */
volatile uint32_t boot_stage;
#define STAGE(n) do { boot_stage = (n); __asm volatile("dsb" ::: "memory"); } while (0)

/* ------------------------------------------------------------------ */
/* Opcode handlers                                                    */
/* ------------------------------------------------------------------ */

static void handle_hello(void)
{
    link_node_info_t info;
    memset(&info, 0, sizeof(info));

    memcpy(info.chip_id, "SNSSLAVE", 8);
    info.package_is_a =
        (*((io_ro_32 *)(SYSINFO_BASE + SYSINFO_PACKAGE_SEL_OFFSET)) & 1u) ? 1 : 0;
    info.fw_version = SLAVE_FW_VERSION;
    info.sys_clk_hz = clock_get_hz(clk_sys);
    info.psram_bytes = 0;      /* the slave's PSRAM goes unused — see
                                * slave/src/snes_alloc.h */
    info.proto_ver  = LINK_PROTO_VER;

    link_s_send_ctrl(&g_sess, LINK_OP_HELLO_ACK, 0, 0, &info, sizeof(info));
}

static void handle_config(void)
{
    link_sound_config_t cfg;
    memcpy(&cfg, g_ctrl_rx + sizeof(link_hdr_t), sizeof(cfg));

    slave_sound_config(&cfg);
    link_s_send_ctrl(&g_sess, LINK_OP_CONFIG_ACK, 0, 0, NULL, 0);
}

static void handle_reset(void)
{
    slave_sound_reset();

    /* Drop the pipelined frame too: it was mixed from DSP state the
     * reset just discarded, so handing it over would play one frame of
     * the old song after the reset. */
    g_have_pending = false;
    g_mix_slot     = 0;
    memset(&g_pending_reply, 0, sizeof(g_pending_reply));

    link_s_send_ctrl(&g_sess, LINK_OP_RESET_ACK, 0, 0, NULL, 0);
}

static void handle_aram_chunk(void)
{
    uint32_t off = link_rx_hdr(&g_sess)->arg0;
    uint32_t len = link_rx_hdr(&g_sess)->arg1;

    if (len > LINK_ARAM_CHUNK_BYTES || off > LINK_ARAM_BYTES ||
        off + len > LINK_ARAM_BYTES) {
        /* Refuse rather than scribble. The master checks the CRC at the
         * end of the upload, so a rejected chunk surfaces there. */
        printf("[slave] bad ARAM chunk: off %lu len %lu\n",
               (unsigned long)off, (unsigned long)len);
        return;
    }

    if (!link_s_bulk_recv(&g_sess, g_chunk, len)) {
        printf("[slave] ARAM chunk %lu did not arrive\n", (unsigned long)off);
        return;
    }

    memcpy(slave_sound_aram() + off, g_chunk, len);
    link_s_send_ctrl(&g_sess, LINK_OP_ARAM_CHUNK_ACK, off, len, NULL, 0);
}

static void handle_aram_end(void)
{
    uint32_t crc = link_crc32(slave_sound_aram(), LINK_ARAM_BYTES);
    uint32_t want = link_rx_hdr(&g_sess)->arg0;

    if (crc != want)
        printf("[slave] APU RAM CRC %08lx, master sent %08lx\n",
               (unsigned long)crc, (unsigned long)want);

    link_s_send_ctrl(&g_sess, LINK_OP_ARAM_END_ACK, crc, 0, NULL, 0);
}

static void handle_frame(void)
{
    uint32_t n_events = link_rx_hdr(&g_sess)->arg0;
    uint32_t arg1     = link_rx_hdr(&g_sess)->arg1;
    uint32_t n_runs   = LINK_FRAME_RUNS(arg1);
    uint32_t chunks   = LINK_FRAME_CHUNKS(arg1);

    if (chunks < 1u) chunks = 1u;

    if (n_events > LINK_MAX_EVENTS || n_runs > LINK_ARAM_MAX_RUNS ||
        chunks > LINK_MAX_CHUNKS) {
        /* The master and this firmware disagree about the limits, which
         * means the two halves were built from different trees. Bail out
         * of the exchange rather than arming a DMA for a length that
         * will not arrive. */
        printf("[slave] frame out of range: %lu events, %lu runs, %lu chunks\n",
               (unsigned long)n_events, (unsigned long)n_runs,
               (unsigned long)chunks);
        g_bad_frames++;
        return;
    }

    /* The run table rode in the control frame's payload — see
     * LINK_OP_FRAME in link_proto.h. Copy it out before the first bulk
     * arms, because g_ctrl_rx is the landing area for the next receive. */
    if (n_runs)
        memcpy(g_runs, g_ctrl_rx + sizeof(link_hdr_t),
               n_runs * sizeof(link_aram_run_t));

#ifdef FRANK_SNES_PPU_SLAVE
    /* Same rule as the run table: the PPU stream length rides in this
       payload and MUST be copied out before the first bulk arms, or
       g_ctrl_rx is overwritten underneath it. */
    uint32_t ppu_want = 0, ppu_exp_sum = 0;
    memcpy(&ppu_want, g_ctrl_rx + sizeof(link_hdr_t) + LINK_PPU_LEN_OFFSET,
           sizeof(ppu_want));
    /* The master's checksum of the very stream this frame is about to carry,
       and its first bytes. Same rule, same reason: out of the payload before
       any bulk arms. */
    memcpy(&ppu_exp_sum, g_ctrl_rx + sizeof(link_hdr_t) + LINK_PPU_SUM_OFFSET,
           sizeof(ppu_exp_sum));
    memcpy((void *)g_ppu_exp_head,
           g_ctrl_rx + sizeof(link_hdr_t) + LINK_PPU_HEAD_OFFSET,
           LINK_PPU_HEAD_BYTES);
    uint32_t ppu_exp_vram_hash = 0;
    memcpy(&ppu_exp_vram_hash,
           g_ctrl_rx + sizeof(link_hdr_t) + LINK_PPU_VRAMHASH_OFFSET,
           sizeof(ppu_exp_vram_hash));
    { extern volatile uint8_t slave_ppu_exp_peek[];
      memcpy((void *)slave_ppu_exp_peek,
             g_ctrl_rx + sizeof(link_hdr_t) + LINK_PPU_VRAMPEEK_OFFSET,
             LINK_PPU_VRAMPEEK_BYTES); }
    uint32_t ppu_exp_block[LINK_PPU_VRAM_BLOCKS];
    for (uint32_t b = 0; b < LINK_PPU_VRAM_BLOCKS; b++)
        memcpy(&ppu_exp_block[b], g_ctrl_rx + sizeof(link_hdr_t)
                                  + LINK_PPU_VRAMBLK_OFFSET + b * 4u, 4u);
#endif

    if (n_events) {
        if (!link_s_bulk_recv(&g_sess, g_events,
                              n_events * sizeof(link_event_t))) {
            g_bad_frames++;
            return;
        }
    }

    if (n_runs) {
        for (uint32_t i = 0; i < n_runs; i++) {
            uint32_t first = g_runs[i].first_page;
            uint32_t count = g_runs[i].page_count;

            if (first + count > LINK_ARAM_PAGES) {
                printf("[slave] bad ARAM run: %lu+%lu\n",
                       (unsigned long)first, (unsigned long)count);
                g_bad_frames++;
                return;
            }
            uint8_t *dst = slave_sound_aram() +
                           (first << LINK_ARAM_PAGE_BITS);
            if (!link_s_bulk_recv(&g_sess, dst,
                                  count << LINK_ARAM_PAGE_BITS)) {
                g_bad_frames++;
                return;
            }
        }
    }

    /*
     * Reply first, with the audio mixed during the previous exchange,
     * then mix this frame. The master is unblocked before this chip
     * starts any real work — see the note on g_samples.
     *
     * Doing it the other way round is not just slower, it does not
     * survive a multi-chunk request: mixing N chunks takes N * ~400 us,
     * and the master's steady-state doorbell patience is a few
     * milliseconds. Once the emulator drops below 60 fps and starts
     * asking for three or four chunks, the master times out waiting for
     * an ACK the slave has not reached yet, and the link drops. That is
     * exactly what it did.
     *
     * The very first frame has nothing buffered, so the reply says zero
     * samples and the master substitutes silence for one frame.
     */
#ifdef C2_NO_PIPELINE
    /*
     * Diagnostic: mix first, then reply, so the DSP register file the
     * master receives describes the frame it just sent rather than the
     * one before it.
     *
     * The pipeline exists because it keeps the master off the critical
     * path — 219 us of stall against 683 us. The cost is that ENDX,
     * ENVX and OUTX come back one frame stale, and a sound driver that
     * polls ENDX to sequence samples can re-trigger a voice on the
     * strength of a stale bit. That is the last remaining candidate for
     * the repeats, and this is how to find out.
     */
    {
        link_frame_reply_t reply;
        uint32_t got = slave_sound_frame(g_events, n_events,
                                         g_samples[0], slave_sound_want(chunks),
                                         &reply);
        link_s_send_ctrl(&g_sess, LINK_OP_FRAME_ACK, 0, 0, &reply, sizeof(reply));
        if (got)
            link_s_bulk_send(&g_sess, g_samples[0],
                             LINK_ALIGN4(got * sizeof(int16_t)));
        g_frames++;
        return;
    }
#endif

#ifdef FRANK_SNES_PPU_SLAVE
    /* The PPU command stream rides in the same exchange, after the sound
       payloads. Receive it now; render it AFTER the reply. */
    uint32_t ppu_len = 0;
    g_ppu_stream = slave_ppu_stage_buf[g_ppu_stage_slot];
    {
        /* Copied out of the payload above, before the bulks armed. */
        uint32_t want = ppu_want;
        g_ppu_last_want = want;

        /* Core 1 replays from a PSRAM staging buffer, not from the landing
           zone, so the landing zone is free the moment the receive completes
           and this normally does not wait at all. It can still wait if core 1
           is more than a whole frame behind, and it must: dropping a record
           corrupts the slave's VRAM mirror permanently rather than
           cosmetically, so stalling is the lesser harm. g_render_waits says
           how often, so this cannot quietly become the old core-0 block
           wearing a new name. */
        if (want && g_render_req && g_ppu_stage_slot == g_render_stage_slot) {
            uint32_t w0 = time_us_32();
            g_render_waits++;
            while (g_render_req) tight_loop_contents();
            g_render_wait_us = time_us_32() - w0;
        }
        __dmb();

        /* Whatever the master announced MUST be taken off the wire. Skipping
           it because it does not fit leaves those bytes in flight and every
           later exchange misaligned - the link then never recovers. Receive
           it, and only then decide whether it is usable. */
        if (want > SLAVE_PPU_STREAM_BYTES) {
            g_ppu_oversize++;
            if (link_s_bulk_recv(&g_sess, g_ppu_stream,
                                 LINK_ALIGN4(SLAVE_PPU_STREAM_BYTES)))
                ppu_len = 0;          /* drained what we could; frame is lost */
        } else if (!want) {
            g_ppu_want_zero++;
        } else {
            /* SRAM when it fits, PSRAM only when it cannot - see the note on
               g_ppu_stream_sram for the measurement that decides this. */
            extern volatile uint32_t slave_ppu_stream_dma;
            uint8_t *stage = slave_ppu_stage_buf[g_ppu_stage_slot];
            bool ok = true;
            /* One bulk per chunk, always into SRAM, then copied to the PSRAM
               staging buffer with the CPU. The DMA never touches PSRAM, so
               there is no oversized case left to fail. */
            /* Summed as it comes OFF THE WIRE, before it is staged, so a
               byte that changes in transit and a byte that changes in the
               SRAM->PSRAM staging are distinguishable. The replay sums the
               staged copy; comparing the two against the master's says which
               half is at fault instead of just that one of them is. */
            uint32_t rxh = 2166136261u;
            for (uint32_t off = 0; off < want; off += LINK_PPU_STREAM_CHUNK) {
                uint32_t n = want - off;
                if (n > LINK_PPU_STREAM_CHUNK) n = LINK_PPU_STREAM_CHUNK;
                if (!link_s_bulk_recv(&g_sess, g_ppu_stream_sram,
                                      LINK_ALIGN4(n))) { ok = false; break; }
                for (uint32_t i = 0; i < n; i++) {
                    rxh ^= g_ppu_stream_sram[i]; rxh *= 16777619u;
                }
                /* Verify the staging copy WHILE THE SOURCE IS STILL HERE.
                 *
                 * Measured with the sum computed both on arrival and after
                 * staging: 28 corrupt streams, wire 0, staging 28, every one
                 * of them a multi-chunk frame. So the bytes are right when
                 * they come off the wire and wrong when core 1 reads them -
                 * this core writes ~100 KB into PSRAM through a 16 KB XIP
                 * cache while core 1 is reading its PSRAM tile caches flat
                 * out, and some of those writes do not survive it.
                 *
                 * The next bulk overwrites this SRAM buffer, so this is the
                 * only moment a bad copy can be repeated. A retry here turns
                 * a permanently wrong VRAM mirror - which is what the
                 * distorted sprites were - into a few microseconds. */
                for (uint32_t try = 0; ; try++) {
                    memcpy(stage + off, g_ppu_stream_sram, n);
                    if (!memcmp(stage + off, g_ppu_stream_sram, n)) break;
                    g_ppu_stage_retry++;
                    if (try >= 3u) { g_ppu_stage_lost++; break; }
                }
            }
            { extern volatile uint32_t slave_ppu_rx_sum;
              slave_ppu_rx_sum = rxh; }
            if (!ok) {
                g_ppu_recv_fail++;
            } else {
                g_ppu_recv_ok++;
                ppu_len = want;
                g_ppu_stream = stage;
                slave_ppu_stream_dma = 0;
                /* Checked inside slave_ppu_replay, which is where the cache
                   invalidate happens - summing before it would compare the
                   master against stale lines and blame the wire for the
                   cache. */
                extern volatile uint32_t slave_ppu_exp_sum;
                slave_ppu_exp_sum = ppu_exp_sum;
            }
        }
    }
#endif

    if (g_have_pending) {
        uint32_t send_slot = g_mix_slot ^ 1u;
        uint32_t got = g_pending_reply.samples;

#ifdef FRANK_SNES_PPU_SLAVE
        /* The picture's length and the slave's diagnostics ride in the sound
           reply. A control frame of their own cost 10.2 ms of a 10.5 ms
           exchange; the 57 KB bulk it carried cost 105 us. */
        {
            extern volatile uint32_t slave_ppu_render_us, slave_ppu_records,
                                     slave_ppu_psram_ok, slave_ppu_impossible,
                                     slave_ppu_alloc_fail;
            g_pending_reply.ppu_fb_bytes = g_ppu_fb_valid ? g_ppu_fb_bytes : 0;
            g_pending_reply.ppu_stat.render_us  = slave_ppu_render_us;
            g_pending_reply.ppu_stat.records    = slave_ppu_records;
            g_pending_reply.ppu_stat.oversize   = g_ppu_oversize;
            g_pending_reply.ppu_stat.psram_ok   = slave_ppu_psram_ok |
                                                  (slave_ppu_alloc_fail << 8);
            g_pending_reply.ppu_stat.impossible = slave_ppu_impossible;
            g_pending_reply.ppu_stat.want       = g_ppu_last_want;
            { extern uint32_t slave_ppu_dbg_pitch_h(void), slave_ppu_dbg_flags(void);
              g_pending_reply.ppu_stat.dbg_pitch_h = slave_ppu_dbg_pitch_h();
              g_pending_reply.ppu_stat.dbg_flags   = slave_ppu_dbg_flags(); }
            { extern volatile uint32_t slave_ppu_stream_len, slave_ppu_stream_sum,
                                       slave_ppu_exp_sum, slave_ppu_sum_ok,
                                       slave_ppu_sum_bad,
                                       slave_ppu_stop_off, slave_ppu_stop_ctx,
                                       slave_ppu_stop_why;
              g_pending_reply.ppu_stat.stream_len = slave_ppu_stream_len;
              g_pending_reply.ppu_stat.stream_sum = slave_ppu_stream_sum;
              g_pending_reply.ppu_stat.exp_sum    = slave_ppu_exp_sum;
              g_pending_reply.ppu_stat.sum_ok     = slave_ppu_sum_ok;
              g_pending_reply.ppu_stat.sum_bad    = slave_ppu_sum_bad;
              g_pending_reply.ppu_stat.stop_off   = slave_ppu_stop_off;
              g_pending_reply.ppu_stat.stop_ctx   = slave_ppu_stop_ctx;
              g_pending_reply.ppu_stat.stop_why   = slave_ppu_stop_why; }
            { extern volatile uint32_t slave_ppu_vram_hash,
                                       slave_ppu_cgram_hash, slave_ppu_oam_hash;
              extern volatile uint32_t slave_ppu_vram_match, slave_ppu_vram_diff;
              { extern volatile uint32_t slave_ppu_blk_bitmap;
                g_pending_reply.ppu_stat.vram_hash = slave_ppu_blk_bitmap; }
              g_pending_reply.ppu_stat.vram_match = slave_ppu_vram_match;
              g_pending_reply.ppu_stat.vram_diff  = slave_ppu_vram_diff; }
        }
#endif
        link_s_send_ctrl(&g_sess, LINK_OP_FRAME_ACK, 0, 0,
                         &g_pending_reply, sizeof(g_pending_reply));
        if (got)
            link_s_bulk_send(&g_sess, g_samples[send_slot],
                             LINK_ALIGN4(got * sizeof(int16_t)));
    } else {
        link_frame_reply_t empty;
        memset(&empty, 0, sizeof(empty));
        link_s_send_ctrl(&g_sess, LINK_OP_FRAME_ACK, 0, 0,
                         &empty, sizeof(empty));
    }

#ifdef FRANK_SNES_PPU_SLAVE
    /* Return the picture rendered from the PREVIOUS stream, for the same
       reason the audio is pipelined: rendering is ~7.6 ms and the master's
       doorbell patience is a few milliseconds. Replying with this frame's
       picture would mean rendering before the reply and dropping the link.
       The cost is one frame of video latency, which was accepted in the
       design; the alternative is not a slower link, it is no link. */
    {
        /* Length and stats already went out in the FRAME_ACK payload above;
           only the bulks remain, and they need no handshake of their own. */
        uint32_t fb_bytes = g_have_pending ? g_pending_reply.ppu_fb_bytes : 0;
        if (fb_bytes)
            link_s_bulk_send(&g_sess, g_ppu_fb[g_ppu_slot ^ 1u],
                             LINK_ALIGN4(fb_bytes));

        /* The palette, every frame and unconditionally. The master no longer
           renders, so it never calls graphics_set_palette itself - without
           this its HDMI has no palette at all. 1 KB/frame is 50 KB/s against
           a 50 MB/s link; making it conditional to save that would risk the
           phase desync that has already cost two debugging rounds. */
        {
            extern uint32_t slave_palette[256];
            link_s_bulk_send(&g_sess, slave_palette, sizeof(slave_palette));
        }
    }
#endif

    /* Now mix, with the wire idle and the master away doing its own
     * frame. How many samples is not carried in the bulk: the count in
     * the reply tells the master what to arm for next time. */
    slave_sound_frame(g_events, n_events,
                      g_samples[g_mix_slot], slave_sound_want(chunks),
                      &g_pending_reply);

#ifdef FRANK_SNES_PPU_SLAVE
    /* Hand the render to CORE 1 and return to the wire immediately.
     *
     * Rendering here, on core 0, is what makes the master wait: the master's
     * next stream bulk cannot complete until this loop comes back round and
     * arms its receive, so the whole render lands inside the master's
     * exchange. Measured on the master: the stream-send phase costs 10,021 us
     * against a ~26 us transfer, and that block starves the master's HDMI
     * scanline interrupt until the display drops lock.
     *
     * Core 1 is idle - core 0 owns the S-DSP and the link - so the render
     * overlaps the master's next frame instead of blocking it, which is what
     * the two-chip design was for. The double buffer already separates the
     * picture being sent from the one being drawn. */
    /* The render is 16 ms. It cannot stay on this core.
     *
     * With it here, the master's exchange measured 12,184 us and the link
     * failed 31 times a minute: the master's next stream bulk cannot complete
     * until this loop comes back round and arms its receive, so the whole
     * render lands inside the master's exchange. The earlier attempt at core 1
     * "entered the replay and never returned" - which is what the SDK's
     * 2 KB default core 1 stack does to a recursive tile renderer, and is
     * fixed by launching with an explicit 16 KB one.
     *
     * g_ppu_slot needs no lock. Core 1 renders into g_ppu_fb[g_ppu_slot] and
     * flips only when it is finished, so core 0's g_ppu_fb[g_ppu_slot ^ 1] is
     * the last COMPLETED frame whether core 1 is mid-render or not. */
    if (ppu_len) {
        g_dbg_len  = ppu_len;
        g_dbg_sram = ((uintptr_t)g_ppu_stream == (uintptr_t)g_ppu_stream_sram);
        memcpy((void *)g_dbg_head, g_ppu_stream, 8);

        { extern volatile uint32_t slave_ppu_stream_sum;
          if (!g_ppu_head_valid && slave_ppu_stream_sum != ppu_exp_sum) {
              uint32_t n = ppu_len < LINK_PPU_HEAD_BYTES ? ppu_len
                                                         : LINK_PPU_HEAD_BYTES;
              for (uint32_t q = 0; q < n; q++)
                  g_ppu_got_head[q] = g_ppu_stream[q];
              g_ppu_head_valid = 1;
          } }

        if (g_render_disable) {
            g_ppu_fb_bytes = 256u * 224u;
            g_ppu_fb_valid = true;
        } else {
            g_render_buf        = g_ppu_stream;
            g_render_len        = ppu_len;
            g_render_stage_slot = g_ppu_stage_slot;
            /* Handed over WITH the request. Core 1 runs up to a frame behind
               core 0, so a global that core 0 overwrites per frame would have
               core 1 checking frame N-1's VRAM against frame N's hash and
               reporting differences that are only the pipeline. */
            g_render_exp_hash   = ppu_exp_vram_hash;
            /* The block hashes need the same frame alignment as the whole
               one, for the same reason: core 0 would otherwise have core 1
               comparing frame N-1's VRAM against frame N's blocks, and the
               resulting map is worse than no map - it said "no block
               differs" while the whole-VRAM hash said every frame did. */
            { extern volatile uint32_t slave_ppu_exp_block[];
              for (uint32_t b = 0; b < LINK_PPU_VRAM_BLOCKS; b++)
                  slave_ppu_exp_block[b] = ppu_exp_block[b]; }
            g_render_kicks++;
            __dmb();
            g_render_req = true;
            __sev();
            /* The next frame stages into the other buffer, so receiving it
               does not overwrite what core 1 is replaying. */
            if (++g_ppu_stage_slot >= G_PPU_STAGE_SLOTS) g_ppu_stage_slot = 0;
        }
    }
#endif

    g_mix_slot ^= 1u;
    g_have_pending = true;

    g_frames++;
}

#ifdef FRANK_SNES_PPU_SLAVE
/* Core 1: the renderer.
 *
 * Nothing here touches the link. Core 0 publishes a stream length, core 1
 * replays it into the back buffer and flips; core 0 ships whatever is finished
 * on the next exchange. A frame that is still rendering when the master asks
 * simply is not sent that frame - one frame of latency, which the design
 * already accepted - rather than stalling the wire. */
static void slave_render_core(void)
{
    /* Per-core, and core 1 has had neither.
     *
     * CP0 (the GPIO coprocessor) and CP10/CP11 (VFP) are enabled through
     * CPACR, which is banked per core - core 0 doing it at boot buys core 1
     * nothing. The renderer reaches VFP quickly, so without this core 1 took
     * exactly one render request and never returned from it: alive=1,
     * start=1, done=0, while core 0 went on kicking 51 requests a second into
     * a core that was already dead. The main() comment above says the same
     * thing about core 0 - "coprocessors first, before anything else". */
    runtime_init_per_core_enable_coprocessors();
    cpacr_ensure();

    g_render_alive = 1;          /* core 1 reached its loop at all */
    for (;;) {
        while (!g_render_req) { g_render_spins++; __wfe(); }
        __dmb();
        g_render_starts++;

        extern void slave_ppu_replay(const uint8_t *rec, uint32_t len);
        extern uint8_t *slave_ppu_screen;
        extern void slave_ppu_arm_frame(void);

        slave_ppu_screen = g_ppu_fb[g_ppu_slot];

        /* Synthetic picture: prove the DELIVERY path on its own.
         *
         * Everything from here to the screen is shared with the real thing -
         * the same framebuffer, the same bulk, the same palette bulk, the
         * same HDMI scanout - but the pixels come from eight lines of code
         * instead of from the renderer, and the palette is eight known
         * colours instead of whatever the game built. So:
         *
         *   pattern appears on HDMI -> transport and display are correct, and
         *     the fault is in what the renderer produces (content), OR
         *   pattern does not appear -> the fault is in the offload mechanism
         *     itself and the renderer is irrelevant.
         *
         * Measured first: an identical master build with FRANK_SNES_PPU_CAPTURE
         * off shows a picture on all 8 grabs (SIGNAL 218), so the board, the
         * cable and the sink are not in question - only the offload path is.
         *
         * Indices 1..8 deliberately: 251-254 are the HDMI driver's sync
         * control symbols, and putting those in the picture is a SEPARATE
         * experiment, not a confound in this one. */
        if (g_ppu_test_pattern) {
            extern uint32_t slave_palette[256];
            uint8_t *fb = slave_ppu_screen;
            for (uint32_t y = 0; y < 224u; y++) {
                uint8_t *row = fb + y * 256u;
                for (uint32_t x = 0; x < 256u; x++)
                    row[x] = (uint8_t)(1u + (x >> 5));   /* 8 vertical bars */
            }
            /* A moving marker, so a frozen picture is distinguishable from a
               live one that happens to be static. */
            { uint32_t t = (g_render_dones >> 3) & 0xffu;
              for (uint32_t y = 8; y < 24u; y++)
                  for (uint32_t x = 0; x < 16u; x++)
                      fb[y * 256u + ((t + x) & 0xffu)] = 8u; }
            static const uint32_t bars[8] = {
                0xff0000u, 0x00ff00u, 0x0000ffu, 0xffff00u,
                0xff00ffu, 0x00ffffu, 0xffffffu, 0x808080u
            };
            for (uint32_t i = 0; i < 8u; i++) slave_palette[1u + i] = bars[i];
            slave_palette[0] = 0x000000u;
        } else {
            /* Sentinel fill: "wrote zeros" and "wrote nothing" are the same
               picture but completely different bugs, and counting non-zero
               pixels cannot tell them apart. Anything the renderer touches
               stops being 0xAA. */
            memset(slave_ppu_screen, 0xAA, 256u * 224u);
            slave_ppu_arm_frame();
            slave_ppu_replay(g_render_buf, g_render_len);
            { uint32_t touched = 0;
              const uint8_t *fb = slave_ppu_screen;
              for (uint32_t q = 0; q < 256u * 224u; q += 37)
                  if (fb[q] != 0xAAu) touched++;
              g_dbg_touched = touched; }
        }

        /* Did the renderer actually put pixels in the buffer, and is it the
           buffer core 0 ships? Sampled here rather than on core 0 because
           only this core knows when the frame is finished. */
        { const uint8_t *drawn = g_ppu_fb[g_ppu_slot];
          const uint8_t *sent  = g_ppu_fb[g_ppu_slot ^ 1u];
          uint32_t nzd = 0, nzs = 0;
          for (uint32_t q = 0; q < 256u * 224u; q += 37) {
              if (drawn[q]) nzd++;
              if (sent[q])  nzs++;
          }
          g_dbg_nz_drawn = nzd;
          g_dbg_nz_sent  = nzs; }

        /* On this core, after the render - see slave_ppu_hash_state. */
        { extern void slave_ppu_hash_state(void);
          extern volatile uint32_t slave_ppu_exp_vram_hash;
          slave_ppu_exp_vram_hash = g_render_exp_hash;
          slave_ppu_hash_state(); }

        g_ppu_fb_bytes = 256u * 224u;   /* SNES_WIDTH * SNES_HEIGHT */
        /* The flip is the publish: everything above must be visible to core 0
           before the slot moves, and the slot before the request clears. */
        __dmb();
        g_ppu_slot ^= 1u;
        g_ppu_fb_valid = true;
        g_render_dones++;
        __dmb();
        g_render_req = false;
        __sev();
    }
}
#endif

/* ------------------------------------------------------------------ */
/* Boot                                                               */
/* ------------------------------------------------------------------ */

/* Automatic BOOTSEL after repeated boot failures.
 *
 * This board has no working SWD and no working UART on the slave, so a
 * firmware that hangs before its main loop is unrecoverable except by a human
 * holding a button. That makes every experiment cost a person, which is a poor
 * property for a chip meant to be iterated on.
 *
 * So: a watchdog reboots a hung boot, a counter in the watchdog's scratch
 * registers survives that reboot, and the third consecutive failure drops the
 * chip into BOOTSEL by itself - where picotool can reach it and flash
 * something that works. The counter is cleared once the main loop has been
 * healthy for a while, so a normal boot never accumulates toward it.
 *
 * The magic word distinguishes a real count from whatever the scratch
 * registers hold at power-on, when they are undefined. */
#define SLAVE_BOOT_MAGIC   0x5A1E0000u
#define SLAVE_BOOT_MAGIC_MASK 0xffff0000u
#define SLAVE_BOOT_FAIL_LIMIT 3u
/* Generous: the main loop blocks up to 1 s in link_s_wait_ctrl, and a frame's
   render is a few ms. This is here to catch a HANG, not to police latency. */
#define SLAVE_WATCHDOG_MS  8000u
/* How long the main loop must run before a boot counts as good. */
#define SLAVE_BOOT_OK_US   10000000u

int main(void)
{
    {
        uint32_t sc = watchdog_hw->scratch[0];
        uint32_t fails = ((sc & SLAVE_BOOT_MAGIC_MASK) == SLAVE_BOOT_MAGIC)
                       ? (sc & 0xffffu) : 0u;

        fails = watchdog_caused_reboot() ? fails + 1u : 0u;

        if (fails >= SLAVE_BOOT_FAIL_LIMIT) {
            /* Clear first: this must not loop if BOOTSEL is itself escaped. */
            watchdog_hw->scratch[0] = SLAVE_BOOT_MAGIC;
            reset_usb_boot(0, 0);
        }
        watchdog_hw->scratch[0] = SLAVE_BOOT_MAGIC | fails;
    }
    /* Both halves must run at the same clock: the receiving PIO program
     * has to finish its loop inside the transmitter's byte period, and
     * each side derives that from its own system clock. The master
     * checks this in its HELLO handling and refuses to proceed on a
     * mismatch rather than running a link that works one way. */
    /* Coprocessors first, before anything else — sleep_ms() below is
     * 64-bit maths and compiles to VFP, so enabling these after it would
     * be too late. See the note further down for why they are off at
     * all when the board free-runs. */
    runtime_init_per_core_enable_coprocessors();
    cpacr_ensure();

#if CPU_CLOCK_MHZ > 252
    vreg_disable_voltage_limit();
    vreg_set_voltage(CPU_VOLTAGE);
    /* QSPI has to be brought inside spec before the core is overclocked,
     * or instruction fetches out of the XIP window come back corrupt. */
    set_flash_timings(CPU_CLOCK_MHZ);
    sleep_ms(100);
#endif
    /* `false`, not `true`: fall back rather than panic if the PLL cannot
     * be configured. A slave stuck in panic() is indistinguishable from a
     * dead one — the master just reports "no sound slave". */
    if (!set_sys_clock_khz(CPU_CLOCK_MHZ * 1000, false))
        set_sys_clock_khz(252 * 1000, true);

    /*
     * Turn the coprocessors on before touching a single GPIO.
     *
     * The SDK does this from its per-core preinit array, and when the
     * core is stepped from a debugger reset that is exactly what
     * happens — so the board comes up fine under a debugger and dies
     * when it free-runs from power-on. Free running it reaches main with
     * CPACR = 0x0000C000, and gpio_put() compiles to a CP0 instruction
     * on RP2350, so the first LED write takes a UsageFault. pico_time's
     * 64-bit maths is VFP, so alarms fault too.
     *
     * This cost most of a debugging session. The symptom — a slave that
     * never answers HELLO, with a core the debugger reports as "in
     * unknown state" — looks exactly like a marginal overclock, and was
     * misdiagnosed as one. frank-genesis's slave documents the same trap.
     */
    runtime_init_per_core_enable_coprocessors();
    cpacr_ensure();

    stdio_init_all();

    /* Wait for the host to attach the USB-CDC console BEFORE anything that
     * can fault.
     *
     * This chip's UART and SWD are both dead at the wire on this board, so
     * CDC is the only channel - and CDC is useless until the host has
     * enumerated and opened it. Everything printed in the first ~2 s goes
     * nowhere. Worse, enumeration is IRQ-driven while delivery runs from
     * tud_task in the main loop, so a hang here leaves the host holding an
     * open port that never produces a byte: present, silent, and easily
     * mistaken for dead hardware. Three seconds of boot latency is a small
     * price for a console that outlives the failure it is meant to explain. */
    sleep_ms(3000);

    /* Armed HERE, before the risky init - not after it.
     *
     * It was originally armed once the main loop was reached, on the reasoning
     * that init is legitimately slow (PSRAM bring-up, 448 KB of tile caches, a
     * 128 KB LUT) and should not be policed. That protected exactly the wrong
     * window: slave_ppu_init is where this firmware actually hangs, and a
     * watchdog that starts afterwards can never fire for it. The point of the
     * thing is recovering a chip whose SWD and UART are both dead, so it has
     * to cover the code most likely to kill it.
     *
     * Generous enough that a slow-but-working init is never mistaken for a
     * hung one, and every stage marker pets it. */
    watchdog_enable(SLAVE_WATCHDOG_MS, 1);

    printf("\n[slave] frank-snes C2 sound slave, fw %u.%02u\n",
           SLAVE_FW_VERSION >> 8, SLAVE_FW_VERSION & 0xff);
    printf("[slave] sys_clk %lu Hz\n", (unsigned long)clock_get_hz(clk_sys));

    STAGE(3);
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    STAGE(4);

    printf("[slave] sound init: entering (heap %u used)\n",
           (unsigned)slave_heap_bytes_used());
    sleep_ms(40);
    slave_sound_init();
    printf("[slave] sound init: returned\n");
    sleep_ms(40);
    STAGE(5);
    printf("[slave] mixer up, heap %u bytes\n",
           (unsigned)slave_heap_bytes_used());

    /* PPU offload: the renderer lives here now. Core 0 keeps the S-DSP, the
       renderer runs on core 1. Measured on the master: handing the renderer
       over frees 7,642 us/frame and takes it from 46 to 50 fps. */
    /* Wait for USB-CDC to enumerate BEFORE touching the renderer.
     *
     * A fault inside slave_ppu_init takes the USB device down with it, and
     * the host then sees no port at all - which is indistinguishable from a
     * board that lost power, and says nothing about where it died. Everything
     * printed before this point is lost for the same reason: the host is not
     * attached yet. Three seconds of dead time at boot buys a console that
     * survives the failure, on a chip whose UART and SWD are both dead at the
     * wire. */
    printf("[slave] ppu init: entering (heap %u bytes used)\n",
           (unsigned)slave_heap_bytes_used());

    { extern bool slave_ppu_init(void);
      if (!slave_ppu_init())
          printf("[slave] FATAL: slave_ppu_init failed (out of memory)\n");
      else {
          printf("[slave] renderer up, heap %u bytes\n",
                 (unsigned)slave_heap_bytes_used());
          /* Only after init: core 1 dereferences everything it allocates.
             With an EXPLICIT 16 KB stack: the SDK's default core 1 stack is a
             couple of KB, and the tile renderer recurses through the draw
             paths far past that. Launched with the default, core 1 accepted
             1,745 render requests and completed none - it died on its first
             frame, silently, while the link stayed perfectly healthy. */
          /* 16 KB, explicitly. The SDK's default core 1 stack is a couple of
             KB and the tile renderer goes far past that; launched with the
             default, core 1 accepted 1,745 render requests and completed
             none - it died on its first frame, silently, while the link
             stayed perfectly healthy. That is almost certainly the whole of
             the "core 1 entered the replay and never returned" result that
             kept the renderer on core 0. */
          multicore_launch_core1_with_stack(slave_render_core, g_render_stack,
                                            sizeof(g_render_stack));
          printf("[slave] render core launched (%u byte stack)\n",
                 (unsigned)sizeof(g_render_stack));
      } }

    STAGE(6);
    link_init(&g_link, LINK_PIO_SLAVE,
              S_LINK_B_DATA_BASE,   /* we transmit on bus B */
              S_LINK_A_DATA_BASE,   /* and receive on bus A */
              S_LINK_DB_OUT, S_LINK_DB_IN,
              S_LINK_FS, false /* the master drives FS */);
    link_set_bulk_clkdiv(&g_link, 1.0f);

    STAGE(7);
    g_sess.link    = &g_link;
    g_sess.ctrl_tx = g_ctrl_tx;
    g_sess.ctrl_rx = g_ctrl_rx;
    g_sess.seq     = 0;
    g_sess.handshake_timeout_us = LINK_HANDSHAKE_TIMEOUT_US;

    printf("[slave] link up, waiting for the master\n");
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    STAGE(8);
    uint32_t last_report = 0;
    bool boot_confirmed = false;

    for (;;) {
        /* A timeout is not an error: the master is simply idle, or
         * showing the ROM selector, or still booting. Loop and wait
         * again. */
        STAGE(9);
        uint16_t op = link_s_wait_ctrl(&g_sess, 1000000u /* 1 s */);

        switch (op) {
        case 0:                                            break;
        case LINK_OP_HELLO:      handle_hello();            break;
        case LINK_OP_RESET:      handle_reset();            break;
        case LINK_OP_CONFIG:     handle_config();           break;
        case LINK_OP_ARAM_BEGIN:
            link_s_send_ctrl(&g_sess, LINK_OP_ARAM_BEGIN_ACK, 0, 0, NULL, 0);
            break;
        case LINK_OP_ARAM_CHUNK: handle_aram_chunk();       break;
        case LINK_OP_ARAM_END:   handle_aram_end();         break;
        case LINK_OP_FRAME:      handle_frame();            break;
        case LINK_OP_PING:
            link_s_send_ctrl(&g_sess, LINK_OP_PONG, 0, 0, NULL, 0);
            break;
        default:
            printf("[slave] unknown opcode 0x%04x\n", op);
            break;
        }

        /* Once a second, say what is happening. The slave has no screen
         * and no other way to be observed short of a debug probe.
         *
         * UNCONDITIONALLY - the `if (g_frames || g_bad_frames)` guard this
         * replaces made silence ambiguous, and that ambiguity cost real
         * debugging time: with the link down the slave printed nothing, which
         * is indistinguishable from a hung slave, a dead UART and a slave that
         * never booted. It was in fact healthy and idle every time. A line
         * with zeroes in it is worth far more than no line, and once a second
         * is nowhere near enough output to perturb anything.
         *
         * The uptime is what makes it diagnostic: a counter that stops moving
         * says "hung after boot", which no snapshot of state can tell you. */
        watchdog_update();

        uint32_t now = time_us_32();
        if (!boot_confirmed && now >= SLAVE_BOOT_OK_US) {
            /* Ten seconds of serving the loop: this image boots. Forget the
               failure history so it cannot accumulate across power cycles. */
            watchdog_hw->scratch[0] = SLAVE_BOOT_MAGIC;
            boot_confirmed = true;
        }

        if (now - last_report >= 1000000u) {
            last_report = now;
#ifdef FRANK_SNES_PPU_SLAVE
            /* Driven from HERE, not from the render core: the render core only
               runs when the master kicks it, and the whole point of the
               self-test is to answer "can this renderer draw?" without the
               master in the picture at all. Once a second is plenty to read a
               counter, and it still reaches the screen if a master IS
               attached. */
            if (g_ppu_selftest) {
                extern void slave_ppu_selftest_build(void);
                extern void slave_ppu_selftest_frame(void);
                extern uint8_t *slave_ppu_screen;
                static bool st_built;
                if (!st_built) { st_built = true; slave_ppu_selftest_build(); }
                slave_ppu_screen = g_ppu_fb[g_ppu_slot];
                memset(slave_ppu_screen, 0xAA, 256u * 224u);
                slave_ppu_selftest_frame();
                { uint32_t nz = 0, touched = 0;
                  const uint8_t *fb = slave_ppu_screen;
                  for (uint32_t q = 0; q < 256u * 224u; q += 37) {
                      if (fb[q] != 0xAAu) touched++;
                      if (fb[q] && fb[q] != 0xAAu) nz++;
                  }
                  g_dbg_touched = touched; g_dbg_selftest_nz = nz; }
                g_ppu_fb_bytes = 256u * 224u;
                g_ppu_slot ^= 1u;
                g_ppu_fb_valid = true;
            }
#endif
            printf("[slave] up %lus %lu frames, %lu bad",
                   (unsigned long)(now / 1000000u),
                   (unsigned long)g_frames, (unsigned long)g_bad_frames);
#ifdef FRANK_SNES_PPU_SLAVE
            /* The renderer's own verdict, on the same line: it has been
               replaying records and drawing nothing, and these are the exact
               values the draw path gates on. */
            { extern volatile uint32_t slave_ppu_render_us, slave_ppu_records,
                                       slave_ppu_psram_ok, slave_ppu_impossible,
                                       slave_ppu_alloc_fail, slave_ppu_heap_max_kb,
                                       slave_ppu_alloc_probe, slave_ppu_stage,
                                       slave_ppu_live_recs, slave_ppu_last_tag;
              (void)slave_ppu_alloc_probe;
              extern uint32_t slave_ppu_dbg_pitch_h(void), slave_ppu_dbg_flags(void);
              uint32_t ph = slave_ppu_dbg_pitch_h();
              printf(" | ppu %luus %lurec psram=%lu imp=%lu pitch=%lu h=%lu"
                     " flags=%08lx len=%lu nzdrawn=%lu nzsent=%lu slot=%lu",
                     (unsigned long)slave_ppu_render_us,
                     (unsigned long)slave_ppu_records,
                     (unsigned long)slave_ppu_psram_ok,
                     (unsigned long)slave_ppu_impossible,
                     (unsigned long)(ph & 0xffffu),
                     (unsigned long)(ph >> 16),
                     (unsigned long)slave_ppu_dbg_flags(),
                     (unsigned long)g_dbg_len,
                     (unsigned long)g_dbg_nz_drawn,
                     (unsigned long)g_dbg_nz_sent,
                     (unsigned long)g_ppu_slot);
              extern volatile uint32_t frank_gfx_fail,
                                       frank_gfx_localstate_bytes,
                                       frank_gfx_zero_bytes;
              printf(" gfxfail=%lu ls=%lu zero=%lu probe=%lu",
                     (unsigned long)frank_gfx_fail,
                     (unsigned long)frank_gfx_localstate_bytes,
                     (unsigned long)frank_gfx_zero_bytes,
                     (unsigned long)slave_ppu_alloc_probe);
              /* The truncation, in full: where the replay stopped, why, the
                 bytes there, and a checksum of what it replayed from. The
                 master prints the same checksum over what it SENT. */
              { extern volatile uint32_t slave_ppu_stream_len,
                                         slave_ppu_stream_sum,
                                         slave_ppu_stop_off,
                                         slave_ppu_stop_ctx,
                                         slave_ppu_stop_why,
                                         slave_ppu_bad_lines;
                extern volatile uint32_t slave_ppu_exp_sum, slave_ppu_sum_ok,
                                         slave_ppu_sum_bad, slave_ppu_bad_multi,
                                         slave_ppu_bad_single, slave_ppu_bad_wire,
                                         slave_ppu_bad_stage;
                printf(" | slen=%lu sum=%08lx exp=%08lx ok=%lu bad=%lu"
                       " stop=%lu why=%lu badm=%lu bads=%lu wire=%lu stage=%lu rt=%lu/%lu",
                       (unsigned long)slave_ppu_stream_len,
                       (unsigned long)slave_ppu_stream_sum,
                       (unsigned long)slave_ppu_exp_sum,
                       (unsigned long)slave_ppu_sum_ok,
                       (unsigned long)slave_ppu_sum_bad,
                       (unsigned long)slave_ppu_stop_off,
                       (unsigned long)slave_ppu_stop_why,
                       (unsigned long)slave_ppu_bad_multi,
                       (unsigned long)slave_ppu_bad_single,
                       (unsigned long)slave_ppu_bad_wire,
                       (unsigned long)slave_ppu_bad_stage,
                       (unsigned long)g_ppu_stage_retry,
                       (unsigned long)g_ppu_stage_lost);
                extern volatile uint32_t slave_ppu_upd_calls, slave_ppu_vram_w,
                                         slave_ppu_cgram_w, slave_ppu_r2100_w,
                                         slave_ppu_r2100_last,
                                         slave_ppu_r2100_seen;
                extern uint32_t slave_ppu_dbg_content(void);
                extern void slave_ppu_dbg_render(uint32_t *);
                extern volatile uint32_t frank_dbg_bg_calls, frank_dbg_obj_calls;
                uint32_t ct = slave_ppu_dbg_content();
                { uint32_t r[6];
                  slave_ppu_dbg_render(r);
                  printf(" | Y=%lu..%lu line=%lu/%lu mode=%lu tm=%02lx ts=%02lx"
                         " clip=%lu/%lu/%lu tiles=%lu cached=%lu touched=%lu bg=%lu obj=%lu stnz=%lu",
                         (unsigned long)(r[0] & 0xffff),
                         (unsigned long)(r[0] >> 16),
                         (unsigned long)(r[1] & 0xffff),
                         (unsigned long)(r[1] >> 16),
                         (unsigned long)(r[2] & 0xff),
                         (unsigned long)((r[2] >> 8) & 0xff),
                         (unsigned long)((r[2] >> 16) & 0xff),
                         (unsigned long)(r[3] & 0xff),
                         (unsigned long)((r[3] >> 8) & 0xff),
                         (unsigned long)((r[3] >> 16) & 0xff),
                         (unsigned long)r[4], (unsigned long)r[5],
                         (unsigned long)g_dbg_touched,
                         (unsigned long)frank_dbg_bg_calls,
                         (unsigned long)frank_dbg_obj_calls,
                         (unsigned long)g_dbg_selftest_nz); }
                printf(" | r2100 n=%lu last=%02lx seen=%04lx vramnz=%lu palnz=%lu",
                       (unsigned long)slave_ppu_r2100_w,
                       (unsigned long)slave_ppu_r2100_last,
                       (unsigned long)slave_ppu_r2100_seen,
                       (unsigned long)(ct & 0xffffu),
                       (unsigned long)(ct >> 16));
                extern uint32_t slave_ppu_dbg_regs(void);
                printf(" | vramat=%08lx upd=%lu vram=%lu cgram=%lu regs=%08lx"
                       " want=%lu rx=%lu/%lu zero=%lu over=%lu/%lu sram=%lu",
                       (unsigned long)(uintptr_t)Memory.VRAM,
                       (unsigned long)slave_ppu_upd_calls,
                       (unsigned long)slave_ppu_vram_w,
                       (unsigned long)slave_ppu_cgram_w,
                       (unsigned long)slave_ppu_dbg_regs(),
                       (unsigned long)g_ppu_last_want,
                       (unsigned long)g_ppu_recv_ok,
                       (unsigned long)g_ppu_recv_fail,
                       (unsigned long)g_ppu_want_zero,
                       (unsigned long)g_ppu_oversize,
                       (unsigned long)g_ppu_oversize_sram,
                       (unsigned long)g_dbg_sram);
                printf(" | core1 alive=%lu kick=%lu done=%lu wait=%lu/%luus",
                       (unsigned long)g_render_alive,
                       (unsigned long)g_render_kicks,
                       (unsigned long)g_render_dones,
                       (unsigned long)g_render_waits,
                       (unsigned long)g_render_wait_us); } }
            /* The two heads, same frame, whenever one has been latched. This
               is the whole point: a checksum says the delivery is wrong, these
               say what it actually is. */
            { extern volatile uint32_t slave_ppu_peek_valid;
              extern volatile uint8_t slave_ppu_exp_peek[], slave_ppu_got_peek[];
              if (slave_ppu_peek_valid) {
                  printf("\n[slave] vram@8000 master:");
                  for (uint32_t q = 0; q < LINK_PPU_VRAMPEEK_BYTES; q++)
                      printf(" %02x", slave_ppu_exp_peek[q]);
                  printf("\n[slave] vram@8000 slave :");
                  for (uint32_t q = 0; q < LINK_PPU_VRAMPEEK_BYTES; q++)
                      printf(" %02x", slave_ppu_got_peek[q]);
                  slave_ppu_peek_valid = 0;
              } }
            if (g_ppu_head_valid) {
                printf("\n[slave] sent:");
                for (uint32_t q = 0; q < LINK_PPU_HEAD_BYTES; q++)
                    printf(" %02x", g_ppu_exp_head[q]);
                printf("\n[slave] got :");
                for (uint32_t q = 0; q < LINK_PPU_HEAD_BYTES; q++)
                    printf(" %02x", g_ppu_got_head[q]);
                g_ppu_head_valid = 0;
            }
#endif
            printf("\n");
            g_frames = g_bad_frames = 0;
        }
    }
}
