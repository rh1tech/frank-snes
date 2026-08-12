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
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/vreg.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/sysinfo.h"

#include "link_bus.h"
#include "link_pins.h"
#include "link_proto.h"
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
static uint8_t  g_ppu_fb[2][LINK_PPU_MAX_BYTES];
static uint32_t g_ppu_fb_bytes;
static uint32_t g_ppu_slot;
static bool     g_ppu_fb_valid;
volatile uint32_t g_ppu_oversize;   /* frames too big for the buffer */
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
    extern uint8_t *slave_ppu_stream_buf;
    g_ppu_stream = slave_ppu_stream_buf;
    if (g_ppu_stream && link_s_wait_ctrl(&g_sess, 100000u) == LINK_OP_PPU_STREAM) {
        uint32_t want = link_rx_hdr(&g_sess)->arg0;

        /* Whatever the master announced MUST be taken off the wire. Skipping
           it because it does not fit leaves those bytes in flight and every
           later exchange misaligned - the link then never recovers. Receive
           it, and only then decide whether it is usable. */
        if (want > SLAVE_PPU_STREAM_BYTES) {
            g_ppu_oversize++;
            if (link_s_bulk_recv(&g_sess, g_ppu_stream,
                                 LINK_ALIGN4(SLAVE_PPU_STREAM_BYTES)))
                ppu_len = 0;          /* drained what we could; frame is lost */
        } else if (want) {
            if (link_s_bulk_recv(&g_sess, g_ppu_stream, LINK_ALIGN4(want)))
                ppu_len = want;
        }
    }
#endif

    if (g_have_pending) {
        uint32_t send_slot = g_mix_slot ^ 1u;
        uint32_t got = g_pending_reply.samples;

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
        uint32_t fb_bytes = g_ppu_fb_valid ? g_ppu_fb_bytes : 0;
        {
            extern volatile uint32_t slave_ppu_render_us, slave_ppu_records,
                                     slave_ppu_psram_ok, slave_ppu_impossible;
            link_ppu_stat_t st;
            st.render_us  = slave_ppu_render_us;
            st.records    = slave_ppu_records;
            st.oversize   = g_ppu_oversize;
            { extern volatile uint32_t slave_ppu_alloc_fail;
              st.psram_ok = slave_ppu_psram_ok | (slave_ppu_alloc_fail << 8); }
            st.impossible = slave_ppu_impossible;
            link_s_send_ctrl(&g_sess, LINK_OP_PPU_FRAME, fb_bytes, 0,
                             &st, sizeof(st));
        }
        if (fb_bytes)
            link_s_bulk_send(&g_sess, g_ppu_fb[g_ppu_slot ^ 1u],
                             LINK_ALIGN4(fb_bytes));
    }
#endif

    /* Now mix, with the wire idle and the master away doing its own
     * frame. How many samples is not carried in the bulk: the count in
     * the reply tells the master what to arm for next time. */
    slave_sound_frame(g_events, n_events,
                      g_samples[g_mix_slot], slave_sound_want(chunks),
                      &g_pending_reply);

#ifdef FRANK_SNES_PPU_SLAVE
    /* Wire idle, master away: render this frame's stream for next time. */
    if (ppu_len) {
        extern void slave_ppu_replay(const uint8_t *rec, uint32_t len);
        extern uint32_t slave_ppu_copy_frame(uint8_t *dst, uint32_t max);
        slave_ppu_replay(g_ppu_stream, ppu_len);
        g_ppu_fb_bytes = slave_ppu_copy_frame(g_ppu_fb[g_ppu_slot],
                                              LINK_PPU_MAX_BYTES);
        g_ppu_slot ^= 1u;
        g_ppu_fb_valid = true;
    }
#endif

    g_mix_slot ^= 1u;
    g_have_pending = true;

    g_frames++;
}

/* ------------------------------------------------------------------ */
/* Boot                                                               */
/* ------------------------------------------------------------------ */

int main(void)
{
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
    sleep_ms(50);

    printf("\n[slave] frank-snes C2 sound slave, fw %u.%02u\n",
           SLAVE_FW_VERSION >> 8, SLAVE_FW_VERSION & 0xff);
    printf("[slave] sys_clk %lu Hz\n", (unsigned long)clock_get_hz(clk_sys));

    STAGE(3);
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    STAGE(4);

    slave_sound_init();
    STAGE(5);
    printf("[slave] mixer up, heap %u bytes\n",
           (unsigned)slave_heap_bytes_used());

    /* PPU offload: the renderer lives here now. Core 0 keeps the S-DSP, the
       renderer runs on core 1. Measured on the master: handing the renderer
       over frees 7,642 us/frame and takes it from 46 to 50 fps. */
    { extern bool slave_ppu_init(void);
      if (!slave_ppu_init())
          printf("[slave] FATAL: slave_ppu_init failed (out of memory)\n");
      else
          printf("[slave] renderer up, heap %u bytes\n",
                 (unsigned)slave_heap_bytes_used()); }

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
         * and no other way to be observed short of a debug probe. */
        uint32_t now = time_us_32();
        if (now - last_report >= 1000000u) {
            last_report = now;
            if (g_frames || g_bad_frames) {
                printf("[slave] %lu frames, %lu bad\n",
                       (unsigned long)g_frames, (unsigned long)g_bad_frames);
                g_frames = g_bad_frames = 0;
            }
        }
    }
}
