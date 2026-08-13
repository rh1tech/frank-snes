/*
 * frank-snes — C2 inter-processor sound link, master half
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * link_master.c — bring-up and the per-frame exchange with the sound
 * slave. Ported from frank-genesis's src/link_master.c; the transport
 * and the doorbell sequencing are identical, only the payloads differ.
 */

#ifdef C2_SOUND_LINK

#include "link_master.h"

#include <stdio.h>
#include <string.h>
#include "ppu_capture.h"

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/clocks.h"

#include "link_bus.h"
#include "link_pins.h"
#include "link_session.h"

#ifndef LOG
#define LOG(...) printf(__VA_ARGS__)
#endif

static link_t         g_link;
static link_session_t g_sess;
static bool           g_online;

/* Control frames are DMA'd straight out of and into these, so they must
 * be word-aligned and must not move. */
static uint8_t __attribute__((aligned(4))) g_ctrl_tx[LINK_CTRL_BYTES];
static uint8_t __attribute__((aligned(4))) g_ctrl_rx[LINK_CTRL_BYTES];

/* Steady-state doorbell patience. Short on purpose: a slave that has
 * died should cost a frame of audio, not stall the emulator. Boot-time
 * probes raise it — the slave may still be coming up.
 *
 * Not *too* short, though. The slave has to finish mixing the previous
 * frame before it can service this one, and a multi-chunk frame is a few
 * milliseconds of mixing. At 3000 us the link dropped as soon as the
 * emulator fell below 60 fps and started asking for extra audio; at
 * 8000 us it still dropped on the occasional long frame. 20 ms is about
 * one emulated frame at 60 fps, which is the right order: longer than
 * any mix, shorter than giving up on a dead slave costs. */
#define LINK_FRAME_TIMEOUT_US   20000u

static uint32_t g_exchanges, g_failures, g_last_us;

/* ------------------------------------------------------------------ */
/* Bring-up                                                           */
/* ------------------------------------------------------------------ */

/* The reason string's address, published so a probe can read WHY the link
   dropped without a console - this board has no usable UART. */
volatile const char *frank_link_why;

static void go_offline(const char *why)
{
    if (g_online) {
        LOG("[link] offline: %s\n", why);
        g_failures++;
        frank_link_why = why;
    }
    g_online = false;
}

bool link_master_init(void)
{
    link_init(&g_link, LINK_PIO_MASTER,
              M_LINK_A_DATA_BASE,   /* we transmit on bus A */
              M_LINK_B_DATA_BASE,   /* and receive on bus B */
              M_LINK_DB_OUT, M_LINK_DB_IN,
              M_LINK_FS, true /* master drives FS */);

    /* Both halves are built at the same CPU_SPEED, so a divider of 1.0
     * is what the wire was characterised at. link_bus.c keeps control
     * frames at LINK_CTRL_CLKDIV regardless. */
    link_set_bulk_clkdiv(&g_link, 1.0f);

    g_sess.link    = &g_link;
    g_sess.ctrl_tx = g_ctrl_tx;
    g_sess.ctrl_rx = g_ctrl_rx;
    g_sess.seq     = 0;
    g_sess.handshake_timeout_us = LINK_HANDSHAKE_TIMEOUT_US;

    g_online = true;   /* provisional, so the HELLO below is allowed out */

    if (!link_m_send_ctrl(&g_sess, LINK_OP_HELLO, 0, 0, NULL, 0)) {
        go_offline("slave did not accept HELLO");
        return false;
    }
    if (!link_m_recv_ctrl(&g_sess)) {
        go_offline("no HELLO_ACK");
        return false;
    }
    if (link_rx_hdr(&g_sess)->op != LINK_OP_HELLO_ACK) {
        go_offline("unexpected reply to HELLO");
        return false;
    }

    const link_node_info_t *info =
        (const link_node_info_t *)(g_ctrl_rx + sizeof(link_hdr_t));

    LOG("[link] slave up: %s, fw %u.%02u, sys_clk %lu Hz, psram %lu bytes\n",
        info->package_is_a ? "RP2350A" : "RP2350B",
        (unsigned)(info->fw_version >> 8), (unsigned)(info->fw_version & 0xff),
        (unsigned long)info->sys_clk_hz, (unsigned long)info->psram_bytes);

    if (info->proto_ver != LINK_PROTO_VER) {
        LOG("[link] protocol mismatch: master %u, slave %lu\n",
            (unsigned)LINK_PROTO_VER, (unsigned long)info->proto_ver);
        go_offline("protocol mismatch");
        return false;
    }

    /* The two halves must agree on the clock: the receiving PIO program
     * has to finish its loop inside the transmitter's byte period, and
     * each side derives that from its own system clock. A mismatch gives
     * a link that works one way and silently drops bytes the other. */
    if (info->sys_clk_hz != clock_get_hz(clk_sys)) {
        LOG("[link] clock mismatch: master %lu Hz, slave %lu Hz — "
            "rebuild both halves at the same CPU_SPEED\n",
            (unsigned long)clock_get_hz(clk_sys),
            (unsigned long)info->sys_clk_hz);
        go_offline("clock mismatch");
        return false;
    }

    /* Steady state from here on. */
    g_sess.handshake_timeout_us = LINK_FRAME_TIMEOUT_US;
#ifdef FRANK_SNES_PPU_CAPTURE
    /* The slave starts with an empty PPU and cannot ask for what it missed. */
    ppucap_request_resync();
#endif
    return true;
}

bool link_master_online(void) { return g_online; }

/*
 * Come back from a failed exchange without re-initialising the PIO.
 *
 * A single failure takes the link offline, which is the right call at the
 * moment it happens — the doorbells are mid-phase and guessing where the
 * peer got to is worse than stopping. What was wrong was that it stayed
 * offline: the slave's loop times out after a second and goes back to
 * waiting for a control frame, so the wire is usable again almost
 * immediately, and yet the machine played silent until the next ROM load.
 * One glitched handshake cost the rest of the session's sound.
 *
 * Recovery is the same three steps a human would take: stop driving,
 * wait for the peer to stop driving, then say hello. link_init() is
 * deliberately not repeated — it claims PIO state machines and DMA
 * channels, and claiming them twice panics.
 */
bool link_master_reprobe(void)
{
    /* Quiesce our half: drop the doorbell and discard any receive the
     * failed exchange left armed, or the next reply lands in the wrong
     * buffer at the wrong offset. */
    link_db_set(&g_link, false);
    link_rx_abort(&g_link);

    /* The slave finishes or times out of whatever phase it was in before
     * it drops its own doorbell. Until then the wire is still busy and a
     * HELLO would just fail again. */
    if (!link_db_wait(&g_link, false, LINK_HANDSHAKE_TIMEOUT_US))
        return false;

    g_sess.handshake_timeout_us = LINK_HANDSHAKE_TIMEOUT_US;
    g_online = true;                 /* provisional, as at boot */

    bool ok = link_m_send_ctrl(&g_sess, LINK_OP_HELLO, 0, 0, NULL, 0) &&
              link_m_recv_ctrl(&g_sess) &&
              link_rx_hdr(&g_sess)->op == LINK_OP_HELLO_ACK;

    if (!ok) {
        g_online = false;
        return false;
    }

    LOG("[link] recovered\n");
    g_sess.handshake_timeout_us = LINK_FRAME_TIMEOUT_US;
#ifdef FRANK_SNES_PPU_CAPTURE
    /* Every exchange missed while offline is a write the slave will never
       see again, so a recovery is exactly as damaging as a cold start. */
    ppucap_request_resync();
#endif
    return true;
}

/* ------------------------------------------------------------------ */
/* Control exchanges                                                  */
/* ------------------------------------------------------------------ */

/* Send one control frame and collect the expected acknowledgement. Any
 * failure takes the link offline: a half-completed exchange has left the
 * doorbells in an unknown phase, and the only clean recovery is to stop
 * using the wire rather than to guess where the peer got to. */
static bool ctrl_round_trip(uint16_t op, uint32_t arg0, uint32_t arg1,
                            const void *payload, uint32_t payload_len,
                            uint16_t expect_op)
{
    if (!g_online) return false;

    if (!link_m_send_ctrl(&g_sess, op, arg0, arg1, payload, payload_len)) {
        go_offline("control send failed");
        return false;
    }
    if (!link_m_recv_ctrl(&g_sess)) {
        go_offline("no control reply");
        return false;
    }
    if (link_rx_hdr(&g_sess)->op != expect_op) {
        go_offline("wrong control reply");
        return false;
    }
    return true;
}

bool link_master_send_config(const link_sound_config_t *cfg)
{
    return ctrl_round_trip(LINK_OP_CONFIG, 0, 0, cfg, sizeof(*cfg),
                           LINK_OP_CONFIG_ACK);
}

bool link_master_reset_and_sync_aram(const uint8_t *aram)
{
    if (!g_online) return false;

    /* Boot-time patience again: a reset lands while the slave may be
     * clearing 64 KB of its own, which takes longer than a frame. */
    uint32_t saved = g_sess.handshake_timeout_us;
    g_sess.handshake_timeout_us = LINK_HANDSHAKE_TIMEOUT_US;

    bool ok = ctrl_round_trip(LINK_OP_RESET, 0, 0, NULL, 0,
                              LINK_OP_RESET_ACK);

    if (ok) ok = ctrl_round_trip(LINK_OP_ARAM_BEGIN, LINK_ARAM_BYTES, 0,
                                 NULL, 0, LINK_OP_ARAM_BEGIN_ACK);

    for (uint32_t off = 0; ok && off < LINK_ARAM_BYTES;
         off += LINK_ARAM_CHUNK_BYTES) {
        uint32_t n = LINK_ARAM_BYTES - off;
        if (n > LINK_ARAM_CHUNK_BYTES) n = LINK_ARAM_CHUNK_BYTES;

        if (!link_m_send_ctrl(&g_sess, LINK_OP_ARAM_CHUNK, off, n, NULL, 0)) {
            go_offline("ARAM chunk header failed");
            ok = false;
            break;
        }
        if (!link_m_bulk_send(&g_sess, aram + off, n)) {
            go_offline("ARAM chunk body failed");
            ok = false;
            break;
        }
        if (!link_m_recv_ctrl(&g_sess) ||
            link_rx_hdr(&g_sess)->op != LINK_OP_ARAM_CHUNK_ACK) {
            go_offline("no ARAM chunk ack");
            ok = false;
            break;
        }
    }

    if (ok) {
        uint32_t crc = link_crc32(aram, LINK_ARAM_BYTES);
        ok = ctrl_round_trip(LINK_OP_ARAM_END, crc, 0, NULL, 0,
                             LINK_OP_ARAM_END_ACK);
        if (ok) {
            uint32_t theirs = link_rx_hdr(&g_sess)->arg0;
            if (theirs != crc) {
                LOG("[link] APU RAM CRC mismatch: sent %08lx, slave saw %08lx\n",
                    (unsigned long)crc, (unsigned long)theirs);
                go_offline("APU RAM CRC mismatch");
                ok = false;
            }
        }
    }

    g_sess.handshake_timeout_us = ok ? saved : LINK_FRAME_TIMEOUT_US;
    return ok;
}

/* ------------------------------------------------------------------ */
/* The per-frame exchange                                             */
/* ------------------------------------------------------------------ */

#ifdef FRANK_SNES_PPU_CAPTURE
/* Staged by the emulation loop before the exchange; consumed inside it so the
 * PPU traffic shares the sound frame's doorbell phases. */
static const uint8_t *g_ppu_stream;
static uint32_t       g_ppu_len;
static uint8_t       *g_ppu_fb;
static uint32_t       g_ppu_fb_max;
static uint32_t       g_ppu_fb_got;

/* The stream goes on the wire from SRAM, never from PSRAM.
 *
 * A bulk DMA reading the XIP window is what was killing HDMI. Bisected on
 * hardware with two live switches, holding everything else constant and with
 * the slave sending a synthetic test pattern so the renderer could not be the
 * explanation:
 *
 *   capture off, nothing sent          -> SIGNAL on 6 of 6 grabs
 *   capture ON, stream NOT sent        -> SIGNAL on 6 of 6 grabs
 *   capture ON, stream sent from PSRAM -> NOSIG  on 4 of 5 grabs
 *
 * So the CPU writing ~800 records a frame into PSRAM is harmless, and
 * xip_cache_clean_all() is harmless - both still happen in the working cases.
 * It is specifically the DMA reading PSRAM: it holds the DMA block long
 * enough on QMI latency that the scanline channel misses its slot and the
 * TMDS serialiser underruns. The interrupt itself is never late (31,507/s,
 * max gap 65 us in the broken case), which is why every timing-based theory
 * came back clean. The same weakness is already recorded from the other
 * direction in src/main.c: an RX DMA into PSRAM could not sustain the
 * framebuffer and took the link offline 12 times in 16 seconds.
 *
 * 16 KB, because the master has little SRAM to spare - ~50 KB between .bss
 * and the heap limit, out of which core 1's stack and the allocator still
 * have to come, and a 16 KB static buffer here once left 7,128 bytes free.
 * It covers the steady state completely: MK3 in play captures 2,199 bytes a
 * frame and 12 KB in the busiest scenes. Only the rare VRAM-upload frames
 * (41,760 bytes measured) exceed it, and those still go straight from PSRAM
 * and still disturb the display - g_ppu_tx_psram counts them so that stays
 * visible rather than becoming a mystery flicker. */
#define PPU_TX_BOUNCE_BYTES LINK_PPU_STREAM_CHUNK
static uint8_t __attribute__((aligned(4))) g_ppu_tx_bounce[PPU_TX_BOUNCE_BYTES];
volatile uint32_t g_ppu_tx_psram;   /* frames too big to bounce through SRAM */

void link_master_ppu_stage(const uint8_t *ppu_stream, uint32_t ppu_len,
                           uint8_t *fb, uint32_t fb_max)
{
    /* Copied here rather than inside the exchange: this is outside the
       doorbell handshake, so it cannot add to the master's stall if it is
       ever slow. Reading PSRAM with the CPU is fine - it is the DMA that
       cannot. */
    g_ppu_stream = ppu_stream;      /* copied out chunk by chunk at send time */
    g_ppu_len    = ppu_len;
    g_ppu_fb     = fb;
    g_ppu_fb_max = fb_max;
}

uint32_t link_master_ppu_got(void) { return g_ppu_fb_got; }

/* Bisect switch: capture the stream but do not put it on the wire.
 *
 * With frank_cap_on=0 - no capture at all - HDMI locks on every grab; with it
 * on, the display never locks, and the scanline interrupt's timing is
 * perfect either way (31,507/s, max gap 65 us). So the capture path breaks
 * the picture without disturbing the interrupt, and the two halves of it need
 * separating: the CPU writing records into PSRAM, and the DMA reading that
 * PSRAM to send it. This switch keeps the first and removes the second.
 *
 * Announcing a zero length is what makes it safe - the slave then never arms
 * a receive, so the exchange stays in step. */
volatile uint32_t g_ppu_tx_disable;

/* What the last exchange actually put on the wire, and how many exchanges
   have carried a stream. Paired with frank_cap_takes this says whether the
   capture and the send run at the same rate. */
volatile uint32_t g_ppu_sent_len, g_ppu_sends;

/* The slave's per-frame diagnostics, published so a probe on the MASTER can
   see inside the slave - there is no console or probe on that chip. */
volatile link_ppu_stat_t g_ppu_stat;

/* Where the exchange's time goes. The total jumped from 370 us to 10,445 us
   the moment the slave began returning real framebuffers, and 57 KB at the
   bulk rate should cost ~0.6 ms - so the time is being spent WAITING, and
   this says on what. */
volatile uint32_t g_ph_sound, g_ph_ev, g_ph_aram, g_ph_ppu_tx, g_ph_ack, g_ph_fb;

/* The slave's palette, and whether one has arrived. The master pushes it into
   the HDMI driver; it no longer computes one itself. */
uint32_t g_ppu_palette[256];
volatile bool g_ppu_pal_valid;

#endif

bool link_master_frame_exchange(const link_event_t *events, uint32_t n_events,
                                const link_aram_run_t *runs, uint32_t n_runs,
                                uint32_t chunks, const uint8_t *aram,
                                int16_t *samples, uint32_t n_samples,
                                link_frame_reply_t *reply)
{
    if (!g_online) return false;

    uint32_t t0 = time_us_32();

#ifdef FRANK_SNES_PPU_CAPTURE
    /* Applied here, once, so the length in the payload and the bulk that
       follows it cannot disagree. */
    if (g_ppu_tx_disable) g_ppu_len = 0;
#endif

    /* --- header (carrying the run table), then this frame's payloads --- */
    if (n_runs > LINK_ARAM_MAX_RUNS) n_runs = LINK_ARAM_MAX_RUNS;

    /* The PPU stream length rides in this frame's payload, exactly as the run
       table does and for the same reason the note on LINK_OP_FRAME gives: a
       separate control frame costs a doorbell round trip whatever it carries.
       Measured, it cost 10.2 ms of a 10.5 ms exchange - the 57 KB framebuffer
       bulk itself was 105 us. Phases, not bytes. */
    {
        uint8_t pl[LINK_PAYLOAD_BYTES];
        uint32_t rt = n_runs * sizeof(link_aram_run_t);
        memset(pl, 0, sizeof(pl));
        memcpy(pl, runs, rt);
        /* The PPU length only exists in an offload build; without it the
           payload is just the run table. Guarding the USES as well as the
           declarations is what lets the non-offload configuration build at
           all - it had stopped, which meant the one experiment that isolates
           the renderer from everything else could not be run. */
#ifdef FRANK_SNES_PPU_CAPTURE
        memcpy(pl + LINK_PPU_LEN_OFFSET, &g_ppu_len, sizeof(uint32_t));
        { extern volatile uint32_t frank_cap_sum;
          uint32_t s = frank_cap_sum;
          memcpy(pl + LINK_PPU_SUM_OFFSET, &s, sizeof(uint32_t)); }
        if (g_ppu_stream && g_ppu_len)
            memcpy(pl + LINK_PPU_HEAD_OFFSET, g_ppu_stream,
                   g_ppu_len < LINK_PPU_HEAD_BYTES ? g_ppu_len
                                                   : LINK_PPU_HEAD_BYTES);
        const uint32_t pl_bytes = LINK_PPU_HEAD_OFFSET + LINK_PPU_HEAD_BYTES;
#else
        const uint32_t pl_bytes = rt;
#endif
        if (!link_m_send_ctrl(&g_sess, LINK_OP_FRAME, n_events,
                              LINK_FRAME_ARG1(n_runs, chunks),
                              pl, pl_bytes)) {
            go_offline("frame header failed");
            return false;
        }
    }

#ifdef FRANK_SNES_PPU_CAPTURE
    g_ph_sound = time_us_32() - t0;
#endif
    if (n_events) {
        if (!link_m_bulk_send(&g_sess, events,
                              n_events * sizeof(link_event_t))) {
            go_offline("event bulk failed");
            return false;
        }
    }

#ifdef FRANK_SNES_PPU_CAPTURE
    g_ph_ev = time_us_32() - t0;
#endif
    if (n_runs) {
        for (uint32_t i = 0; i < n_runs; i++) {
            const uint8_t *src = aram +
                ((uint32_t)runs[i].first_page << LINK_ARAM_PAGE_BITS);
            uint32_t bytes =
                (uint32_t)runs[i].page_count << LINK_ARAM_PAGE_BITS;
            if (!link_m_bulk_send(&g_sess, src, bytes)) {
                go_offline("APU RAM run failed");
                return false;
            }
        }
    }

#ifdef FRANK_SNES_PPU_CAPTURE
    /* The PPU command stream rides here: after the sound payloads, before
       the reply, so it costs no extra doorbell phase. */
#ifdef FRANK_SNES_PPU_CAPTURE
    g_ph_aram = time_us_32() - t0;
#endif

    /* No control frame of its own: the length was in the FRAME payload. */
    g_ppu_fb_got = 0;
    /* LINK_ALIGN4, matching the slave's arm exactly. The sample bulk aligns
       on both sides; aligning on only one leaves the receiver's DMA waiting
       for the padding bytes that were never sent - it times out and the
       sender stalls. Measured: 10.2 ms of a 10.4 ms exchange, for three
       missing bytes. */
    g_ppu_sent_len = g_ppu_len;
    if (g_ppu_len) g_ppu_sends++;
    /* Chunked, through SRAM, so no DMA ever reads the XIP window. The CPU
       copy out of PSRAM is fine - it is the DMA that is not. */
    for (uint32_t off = 0; off < g_ppu_len; off += LINK_PPU_STREAM_CHUNK) {
        uint32_t n = g_ppu_len - off;
        if (n > LINK_PPU_STREAM_CHUNK) n = LINK_PPU_STREAM_CHUNK;
        memcpy(g_ppu_tx_bounce, g_ppu_stream + off, n);
        if (!link_m_bulk_send(&g_sess, g_ppu_tx_bounce, LINK_ALIGN4(n))) {
            go_offline("ppu stream bulk failed");
            return false;
        }
    }
#endif

#ifdef FRANK_SNES_PPU_CAPTURE
    g_ph_ppu_tx = time_us_32() - t0;
#endif

    /* --- the reply --- */
    if (!link_m_recv_ctrl(&g_sess)) {
        go_offline("no frame ack");
        return false;
    }
    if (link_rx_hdr(&g_sess)->op != LINK_OP_FRAME_ACK) {
        go_offline("wrong frame ack");
        return false;
    }

    memcpy(reply, g_ctrl_rx + sizeof(link_hdr_t), sizeof(*reply));

    /* The slave rounds the sample count the same way from the same
     * number in the reply, so both sides move identical byte counts and
     * the bulk stays word-aligned.
     *
     * A count larger than we asked for is not clamped, it is fatal: the
     * slave is about to put that many bytes on the wire whatever we do,
     * and receiving fewer would leave the surplus in flight and
     * desynchronise every exchange after it. Dropping the link is
     * recoverable; a desynchronised wire is not. */
#ifdef FRANK_SNES_PPU_CAPTURE
    g_ph_ack = time_us_32() - t0;
#endif
    uint32_t got = reply->samples;
    if (got > n_samples) {
        go_offline("slave returned more samples than requested");
        return false;
    }

    if (got) {
        if (!link_m_bulk_recv(&g_sess, samples,
                              LINK_ALIGN4(got * sizeof(int16_t)))) {
            go_offline("sample bulk failed");
            return false;
        }
    }
    reply->samples = got;

#ifdef FRANK_SNES_PPU_CAPTURE
    /* The rendered picture comes back last, still inside this exchange.
       Same rule as the sample bulk above: a length larger than we can hold
       is fatal, not clamped - the slave is about to put those bytes on the
       wire whatever we do, and receiving fewer desynchronises every exchange
       after it. */
    if (g_ppu_fb) {
        uint32_t fb = reply->ppu_fb_bytes;
        g_ppu_stat  = reply->ppu_stat;
        if (fb > g_ppu_fb_max) {
            go_offline("slave returned an oversized framebuffer");
            return false;
        }
        if (fb) {
            if (!link_m_bulk_recv(&g_sess, g_ppu_fb, LINK_ALIGN4(fb))) {
                go_offline("ppu framebuffer bulk failed");
                return false;
            }
        }
        /* The palette always follows the picture - see the slave. */
        if (!link_m_bulk_recv(&g_sess, (void *)g_ppu_palette,
                              sizeof(g_ppu_palette))) {
            go_offline("ppu palette bulk failed");
            return false;
        }
        g_ppu_pal_valid = true;
        g_ppu_fb_got = fb;
        g_ph_fb = time_us_32() - t0;
    }
#endif

    g_last_us = time_us_32() - t0;
    g_exchanges++;
    return true;
}

void link_master_get_stats(uint32_t *exchanges, uint32_t *failures,
                           uint32_t *last_us)
{
    if (exchanges) *exchanges = g_exchanges;
    if (failures)  *failures  = g_failures;
    if (last_us)   *last_us   = g_last_us;
}

#endif /* C2_SOUND_LINK */
