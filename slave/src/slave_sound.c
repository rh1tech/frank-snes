/*
 * frank-snes — C2 slave firmware
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * slave_sound.c — runs the accurate S-DSP from the master's event
 * stream.
 *
 * The DSP itself is src/snes9x/spc_dsp.c, compiled from the shared tree
 * exactly as the M1/M2 master compiles it. What lives here is only the
 * plumbing an SPC700 would otherwise provide: replaying the master's
 * register writes at the point in the frame they happened, and running
 * the DSP forward between them.
 *
 * This is where the second processor finally earns its place. The
 * accurate DSP is a per-sample cycle pipeline rather than the legacy
 * block mixer, and it costs enough that the master drops from 60 fps to
 * 43 running it locally. Here it runs in the time the master spends
 * emulating, and costs the master nothing but the exchange.
 */

#include "slave_sound.h"

#include <stdio.h>
#include <string.h>

#include "spc_dsp.h"
#include "pico/time.h"

/* ------------------------------------------------------------------ */
/* State                                                              */
/* ------------------------------------------------------------------ */

/* The master's APU RAM, mirrored here. The DSP reads BRR data straight
 * out of it, which is why the mirror exists at all. */
static uint8_t __attribute__((aligned(4))) aram[LINK_ARAM_BYTES];

static bool     cfg_mono = true;
static uint32_t cfg_samples_per_frame = LINK_SAMPLES_PER_CHUNK;
static uint32_t cfg_rate = 32040;

/* Sub-frame timing. `frame_span` is the master's timebase for one frame
 * (V_Counter * H_Max + CPU.Cycles, so it must match what the master
 * stamps events with); `frame_clocks` is what the DSP owes for it. */
/* Overwritten by the first LINK_OP_CONFIG, which the master always sends
 * before any frame. The default is NTSC (262 lines x 1364 master cycles)
 * purely so a frame arriving out of order cannot divide by something
 * absurd — the span is region-dependent and is never assumed here. */
static uint32_t frame_span   = 262u * 1364u;
static uint32_t frame_clocks;
static uint32_t clocks_done;

/* The DSP writes stereo pairs here; slave_sound_frame() converts to
 * whatever the master asked for. */
static int16_t  pairs_buf[LINK_MAX_SAMPLES * 2];
static uint32_t stat_overflows;

uint8_t *slave_sound_aram(void) { return aram; }

uint32_t slave_sound_want(uint32_t chunks)
{
    uint32_t per = cfg_mono ? cfg_samples_per_frame
                            : cfg_samples_per_frame * 2u;
    uint32_t want = per * (chunks ? chunks : 1u);
    return want > LINK_MAX_SAMPLES ? LINK_MAX_SAMPLES : want;
}

/* How many stereo pairs `want` int16 samples comes to. */
static uint32_t want_pairs(uint32_t want)
{
    return cfg_mono ? want : want / 2u;
}

void slave_sound_init(void)
{
    memset(aram, 0, sizeof(aram));
    spc_dsp_init(aram);
    /* The master owns the IPL ROM window and its hidden RAM, so the
     * overlay is not modelled here — an echo buffer placed at $FFC0 is
     * vanishingly rare and would cost a round trip to track. */
    spc_dsp_set_rom_window(0, NULL, NULL);
    slave_sound_reset();
}

void slave_sound_reset(void)
{
    spc_dsp_reset();
    spc_dsp_set_output(pairs_buf, (int32_t)(LINK_MAX_SAMPLES * 2));
    clocks_done    = 0;
    stat_overflows = 0;
}

void slave_sound_config(const link_sound_config_t *cfg)
{
    cfg_mono              = cfg->mono ? true : false;
    cfg_samples_per_frame = cfg->samples_per_frame ? cfg->samples_per_frame
                                                   : LINK_SAMPLES_PER_CHUNK;
    if (cfg->playback_rate)
        cfg_rate = cfg->playback_rate;

    if (cfg->frame_span)
        frame_span = cfg->frame_span;
}

/* ------------------------------------------------------------------ */
/* The frame                                                          */
/* ------------------------------------------------------------------ */

/* Run the DSP forward to `when` in the master's frame timebase. */
static void sync_to(uint32_t when)
{
    if (when > frame_span) when = frame_span;

    uint32_t target = (uint32_t)(((uint64_t)when * frame_clocks) / frame_span);
    if (target <= clocks_done) return;

    spc_dsp_run((int32_t)(target - clocks_done));
    clocks_done = target;
}

uint32_t slave_sound_frame(const link_event_t *events, uint32_t n_events,
                           int16_t *out, uint32_t want,
                           link_frame_reply_t *reply)
{
    uint32_t t0 = time_us_32();

    if (want > LINK_MAX_SAMPLES) want = LINK_MAX_SAMPLES;
    uint32_t pairs = want_pairs(want);

    /* One frame's worth of DSP clocks for the audio being asked for.
     * Asking for several chunks stretches the same frame of events over
     * proportionally more output, which is what keeps the catch-up path
     * in the master's main loop fed. */
    frame_clocks = pairs * SPC_DSP_CLOCKS_PER_SAMPLE;
    clocks_done  = 0;
    spc_dsp_set_output(pairs_buf, (int32_t)(pairs * 2));

    /* Replay: run the DSP up to each write, then apply it. This is the
     * whole reason for the rewrite — the legacy mixer could only apply
     * every write and then render the frame, so a key-on a third of the
     * way down the screen was rendered as if it had happened at the
     * top. */
    for (uint32_t i = 0; i < n_events; i++) {
        const link_event_t *e = &events[i];

        switch (e->type) {
        case LINK_EV_DSP_WRITE:
            sync_to(e->cycles);
            spc_dsp_write((uint8_t)e->addr, e->val);
            break;
        case LINK_EV_RUN_UNTIL:
            sync_to(e->cycles);
            break;
        default:
            break;
        }
    }

#ifdef C2_DISABLE_ECHO
    /*
     * Force the echo unit off.
     *
     * On real hardware the echo delay line lives *inside* APU RAM: the
     * DSP writes it at ESA and reads it back for feedback. On C2 only
     * this chip's copy of APU RAM gets those writes — the master's copy
     * has never seen them — so the two mirrors necessarily diverge over
     * the echo region, and any dirty-page push that covers it drops a
     * stale delay line on top of the live one. A delay line replaying
     * old content sounds like samples repeating, which is the one
     * symptom that has outlived every other fix.
     *
     * FLG bit 5 stops echo writes; zeroing EVOLL/EVOLR stops the echo
     * being mixed into the output at all. Both are re-applied every
     * frame because the game keeps writing its own values.
     */
    {
        uint8_t *r = spc_dsp_regs();
        r[0x6C] |= 0x20;   /* FLG: echo write disable */
        r[0x2C]  = 0;      /* EVOLL */
        r[0x3C]  = 0;      /* EVOLR */
    }
#endif

    /* Finish the frame. */
    sync_to(frame_span);

    /* The DSP stops early only if it ran out of output room; pad so the
     * master always gets exactly what it armed for. */
    uint32_t got_pairs = (uint32_t)spc_dsp_samples_written() / 2u;
    if (got_pairs > pairs) got_pairs = pairs;

    if (cfg_mono) {
        for (uint32_t i = 0; i < got_pairs; i++)
            out[i] = (int16_t)((pairs_buf[i * 2] + pairs_buf[i * 2 + 1]) >> 1);
        if (got_pairs < want)
            memset(out + got_pairs, 0, (want - got_pairs) * sizeof(int16_t));
    } else {
        memcpy(out, pairs_buf, got_pairs * 2 * sizeof(int16_t));
        if (got_pairs * 2 < want)
            memset(out + got_pairs * 2, 0,
                   (want - got_pairs * 2) * sizeof(int16_t));
    }

    memset(reply, 0, sizeof(*reply));
    reply->samples = want;
    memcpy(reply->dsp_regs, spc_dsp_regs(), sizeof(reply->dsp_regs));
    reply->events_replayed = n_events;
    reply->overflows       = stat_overflows;
    reply->mix_us          = time_us_32() - t0;

    return want;
}
