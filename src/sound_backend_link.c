/*
 * frank-snes — C2 sound backend over the inter-processor link
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * sound_backend_link.c — the master's half of the C2 sound split.
 *
 * On M1 and M2, src/snes9x/soundux.c generates the audio: it decodes BRR
 * blocks out of APU RAM, runs eight voices with ADSR, applies echo and
 * the FIR, and mixes. On C2 that file is not built into the master at
 * all. This file stands in its place and does three things instead:
 *
 *   1. Captures every DSP register write the SPC700 makes, timestamped
 *      with CPU.Cycles, into a per-frame event list.
 *   2. Tracks which 256-byte pages of APU RAM the SPC700 has written, so
 *      sample data reaches the slave without shipping 64 KB a frame.
 *   3. Hands both to the slave once per frame and takes back the mixer's
 *      output plus the handful of DSP values the SPC700 can read.
 *
 * The SPC700 itself, APU RAM and the $2140..$2143 port handshake all
 * stay here, untouched and bit-identical to M1/M2. See
 * docs/C2_SOUND_SPLIT.md for why the split is at the DSP and not at the
 * SPC700.
 */

#ifdef C2_SOUND_LINK

#include <stdio.h>
#include <string.h>

#include "pico/time.h"

#include "snes9x.h"
#include "apu.h"
#ifdef SPC700_ACCURATE
#include "spc700_blargg.h"
#include "spc_dsp.h"
#else
#include "spc700.h"
#include "soundux.h"
#endif
#include "cpuexec.h"
#include "memmap.h"

#include "link_aram.h"
#include "link_master.h"
#include "link_proto.h"

#ifndef LOG
#define LOG(...) printf(__VA_ARGS__)
#endif

/* Where APU RAM lives, which depends on which SPC700 is built. */
#ifdef SPC700_ACCURATE
#define C2_APU_RAM  spc_apuram()
#else
#define C2_APU_RAM  IAPU.RAM
#endif

/* ------------------------------------------------------------------ */
/* State soundux.c would otherwise own                                */
/* ------------------------------------------------------------------ */

/* Frame counter for main.c's diagnostics. apu.c owns it when the
 * original SPC700 is built; only the accurate core needs it here. */
#ifdef SPC700_ACCURATE
volatile uint32_t dsp_log_frame;
volatile uint32_t dsp_write_count;
#endif

/* ------------------------------------------------------------------ */
/* Per-frame event list                                               */
/* ------------------------------------------------------------------ */

/* Frame-position timestamp, matching src/snes9x/apu_dsp.c.
 *
 * CPU.Cycles alone will not do: it is rewound by H_Max at every
 * scanline. V_Counter * H_Max + CPU.Cycles fixes that within a frame,
 * but the batch boundary is where s9x_link_frame() runs, and that does
 * not coincide with V_Counter wrapping — measured at exactly one wrap
 * per batch on hardware. The stragglers after the wrap come out with
 * near-zero timestamps, and since the slave replays in order and never
 * rewinds the DSP, every write after such an inversion collapses onto
 * one instant. That is heard as samples repeating and playing only
 * partially.
 *
 * Clamping to the last value issued is the fix: the handful of events
 * past the wrap belong to the next frame anyway, so landing them at the
 * end of this one is a sub-sample error, and time never runs backwards. */
static uint32_t ev_last_when;

/* The frame is as long as the region says it is. Assuming 262 lines on a
 * PAL game clamps every event from scanline 262 to 311 onto one timestamp
 * at the end of the frame — and that band contains most of PAL V-blank,
 * which is where a sound driver does its work. Key-ons then arrive bunched
 * at the frame edge instead of where they belong. */
static INLINE uint32_t link_frame_span(void)
{
    return (uint32_t)(Settings.PAL ? SNES_MAX_PAL_VCOUNTER
                                   : SNES_MAX_NTSC_VCOUNTER)
         * (uint32_t)Settings.H_Max;
}

static INLINE uint32_t link_dsp_now(void)
{
    uint32_t raw = (uint32_t)CPU.V_Counter * (uint32_t)Settings.H_Max
                 + (uint32_t)CPU.Cycles;
    uint32_t span = link_frame_span();

    if (raw > span)       raw = span;
    if (raw < ev_last_when) raw = ev_last_when;
    ev_last_when = raw;
    return raw;
}

static link_event_t ev_buf[LINK_MAX_EVENTS];
static uint32_t     ev_count;
static uint32_t     ev_overflows;

/* Diagnostic: inversions that survive the clamp above. Should stay 0. */
uint32_t ev_inversions;
uint32_t ev_max_backstep;

static inline void ev_push(uint8_t type, uint16_t addr, uint8_t val)
{
    if (ev_count >= LINK_MAX_EVENTS) {
        /* Truncate rather than drop the frame. Losing the tail of a
         * frame's writes degrades one frame of sound; losing frame
         * alignment desynchronises the stream from here on. */
        ev_overflows++;
        return;
    }
    link_event_t *e = &ev_buf[ev_count++];
    uint32_t when = link_dsp_now();
    if (ev_count > 1 && when < e[-1].cycles) {
        uint32_t back = e[-1].cycles - when;
        ev_inversions++;
        if (back > ev_max_backstep) ev_max_backstep = back;
    }
    e->cycles = when;
    e->type   = type;
    e->val    = val;
    e->addr   = addr;
}

/* ------------------------------------------------------------------ */
/* The master's shadow of the DSP register file                       */
/* ------------------------------------------------------------------ */

/*
 * Writes go two places: into the event list bound for the slave, and
 * into this shadow so a read that follows a write in the same frame sees
 * its own value. Everything the *DSP* changes — ENVX, OUTX, ENDX — is
 * refreshed wholesale from the slave's reply once a frame.
 *
 * That means DSP-authored bytes are one frame stale, the same bargain
 * frank-genesis makes with the YM2612 status byte. Master-authored bytes
 * never are, because the shadow is written directly here and the reply
 * is applied before this frame's writes rather than after.
 *
 * It is declared here rather than beside the read/write pair below
 * because the APU RAM tracking needs ESA and EDL out of it.
 */
static uint8_t dsp_shadow[128];

/* ------------------------------------------------------------------ */
/* APU RAM dirty tracking                                             */
/* ------------------------------------------------------------------ */

/* The tracking itself is in link_aram.c, which depends on nothing but
 * the wire protocol and so can be tested on a host. This is the
 * emulator-facing name spc700.c calls. */
static link_aram_run_t aram_runs[LINK_ARAM_MAX_RUNS];

void s9x_apu_ram_dirty(uint32_t address)
{
    link_aram_mark(address);
}

/* ESA and EDL come out of the shadow, which is the slave's own register
 * file as of last frame — so the echo region link_aram_collect() protects
 * is the one its DSP is actually writing. */
static uint32_t aram_collect_runs(void)
{
    return link_aram_collect(aram_runs, dsp_shadow[0x6D], dsp_shadow[0x7D]);
}

/* ------------------------------------------------------------------ */
/* DSP register access — the seam                                     */
/* ------------------------------------------------------------------ */

/*
 * The seam, now that the accurate SPC700 is in place.
 *
 * Every DSP register write the SPC700 makes funnels through
 * smp_dsp_write() in spc700_blargg.c, which calls this. Reads come back
 * out of the shadow the slave refreshes each frame. Nothing else on the
 * master touches the DSP — spc_dsp.c is not even clocked here.
 */
void c2_dsp_write(uint8_t addr, uint8_t data)
{
    addr &= 0x7f;

    ev_push(LINK_EV_DSP_WRITE, addr, data);
    dsp_shadow[addr] = data;

    /* ENDX is write-to-clear on the real DSP, whatever value is stored. */
    if (addr == 0x7C)
        dsp_shadow[0x7C] = 0;
}

uint8_t c2_dsp_read(uint8_t addr)
{
    return dsp_shadow[addr & 0x7f];
}

#ifndef SPC700_ACCURATE
/*
 * With the original SPC700 the funnel is apu.c's S9xSetAPUDSP() rather
 * than blargg's smp_dsp_write(), but it is the same seam: the register
 * number is latched in $00F2 and the data is the store to $00F3.
 *
 * This pairing — original SPC700, accurate DSP on the slave — is the C2
 * default. blargg's CPU is more correct but costs the master about a
 * third of its frame rate in a busy game, and a slow machine is a worse
 * outcome than an approximated port handshake.
 */
void S9xSetAPUDSP(uint8_t byte)
{
    c2_dsp_write(IAPU.RAM[0xf2], byte);
}

uint8_t S9xGetAPUDSP(void)
{
    return c2_dsp_read(IAPU.RAM[0xf2]);
}
#endif

/* ------------------------------------------------------------------ */
/* The frame exchange                                                 */
/* ------------------------------------------------------------------ */

/* Which mixer main.c will ask for. Fixed at build time by the same flag
 * that decides which S9xMixSamples* it calls, so the very first exchange
 * already requests the right number of samples — the drain path below
 * corrects it too, but only after a frame of half-length audio. */
#ifdef FRANK_SNES_FAST_MODE
static bool want_mono = true;
#else
static bool want_mono = false;
#endif

static uint32_t frames_silent;      /* frames the link could not serve  */

/*
 * Time-stretching.
 *
 * The slave always renders exactly one emulated frame of DSP time —
 * 534 samples — however slowly the emulator happens to be running.
 * Those samples are then stretched to cover the wall-clock interval the
 * frame actually took.
 *
 * The alternative, and what this did first, is to ask the slave for
 * more chunks when the emulator falls behind. That keeps the DAC fed
 * but runs the DSP ahead of the game: at the 42 fps Mortal Kombat 3
 * drops to in a fight, two chunks per frame advances every voice ~1.4x
 * faster than the game's own sequencing, so looping voices loop extra
 * times and one-shots end before the game expects them. That is heard
 * as samples repeating and being cut — and no amount of work on the APU
 * fixes it, because the APU is being told to run too fast. Replacing
 * the mixer, then the whole APU, changed nothing for exactly that
 * reason.
 *
 * Stretching keeps sound and game locked together. The cost is pitch:
 * at 42 fps everything plays about 30% low, like a tape slowing down.
 * That is the same bargain frank-genesis makes in audio_submit(), and
 * it is the honest one — the machine really is running slow, and the
 * audio says so rather than desynchronising.
 */
static uint32_t playback_rate = 32040u;
static uint32_t frame_requested;
static uint32_t underruns;

/* Two frames' worth, so the resampler can interpolate across a frame
 * boundary instead of restarting at it.
 *
 * Restarting is what the first version did, and it clicks: the cursor
 * almost never lands exactly on the last sample of a frame, so every
 * boundary either discarded the unconsumed tail or held the final
 * sample — a step discontinuity 42 to 60 times a second. The unconsumed
 * tail is now kept and the new frame appended behind it, so the cursor
 * runs continuously. */
static int16_t  frame_buf[LINK_SAMPLES_PER_CHUNK * 4];
static uint32_t frame_len;
static uint32_t frame_pos_q16;
static uint32_t frame_step_q16 = 1u << 16;
static uint32_t last_frame_us;

/* Smoothed frame interval, in microseconds.
 *
 * The instantaneous interval is far too noisy to stretch against: the
 * emulation loop paces itself against a deadline, so a heavy frame takes
 * 30 ms and the next one runs back-to-back in 8 ms while it catches up.
 * Using that directly makes the resample ratio swing 4:1 frame to frame,
 * which is audible as warbling pitch. What matters is the *average* rate
 * the emulator is sustaining, so this is a slow EMA. */
static uint32_t frame_interval_us = 16667u;

/* Where the link drops each frame before it is appended above. */
static int16_t staging_buf[LINK_SAMPLES_PER_CHUNK * 2];

/* How much unplayed audio to keep queued. One frame: deep enough that
 * the resampler never reaches the end of the data, shallow enough that
 * it costs only ~17 ms of latency. */
#define C2_FILL_TARGET  LINK_SAMPLES_PER_CHUNK

/* The last reply the slave sent, kept so the exchange cost and the
 * slave-side mix cost can be read back — over SWD during bring-up, or
 * by a profile build. Without this the one number that says whether the
 * offload is worth anything (mix_us against link_master's last_us) is
 * computed on the slave and then thrown away.
 *
 * volatile because nothing in the firmware reads it — a debugger does.
 * Without that the compiler is entirely right to delete the store. */
static volatile link_frame_reply_t last_reply;

/* Take the slave's register file as the new truth for everything the
 * DSP owns. Applied before this frame's writes are captured, so a write
 * the master makes afterwards still wins. */
static void apply_reply(const link_frame_reply_t *r)
{
    memcpy(dsp_shadow, r->dsp_regs, sizeof(dsp_shadow));
}

static void push_config(void);

/* Frames to wait before the next recovery attempt. */
static uint32_t link_retry_in;

/* How many times the link has come back from a failure. Nothing in the
 * firmware reads it; a debugger does, and it is the one number that says
 * whether the wire is glitching in the field. */
volatile uint32_t link_recoveries;

/*
 * Bring a dropped link back and put the slave's DSP back in step.
 *
 * Re-establishing the wire is only half of it. Every register write and
 * every dirty page sent while the link was down is gone, so replaying
 * from here would leave the DSP holding whichever patch it had when the
 * link dropped — voices keyed on samples that have since been replaced,
 * volumes from another scene. That is precisely the desync this is
 * supposed to prevent, so recovery pushes the whole of the state the
 * mixer reads: 64 KB of APU RAM, then all 128 DSP registers out of the
 * shadow. The shadow is authoritative here — the master wrote every one
 * of those bytes itself, and the DSP-authored ones it does not own
 * (ENVX, OUTX, ENDX) are recomputed on the first frame anyway.
 *
 * Attempts are spaced a second apart. A HELLO round trip is cheap, but
 * the slave takes up to a second to time out of a half-finished phase,
 * and hammering it inside that window only fails repeatedly.
 */
static void link_try_recover(void)
{
    if (link_retry_in) { link_retry_in--; return; }
    link_retry_in = 60;

    if (!link_master_reprobe())
        return;

    link_aram_force_resync();
    if (!link_master_reset_and_sync_aram(C2_APU_RAM))
        return;
    push_config();

    /* Stamped at zero and written straight into the list rather than
     * through ev_push(): these belong to the very start of the next
     * frame, whereas ev_push() would stamp them with wherever the 65816
     * happens to be now and every real write after them would then look
     * like time running backwards.
     *
     * KON is the one register that must not be replayed. It is a latch
     * the DSP consumes, not a level the game holds, so re-issuing the
     * last value keys those voices a second time — a duplicated sample,
     * which is the exact fault this whole path exists to avoid. Which
     * voices were sounding is not recoverable anyway; zero is the honest
     * answer and the driver keys its next note normally. */
    ev_count = 0;
    ev_last_when = 0;
    for (uint32_t i = 0; i < sizeof(dsp_shadow); i++) {
        link_event_t *e = &ev_buf[ev_count++];
        e->cycles = 0;
        e->type   = LINK_EV_DSP_WRITE;
        e->addr   = (uint16_t)i;
        e->val    = (i == 0x4C) ? 0 : dsp_shadow[i];
    }

    link_recoveries++;
}

/*
 * Called once per emulated frame from the emulation loop, before the
 * audio is consumed. Ships the frame's events and dirty APU RAM, takes
 * back the samples.
 *
 * This runs on core 0. frank-genesis puts the equivalent exchange on
 * core 1, but core 1 here is the HDMI scanline pump and cannot be
 * borrowed. The cost is a synchronous stall of roughly the wire time for
 * the payload — a few hundred microseconds for a typical frame — which
 * is why the dirty-page scheme matters so much more here than the ROM
 * upload did there.
 */
void s9x_link_frame(void)
{
    if (!link_master_online()) {
        /* The dirty set is deliberately left alone: recovery pushes the
         * whole image anyway, and consuming it here would eat the
         * pending resync flag with it. */
        ev_count = 0;
        frames_silent++;
        link_try_recover();
        return;
    }

    /* A time marker at the end of the frame tells the slave how far to
     * run its mixer, in the same emulated timebase the writes carry. */
    ev_push(LINK_EV_RUN_UNTIL, 0, 0);

    uint32_t n_runs = aram_collect_runs();

    /* Always one frame of DSP time. The stretch below is what adapts to
     * however long the frame took in the real world. */
    uint32_t want = want_mono ? LINK_SAMPLES_PER_CHUNK
                              : LINK_SAMPLES_PER_CHUNK * 2u;

    /* New batch starts here: let the clamp track from zero again. */
    ev_last_when = 0;

    link_frame_reply_t reply;
    bool ok = link_master_frame_exchange(ev_buf, ev_count,
                                         aram_runs, n_runs, 1u,
                                         C2_APU_RAM,
                                         staging_buf, want, &reply);
    ev_count = 0;

    if (!ok) {
        /* The exchange left the doorbells mid-phase; link_master has
         * already taken the link offline. Re-sync APU RAM in full when
         * it comes back, because the pages dirtied this frame were
         * consumed by the staging above and are gone. */
        link_aram_force_resync();
        frames_silent++;
        link_retry_in = 0;          /* the slave may already be idle */
        return;
    }

    apply_reply(&reply);
    last_reply = reply;

    /* Stretch this frame's audio over the wall-clock interval it took.
     * A frame that ran in 16.7 ms plays back 1:1; one that took 23.8 ms
     * (42 fps) is spread over 1.43x as many output samples. */
    uint32_t now = time_us_32();
    uint32_t elapsed = last_frame_us ? (now - last_frame_us) : 16667u;
    last_frame_us = now;

    /* Clamp before smoothing: a pause, a ROM load or a debugger halt
     * must not drag the average with it. */
    if (elapsed < 4000u)  elapsed = 4000u;
    if (elapsed > 50000u) elapsed = 50000u;

    /* EMA, 1/8 weight — a few frames of memory, enough to ride out the
     * loop's deadline pacing without lagging a real speed change. */
    frame_interval_us += ((int32_t)elapsed - (int32_t)frame_interval_us) / 8;

    uint32_t out_samples = (uint32_t)(((uint64_t)frame_interval_us
                                       * playback_rate) / 1000000ull);
    if (!want_mono) out_samples *= 2u;
    if (out_samples < 1u) out_samples = 1u;

    /* Keep whatever the cursor has not reached yet, and append the new
     * frame behind it. */
    uint32_t consumed = frame_pos_q16 >> 16;
    if (consumed > frame_len) consumed = frame_len;
    uint32_t carry = frame_len - consumed;

    /* Bound the carry: if the emulator is producing faster than main.c
     * drains, drop the oldest rather than growing latency without end. */
    const uint32_t carry_max = LINK_SAMPLES_PER_CHUNK * 2u;
    if (carry > carry_max) {
        memmove(frame_buf, frame_buf + (carry - carry_max),
                carry_max * sizeof(int16_t));
        carry = carry_max;
    } else if (consumed) {
        memmove(frame_buf, frame_buf + consumed, carry * sizeof(int16_t));
    }

    uint32_t added = reply.samples;
    if (carry + added > (sizeof frame_buf / sizeof frame_buf[0]))
        added = (sizeof frame_buf / sizeof frame_buf[0]) - carry;
    memcpy(frame_buf + carry, staging_buf, added * sizeof(int16_t));

    frame_len     = carry + added;
    frame_pos_q16 &= 0xFFFFu;     /* keep the sub-sample phase */

    /*
     * Set the resample rate by servoing on buffer fill, not by timing.
     *
     * Deriving the rate from the measured frame interval is open loop:
     * any error accumulates, the cursor eventually overruns the data,
     * and the last sample gets held — measured at 332 held samples a
     * second, which is a flat spot and a step several times per frame.
     * That is heard as clicking, and a held-then-jumped waveform reads
     * as a repeat.
     *
     * Holding a target backlog and nudging the rate towards it closes
     * the loop: if the buffer is draining the rate slows, if it is
     * filling the rate rises, and it cannot run dry as long as the
     * target is a frame deep. It also needs no clock at all — it adapts
     * to whatever speed the emulator is actually managing.
     */
    {
        uint32_t avail = frame_len - (frame_pos_q16 >> 16);
        int32_t  err   = (int32_t)avail - (int32_t)C2_FILL_TARGET;

        /* +/-5% of rate per frame of error, clamped so a transient
         * cannot produce an audible pitch jump. */
        int32_t adj = (err * (int32_t)(1u << 16)) / (int32_t)(C2_FILL_TARGET * 20);
        int32_t st  = (int32_t)(1u << 16) + adj;

        const int32_t st_min = (int32_t)((1u << 16) * 4 / 5);   /* 0.80 */
        const int32_t st_max = (int32_t)((1u << 16) * 5 / 4);   /* 1.25 */
        if (st < st_min) st = st_min;
        if (st > st_max) st = st_max;

        frame_step_q16 = (uint32_t)st;
    }
    (void)out_samples;
}

/* ------------------------------------------------------------------ */
/* What main.c calls                                                  */
/* ------------------------------------------------------------------ */

/* Hand out the next chunk of what the slave sent. Silence once the
 * frame's samples are used up — main.c's wall-clock catch-up asks for
 * extra chunks after a stall and there is nothing to give it, which is
 * the same outcome M1/M2 get when the mixer has no keyed voices. */
static void drain(int16_t *buffer, int32_t count)
{
    frame_requested += (uint32_t)count;

    for (int32_t i = 0; i < count; i++) {
        uint32_t idx = frame_pos_q16 >> 16;

        if (idx + 1 >= frame_len) {
            /* Past the end of this frame's audio. Hold the last sample
             * rather than emitting a zero: a step to silence and back is
             * a click, and the next frame is a fraction of a millisecond
             * away. */
            buffer[i] = frame_len ? frame_buf[frame_len - 1] : 0;
            underruns++;
            continue;
        }

        /* Linear interpolation between neighbouring samples. The stretch
         * ratio is close to 1 and the source is already band-limited by
         * the DSP's own gaussian, so this is enough. */
        int32_t a = frame_buf[idx];
        int32_t b = frame_buf[idx + 1];
        uint32_t f = frame_pos_q16 & 0xFFFFu;
        buffer[i] = (int16_t)(a + (((b - a) * (int32_t)f) >> 16));

        frame_pos_q16 += frame_step_q16;
    }
}

void S9xMixSamples(int16_t *buffer, int32_t sample_count)
{
    want_mono = false;
    drain(buffer, sample_count);
}

void S9xMixSamplesMono(int16_t *buffer, int32_t sample_count)
{
    want_mono = true;
    drain(buffer, sample_count);
}

void S9xMixSamplesLowPass(int16_t *buffer, int32_t sample_count,
                          int32_t low_pass_range)
{
    (void)low_pass_range;
    drain(buffer, sample_count);
}

/* ------------------------------------------------------------------ */
/* Configuration and reset                                            */
/* ------------------------------------------------------------------ */

static void push_config(void)
{
    link_sound_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    /* The accurate DSP models FLG/echo/interpolation itself from the
     * register writes, so the only thing the slave still needs told is
     * the shape of the audio the master wants back. */
    cfg.sound_enabled     = 1;
    cfg.mono              = want_mono ? 1 : 0;
    cfg.playback_rate     = playback_rate;
    cfg.samples_per_frame = LINK_SAMPLES_PER_CHUNK;
    /* Must match apu_dsp.c's APU_DSP_LINES_PER_FRAME * H_Max, and must
     * match what link_dsp_now() stamps events with. */
    cfg.frame_span        = link_frame_span();

    link_master_send_config(&cfg);
}

/*
 * main.c calls this once, before the emulator starts. On M1/M2 it
 * allocates soundux's 96 KB echo buffer; here that buffer lives on the
 * slave, so the only work is bringing up the wire.
 *
 * A slave that does not answer is not a failure: link_master_init()
 * reports it, everything downstream short-circuits on
 * link_master_online(), and the machine runs silent. Refusing to boot
 * because one of two chips is unprogrammed would be a far worse
 * outcome than a quiet SNES, and it is exactly the state a board is in
 * the first time it is flashed.
 */
bool S9xInitSound(int32_t buffer_ms, int32_t lag_ms)
{
    (void)buffer_ms; (void)lag_ms;

    if (!link_master_init())
        LOG("[link] no sound slave — audio disabled\n");

    S9xResetSound(true);
    return true;
}

void S9xSetPlaybackRate(uint32_t rate)
{
    playback_rate = rate ? rate : 32040u;
    /* freqbase is soundux's fixed-point step and is derived on the slave
     * from the same rate, so it is not mirrored here. */
    push_config();
}

/*
 * S9xResetSound() and a savestate load both leave APU RAM and the DSP in
 * a state the slave cannot reach by replaying events, so both force a
 * full re-sync rather than a dirty-page diff.
 */
void S9xResetSound(bool full)
{
    (void)full;
    ev_count  = 0;
    link_aram_force_resync();

    if (link_master_online()) {
        link_master_reset_and_sync_aram(C2_APU_RAM);
        push_config();
    }
}

void S9xFixSoundAfterSnapshotLoad(void)
{
    link_aram_force_resync();
    if (link_master_online()) {
        link_master_reset_and_sync_aram(C2_APU_RAM);
        push_config();
    }
}

/* ------------------------------------------------------------------ */
/* soundux.c entry points the rest of the emulator still references    */
/* ------------------------------------------------------------------ */

/*
 * apu.c calls S9xSetEchoEnable() from S9xResetAPU() and the SFX helpers
 * are reachable from the input path. On C2 every one of these is a
 * consequence of a DSP register write that has already been forwarded,
 * so the slave will do the equivalent work when it replays the event.
 * They stay as no-ops rather than being deleted so that apu.c, ppu.c and
 * snapshot.c compile unchanged on all three boards.
 */
#ifndef SPC700_ACCURATE
/* soundux.h declares this; apu.h supplies an inline in the accurate
 * build. The legacy mixer's SFX auto-release is disabled either way, so
 * there is nothing to do. */
void S9xNotifyButtonPress(void)                         { }
#endif

void S9xSetEchoEnable(uint8_t byte)                     { (void)byte; }
void S9xSetEchoWriteEnable(uint8_t byte)                { (void)byte; }
void S9xSFXAutoReleaseTick(void)                        { }
void S9xSFXLoopRelease(int channel)                     { (void)channel; }
void S9xSFXCheckKON(int channel)                        { (void)channel; }
void S9xDSPQueueEvent(uint8_t type, uint8_t data, int32_t cycle)
{
    (void)type; (void)data; (void)cycle;
}
void S9xDSPSetFrameStart(int32_t cycle)                 { (void)cycle; }

/* ------------------------------------------------------------------ */
/* Diagnostics                                                        */
/* ------------------------------------------------------------------ */

void s9x_link_sound_stats(uint32_t *events, uint32_t *overflows,
                          uint32_t *silent)
{
    if (events)    *events    = ev_count;
    if (overflows) *overflows = ev_overflows;
    if (silent)    *silent    = frames_silent;
}

#endif /* C2_SOUND_LINK */
