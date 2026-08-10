/*
 * apu_dsp.c — audio output adapter for the accurate APU.
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * With SOUND_CORE=DSP the SPC700 (spc700_blargg.c) clocks the DSP
 * (spc_dsp.c) itself, so nothing here decides *when* audio is produced —
 * it only owns the buffer the DSP writes into and hands it to main.c in
 * the shape main.c expects.
 *
 * That is the whole difference from the legacy path: soundux.c had to be
 * told "render a frame now", and could only apply a frame's register
 * writes beforehand. Here the DSP has already been clocked, write by
 * write, in step with the SPC700.
 *
 * M1 and M2 do not build this file; they keep soundux.c.
 */

#include <string.h>

#include "snes9x.h"
#include "spc_dsp.h"
#ifdef SPC700_ACCURATE
#include "spc700_blargg.h"
/* blargg's core owns APU RAM. */
#define APU_DSP_RAM  spc_apuram()
#else
#include "apu.h"
#include "spc700.h"
#include "cpuexec.h"
#include "soundux.h"
/* The original core keeps it in IAPU. */
#define APU_DSP_RAM  IAPU.RAM
#endif

/* Frame counter used by main.c's diagnostics and by the DSP log on the
 * legacy path, where apu.c owns it. */
#ifdef SPC700_ACCURATE
volatile uint32_t dsp_log_frame;
volatile uint32_t dsp_write_count;
#endif

/* Stereo pairs. Sized for several frames so a late drain never loses
 * audio — main.c's wall-clock catch-up can ask for up to seven chunks
 * after a stall. */
#define APU_OUT_PAIRS 4096

static int16_t  out_buf [APU_OUT_PAIRS * 2];
static uint32_t out_head;      /* next pair to hand out */
static bool     ready;

#ifndef SPC700_ACCURATE
/* ------------------------------------------------------------------ */
/* Driving the DSP from the original SPC700                            */
/* ------------------------------------------------------------------ */

/*
 * blargg's SPC700 clocks the DSP itself; the original one does not, so
 * this side has to. The DSP is run forward to the point in the frame a
 * register write happened, then the write is applied — which is what
 * gives sub-frame accuracy the legacy mixer never had.
 *
 * The timebase is V_Counter * H_Max + CPU.Cycles, clamped so it cannot
 * run backwards: it is monotonic within a frame, but the frame boundary
 * here is the drain, and that does not coincide with V_Counter
 * wrapping.
 */
/* Scanlines per frame is a region property, and so is how much audio a
 * frame contains. Hardcoding the NTSC pair clamped every DSP write in
 * scanlines 262..311 of a PAL frame onto one instant — and that band is
 * most of PAL V-blank, which is exactly where a sound driver does its
 * work. The link path already derives both; this one did not. */
static uint32_t dsp_span;
static uint32_t dsp_clocks_per_frame;
static uint32_t dsp_clocks_done;
/* Cycles since the last drain, and the raw V_Counter position it was last
 * derived from. See apu_dsp_now(). */
static uint32_t dsp_rel;
static uint32_t dsp_prev_raw;
static uint32_t dsp_rate = 32040u;
static bool     dsp_geom_pal;
static bool     dsp_geom_valid;

/* The ROM's region is not known when S9xSetPlaybackRate() first runs —
 * snes9x_init() precedes LoadROM() — so the geometry is recomputed
 * whenever Settings.PAL disagrees with what it was built from. */
static void apu_dsp_geometry(void)
{
   if (dsp_geom_valid && dsp_geom_pal == (Settings.PAL != 0))
      return;
   dsp_geom_pal   = (Settings.PAL != 0);
   dsp_geom_valid = true;
   dsp_span = (uint32_t)(dsp_geom_pal ? SNES_MAX_PAL_VCOUNTER
                                      : SNES_MAX_NTSC_VCOUNTER)
            * (uint32_t)Settings.H_Max;
   dsp_clocks_per_frame = (dsp_rate / (dsp_geom_pal ? 50u : 60u))
                        * SPC_DSP_CLOCKS_PER_SAMPLE;
}

static void apu_dsp_finish_frame(void);

/*
 * Position within the current batch, in master cycles.
 *
 * V_Counter * H_Max + CPU.Cycles is only monotonic *within* a video frame,
 * and the batch boundary is the drain in main.c, which does not coincide
 * with V_Counter wrapping — measured at exactly one wrap per batch. The old
 * code clamped: writes after the wrap came out with near-zero positions,
 * were dragged forward to the previous value, and every one of them
 * collapsed onto a single instant. The wrap lands in V-blank, which is
 * where a sound driver does its work, so that is most of a frame's key-ons
 * landing together — heard as samples repeating and playing only partially.
 *
 * Carrying the wrap instead of clamping keeps the timebase monotonic across
 * it, and keeps the accumulator bounded by resetting it every drain.
 */
static INLINE uint32_t apu_dsp_now(void)
{
   apu_dsp_geometry();

   int32_t cyc = (int32_t)CPU.Cycles;
   uint32_t raw = (uint32_t)CPU.V_Counter * (uint32_t)Settings.H_Max
                + (uint32_t)(cyc > 0 ? cyc : 0);

   uint32_t delta = (raw >= dsp_prev_raw) ? raw - dsp_prev_raw
                                          : raw + dsp_span - dsp_prev_raw;
   if (delta > dsp_span) delta = dsp_span;   /* never trust a wild jump */
   dsp_prev_raw = raw;
   dsp_rel += delta;
   return dsp_rel;
}

static void apu_dsp_sync(uint32_t when)
{
   if (!ready || !dsp_span) return;

   uint32_t target = (uint32_t)(((uint64_t)when * dsp_clocks_per_frame)
                                / dsp_span);
   if (target <= dsp_clocks_done) return;

   spc_dsp_run((int32_t)(target - dsp_clocks_done));
   dsp_clocks_done = target;
}
#endif


/* Re-arm the DSP's output window at the head of the free space. */
static void rearm(void)
{
#ifdef SPC700_ACCURATE
   int32_t written = S9xAPUSamplesWritten();      /* int16s, not pairs */
#else
   int32_t written = spc_dsp_samples_written();
#endif
   if (written < 0) written = 0;

   uint32_t used = out_head * 2u + (uint32_t)written;
   if (used >= APU_OUT_PAIRS * 2u)
   {
      /* Ran the buffer dry-side full: restart. Losing a fragment is
       * better than writing past the end. */
      out_head = 0;
#ifdef SPC700_ACCURATE
      S9xAPUResetOutputCount();
      S9xAPUSetOutput(out_buf, APU_OUT_PAIRS * 2);
#else
      /* Same job for the original SPC700's DSP. Rewinding out_head
       * without rewinding the DSP's write pointer leaves it writing on
       * past the end of out_buf. */
      spc_dsp_set_output(out_buf, APU_OUT_PAIRS * 2);
#endif
   }
}

bool S9xInitSound(int32_t buffer_ms, int32_t lag_ms)
{
   (void)buffer_ms; (void)lag_ms;

   out_head = 0;
#ifdef SPC700_ACCURATE
   S9xAPUResetOutputCount();
   S9xAPUSetOutput(out_buf, APU_OUT_PAIRS * 2);
#else
   /*
    * Nothing else in this configuration ever hands the DSP its RAM
    * pointer or its output window: spc_dsp_init() is called only by the
    * accurate SPC700, which is not compiled here, and by the slave.
    * Without it dsp_m.ram stays NULL and the first BRR fetch reads
    * NULL + the sample offset — a precise bus error at 0x8000 a couple
    * of seconds after a ROM loads, which is why this build never ran.
    */
   spc_dsp_init(APU_DSP_RAM);
   spc_dsp_reset();
   spc_dsp_set_output(out_buf, APU_OUT_PAIRS * 2);
   dsp_clocks_done = 0;
   dsp_rel         = 0;
   dsp_prev_raw    = 0;
   dsp_geom_valid  = false;
#endif
   ready = true;
   return true;
}

void S9xResetSound(bool full)
{
   (void)full;
   if (!ready) return;
   out_head = 0;
#ifdef SPC700_ACCURATE
   S9xAPUResetOutputCount();
   S9xAPUSetOutput(out_buf, APU_OUT_PAIRS * 2);
#else
   spc_dsp_set_output(out_buf, APU_OUT_PAIRS * 2);
   dsp_clocks_done = 0;
   dsp_rel         = 0;
   dsp_prev_raw    = 0;
#endif
}

void S9xFixSoundAfterSnapshotLoad(void)
{
   /* The DSP's saved state carries a stale `ram` pointer and a stale
    * output window; both are re-pointed here. */
   spc_dsp_init_ram(APU_DSP_RAM);
   S9xResetSound(true);
}

void S9xSetPlaybackRate(uint32_t rate)
{
   /* The DSP's output rate is fixed by the hardware it emulates
    * (32 kHz); this port runs its DAC at 32040 and does not resample,
    * which is within a tenth of a percent. */
   if (!rate) rate = 32040u;
#ifndef SPC700_ACCURATE
   dsp_rate        = rate;
   dsp_geom_valid  = false;      /* region may not be known yet */
   dsp_clocks_done = 0;
   dsp_rel         = 0;
   dsp_prev_raw    = 0;
#endif
}

/* ------------------------------------------------------------------ */
/* What main.c calls                                                  */
/* ------------------------------------------------------------------ */

static void drain(int16_t *buffer, int32_t pairs, bool mono)
{
#ifndef SPC700_ACCURATE
   apu_dsp_finish_frame();
#endif
#ifdef SPC700_ACCURATE
   int32_t written = S9xAPUSamplesWritten();
#else
   int32_t written = spc_dsp_samples_written();
#endif
   if (written < 0) written = 0;

   uint32_t have = (uint32_t)written / 2u;
   have = have > out_head ? have - out_head : 0u;

   uint32_t n = (uint32_t)pairs < have ? (uint32_t)pairs : have;

   for (uint32_t i = 0; i < n; i++)
   {
      const int16_t *p = &out_buf[(out_head + i) * 2];
      if (mono)
         buffer[i] = (int16_t)((p[0] + p[1]) >> 1);
      else
      {
         buffer[i * 2    ] = p[0];
         buffer[i * 2 + 1] = p[1];
      }
   }

   if (n < (uint32_t)pairs)
   {
      if (mono) memset(buffer + n, 0, (pairs - n) * sizeof(int16_t));
      else      memset(buffer + n * 2, 0, (pairs - n) * 2 * sizeof(int16_t));
   }

   out_head += n;
   rearm();
}

void S9xMixSamples(int16_t *buffer, int32_t sample_count)
{
   drain(buffer, sample_count / 2, false);
}

void S9xMixSamplesMono(int16_t *buffer, int32_t sample_count)
{
   drain(buffer, sample_count, true);
}

void S9xMixSamplesLowPass(int16_t *buffer, int32_t sample_count,
                          int32_t low_pass_range)
{
   (void)low_pass_range;
   drain(buffer, sample_count / 2, false);
}

#ifndef SPC700_ACCURATE
void S9xSetAPUDSP(uint8_t byte)
{
   apu_dsp_sync(apu_dsp_now());
   spc_dsp_write(IAPU.RAM[0xf2], byte);
}

uint8_t S9xGetAPUDSP(void)
{
   apu_dsp_sync(apu_dsp_now());
   return spc_dsp_read(IAPU.RAM[0xf2]);
}

static void apu_dsp_finish_frame(void)
{
   /* Run to where the CPU actually is, not to a nominal frame end: a batch
    * can be a little longer or shorter than one frame. */
   apu_dsp_sync(apu_dsp_now());
   dsp_clocks_done = 0;
   dsp_rel         = 0;
}
#endif /* !SPC700_ACCURATE */

/* ------------------------------------------------------------------ */
/* soundux.c entry points the rest of the emulator still references    */
/* ------------------------------------------------------------------ */

/*
 * With a real DSP in place these are all consequences of a register
 * write that has already reached it, so there is nothing left to do.
 * They stay so apu.c, ppu.c and snapshot.c compile unchanged against
 * either sound core.
 */
void S9xSetEchoEnable(uint8_t byte)                  { (void)byte; }
void S9xSetEchoWriteEnable(uint8_t byte)             { (void)byte; }
void S9xSFXAutoReleaseTick(void)                     { }
void S9xSFXLoopRelease(int channel)                  { (void)channel; }
void S9xSFXCheckKON(int channel)                     { (void)channel; }
void S9xDSPQueueEvent(uint8_t type, uint8_t data, int32_t cycle)
{
   (void)type; (void)data; (void)cycle;
}
void S9xDSPSetFrameStart(int32_t cycle)              { (void)cycle; }

#ifndef SPC700_ACCURATE
/* apu.h supplies an inline of this for the accurate build; soundux.h
 * only declares it for the original SPC700. */
void S9xNotifyButtonPress(void)                      { }

/* soundux.h declares the legacy mixer's sub-frame KON queue. Nothing
 * uses it — the DSP's timing comes from when writes are applied. */
DSPEvent dsp_events[DSP_EVENT_MAX];
uint8_t  dsp_event_count;
int32_t  dsp_frame_start_cycle;
#endif
