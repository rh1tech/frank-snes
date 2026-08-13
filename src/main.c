/*
 * FRANK SNES - SNES Emulator for RP2350
 * Based on Snes9x and pico-snes
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "hardware/structs/qmi.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>


/* main.c is compiled as C++ when CPU_CORE=S9X16, because the 1.6x core's
 * headers are C++. Everything else this file talks to - the drivers, the
 * UI, fatfs - is C, and those headers have no extern "C" guards of their
 * own. Rather than churn a dozen shared headers, the C includes are
 * wrapped here. */
#ifdef __cplusplus
#define FRANK_C_BEGIN extern "C" {
#define FRANK_C_END   }
#else
#define FRANK_C_BEGIN
#define FRANK_C_END
#endif

FRANK_C_BEGIN
#include "board_config.h"
#include "HDMI.h"
// USB CDC serial used for debug output (dev builds only)
#include "psram_init.h"
#include "psram_allocator.h"
#include "ff.h"
FRANK_C_END

// Snes9x includes
#ifdef FRANK_SNES_CPU_CORE_S9X16
/* The 1.6x core. These headers are C++ — this file is compiled as C++ when
 * CPU_CORE=S9X16 for exactly that reason. soundux.h and srtc.h have no
 * counterpart: the block mixer and the S-RTC are 1.43-only. */
#include "snes9x.h"
#include "memmap.h"
#include "apu/apu.h"
#include "display.h"
#include "gfx.h"
#include "cpuexec.h"
#include "ppu.h"
#include "s9x16_api.h"
#else
#include "snes9x/snes9x.h"
#include "snes9x/soundux.h"
#include "snes9x/memmap.h"
#include "snes9x/apu.h"
#include "snes9x/display.h"
#include "snes9x/gfx.h"
#include "snes9x/cpuexec.h"
#include "snes9x/srtc.h"
#endif

/*
 * APU RAM and the DSP register file, wherever the built sound core keeps
 * them. The key-on and state-fingerprint diagnostics below are the same
 * whichever SPC700 is compiled in — only the accessors differ — and
 * naming IAPU/APU directly is what stopped SOUND_CORE=ACCURATE building
 * at all, which is the one configuration worth comparing against the
 * known-good 1.6x engine.
 */
#ifdef SPC700_ACCURATE
#include "snes9x/spc700_blargg.h"
#include "snes9x/spc_dsp.h"
#define DIAG_APU_RAM   spc_apuram()
#define DIAG_DSP_REGS  spc_dsp_regs()
#elif defined(FRANK_SNES_CPU_CORE_S9X16)
/* bapu keeps APU RAM and the DSP register file inside SNES::smp / SNES::dsp
 * rather than in the flat IAPU/APU globals the 1.43 core exposed. The two
 * diagnostics that used these are 1.43-only; NULL makes any attempt to use
 * them fail loudly instead of fingerprinting whatever happens to be at
 * address zero. */
#define DIAG_APU_RAM   ((const uint8_t *)NULL)
#define DIAG_DSP_REGS  ((const uint8_t *)NULL)
#else
#define DIAG_APU_RAM   IAPU.RAM
#define DIAG_DSP_REGS  APU.DSP
#endif

FRANK_C_BEGIN

// APU on Core 1
#ifndef FRANK_SNES_CPU_CORE_S9X16
#include "snes9x/apu_core1.h"
#endif
#ifdef C2_SOUND_LINK
/* C2 only: hand the frame's sound work to the slave. Defined in
 * src/sound_backend_link.c. */
void s9x_link_frame(void);
#endif

// Audio driver (exact copy from pico-snes-master)
#include "audio.h"

// Audio optimizations
#include "audio_opt.h"

#include "snes9x/ppu_capture.h"
#include "link_master.h"

#ifdef FRANK_SNES_HDMI_ALT
// HDMI_ALT (libdvi-backed) entry points and audio ring write.
#include "hdmi_alt.h"
extern void hdmi_alt_run_core1(void);
#endif

// Input drivers
#include "nespad/nespad.h"
#include "ps2kbd/ps2kbd_wrapper.h"
#include "ps2/ps2.h"
#ifdef USB_HID_ENABLED
#include "usbhid/usbhid.h"
#include "usbhid/gamepad_cal.h"
#endif

// ROM selector and settings
#include "rom_selector.h"
#include "settings.h"
#include "menu_ui.h"

#ifdef FRANK_SNES_PROFILE
#include "frank_snes_profile.h"
#endif

/* Defined in the C drivers. Declared here rather than in a function body
   because an extern "C" block cannot be opened at block scope. */
extern volatile bool g_palette_needs_update;
extern volatile uint32_t dsp_log_frame;
#ifdef FRANK_SNES_CPU_CORE_S9X16
void S9xPushPaletteToDisplay(void);
#endif

FRANK_C_END

//=============================================================================
// Configuration
//=============================================================================

#define SCREEN_WIDTH     SNES_WIDTH    // 256
#define SCREEN_HEIGHT    SNES_HEIGHT   // 224

// Audio sample rate.  Use 32040 on both paths so AUDIO_BUFFER_LENGTH
// divides exactly by 60 (534 samples/frame) and the producer doesn't
// drift relative to the wall clock.  HDMI declares 32 kHz on the wire
// (the closest CEA-861 standard rate); the resulting 0.125% pitch
// shift is well below audible.
#define AUDIO_SAMPLE_RATE   (32040)
#define AUDIO_BUFFER_LENGTH (AUDIO_SAMPLE_RATE / 60)

/*
 * How much audio ONE EMULATED FRAME is worth.
 *
 * AUDIO_BUFFER_LENGTH is a DAC chunk. The DAC's rate is fixed, so that
 * number is fixed too. How much audio an emulated frame contains is not:
 * a frame is 1/60 s on an NTSC machine and 1/50 s on a PAL one, so 534 or
 * 641 samples. These are different quantities and only coincide on NTSC.
 *
 * Passing the NTSC figure to the mixer on a PAL ROM asks for five sixths
 * of the frame and abandons the rest. It hid for so long because
 * 60 x 534 is *exactly* the DAC rate: the pipeline stays perfectly
 * balanced while doing it, so no underrun, discard or starvation counter
 * ever moves. The loss is upstream of all of them. That silent 16.7% is
 * what "some sounds are skipped" was, on every board, for every PAL ROM.
 */
#define AUDIO_FRAME_SAMPLES_NTSC (AUDIO_SAMPLE_RATE / 60)
#define AUDIO_FRAME_SAMPLES_PAL  (AUDIO_SAMPLE_RATE / 50)

/*
 * ...and the frame is not always a nominal frame long.
 *
 * memmap.c carries snes9x's per-game CPU timing hacks, which stretch
 * Settings.H_Max: 103% for Power Rangers, 130% for Alien vs Predator, 200%
 * for Home Improvement — and 110% for Mortal Kombat 3, whose comment reads
 * "Fixes cut off speech sample". A stretched scanline means more emulated
 * cycles per frame, so the APU generates proportionally more audio: MK3's
 * frame holds ~705 samples, not 641.
 *
 * Asking the mixer for the nominal count on such a game throws the
 * difference away every frame — 9% of MK3's audio — which is exactly the
 * cut-off speech the hack was written to prevent.
 *
 * Clamped because soundux mixes into SOUND_BUFFER_SIZE (2133 stereo
 * entries), so no more than 1066 sample frames can be asked for at once.
 */
#ifdef FRANK_SNES_CPU_CORE_S9X16
/* No H_Max stretching on this core, so a frame is never longer than the
   nominal PAL count. The 1066 below exists only for the 1.43 core's
   per-game scanline stretch. */
#define AUDIO_FRAME_SAMPLES_MAX  704u
#else
#define AUDIO_FRAME_SAMPLES_MAX  1066u
#endif
#define AUDIO_FRAME_SAMPLES_BASE (Settings.PAL ? AUDIO_FRAME_SAMPLES_PAL \
                                               : AUDIO_FRAME_SAMPLES_NTSC)
static inline uint32_t audio_frame_samples(void)
{
#ifdef FRANK_SNES_CPU_CORE_S9X16
    /* No H_Max to scale by: the 1.6x core charges cycles inside the memory
     * accessors, so a scanline is a scanline and there is no per-game
     * stretch factor to compensate for. The nominal count is the count. */
    return AUDIO_FRAME_SAMPLES_BASE;
#else
    uint32_t n = (uint32_t)(((uint64_t)AUDIO_FRAME_SAMPLES_BASE *
                             (uint32_t)Settings.H_Max +
                             (SNES_CYCLES_PER_SCANLINE / 2u)) /
                            SNES_CYCLES_PER_SCANLINE);
    if (n > AUDIO_FRAME_SAMPLES_MAX) n = AUDIO_FRAME_SAMPLES_MAX;
    return n;
#endif
}
#define AUDIO_FRAME_SAMPLES      audio_frame_samples()

//=============================================================================
// Screen Buffers
//=============================================================================

// Screen buffers - 256x224 8-bit palette-indexed (HDMI driver maps index to color)
uint8_t __attribute__((aligned(4))) SCREEN[2][SNES_WIDTH * SNES_HEIGHT];

#ifdef FRANK_SNES_CPU_CORE_S9X16
/* The 1.6x core allocates its own depth buffers in S9xGraphicsInit. */
#else
static uint8_t __attribute__((aligned(4))) ZBuffer[SNES_WIDTH * SNES_HEIGHT];
static uint8_t __attribute__((aligned(4))) SubZBuffer[SNES_WIDTH * SNES_HEIGHT];
#endif

// Separate sub-screen buffer for transparency.  Lives in the PSRAM
// scratch region (first 512 KB, reserved by psram_allocator and not
// touched by psram_malloc / psram_reset), so it survives across
// ROM launches and doesn't compete with ROM / RAM / VRAM allocations.
//
// Before this, the pointer was psram_malloc()'d inside a session on
// cold boot.  On a 6 MB SuperFX ROM (DOOM) that allocation plus the
// emulator's state used the last of permanent PSRAM, and S9xInitGFX's
// later 22 KB calloc() for LocalState silently returned NULL — which
// left GFX.OBJLines dereferencing NULL and the emulator rendering one
// stray scanline on the first launch.  Going back to the selector and
// re-launching leaked the old SubScreenBuffer pointer, which happened
// to save exactly enough PSRAM for LocalState to succeed on the
// second launch.
#define SUB_SCREEN_OFFSET (256 * 1024)  /* into 512 KB scratch region */
static uint8_t *SubScreenBuffer = (uint8_t *)(0x11000000 + SUB_SCREEN_OFFSET);

#ifdef FRANK_SNES_CPU_CORE_S9X16
/* The 1.6x tile renderer writes 16-bit pixels and cannot be talked out of
 * it without rewriting tile.c, so it renders into its own buffer and the
 * frame is narrowed to the 8-bit indices the HDMI driver scans out. With
 * FRANK_SNES_INDEXED_SCREEN those 16-bit values are already CGRAM indices,
 * so "narrowing" is a truncation and not a colour lookup.
 *
 * These live in PSRAM: 224 KB of framebuffer does not fit in SRAM beside
 * the core. That puts the renderer's pixel writes on the PSRAM bus, which
 * is the one cost of this arrangement worth measuring.
 */
#define SCREEN16_PIXELS   (SNES_WIDTH * SNES_HEIGHT)

/* The core allocates its own framebuffers in S9xGraphicsInit (in PSRAM,
 * at native size), so there is nothing for the front end to place. This
 * used to allocate them here at fixed offsets in the 512 KB scratch
 * region - which is already carved into a decompression buffer, a
 * conversion buffer and a 256 KB file-load buffer, and writing over all
 * three is what locked the first build up. */

/* 256x224 truncations per frame. Kept in one place so the cost is visible
 * and so the loop can be replaced wholesale if it shows up in a profile. */
static void screen16_to_indexed(uint8_t *dst)
{
    const uint16_t *src = (const uint16_t *)GFX.Screen;
    if (!src)
        return;
    for (uint32_t i = 0; i < SCREEN16_PIXELS; i++)
        dst[i] = (uint8_t)src[i];
}
#endif

// Current display buffer (double buffering) - accessed by HDMI driver
volatile uint32_t current_buffer = 0;

#ifdef FRANK_SNES_PPU_CAPTURE
/* PSRAM landing area for the slave's picture - see the staging call. */
static uint8_t *g_ppu_stage;
#endif

//=============================================================================
// Audio - Core 0 mixes into buffer, Core 1 plays
//=============================================================================

// Audio handoff: Core 0 mixes int16 stereo, then applies gain/limiting and packs
// into 32-bit stereo frames for I2S (L in high 16, R in low 16). Core 1 then
// streams these packed frames to pico_audio_i2s.
//
// Key goal: keep Core 1 work minimal so HDMI activity doesn't starve audio.
// 16 frames (~267ms) - absorbs CPU spikes during scene transitions
//
// On HDMI_ALT, Core 0 forwards each packed chunk directly into the
// dvi0.audio_ring after pack — there is no I2S consumer, so the
// AUDIO_QUEUE_DEPTH SRAM ring is dead weight.  Shrink it to 2 to
// recover ~12 KB SRAM that the libdvi TMDS buffers can use.
#ifdef FRANK_SNES_HDMI_ALT
/* The HDMI ring is the real consumer queue; this SRAM ring just holds
 * the in-flight chunk Core 0 just packed.  Depth 1 is sufficient. */
#define AUDIO_QUEUE_DEPTH 1
#else
#define AUDIO_QUEUE_DEPTH 8
#endif
// NOTE: With fixed 60Hz emulation producing exactly one audio chunk per frame,
// the producer cannot stay "ahead" of the consumer by >1 chunk in steady state.
// Using queue-fill watermarks to decide frame skipping will therefore
// permanently starve video (e.g. render ~12fps with MAX_FRAME_SKIP=4).
// We keep this watermark only for choosing the cheaper limiter path.
#define AUDIO_LOW_WATERMARK 4
static uint32_t __attribute__((aligned(32))) audio_packed_buffer[AUDIO_QUEUE_DEPTH][AUDIO_BUFFER_LENGTH];
static uint32_t __attribute__((aligned(32))) audio_packed_discard[AUDIO_BUFFER_LENGTH];

/* Audio pacing diagnostics. A discard is a whole 534-sample chunk the
 * emulator produced and threw away because the I2S ring was full — the
 * emulated frame rate running ahead of the DAC. An underrun is the
 * opposite. Either one is an audible seam: a discard skips ~8.9 ms of
 * the song, an underrun fades it to silence and back. Both are common
 * to every sound core and every board, which is why they survived
 * replacing the mixer. */
static uint64_t g_ship_emul_sum = 0;
static uint32_t g_ship_emul_n = 0;
static uint32_t g_render_cost_us = 4000;   /* estimated cost of rendering  */
static uint32_t g_emu_only_us   = 8000;   /* cost of a skipped-render frame */
volatile uint32_t audio_discards;
/* Delivery-path health that no existing counter covered. */
/*
 * Fingerprint of what actually reached the DAC.
 *
 * The emulator is deterministic: identical ROM and identical input must
 * give identical audio, every run. The reported fault is NOT deterministic
 * — same fight, different samples cut each time — so something between the
 * emulator and the DAC varies. Hashing the delivered frames and snapshotting
 * that hash once a second lets two runs be diffed, and the first differing
 * second says when the divergence starts.
 */
#ifdef FRANK_SNES_AUDIO_FINGERPRINT
volatile uint32_t audio_hash = 2166136261u;
#define AUDIO_HASH_LOG 160
volatile uint32_t audio_hash_log[AUDIO_HASH_LOG];
volatile uint32_t audio_hash_n;
static uint32_t   audio_hash_chunks;

/* Same fingerprint taken at the DAC, so one run reports determinism at both
 * ends: the producer must be bit-identical run to run, the DAC never can be
 * (it adapts to the real clock) — what matters is how far it drifts. */
volatile uint32_t dac_hash = 2166136261u;
volatile uint32_t dac_hash_log[AUDIO_HASH_LOG];
volatile uint32_t dac_hash_n;
static uint32_t   dac_hash_chunks;
#endif /* FRANK_SNES_AUDIO_FINGERPRINT */

volatile uint32_t ratio_floor_hits;   /* servo pinned at maximum stretch */
volatile uint32_t ratio_ceil_hits;    /* servo pinned at maximum squeeze */
volatile uint32_t sfifo_short;        /* chunk skipped: ring below need   */
volatile int32_t  ratio_min = 0x7fffffff;   /* servo excursion, q16 */
volatile int32_t  ratio_max;
/* Headroom telemetry. The mixer sums eight voices, so a dense scene is a
 * loud one; if the post-gain peak is riding into the soft limiter then
 * dense passages are being compressed and quiet voices buried under the
 * loud ones — which would present as "samples missing when many play at
 * once" without any key-on ever being lost. */
volatile int32_t  mix_peak_max;        /* largest |mix16| seen, pre-gain */
volatile uint32_t mix_limit_frames;    /* frames whose peak enters the limiter */
volatile uint32_t mix_clip_frames;     /* frames whose peak would hard-clip */
volatile uint32_t mix_frames;
volatile int32_t  mix_gain_num, mix_gain_den;

volatile int32_t  pace_adj_min = 0x7fffffff;  /* frame-period trim, us */
volatile int32_t  pace_adj_max = -0x7fffffff;
volatile uint32_t audio_underruns;

/* --- Audio pacing -------------------------------------------------------
 *
 * Audio is produced at the rate emulation advances and consumed at a fixed
 * 32040 Hz. When emulation runs below 60 fps the two diverge, and the old
 * fix was to call S9xMixSamples extra times to top the ring up.
 *
 * That is not a rendering-only operation. The mixer writes ENDX (and clears
 * KON) back into APU.DSP, and the emulated SPC700 reads those registers to
 * decide when a sample has finished. Mixing ahead of emulated time therefore
 * tells the driver a sample ended before it did, and the driver keys it
 * again — the same sound plays twice. It scales with load, because the
 * number of extra mixes is exactly how far behind 60 fps we are.
 *
 * So the mixer now runs exactly once per emulated frame, and the shortfall
 * is covered by resampling audio that was really produced. Emulator state
 * is never advanced by the audio clock.
 */
#if defined(FRANK_SNES_CPU_CORE_S9X16)
/* The 1.6x S-DSP writes stereo and offers no mono path, so FAST_MODE's
   half-rate trick does not apply to it. */
#define AUDIO_CH 2
#elif defined(FRANK_SNES_FAST_MODE)
#define AUDIO_CH 1                  /* S9xMixSamplesMono output */
#else
#define AUDIO_CH 2
#endif
/* Buffer depth decides how long an fps dip can last before the resampler
 * has to stretch hard enough to be audible. M1 has almost no SRAM spare,
 * so only the larger boards get the deeper FIFO. */
#if defined(BOARD_C2) && !defined(FRANK_SNES_CPU_CORE_S9X16)
#define SFIFO_CHUNKS 8
#elif defined(BOARD_C2)
#define SFIFO_CHUNKS 8
#else
#define SFIFO_CHUNKS 4
#endif
/* Ring capacity is sized on a nominal frame, not the stretched worst case:
 * a longer producer frame simply occupies more of the ring. Only mix16 has
 * to be able to hold one whole stretched frame. */
#define SFIFO_FRAMES (AUDIO_FRAME_SAMPLES_PAL * SFIFO_CHUNKS)

/*
 * The FIFO itself lives in a header so tests/audio_path_test.c compiles
 * the *same text* the firmware does. It used to be inlined here and
 * duplicated in the test, which meant the test could pass against code
 * the device was not running — and it did: this copy still emitted zeros
 * when the ring ran dry, which punches an audible hole rather than a
 * click-free held sample.
 */
#include "audio_rate.h"

/* --- end audio pacing --- */

/* PSRAM integrity check.
 *
 * The ROM — and therefore every BRR sample the game uploads to the APU —
 * lives in PSRAM. apu.c already documents that "PSRAM read latency at
 * 504MHz/166MHz can cause SPC700 to read corrupted data", which is why
 * APU RAM itself was moved to SRAM. If the ROM image in PSRAM is not
 * stable at the speeds build.sh uses, the sample data the game uploads
 * is occasionally wrong, and that is indistinguishable by ear from the
 * emulator mishandling the sample: bits of speech repeat, cut, or turn
 * to noise, no matter which sound core renders them.
 *
 * This re-hashes the ROM once a second and counts mismatches against
 * the first pass. Non-zero means the PSRAM overclock is the problem and
 * no amount of work on the audio path will fix it. */
/* Audio capture.
 *
 * Records the exact stream handed to the I2S DMA — post-gain, post-pack,
 * the last thing before the DAC — into PSRAM, so it can be pulled off
 * over SWD and looked at. Every diagnosis so far has been indirect;
 * this is the actual signal.
 */
volatile uint32_t *audio_cap_buf;      /* PSRAM, stereo pairs as u32   */
volatile uint32_t  audio_cap_len;      /* chunks captured (total)      */
volatile uint32_t  audio_cap_max;      /* capacity in chunks           */
volatile uint32_t  audio_cap_arm;      /* 1 = recording, 0 = frozen    */
volatile uint32_t  audio_cap_wr;       /* write cursor, wraps at _max  */
volatile uint32_t  audio_cap_start;    /* first frame to capture        */

/* Sound diagnosis: the mixer emitting exact zeros for many consecutive
 * chunks means no voice is playing at all. That is what "no FIGHT" looks
 * like in the capture. When it happens, freeze a snapshot of everything
 * the mixer decides from — APU RAM, the DSP file, and the recent key-on
 * history with the sample data each one resolved to — so the failure can
 * be read out over SWD instead of guessed at. */
typedef struct {
   uint32_t frame, ch, srcn, start, loop;
   uint8_t  brr[16];        /* BRR header + first block at `start` */
} kon_rec_t;

/* Long-run key-on log in PSRAM. A duplicated sample is, by definition, an
 * extra key-on, so comparing this sequence against a host render of the
 * same ROM shows divergence directly and covers minutes in a few hundred
 * KB — raw audio for the same span would not fit beside the ROM. */
typedef struct { uint32_t frame; uint16_t start; uint8_t ch, srcn; } konlog_rec_t;
#define KONLOG_MAX 65536
volatile uint32_t *diag_statecrc;   /* per frame: APU RAM crc, DSP crc */
volatile konlog_rec_t *diag_konlog;
volatile uint32_t diag_konlog_n;
volatile uint32_t diag_emu_frame;
#define KON_RING 64
volatile kon_rec_t diag_kon[KON_RING];
volatile uint32_t  diag_kon_wr;
volatile uint32_t  diag_kon_total;
volatile uint32_t  diag_silent_run;   /* consecutive all-zero chunks    */
volatile uint32_t  diag_tripped;      /* 1 once the snapshot is taken   */
volatile uint8_t  *diag_apuram_snap;  /* 64 KB copy of APU RAM          */
#ifdef FRANK_SNES_CPU_CORE_S9X16
/* Diagnostics from the 1.43 sound investigation. The 1.6x core needs the
   SRAM for APU RAM; these are kept as one-element stubs so the code that
   references them still builds. */
volatile uint8_t   diag_ram_head[1];
volatile uint8_t   diag_dsp_snap[1];
#else
volatile uint8_t   diag_ram_head[0x800]; /* zero page + directory, SRAM */
volatile uint8_t   diag_dsp_snap[128];
#endif

/* Every SPC700 write into the sample directory, so a directory entry
 * found half-written can be traced to either a lost write (the store
 * happened, the memory did not keep it) or a write that never issued. */
typedef struct { uint32_t seq, addr; uint8_t val; } dirw_rec_t;
#ifdef FRANK_SNES_CPU_CORE_S9X16
/* A DSP-write trace ring, 6 KB of SRAM. The 1.6x core needs that space for
   APU RAM, which is on every SPC700 instruction's path; this only matters
   when the DSP-write diagnostic is being read. */
#define DIRW_RING 16
#else
#define DIRW_RING 512
#endif
volatile dirw_rec_t diag_dirw[DIRW_RING];
volatile uint32_t   diag_dirw_wr, diag_dirw_total;

void s9x_diag_dirw(unsigned addr, uint8_t val)
{
   if (diag_tripped) return;
   uint32_t i = diag_dirw_wr % DIRW_RING;
   diag_dirw[i].seq  = diag_dirw_total;
   diag_dirw[i].addr = addr;
   diag_dirw[i].val  = val;
   diag_dirw_wr = i + 1;
   diag_dirw_total++;
}

/* Set by the mixer when a voice is keyed onto a sample whose directory
 * entry has no start address: the voice then decodes whatever sits at
 * APU RAM 0x0000. That is what "clicks instead of FIGHT" is, and it is
 * a far sharper trigger than waiting for silence, which also fires on
 * legitimate sound-bank switches. */
volatile uint32_t diag_trip_on_null_kon = 0;  /* off: it disarmed the audio capture */
volatile uint32_t diag_null_kon_count;

void s9x_diag_kon(int ch, int srcn, unsigned start, unsigned loop,
                  const uint8_t *brr)
{
   if (diag_konlog && diag_konlog_n < KONLOG_MAX) {
      uint32_t i = diag_konlog_n;
      diag_konlog[i].frame = diag_emu_frame;
      diag_konlog[i].start = (uint16_t)start;
      diag_konlog[i].ch    = (uint8_t)ch;
      diag_konlog[i].srcn  = (uint8_t)srcn;
      diag_konlog_n = i + 1;
   }
   if (diag_tripped) return;
   /* The driver legitimately keys voices onto empty slots while a sound
    * bank is still loading, so ignore null key-ons until the game has
    * been playing for a while. */
   if (start == 0) {
      diag_null_kon_count++;
      if (diag_trip_on_null_kon && diag_kon_total >= 200) {
         uint32_t i2 = diag_kon_wr % KON_RING;
         diag_kon[i2].frame = diag_kon_total;
         diag_kon[i2].ch = ch; diag_kon[i2].srcn = srcn;
         diag_kon[i2].start = start; diag_kon[i2].loop = loop;
         for (int k = 0; k < 16; k++) diag_kon[i2].brr[k] = brr[k];
         diag_kon_wr = i2 + 1; diag_kon_total++;
         /* Small and SRAM-resident: a 64 KB copy into PSRAM from inside
          * the audio path starves Core 1's HDMI streaming long enough to
          * lock it up. The zero page and the directory are all that is
          * needed to see which sample a voice resolved to. */
         memcpy((void *)diag_ram_head, DIAG_APU_RAM, sizeof diag_ram_head);
         memcpy((void *)diag_dsp_snap, DIAG_DSP_REGS, 128);
         __dmb();
         diag_tripped = 1;
         audio_cap_arm = 0;
         return;
      }
   }
   uint32_t i = diag_kon_wr % KON_RING;
   diag_kon[i].frame = diag_kon_total;
   diag_kon[i].ch = ch; diag_kon[i].srcn = srcn;
   diag_kon[i].start = start; diag_kon[i].loop = loop;
   for (int k = 0; k < 16; k++) diag_kon[i].brr[k] = brr[k];
   diag_kon_wr = i + 1;
   diag_kon_total++;
}

volatile uint32_t psram_crc_checks;
volatile uint32_t psram_crc_errors;
volatile uint32_t psram_crc_first;
volatile uint32_t psram_crc_last;
static volatile uint32_t audio_prod_seq = 0; // total chunks produced
static volatile uint32_t audio_cons_seq = 0; // total chunks consumed

//=============================================================================
// Sync flags
//=============================================================================
static volatile bool core1_ready = false;
static volatile bool menu_active = false;  // When true, Core 1 stops overriding HDMI buffer
static bool hotkey_consumed = false;       // Set when hotkey opens menu (tells Start buffer to discard)

//=============================================================================
// FatFS
//=============================================================================
static FATFS fs;

/* ROM name for save state file paths (set after ROM loads) */
char g_rom_name[64];

//=============================================================================
// Flash timing configuration for overclocking
//=============================================================================
static void __no_inline_not_in_flash_func(set_flash_timings)(int cpu_mhz, int flash_max_mhz) {
    const int clock_hz = cpu_mhz * 1000000;
    const int max_flash_freq = flash_max_mhz * 1000000;
    
    int divisor = (clock_hz + max_flash_freq - (max_flash_freq >> 4) - 1) / max_flash_freq;
    if (divisor == 1 && clock_hz >= 166000000) {
        divisor = 2;
    }
    
    int rxdelay = divisor;
    if (clock_hz / divisor > 100000000 && clock_hz >= 166000000) {
        rxdelay += 1;
    }
    
    qmi_hw->m[0].timing = 0x60007000 |
                        rxdelay << QMI_M0_TIMING_RXDELAY_LSB |
                        divisor << QMI_M0_TIMING_CLKDIV_LSB;
}

//=============================================================================
// Logging
//=============================================================================
#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)

/* Telemetry the debug probe can read while the board runs.
 *
 * C2 has no usable console (USB is the HID host, and the UART header reads as
 * garbage through the probe) and OpenOCD cannot halt this firmware, so the
 * only way to get numbers off the board is to read them out of RAM with a
 * mem_ap target. Deliberately NOT static: it has to appear in the symbol
 * table for the probe to find it. Update it last in the report block and bump
 * seq around the write so a reader can tell it caught a torn sample. */
typedef struct {
    uint32_t magic;        /* 0x4B4E5246 = "FRNK" */
    uint32_t seq;          /* odd while being written */
    uint32_t emu_fps;
    uint32_t rend_fps;
    uint32_t skip_fps;
    uint32_t avg_emul_us;
    uint32_t max_emul_us;
    uint32_t avg_mix_us;
    uint32_t avg_pack_us;
    uint32_t avg_upd_us;   /* S9xUpdateScreen  */
    uint32_t avg_rs_us;    /* RenderScreen     */
    uint32_t avg_zclear_us;
    uint32_t avg_colormath_us;
    uint32_t avg_scale_us;
    uint32_t avg_obj_us;
    uint32_t avg_bg0_us;
    uint32_t instr_per_frame;
    uint32_t events_per_frame;
    uint32_t event_us_per_frame;
    uint32_t ev_us_type[7];
    uint32_t ev_n_type[7];
    uint32_t hdma_bytes_per_frame;
    uint32_t hdma_chan_per_frame;
    uint32_t hdma_calls_per_frame;
    uint32_t getmemptr_per_frame;
    /* Renderer breakdown, us PER FRAME (not per call, so it is directly
       comparable with avg_emul_us). Index order in rend_us_name[] below. */
    uint32_t rend_us[15];
    /* Always-on frame budget, ship builds included. The profiling build's
       per-event timers inflate everything measured inside an event (~3x on
       event_us), so any frame-budget claim must come from these instead. */
    uint32_t ship_emul_us;         /* avg us inside S9xMainLoop, all frames */
    uint32_t ship_emu_only_us;     /* last skipped frame: emulation without render */
    uint32_t ship_render_cost_us;  /* rolling estimate of what rendering adds */
    uint32_t ship_mainloop_calls;  /* S9xMainLoop entries per frame x100 */
    uint32_t ship_upd_us;          /* S9xUpdateScreen total, us/frame */
    uint32_t ship_rs_us;           /* RenderScreen total, us/frame */
    uint32_t ship_rs_calls;        /* RenderScreen calls/frame x100 */
    uint32_t ship_rs_sub_us;       /* subscreen half, us/frame */
    uint32_t ship_rs_sub_calls;    /* subscreen calls/frame x100 */
    uint32_t ship_tile_calls;      /* DrawTile16 calls/frame */
    uint32_t ship_tile_lines;      /* tile LINES drawn/frame */
    uint32_t ship_norender;        /* 1 = this sample had rendering skipped */
    uint32_t ship_cap_bytes;       /* PPU stream bytes captured per frame */
    uint32_t ship_cap_on;          /* 1 = capture was enabled for this sample */
    uint32_t ship_link_us;         /* duration of the last link exchange */
    uint32_t ship_link_fail;       /* cumulative exchange failures */
    uint32_t ship_link_why;        /* address of the last go_offline reason */
    uint32_t slave_render_us;
    uint32_t slave_records;
    uint32_t slave_oversize;
    uint32_t slave_psram_ok;
    uint32_t slave_impossible;
    uint32_t ship_cap_overflow;    /* master records dropped: frame is WRONG */
    uint32_t ph_sound, ph_ppu_tx, ph_ack, ph_fb, ph_ev, ph_aram;
    uint32_t slave_want;
    uint32_t ppu_fb_got;   /* framebuffer bytes the master actually received */
    uint32_t slave_pitch_h, slave_flags;
    uint32_t fb_hash;      /* FNV of the received picture: 0 or constant = blank */
    uint32_t fb_nonzero;   /* how many pixels are not colour 0 */
    uint32_t pal0, pal1;   /* two palette entries, to see if colours arrived */
    /* The truncation check. ship_cap_sum is over the bytes this chip handed
       to the link; slave_stream_* is what the other chip actually replayed
       from. See link_ppu_stat_t for how the pair is read. */
    uint32_t ship_cap_sum;
    uint32_t slave_stream_len, slave_stream_sum;
    uint32_t slave_stop_off, slave_stop_ctx, slave_stop_why;
    /* The slave's own verdict on the delivery, made inside one exchange
       against the checksum this chip put in the control frame. sum_bad > 0
       means the wire changed the bytes. */
    uint32_t ppu_sum_ok, ppu_sum_bad, ppu_exp_sum;
    /* The master's telemetry says one stream size and the slave receives
       another, on a link that now checksums clean. These say whether that is
       a selection effect (the sample lands on a big frame, the slave sees the
       small ones) or the send path dropping frames: min/max of the captured
       length over the sample window, what the last exchange actually put on
       the wire, and how many takes there were per send. */
    uint32_t cap_min, cap_max, ppu_sent_len, ppu_takes, ppu_sends;
    uint32_t cap_vram_w, cap_cgram_w, cap_oam_w;
    /* The master's own $2100/$2105/$212c/$212d, packed exactly as the slave
       packs its mirror. The slave renders an all-black frame and reports
       forced blank; this says whether that is a faithful replay of the
       master's PPU or a slave that has lost the register. */
    uint32_t master_regs;
} frank_telemetry_t;
/* 0 upd 1 rs 2 obj 3 bg0 4 bg1 5 bg2 6 bg3 7 mode7 8 zclear 9 sub 10 main
   11 colormath 12 backdrop 13 scale 14 tileconv */
volatile frank_telemetry_t frank_telemetry =
    { 0x4B4E5246u };

#ifdef FRANK_SNES_PROFILE
typedef struct {
    uint32_t last_report_us;
    uint32_t frames;
    uint32_t rendered;
    uint32_t skipped;

    uint64_t sum_emul_us;
    uint64_t sum_emul_render_us;
    uint64_t sum_emul_skip_us;
    uint64_t sum_mix_us;
    uint64_t sum_pack_us;

    uint32_t frames_render;
    uint32_t frames_skip;

    uint32_t max_emul_us;
    uint32_t max_emul_render_us;
    uint32_t max_emul_skip_us;
    uint32_t max_mix_us;
    uint32_t max_pack_us;

    int32_t max_late_us;
    uint32_t min_q_fill;
    uint32_t max_q_fill;
} perf_stats_t;

static perf_stats_t g_perf;


static inline void perf_reset_window(uint32_t now_us) {
    g_perf.last_report_us = now_us;
    g_perf.frames = 0;
    g_perf.rendered = 0;
    g_perf.skipped = 0;
    g_perf.sum_emul_us = 0;
    g_perf.sum_emul_render_us = 0;
    g_perf.sum_emul_skip_us = 0;
    g_perf.sum_mix_us = 0;
    g_perf.sum_pack_us = 0;
    g_perf.frames_render = 0;
    g_perf.frames_skip = 0;
    g_perf.max_emul_us = 0;
    g_perf.max_emul_render_us = 0;
    g_perf.max_emul_skip_us = 0;
    g_perf.max_mix_us = 0;
    g_perf.max_pack_us = 0;
    g_perf.max_late_us = 0;
    g_perf.min_q_fill = 0xFFFFFFFFu;
    g_perf.max_q_fill = 0;

    frank_snes_prof_reset_window();
}

static inline void perf_max_u32(uint32_t *dst, uint32_t v) {
    if (v > *dst) *dst = v;
}

static inline void perf_min_u32(uint32_t *dst, uint32_t v) {
    if (v < *dst) *dst = v;
}
#endif

//=============================================================================
// Snes9x Display Interface Implementation
//=============================================================================

bool S9xInitDisplay(void) {
#ifdef FRANK_SNES_CPU_CORE_S9X16
    /* Nothing to do: S9xGraphicsInit owns Pitch, Screen, SubScreen and both
     * Z buffers, and has already sized them for the native frame. */
#else
    GFX.Pitch = SNES_WIDTH;  // 8-bit pixels: 1 byte per pixel
    GFX.ZPitch = SNES_WIDTH;
    GFX.Screen = SCREEN[current_buffer];

    // SubScreenBuffer lives in the PSRAM scratch region (see declaration
    // above) — always valid, no allocation needed.  Zero it so a stale
    // sub-screen from the previous ROM doesn't leak into color math.
    memset(SubScreenBuffer, 0, SNES_WIDTH * SNES_HEIGHT);
    GFX.SubScreen = g_settings.transparency_enabled ? SubScreenBuffer : GFX.Screen;
#endif

#ifndef FRANK_SNES_CPU_CORE_S9X16
    GFX.ZBuffer = (uint8_t *)ZBuffer;
    GFX.SubZBuffer = (uint8_t *)SubZBuffer;
#endif
    return true;
}

void S9xDeinitDisplay(void) {
}

/* SNES button masks indexed by BTNMAP_* (A, B, X, Y, L, R, Start, Select) */
static const uint32_t snes_masks[BTNMAP_COUNT] = {
    SNES_A_MASK, SNES_B_MASK, SNES_X_MASK, SNES_Y_MASK,
    SNES_TL_MASK, SNES_TR_MASK, SNES_START_MASK, SNES_SELECT_MASK
};

/* Physical button bits for SNES pads indexed by BTNMAP_*.
 * The DPAD_* names are NES-era: on an SNES pad the shift-register order is
 * B, Y, Select, Start, D-pad, A, X, L, R — so physical SNES
 *   A=DPAD_Y, B=DPAD_A, X=DPAD_X, Y=DPAD_B, L=DPAD_LT, R=DPAD_RT. */
static const uint32_t nespad_bits_snes[BTNMAP_COUNT] = {
    DPAD_Y, DPAD_A, DPAD_X, DPAD_B,
    DPAD_LT, DPAD_RT, DPAD_START, DPAD_SELECT
};

/* Physical button bits for NES pads indexed by BTNMAP_*.
 * NES pads only have A and B; we send the NES "A" to SNES A and NES "B" to
 * SNES B (so the thumb-adjacent button acts as A). X/Y/L/R have no source,
 * so we leave those entries 0 (no physical bit will match). */
static const uint32_t nespad_bits_nes[BTNMAP_COUNT] = {
    DPAD_A, DPAD_B, 0, 0,
    0, 0, DPAD_START, DPAD_SELECT
};

/* Keyboard state bits indexed by BTNMAP_* */
static const uint16_t kbd_bits[BTNMAP_COUNT] = {
    KBD_STATE_A, KBD_STATE_B, KBD_STATE_X, KBD_STATE_Y,
    KBD_STATE_L, KBD_STATE_R, KBD_STATE_START, KBD_STATE_SELECT
};

/* USB gamepad button bits indexed by BTNMAP_* */
static const uint16_t usbgp_bits[BTNMAP_COUNT] = {
    0x0001, 0x0002, 0x0004, 0x0008,
    0x0010, 0x0020, 0x0040, 0x0080
};

/* Helper: merge NES/SNES pad bits into SNES joypad mask (with button remap).
 * is_snes selects the physical-bit table: SNES pads carry A/X/L/R on the
 * upper shift-register bits, NES pads only carry A/B. See nespad.h for how
 * this flag is latched by the PIO reader. */
static inline uint32_t nespad_to_snes(uint32_t pad, bool is_snes) {
    uint32_t j = 0;
    /* D-pad is always direct (no remap) */
    if (pad & DPAD_UP)     j |= SNES_UP_MASK;
    if (pad & DPAD_DOWN)   j |= SNES_DOWN_MASK;
    if (pad & DPAD_LEFT)   j |= SNES_LEFT_MASK;
    if (pad & DPAD_RIGHT)  j |= SNES_RIGHT_MASK;
    /* Face/shoulder buttons use remap table */
    const uint32_t *bits = is_snes ? nespad_bits_snes : nespad_bits_nes;
    const uint8_t *map = g_settings.btnmap_nes.map;
    for (int i = 0; i < BTNMAP_COUNT; i++) {
        if (bits[i] && (pad & bits[i]))
            j |= snes_masks[map[i]];
    }
    return j;
}

/* Helper: merge PS/2+USB keyboard state bits into SNES joypad mask (with remap) */
static inline uint32_t kbd_to_snes(uint16_t kbd) {
    uint32_t j = 0;
    if (kbd & KBD_STATE_UP)     j |= SNES_UP_MASK;
    if (kbd & KBD_STATE_DOWN)   j |= SNES_DOWN_MASK;
    if (kbd & KBD_STATE_LEFT)   j |= SNES_LEFT_MASK;
    if (kbd & KBD_STATE_RIGHT)  j |= SNES_RIGHT_MASK;
    const uint8_t *map = g_settings.btnmap_kbd.map;
    for (int i = 0; i < BTNMAP_COUNT; i++) {
        if (kbd & kbd_bits[i])
            j |= snes_masks[map[i]];
    }
    return j;
}

#ifdef USB_HID_ENABLED
/* Helper: merge USB gamepad state into SNES joypad mask (with remap) */
static inline uint32_t usbgp_to_snes(usbhid_gamepad_state_t *gp) {
    uint32_t j = 0;
    if (gp->dpad & 0x01) j |= SNES_UP_MASK;
    if (gp->dpad & 0x02) j |= SNES_DOWN_MASK;
    if (gp->dpad & 0x04) j |= SNES_LEFT_MASK;
    if (gp->dpad & 0x08) j |= SNES_RIGHT_MASK;
    const uint8_t *map = g_settings.btnmap_usb.map;
    for (int i = 0; i < BTNMAP_COUNT; i++) {
        if (gp->buttons & usbgp_bits[i])
            j |= snes_masks[map[i]];
    }
    return j;
}
#endif

uint32_t S9xReadJoypad(const int32_t port) {
    // Read input devices
    nespad_read();
    ps2kbd_tick();
#ifdef USB_HID_ENABLED
    usbhid_task();
#endif

    uint32_t joypad = 0;
    uint8_t mode = (port == 0) ? g_settings.p1_mode : g_settings.p2_mode;

    if (mode == INPUT_MODE_DISABLED)
        return 0;

    if (mode == INPUT_MODE_ANY) {
        // Merge ALL input sources
        joypad |= nespad_to_snes(nespad_state, nespad_is_snes);
        joypad |= nespad_to_snes(nespad_state2, nespad2_is_snes);
        uint16_t kbd = ps2kbd_get_state();
#ifdef USB_HID_ENABLED
        kbd |= usbhid_get_kbd_state();
#endif
        joypad |= kbd_to_snes(kbd);
#ifdef USB_HID_ENABLED
        if (usbhid_gamepad_connected()) {
            usbhid_gamepad_state_t gp;
            usbhid_get_gamepad_state(&gp);
            joypad |= usbgp_to_snes(&gp);
        }
#endif
    } else {
        // Specific input mode
        switch (mode) {
            case INPUT_MODE_NES1:
                joypad |= nespad_to_snes(nespad_state, nespad_is_snes);
                break;
            case INPUT_MODE_NES2:
                joypad |= nespad_to_snes(nespad_state2, nespad2_is_snes);
                break;
            case INPUT_MODE_KEYBOARD: {
                uint16_t kbd = ps2kbd_get_state();
#ifdef USB_HID_ENABLED
                kbd |= usbhid_get_kbd_state();
#endif
                joypad |= kbd_to_snes(kbd);
                break;
            }
#ifdef USB_HID_ENABLED
            case INPUT_MODE_USB1:
                if (usbhid_gamepad_connected_idx(0)) {
                    usbhid_gamepad_state_t gp;
                    usbhid_get_gamepad_state_idx(0, &gp);
                    joypad |= usbgp_to_snes(&gp);
                }
                break;
            case INPUT_MODE_USB2:
                if (usbhid_gamepad_connected_idx(1)) {
                    usbhid_gamepad_state_t gp;
                    usbhid_get_gamepad_state_idx(1, &gp);
                    joypad |= usbgp_to_snes(&gp);
                }
                break;
#endif
            default:
                break;
        }
    }


    /* Buffer Start and Select: don't send either to the game while held.
     * On release, send one frame of the button UNLESS the other was also
     * pressed during the hold (= hotkey combo → discard both).
     * hotkey_consumed is set by the main loop when it opens the menu. */
    if (port == 0) {
        static bool start_held = false;
        static bool select_held = false;
        static bool combo_seen = false;
        bool cur_start = (joypad & SNES_START_MASK) != 0;
        bool cur_select = (joypad & SNES_SELECT_MASK) != 0;

        if (hotkey_consumed) {
            combo_seen = true;
            hotkey_consumed = false;
        }

        /* While either hotkey button is held, suppress both and watch for combo */
        if (cur_start || cur_select) {
            if (cur_start && cur_select)
                combo_seen = true;
            if (cur_start)  { joypad &= ~SNES_START_MASK;  start_held = true; }
            if (cur_select) { joypad &= ~SNES_SELECT_MASK; select_held = true; }
        }

        /* When BOTH are released, decide what to do */
        if (!cur_start && !cur_select && (start_held || select_held)) {
            if (!combo_seen) {
                /* Solo press — inject one frame */
                if (start_held)  joypad |= SNES_START_MASK;
                if (select_held) joypad |= SNES_SELECT_MASK;
            }
            start_held = false;
            select_held = false;
            combo_seen = false;
        }
    }

    /* Detect new button presses — notify SFX auto-release system */
    if (port == 0) {
        static uint32_t prev_joypad = 0;
        uint32_t new_buttons = joypad & ~prev_joypad;
        if (new_buttons)
#ifndef FRANK_SNES_CPU_CORE_S9X16
            S9xNotifyButtonPress();   /* SFX auto-release: 1.43 only */
#endif
        prev_joypad = joypad;
    }

#ifdef FRANK_SNES_PADSCRIPT_MK3
    /*
     * Deterministic MK3 menu walk, for comparing this emulator against the
     * same core running on a host.
     *
     * Six single-frame START presses at fixed frames, then NOTHING is
     * pressed for the rest of the run. The frames are the ones the host
     * harness (scratchpad lr.c, screen-driven) chose for the 1.43 core, so
     * both sides see identical input and their key-on streams can be
     * diffed directly.
     *
     * It must stay single-frame and must stop: START pauses MK3, so a
     * script that keeps pressing it re-triggers the announcer and restarts
     * rounds by itself, which invalidated every earlier comparison.
     */
    if (port == 0) {
        /* Key off the EMULATED FRAME, not the number of times the game has
         * polled the pad. MK3 reads the joypad more than once per frame, so
         * a call counter runs ahead of the host harness and the two stop
         * being the same experiment — which invalidated the first
         * device-vs-host comparison. */
        static const uint32_t press_at[] = { 46, 196, 346, 496, 652, 1440 };
        uint32_t f = (uint32_t)ICPU.Frame;
        for (unsigned i = 0; i < sizeof(press_at) / sizeof(press_at[0]); i++)
            if (f == press_at[i]) joypad |= SNES_START_MASK;
    }
#endif

#ifdef FRANK_SNES_AUTOPAD
    /* Scripted pad driver for SNES DOOM: drives through logos, title,
     * menus, difficulty/episode select, then moves forward with D-pad UP.
     * Press events are 8 frames ON, 12 frames OFF — DOOM's menu debouncer
     * needs at least 6 frames between presses. */
    if (port == 0) {
        static uint32_t autopad_frame = 0;
        autopad_frame++;
        uint32_t f = autopad_frame;

        /* Phase plan (frame counts at ~24 emulated FPS so beat is ~0.33s/press):
         *   0..240   : wait through publisher logos (~10s at 24fps)
         *   240..250 : START (skip title "Press Start")
         *   270..280 : A (confirm "New Game" main menu)
         *   300..310 : A (confirm default difficulty)
         *   330..340 : A (confirm first episode)
         *   360..480 : wait for level load (~5s)
         *   480+     : hold UP continuously to move forward in the level
         */
        if (f >= 240 && f < 248)          joypad |= SNES_START_MASK;
        else if (f >= 270 && f < 278)     joypad |= SNES_A_MASK;
        else if (f >= 300 && f < 308)     joypad |= SNES_A_MASK;
        else if (f >= 330 && f < 338)     joypad |= SNES_A_MASK;
        else if (f >= 360 && f < 368)     joypad |= SNES_A_MASK;  /* extra safety confirm */
        else if (f >= 480) {
            /* Hold UP to walk forward, alternate occasional strafing */
            joypad |= SNES_UP_MASK;
            /* Wiggle left/right every 60 frames to explore */
            uint32_t phase = (f - 480) / 60;
            if (phase & 1) joypad |= SNES_LEFT_MASK;
        }
    }
#endif

    return joypad;
}

//=============================================================================
// SNES Mouse (plugged into controller port 2).
//
// Pointer lives in screen space [0..255, 0..223]. Host mouse deltas (PS/2
// and USB) are accumulated into this position each frame. Snes9x reads the
// position via S9xReadMousePosition() when the emulated controller is
// SNES_MOUSE and turns position deltas into SNES mouse packets.
//=============================================================================

static int32_t  snes_mouse_x = 128;
static int32_t  snes_mouse_y = 112;
static uint32_t snes_mouse_buttons = 0;

static inline void snes_mouse_apply_delta(int16_t dx, int16_t dy, uint8_t buttons) {
    // Free-running accumulator — do NOT clamp to screen bounds. Snes9x
    // reads our value and diffs it against its own previous sample to
    // produce per-frame motion packets for the SNES mouse, then the game
    // handles on-screen cursor bounds itself. Clamping here silently
    // swallows real mouse motion once the accumulator hits 0 or 255,
    // which shows up as "can't move left/up anymore until I move the
    // opposite direction first".
    snes_mouse_x += (int32_t)dx;
    snes_mouse_y += (int32_t)dy;

    // PS/2/USB buttons: bit0=left, bit1=right, bit2=middle.
    // SNES mouse returns bit0=left, bit1=right (via the PPU code).
    snes_mouse_buttons = (uint32_t)(buttons & 0x03);
}

static inline bool mouse_is_connected(void) {
    if (ps2_mouse_is_initialized()) return true;
#ifdef USB_HID_ENABLED
    if (usbhid_mouse_connected()) return true;
#endif
    return false;
}

static void poll_host_mouse(void) {
    // No user on/off toggle any more — if a mouse is physically attached we
    // always drive the pointer and let the menu decide which SNES port it
    // lands on via g_settings.mouse_port.

    // PS/2 mouse (streaming via ps2_mouse_poll + get_state)
    if (ps2_mouse_is_initialized()) {
        int16_t dx = 0, dy = 0;
        int8_t wheel = 0;
        uint8_t btns = 0;
        if (ps2_mouse_get_state(&dx, &dy, &wheel, &btns)) {
            // On this board/driver the raw axes are swapped AND both
            // signs are inverted relative to screen space.
            //   raw dy -> screen X, negated  (right hand = +x on screen)
            //   raw dx -> screen Y, negated  (down hand  = +y on screen)
            snes_mouse_apply_delta((int16_t)-dy, (int16_t)dx, btns);
            LOG("[mouse ps2] dx=%d dy=%d btn=0x%02X -> pos=(%ld,%ld)\n",
                (int)dx, (int)dy, btns,
                (long)snes_mouse_x, (long)snes_mouse_y);
        } else {
            // Still need to refresh button state even when no motion.
            uint32_t new_btn = (uint32_t)(btns & 0x03);
            if (new_btn != snes_mouse_buttons) {
                LOG("[mouse ps2] btn-only 0x%02X -> 0x%02X\n",
                    (unsigned)snes_mouse_buttons, (unsigned)new_btn);
                snes_mouse_buttons = new_btn;
            }
        }
    }

#ifdef USB_HID_ENABLED
    if (usbhid_mouse_connected()) {
        usbhid_mouse_state_t m;
        usbhid_get_mouse_state(&m);
        if (m.has_motion || m.dx || m.dy || m.buttons) {
            // Same axis swap + inversion the PS/2 path needed on this
            // build: raw dx/dy are rotated/flipped relative to screen
            // space. raw dy -> screen -x, raw dx -> screen +y.
            snes_mouse_apply_delta((int16_t)-m.dy, (int16_t)m.dx, m.buttons);
            LOG("[mouse usb] dx=%d dy=%d btn=0x%02X -> pos=(%ld,%ld)\n",
                (int)m.dx, (int)m.dy, m.buttons,
                (long)snes_mouse_x, (long)snes_mouse_y);
        }
    }
#endif

    // Once-per-second heartbeat so we can tell whether the mouse pipeline
    // is alive even when no motion is arriving. Shows PS/2 init state,
    // raw-byte count from the IRQ, packet count, ring depth, and errors.
    // Skip entirely when nothing is plugged in — otherwise it spams the
    // UART every second for the 99% of users who never attach a mouse.
    if (!mouse_is_connected()) return;
    static uint32_t last_heartbeat_us = 0;
    uint32_t now_us = time_us_32();
    if ((now_us - last_heartbeat_us) >= 1000000u) {
        last_heartbeat_us = now_us;
        uint32_t raw = 0, pkts = 0, ring = 0;
        uint32_t ferr = 0, perr = 0, serr = 0;
        ps2_mouse_get_counters(&raw, &pkts, &ring);
        ps2_mouse_get_errors(&ferr, &perr, &serr);
        uint32_t fifo = ps2_mouse_pio_fifo_level();
        LOG("[mouse hb] ps2_init=%d wheel=%d fifo=%lu raw=%lu pkts=%lu ring=%lu "
            "frame=%lu parity=%lu sync=%lu "
#ifdef USB_HID_ENABLED
            "usb=%d "
#endif
            "pos=(%ld,%ld) btn=0x%02X enabled=%d\n",
            (int)ps2_mouse_is_initialized(),
            (int)ps2_mouse_has_wheel(),
            (unsigned long)fifo,
            (unsigned long)raw, (unsigned long)pkts, (unsigned long)ring,
            (unsigned long)ferr, (unsigned long)perr, (unsigned long)serr,
#ifdef USB_HID_ENABLED
            (int)usbhid_mouse_connected(),
#endif
            (long)snes_mouse_x, (long)snes_mouse_y,
            (unsigned)snes_mouse_buttons,
            (int)mouse_is_connected());
    }
}

bool S9xReadMousePosition(int32_t which1, int32_t *x, int32_t *y, uint32_t *buttons) {
    if (which1 != 0) return false;
    if (!mouse_is_connected()) return false;
    if (x) *x = snes_mouse_x;
    if (y) *y = snes_mouse_y;
    if (buttons) *buttons = snes_mouse_buttons;
    return true;
}

bool S9xReadSuperScopePosition(int32_t *x, int32_t *y, uint32_t *buttons) {
    return false;
}

bool JustifierOffscreen(void) {
    return true;
}

void JustifierButtons(uint32_t *justifiers) {
    (void)justifiers;
}

//=============================================================================
// Snes9x Initialization
//=============================================================================

static inline void snes9x_init(void) {
#ifdef FRANK_SNES_CPU_CORE_S9X16
    /* s9x16_init() owns Settings for this core - including the cycle costs
     * that make it time the machine correctly. Only the front end's own
     * preferences are applied here, and only the ones 1.6x still has.
     *
     * Gone on purpose: CyclesPercentage, H_Max and HBlankStart. 1.6x
     * charges cycles inside the memory accessors, so there is no scanline
     * length to stretch - which is the whole reason for this port. Also
     * gone: SoundPlaybackRate (the DSP emits at a fixed 32040 Hz),
     * InterpolatedSound and DisableSoundEcho (the S-DSP does both properly
     * now), and the Mouse fields (1.6x routes controllers through
     * S9xSetController). */
    bool have_mouse = mouse_is_connected() &&
                      (g_settings.mouse_port != MOUSE_PORT_OFF);
    Settings.Mute = (g_settings.volume == 0);
    Settings.MouseMaster = have_mouse;
#else
    Settings.CyclesPercentage = 100;
    Settings.H_Max = SNES_CYCLES_PER_SCANLINE;
    Settings.FrameTimePAL = 20000;
    Settings.FrameTimeNTSC = 16667;
    // If a mouse is physically attached at boot, put the controller in
    // SNES_MOUSE mode so S9xProcessMouse() actually drives the bus.
    // MOUSE_PORT_OFF lets the user force the mouse off — needed for games
    // like King Arthur & The Knights of Justice that lock up on any mouse.
    // Settings.MousePort decides whether the packet lands on port 1 or 2.
    bool have_mouse = mouse_is_connected() &&
                      (g_settings.mouse_port != MOUSE_PORT_OFF);
    Settings.ControllerOption = have_mouse ? SNES_MOUSE : SNES_JOYPAD;
    Settings.HBlankStart = (256 * Settings.H_Max) / SNES_HCOUNTER_MAX;
    Settings.SoundPlaybackRate = AUDIO_SAMPLE_RATE;
    Settings.DisableSoundEcho = !g_settings.echo_enabled;
    Settings.InterpolatedSound = g_settings.interpolation;
    Settings.Mute = (g_settings.volume == 0);
    Settings.Mouse = have_mouse;
    Settings.MouseMaster = have_mouse;
    Settings.MousePort = (g_settings.mouse_port == MOUSE_PORT_1) ? 0 : 1;
#ifdef FRANK_SNES_AUDIO_CAPTURE
    /* Pin every sound-affecting setting so a device capture is directly
     * comparable with a host render of the same ROM. */
    Settings.DisableSoundEcho = false;
    Settings.InterpolatedSound = true;
    Settings.SoundEnvelopeHeightReading = false;
    Settings.Mute = false;
#endif
#endif  /* !FRANK_SNES_CPU_CORE_S9X16 */

#ifdef FRANK_SNES_CPU_CORE_S9X16
    /* The 1.6x core owns its own init order and its own APU, and it does
     * not resample - it emits at 32040 Hz, which is already this port's
     * I2S rate. So there is no playback rate to set here, and asking for
     * one would be a lie the core could not honour. */
    S9xInitDisplay();
    if (!s9x16_init()) {
        LOG("FATAL: s9x16_init failed (out of memory)\n");
    }
    s9x16_set_render(true);
#else
    S9xInitDisplay();
    S9xInitMemory();
#ifdef FRANK_SNES_PPU_CAPTURE
    /* The PPU command store is 48 KB and lives in PSRAM: master SRAM has only
       ~22 KB free, and a static buffer there hung the board. */
    if (!ppucap_init())
        LOG("FATAL: ppucap_init failed (no PSRAM for the PPU stream)\n");
    g_ppu_stage = (uint8_t *)psram_malloc(SNES_WIDTH * SNES_HEIGHT);
    if (!g_ppu_stage)
        LOG("FATAL: no PSRAM for the PPU staging buffer\n");
#endif
    S9xInitAPU();
    S9xInitSound(0, 0);
    if (!S9xInitGFX()) {
        LOG("FATAL: S9xInitGFX failed (out of memory)\n");
    }
    S9xSetPlaybackRate(Settings.SoundPlaybackRate);
    IPPU.RenderThisFrame = 1;
#endif
}

//=============================================================================
// ROM Loading from SD Card
//=============================================================================

#ifdef FRANK_SNES_CPU_CORE_S9X16
static size_t g_rom_content_size = 0;
#endif

static bool load_rom_from_sd(const char *filename) {
    static FIL file;
    UINT bytes_read;
    
    LOG("Opening ROM: %s\n", filename);
    
    FRESULT res = f_open(&file, filename, FA_READ);
    if (res != FR_OK) {
        LOG("Failed to open ROM file: %d\n", res);
        return false;
    }
    
    FSIZE_t file_size = f_size(&file);
    LOG("ROM size: %lu bytes\n", (unsigned long)file_size);
    
    // Maximum ROM size for SNES (6 MB for largest commercial games)
    const size_t MAX_ROM_SIZE = 6 * 1024 * 1024;
    
    if (file_size > MAX_ROM_SIZE) {
        LOG("ROM too large! Max: %lu bytes\n", (unsigned long)MAX_ROM_SIZE);
        f_close(&file);
        return false;
    }
    
    // Allocate ROM buffer in PSRAM
    // Peek at ROM header to detect SuperFX before allocating
    size_t alloc_size = (file_size + 0xFFFF) & ~0xFFFF;  // Round up to 64KB boundary
    bool might_be_superfx = false;
    if (file_size >= 0x8000) {
        uint8_t rom_type_byte = 0;
        UINT peek_br;
        /* Cartridge type is at ROM offset 0x7FD6 (NOT 0x7FD5 which is map mode).
         * Account for possible 512-byte copier header. */
        size_t hdr_off = (file_size & 0x3FF) == 0x200 ? 0x200 : 0;
        f_lseek(&file, hdr_off + 0x7FD6);
        f_read(&file, &rom_type_byte, 1, &peek_br);
        f_lseek(&file, 0);
        might_be_superfx = (rom_type_byte & 0xF0) == 0x10;
    }
    if (might_be_superfx && alloc_size < 0x600000)
        alloc_size = 0x600000;  // SuperFX needs ROM duplication at +2MB offset
    else
        alloc_size += 0x10000;  // Extra 64KB for mapping safety (non-SuperFX)
    alloc_size += 0x200;  // Header alignment
#ifdef FRANK_SNES_CPU_CORE_S9X16
    /* The 1.6x core allocated its ROM buffer in Memory.Init, so the file is
     * read straight into it. Allocating a second one here would be two
     * ROM-sized blocks in 8 MB of PSRAM. There is no ForceSuperFX hint
     * either - this core sizes SRAM from the ROM header it parses. */
    {
        if (!s9x16_alloc_rom((uint32_t)file_size)) {
            LOG("Failed to allocate ROM buffer for %lu bytes\n",
                (unsigned long)file_size);
            f_close(&file);
            return false;
        }
        uint32_t rom_capacity = 0;
        uint8_t *rom_dst = s9x16_rom_storage(&rom_capacity);
        if (rom_dst == NULL || file_size > rom_capacity) {
            LOG("ROM too large for core buffer (%lu > %lu)\n",
                (unsigned long)file_size, (unsigned long)rom_capacity);
            f_close(&file);
            return false;
        }
        Memory.ROM = rom_dst;
    }
    (void)alloc_size;
    (void)might_be_superfx;
#else
    Memory.ROM = (uint8_t *)psram_malloc(alloc_size);
    if (Memory.ROM == NULL) {
        LOG("Failed to allocate ROM buffer (%lu bytes)!\n", (unsigned long)alloc_size);
        f_close(&file);
        return false;
    }
    LOG("Allocated %lu bytes for ROM in PSRAM%s\n", (unsigned long)alloc_size,
        might_be_superfx ? " (SuperFX)" : "");

    Memory.ROM_AllocSize = file_size; /* Content size for ROM parser; buffer may be larger */
    Settings.ForceSuperFX = might_be_superfx; /* Hint for S9xInitMemory to allocate 128KB SRAM */
#endif

    // Read ROM into buffer
    res = f_read(&file, Memory.ROM, file_size, &bytes_read);
    f_close(&file);
    
    if (res != FR_OK || bytes_read != file_size) {
        LOG("Failed to read ROM: res=%d, read=%lu\n", res, (unsigned long)bytes_read);
        return false;
    }
    
    LOG("ROM loaded: %lu bytes\n", (unsigned long)bytes_read);
#ifdef FRANK_SNES_CPU_CORE_S9X16
    /* The 1.6x parser is handed the byte count separately - the buffer it
     * sits in is larger than the image. */
    g_rom_content_size = (size_t)bytes_read;
#endif
    return true;
}

//=============================================================================
// Render Core (Core 1) - HDMI & Audio output (exact copy from pico-snes-master)
//=============================================================================

// Test tone buffer in SRAM (not PSRAM)
static int16_t __attribute__((aligned(4))) test_tone[512];

static inline int16_t clamp16(int32_t v) {
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

// Gentle soft limiter to keep boosted peaks from hard-clipping
static inline int16_t soft_limit16(int32_t v) {
    const int32_t knee = 30000; // start soft limiting near full scale
    if (v > knee) {
        v = knee + (v - knee) / 4;
    } else if (v < -knee) {
        v = -knee + (v + knee) / 4;
    }
    return clamp16(v);
}

/* Set by core 0 once it has given up the HDMI scanline interrupt; core 1 then
   takes it. See graphics_hdmi_irq_take_this_core() for why the display cannot
   stay on core 0 once the PPU offload is blocking it. */
/* Diagnostic: skip pushing the slave's palette into the HDMI driver.
   graphics_set_palette_hdmi() encodes straight into the LIVE conv_color TMDS
   table, and its own comment claims that is safe "because
   S9xFixColourBrightness is called between frames" - which was true when the
   renderer drove it and is not true now. Set to 1 to take the palette out of
   the picture entirely while testing whether the display holds lock. */
volatile uint32_t g_pal_push_disable = 0;
volatile bool g_hdmi_irq_core1_ready;   /* core 1 is at its service loop */
volatile bool g_hdmi_irq_released;      /* core 0 has given the IRQ up */
extern void graphics_hdmi_irq_take_this_core(void);
extern void graphics_hdmi_irq_release_this_core(void);

void __time_critical_func(render_core)(void) {
    // Pre-generate test tone - 440Hz square wave
    for (int i = 0; i < 256; i++) {
        int16_t sample = ((i / 25) & 1) ? 8000 : -8000;
        test_tone[i * 2] = sample;      // Left
        test_tone[i * 2 + 1] = sample;  // Right
    }

    // Initialize APU Core 1 support
#if APU_ON_CORE1
    apu_core1_init();
#endif


#ifdef FRANK_SNES_HDMI_ALT
    // HDMI_ALT path: Core 1 is the libdvi worker.  Audio rides HDMI
    // data-island packets, fed directly from Core 0 after packing —
    // there is no I2S consumer here.  Signal ready then enter the
    // libdvi loop, which never returns.
    __dmb();
    core1_ready = true;
    __dmb();
    hdmi_alt_run_core1();
    __builtin_unreachable();
#else
    // Initialize audio on Core 1
    static i2s_config_t i2s_config;
    i2s_config = i2s_get_default_config();
    i2s_config.sample_freq = AUDIO_SAMPLE_RATE;
    i2s_config.dma_trans_count = AUDIO_BUFFER_LENGTH;
    i2s_volume(&i2s_config, 0);
    i2s_init(&i2s_config);

    // HDMI is already initialized on Core 0
    // Signal ready with memory barrier
    __dmb();
    /* Take the HDMI scanline interrupt over from core 0.
     *
     * graphics_init() runs on core 0 before this core exists (main.c: "on Core
     * 0 ... critical for the ROM selector"), so despite its name
     * irq_set_exclusive_handler_DMA_core1() leaves the interrupt enabled on
     * CORE 0. That is fatal once the C2 PPU offload is on: core 0 then also
     * absorbs a 57 KB framebuffer DMA every frame and blocks in the link, so
     * the scanline interrupt misses its deadline and the sink drops lock - the
     * display reads NO SIGNAL while the emulator runs at 51 fps and the slave
     * renders happily. Measured: with the slave rendering, HDMI never locked;
     * with the slave idle, it was rock solid.
     *
     * Ordering: core 0 waits for this flag, disables on itself, then releases;
     * this core enables only afterwards. Enabled on both at once would let two
     * handlers race the same DMA pointer state. */
    g_hdmi_irq_core1_ready = true;
    __dmb();
    while (!g_hdmi_irq_released) tight_loop_contents();
    graphics_hdmi_irq_take_this_core();

    core1_ready = true;
    __dmb();

    // Audio playback - continuously stream from ring buffer to DMA
    static uint32_t __attribute__((aligned(32))) fadeout_buf[AUDIO_BUFFER_LENGTH];
    memset(fadeout_buf, 0, sizeof(fadeout_buf));
    uint32_t last_displayed_buffer = 0;
    uint8_t underrun_count = 0;  /* consecutive underruns for progressive fade */
    bool was_underrun = false;   /* previous chunk was underrun — need fade-in */
    uint32_t total_underruns = 0;
    uint32_t total_chunks = 0;
    uint32_t diag_timer = 0;
    uint32_t max_gap_us = 0;   /* longest gap between consecutive consumer calls */
    uint32_t prev_consume_us = 0;
    while (true) {
        // Run APU batch on Core 1 - catch up to CPU target cycles
#if APU_ON_CORE1
        apu_core1_run_batch();
#endif

        // Skip HDMI buffer management when menu is active —
        // Core 0 controls current_buffer directly for the menu.
        if (!menu_active) {
            uint32_t current_buf = current_buffer;
            if (current_buf != last_displayed_buffer) {
                last_displayed_buffer = current_buf;
            }
        }

        // Consume next mixed chunk if available; fade out on underrun.
        uint32_t prod = audio_prod_seq;
        uint32_t cons = audio_cons_seq;
        const uint32_t *audio_src;
        total_chunks++;
        if (prod != cons) {
            uint32_t idx = cons % AUDIO_QUEUE_DEPTH;
            __dmb();
            audio_src = audio_packed_buffer[idx];

            // After underrun, apply fade-in to avoid click at resume boundary.
            // Ramp first FADE_IN_SAMPLES from 0→1 so the waveform starts from
            // silence instead of jumping to a non-zero value.
            #define FADE_IN_SAMPLES 32
            if (was_underrun) {
                // Copy to fadeout_buf so we can modify in-place
                memcpy(fadeout_buf, audio_src, AUDIO_BUFFER_LENGTH * sizeof(uint32_t));
                int16_t *p = (int16_t *)fadeout_buf;
                for (uint32_t i = 0; i < FADE_IN_SAMPLES; i++) {
                    p[i * 2]     = (int16_t)((p[i * 2]     * (int32_t)i) / FADE_IN_SAMPLES);
                    p[i * 2 + 1] = (int16_t)((p[i * 2 + 1] * (int32_t)i) / FADE_IN_SAMPLES);
                }
                audio_src = fadeout_buf;
            }

            was_underrun = false;
            underrun_count = 0;
        } else {
            total_underruns++;
            audio_underruns++;
            was_underrun = true;
            // Underrun: ramp from last sample to zero (no buffer replay).
            if (underrun_count == 0) {
                int16_t *fade = (int16_t *)fadeout_buf;
                int16_t last_l = fade[(AUDIO_BUFFER_LENGTH - 1) * 2];
                int16_t last_r = fade[(AUDIO_BUFFER_LENGTH - 1) * 2 + 1];
                for (uint32_t i = 0; i < AUDIO_BUFFER_LENGTH; i++) {
                    int32_t t = AUDIO_BUFFER_LENGTH - i;
                    fade[i * 2]     = (int16_t)((last_l * t) / (int32_t)AUDIO_BUFFER_LENGTH);
                    fade[i * 2 + 1] = (int16_t)((last_r * t) / (int32_t)AUDIO_BUFFER_LENGTH);
                }
                underrun_count++;
            } else {
                memset(fadeout_buf, 0, sizeof(fadeout_buf));
            }
            audio_src = fadeout_buf;
        }

        // Stream to I2S DMA (blocks until a DMA buffer is free)
        i2s_dma_write(&i2s_config, (const int16_t *)audio_src);

        // Track consumer cadence
        uint32_t now_us = time_us_32();
        if (prev_consume_us) {
            uint32_t gap = now_us - prev_consume_us;
            if (gap > max_gap_us) max_gap_us = gap;
        }
        prev_consume_us = now_us;

        // Advance consumer AFTER DMA copy is complete
        if (prod != cons) {
            // Save unmodified buffer for potential fade-out on next underrun.
            // (audio_src may point to fadeout_buf with fade-in applied, so
            //  re-copy from the original ring buffer slot.)
            uint32_t idx2 = cons % AUDIO_QUEUE_DEPTH;
            memcpy(fadeout_buf, audio_packed_buffer[idx2], AUDIO_BUFFER_LENGTH * sizeof(uint32_t));
            __dmb();
            audio_cons_seq = cons + 1;
        }

        // Reset underrun stats periodically (logging disabled)
        if (++diag_timer >= 300) {
            total_underruns = 0;
            total_chunks = 0;
            max_gap_us = 0;
            diag_timer = 0;
        }
    }
#endif /* !FRANK_SNES_HDMI_ALT */
}

//=============================================================================
// Main Emulation Loop
//=============================================================================

// Deferred palette update flag from PPU
/* declared with C linkage at file scope, see the include block */
extern void S9xFixColourBrightness(void);

/*
 * Wall time for one emulated frame.
 *
 * A PAL machine's frame is 20 ms, not 16.67. Pacing every ROM at 60 Hz ran
 * PAL games 20% fast, and — now that the mixer is asked for a whole
 * emulated frame — would also overproduce audio by 20% against a
 * fixed-rate DAC. Set from the ROM's region once it is known; the default
 * covers the menu, before any ROM is loaded.
 */
static uint32_t g_target_frame_us = 16667;
#define TARGET_FRAME_US g_target_frame_us

void audio_set_region_pacing(bool pal)
{
    g_target_frame_us = pal ? 20000u : 16667u;
    LOG("[pace] %s: %u us/frame, %u samples/frame\n",
        pal ? "PAL" : "NTSC", (unsigned)g_target_frame_us,
        (unsigned)(pal ? AUDIO_FRAME_SAMPLES_PAL : AUDIO_FRAME_SAMPLES_NTSC));
}

//=============================================================================
// Constant Frameskip Configuration (from murmgenesis)
//=============================================================================
// Frameskip pattern:
// - Pattern length is in frames
// - Bit i (LSB=frame 0) indicates whether to render that frame (1) or skip (0)
// Configurable via -DFRAMESKIP_LEVEL=N where:
//   0 = render all frames (60 fps target)
//   1 = render 5/6 frames (~50 fps)
//   2 = render 3/6 frames (~30 fps) with blink-friendly pattern
//   3 = render 2/6 frames (~20 fps) - DEFAULT
//   4 = render 2/6 frames (~20 fps)
#ifndef FRAMESKIP_LEVEL
#ifdef FRANK_SNES_FAST_MODE
#define FRAMESKIP_LEVEL 2  // Fast mode: 40fps with reduced quality
#else
#define FRAMESKIP_LEVEL 3  // Normal mode: 30fps with full quality
#endif
#endif

// Frameskip patterns: [len, mask] for each level
static const uint8_t frameskip_patterns[5][2] = {
    {1, 0x01},  // 0: none - render every frame (60fps)
    {6, 0x1F},  // 1: low - render frames 0-4, skip frame 5 (~50fps)
    {6, 0x19},  // 2: medium - render frames 0,3,4 (~30fps), consecutive pair for blink visibility
    {6, 0x09},  // 3: high - render frames 0,3 (~20fps)
    {6, 0x03},  // 4: extreme - render frames 0,1 (~20fps)
};

// Runtime frameskip settings
static uint32_t frameskip_pattern_len = 6;
static uint32_t frameskip_pattern_mask = 0x09;  // Default: level 3 (20fps)

// Set frameskip level at runtime
#ifdef __cplusplus
extern "C"
#endif
void set_frameskip_level(uint8_t level) {
    if (level > 4) level = 3;  // Clamp to valid range
    frameskip_pattern_len = frameskip_patterns[level][0];
    frameskip_pattern_mask = frameskip_patterns[level][1];
}

// Safety: always render at least once every N frames even if pattern says skip
#define FRAMESKIP_MAX_CONSECUTIVE 4

// Don't treat tiny overshoots (scheduler jitter) as being "behind".
#define LATE_TOLERANCE_US 1000
// If we fall too far behind, resync the deadline instead of accumulating lateness.
#define LATE_RESYNC_US (TARGET_FRAME_US * 4)

static bool __time_critical_func(emulation_loop)(void) {  /* returns true if user wants ROM selector */
#ifdef FRANK_SNES_AUDIO_CAPTURE
    /* ~12 s of 32040 Hz stereo. */
    audio_cap_max = 900;    /* 15 s of final packed stereo output */
    audio_cap_buf = (volatile uint32_t *)
        psram_malloc(audio_cap_max * AUDIO_BUFFER_LENGTH * sizeof(uint32_t));
    audio_cap_len = 0;
    audio_cap_wr  = 0;
    /* audio_cap_start is set over SWD before emulation starts */
    audio_cap_arm = audio_cap_buf ? 1 : 0;
    diag_apuram_snap = (volatile uint8_t *)psram_malloc(0x10000);
    diag_konlog = (volatile konlog_rec_t *)
        psram_malloc(KONLOG_MAX * sizeof(konlog_rec_t));
    diag_statecrc = (volatile uint32_t *)psram_malloc(1800 * 8 * sizeof(uint32_t));
    diag_konlog_n = 0; diag_emu_frame = 0;
    LOG("[cap] konlog=%p\n", (void *)diag_konlog);
    diag_tripped = 0; diag_silent_run = 0; diag_kon_wr = 0; diag_kon_total = 0;
    LOG("[cap] apuram snapshot=%p\n", (void *)diag_apuram_snap);
    LOG("[cap] buffer=%p chunks=%u\n", (void *)audio_cap_buf,
        (unsigned)audio_cap_max);
#endif

    LOG("Starting emulation loop...\n");
    LOG("[build] %s %s | TARGET_FRAME_US=%u FRAMESKIP_LEVEL=%u LATE_RESYNC_US=%u\n",
        __DATE__, __TIME__,
        (unsigned)TARGET_FRAME_US,
        (unsigned)FRAMESKIP_LEVEL,
        (unsigned)LATE_RESYNC_US);
#ifdef FRANK_SNES_PROFILE
    LOG("[perf] enabled\n");
#else
    LOG("[perf] disabled (rebuild with FRANK_SNES_PROFILE=ON)\n");
#endif

    // Fixed-timestep scheduling: keep emulation/audio running at ~60Hz.
    // If rendering is slow, we skip video frames to catch up rather than slowing audio.
    uint32_t next_frame_deadline = time_us_32() + TARGET_FRAME_US;
    uint32_t frame_num = 0;
    uint32_t consecutive_skipped_frames = 0;

    // Wall-clock audio accumulator: tracks how much real time has elapsed
    // that hasn't been "covered" by an audio chunk yet.  Each TARGET_FRAME_US
    // of accumulated time = one chunk owed.  The normal per-frame mix covers
    // one; any remainder triggers extra S9xMixSamples calls so the I2S output
    // never starves even when emulation runs below 60 fps.
    uint32_t audio_acc_us = 0;
    uint32_t audio_last_us = time_us_32();

    // Initialize frameskip from settings (runtime overrides compile-time default)
    set_frameskip_level(g_settings.frameskip);
#ifdef FRANK_SNES_FORCE_FRAMESKIP
    /* Test override: the emulator's own frameskip setting decides how
     * many frames are rendered, but emulation still has to run every
     * frame. When the CPU cannot sustain 60 fps of emulation, rendering
     * fewer frames is the cheapest time back. */
    set_frameskip_level(FRANK_SNES_FORCE_FRAMESKIP);
#endif
    static const char* frameskip_level_names[] = {"NONE (60fps)", "LOW (50fps)", "MEDIUM (30fps)", "HIGH (20fps)", "EXTREME (20fps)"};
    LOG("[frameskip] level=%d (%s) pattern_len=%u mask=0x%02X\n",
        g_settings.frameskip, frameskip_level_names[g_settings.frameskip],
        (unsigned)frameskip_pattern_len, (unsigned)frameskip_pattern_mask);

#ifdef FRANK_SNES_PROFILE
    perf_reset_window(time_us_32());
#endif

    while (true) {
        uint32_t now = time_us_32();
        int32_t late_us = (int32_t)(now - next_frame_deadline);

        // If we're way behind, drop accumulated lateness and realign.
        // This prevents the loop from going into a long "catch up" phase where
        // it never sleeps and video stays permanently in skip mode.
        if (late_us > (int32_t)LATE_RESYNC_US) {
            next_frame_deadline = now + TARGET_FRAME_US;
            late_us = 0;
            consecutive_skipped_frames = 0;
        }

        // If we're ahead of schedule, wait until it's time for the next emulated frame.
        if (late_us < 0) {
            busy_wait_us_32((uint32_t)(-late_us));
            now = time_us_32();
            late_us = (int32_t)(now - next_frame_deadline);
        }

        // Clamp tiny "late" values caused by wake-up jitter.
        if (late_us > 0 && late_us <= (int32_t)LATE_TOLERANCE_US) {
            late_us = 0;
        }

        // Audio queue depth (before producing this frame's chunk).
        uint32_t q_prod = audio_prod_seq;
        uint32_t q_cons = audio_cons_seq;
        uint32_t q_fill = q_prod - q_cons;

        // Deterministic render/skip pattern (from murmgenesis).
        // Uses constant pattern based on FRAMESKIP_LEVEL setting.
        bool render_this_frame = true;
        const uint32_t pat_idx = (frameskip_pattern_len ? (frame_num % frameskip_pattern_len) : 0u);
        render_this_frame = ((frameskip_pattern_mask >> pat_idx) & 1u) != 0u;

        // Dynamic frameskip: when emulation is too slow, accumulate "overrun"
        // time and skip renders until we've recovered.  This adapts to any
        // TARGET_FRAME_US / frameskip level and prevents ANY perceptible
        // slowdown during heavy scenes (e.g. Contra III beam weapon).
        static int32_t emu_overrun_us = 0;
        if (render_this_frame && emu_overrun_us > 0) {
            render_this_frame = false;
            /* Credit only what skipping a render actually saves. Crediting a
             * whole frame here assumed rendering costs the entire budget,
             * so the loop stopped skipping while still behind and never
             * recovered — emulation stayed below 60 fps and the audio
             * pipeline starved. */
            emu_overrun_us -= (int32_t)g_render_cost_us;
            if (emu_overrun_us < 0) emu_overrun_us = 0;
        }

        /* Video yields to audio. The ring draining means emulation is behind
         * real time; dropping a video frame is the only way to catch up that
         * does not touch emulator state. Audio must never be the thing that
         * gives way — a dropped frame is invisible, a starved chunk is not. */
        if (render_this_frame && q_fill <= (AUDIO_QUEUE_DEPTH / 4))
            render_this_frame = false;

        // Safety: always render at least once every FRAMESKIP_MAX_CONSECUTIVE frames
        if (consecutive_skipped_frames >= FRAMESKIP_MAX_CONSECUTIVE) {
            render_this_frame = true;
        }

        bool skip_render = !render_this_frame;

        IPPU.RenderThisFrame = !skip_render;

        // Poll input early so settings_check_hotkey sees fresh state
        nespad_read();
        ps2kbd_tick();
#ifdef USB_HID_ENABLED
        usbhid_task();
#endif
        poll_host_mouse();

        /* F11 = back to ROM selector. Edge-triggered so holding F11 past
         * the selector return doesn't immediately fire again. */
        {
            uint16_t kbd_state = ps2kbd_get_state();
#ifdef USB_HID_ENABLED
            kbd_state |= usbhid_get_kbd_state();
#endif
            static bool prev_f11 = false;
            bool f11 = (kbd_state & KBD_STATE_F11) != 0;
            if (f11 && !prev_f11) {
                prev_f11 = true;
                return true;  /* signal main loop: back to ROM selector */
            }
            prev_f11 = f11;
        }

        /* Ctrl+Alt+Del = soft-reset the currently loaded ROM (same effect
         * as Settings → Restart Game). Edge-triggered on the chord so one
         * press fires exactly one reset. */
        {
            bool cad = ps2kbd_ctrl_alt_del_pressed() != 0;
#ifdef USB_HID_ENABLED
            cad = cad || usbhid_ctrl_alt_del_pressed() != 0;
#endif
            static bool prev_cad = false;
            if (cad && !prev_cad) {
                S9xSoftReset();
            }
            prev_cad = cad;
        }

        // Check for settings menu hotkey BEFORE emulation runs,
        // so the game never processes buttons on the hotkey frame.
        if (settings_check_hotkey()) {
            hotkey_consumed = true;

            // Tell Core 1 to stop overriding the HDMI buffer
            menu_active = true;
            __dmb();

            // Disable CRT effect for settings menu
            graphics_set_crt_active(false);

            // Use SCREEN[0] for menu drawing, tell HDMI to display it
            graphics_set_buffer(SCREEN[0]);

            settings_result_t sresult = settings_menu_show(SCREEN[0], true);

            if (sresult == SETTINGS_RESULT_ROM_SELECT) {
                // CRT stays off (already disabled above for menu)
                // Re-enable Core 1 before returning
                __dmb();
                menu_active = false;
                return true;
            }

            if (sresult == SETTINGS_RESULT_RESTART) {
                // Soft reset: simulate the SNES Reset button. Keeps the
                // ROM loaded but re-initialises CPU/PPU/APU so gameplay
                // starts from the title screen. CRT/settings get
                // re-applied below along with the normal menu-exit path.
                S9xSoftReset();
            }

            // Wait for all buttons to be released before resuming emulation
            for (int w = 0; w < 60; w++) {
                nespad_read();
                ps2kbd_tick();
#ifdef USB_HID_ENABLED
                usbhid_task();
#endif
                uint32_t pad = nespad_state | nespad_state2;
                uint16_t kbd = ps2kbd_get_state();
#ifdef USB_HID_ENABLED
                kbd |= usbhid_get_kbd_state();
#endif
                if (pad == 0 && kbd == 0) break;
                sleep_ms(16);
            }

            // Apply runtime settings (frameskip, echo, CRT, etc.)
            settings_apply_runtime();

            // Restore emulation: renderer writes to SCREEN[0], HDMI shows SCREEN[!0]=SCREEN[1]
            current_buffer = 0;
#ifndef FRANK_SNES_CPU_CORE_S9X16
            GFX.Screen = SCREEN[0];
            GFX.SubScreen = (g_settings.transparency_enabled && SubScreenBuffer)
                            ? SubScreenBuffer : GFX.Screen;
#endif  /* S9X16 renders into Screen16 always; only the 8-bit side flips. */

            // Restore emulation palette
            S9xFixColourBrightness();
            g_palette_needs_update = false;

            // Clear stale joypad state so the game doesn't see buttons
            // from before the menu on the first frame of resumed emulation
            // (game can read $4218/$4016 before VBlank updates them)
#ifdef FRANK_SNES_CPU_CORE_S9X16
            /* 1.6x keeps pad state in controls.cpp, not in IPPU. */
            for (int j = 0; j < 5; j++)
                s9x16_set_joypad(j, 0);
#else
            for (int j = 0; j < 5; j++)
                IPPU.Joypads[j] = 0;
#endif
            Memory.FillRAM[0x4218] = 0;
            Memory.FillRAM[0x4219] = 0;
            Memory.FillRAM[0x421a] = 0;
            Memory.FillRAM[0x421b] = 0;
            Memory.FillRAM[0x421c] = 0;
            Memory.FillRAM[0x421d] = 0;
            Memory.FillRAM[0x421e] = 0;
            Memory.FillRAM[0x421f] = 0;

            // Re-enable Core 1 buffer management
            __dmb();
            menu_active = false;

            // Resync timing
            next_frame_deadline = time_us_32() + TARGET_FRAME_US;
            audio_acc_us = 0;
            audio_last_us = time_us_32();
            frame_num = 0;
            consecutive_skipped_frames = 0;
            continue;
        }

        { dsp_log_frame++; }

        // Run one SNES frame of emulation.
        uint32_t _diag_t0 = time_us_32();
    #ifdef FRANK_SNES_PROFILE
        uint32_t t0 = _diag_t0;
    #endif
        S9xMainLoop();
        uint32_t _diag_t1 = time_us_32();
    #ifdef FRANK_SNES_PROFILE
        uint32_t t1 = _diag_t1;
    #endif
        /* Feed dynamic frameskip: if this frame exceeded the budget, accumulate overrun */
        {
            uint32_t this_emu_us = _diag_t1 - _diag_t0;
            g_ship_emul_sum += this_emu_us;
            g_ship_emul_n++;
            /* Rolling estimate of what rendering adds to a frame, used above
             * to credit skips honestly. */
            if (!skip_render) {
                if (this_emu_us > g_emu_only_us)
                    g_render_cost_us = this_emu_us - g_emu_only_us;
            } else {
                g_emu_only_us = this_emu_us;
            }
            if (this_emu_us > TARGET_FRAME_US) {
                emu_overrun_us += (int32_t)(this_emu_us - TARGET_FRAME_US);
            } else {
                /* Good frame: drain overrun (but don't go below 0) */
                emu_overrun_us -= (int32_t)(TARGET_FRAME_US - this_emu_us);
                if (emu_overrun_us < 0) emu_overrun_us = 0;
            }
        }

        (void)_diag_t0;
        (void)_diag_t1;

#ifdef FRANK_SNES_PSRAM_CHECK
        if ((frame_num % 60u) == 0u && Memory.ROM && Memory.CalculatedSize) {
            const uint8_t *p = Memory.ROM;
            uint32_t n = Memory.CalculatedSize;
            uint32_t h = 2166136261u;          /* FNV-1a, cheap enough */
            for (uint32_t i = 0; i < n; i++) { h ^= p[i]; h *= 16777619u; }
            psram_crc_last = h;
            if (psram_crc_checks == 0) psram_crc_first = h;
            else if (h != psram_crc_first) psram_crc_errors++;
            psram_crc_checks++;
        }
#endif

#ifdef FRANK_SNES_PPU_CAPTURE
        /* PPU offload: hand this frame's command stream to the link and say
           where the slave's finished picture should land. The capture is
           complete here - S9xEndScreenRefresh, which closes the stream, ran
           inside S9xMainLoop above - and the exchange happens in
           s9x_link_frame() just below, so the stream rides the sound frame's
           doorbell phases instead of buying its own. */
        {
            uint32_t cap_len = 0;
            const uint8_t *cap = ppucap_take(&cap_len);
            /* Straight into SCREEN[current_buffer], which is the buffer the
               display is NOT scanning out (see the double-buffer note above:
               the renderer writes one while HDMI shows the other), so the
               link's RX DMA never touches the live picture.
               Two alternatives were tried and are worse. PSRAM: the RX DMA
               cannot sustain 57 KB into the XIP window and the bulk times out
               - the link went offline with "ppu framebuffer bulk failed" 12
               times in 16 seconds. A separate SRAM buffer: 57 KB does not
               fit, the link script overflows RAM by 9 KB. */
            link_master_ppu_stage(cap, cap_len, SCREEN[current_buffer],
                                  SNES_WIDTH * SNES_HEIGHT);
        }
#endif

#ifdef FRANK_SNES_PPU_CAPTURE
        /* Hand the slave's picture to the display. */
        /* Push the slave's palette into the HDMI driver. Without this the
           master has no palette at all: it stopped rendering, and rendering
           is what used to call graphics_set_palette. */
        if (g_ppu_pal_valid && !g_pal_push_disable) {
            /* 0..BASE_HDMI_CTRL_INX-1 only. 251-254 are the HDMI driver's
               RESERVED control indices: it refuses to write them to the
               hardware palette and maintains a substitution map so that no
               pixel is ever emitted as a TMDS control symbol. Pushing the
               slave's palette over all 256 entries overwrote those reserved
               entries and recomputed the substitute map every frame, and a
               corrupted control symbol does not show a wrong colour - it
               breaks sync, which is why the display reported NO SIGNAL while
               the emulator ran at 51 fps and delivered full framebuffers.
               Measured: identical build with the offload disabled shows the
               picture, so the fault was on this path and not in the board,
               the cable or the power. */
            /* Only entries that actually CHANGED.
             *
             * graphics_set_palette_hdmi encodes straight into the live
             * conv_color TMDS table, and its own comment says that is safe
             * "because S9xFixColourBrightness is called between frames". That
             * was true when the RENDERER drove the palette; the offload calls
             * it from a different point in the main loop, so 251 unconditional
             * writes/frame land in the table while the scanline DMA is reading
             * it. The display was seen losing and regaining lock several times
             * a second with the picture otherwise healthy.
             *
             * A SNES palette changes a handful of entries per frame, so this
             * turns ~251 writes/frame into approximately none. */
            static uint32_t last_pal[251];
            static bool last_pal_valid;
            for (int pi = 0; pi < 251; pi++) {
                if (last_pal_valid && last_pal[pi] == g_ppu_palette[pi])
                    continue;
                last_pal[pi] = g_ppu_palette[pi];
                graphics_set_palette((uint8_t)pi, g_ppu_palette[pi]);
            }
            last_pal_valid = true;
            { extern void graphics_request_palette_update(void);
              graphics_request_palette_update(); }
            g_ppu_pal_valid = false;
        }
#endif

#ifdef C2_SOUND_LINK
        // C2: the mixer lives on the sound slave.  Ship this frame's DSP
        // writes and dirty APU RAM and collect the samples it produced,
        // so the S9xMixSamples* calls below have something to hand out.
        // Must come after the emulated frame and before the first mix.
        s9x_link_frame();

#endif

        // Mix audio on Core 0 (always, even when skipping render), then apply
        // gain/limiting and pack to 32-bit stereo frames.
        static int16_t __attribute__((aligned(32)))
            mix16[AUDIO_FRAME_SAMPLES_MAX * 2];
        const uint32_t frame_samples = AUDIO_FRAME_SAMPLES;
    #ifdef FRANK_SNES_PROFILE
        uint32_t t2 = time_us_32();
    #endif
        {
    #if defined(FRANK_SNES_CPU_CORE_S9X16)
        /* The 1.6x core does not mix on demand - the S-DSP has already
         * written this frame's samples and hands them over. Take what it
         * produced, up to what the packer can hold; short is a real answer
         * and the elastic FIFO downstream absorbs it. Padding the tail with
         * silence would be inventing audio the emulator never generated. */
        {
            int drained = 0;
            const int16_t *src = s9x16_drain_audio(&drained);
            uint32_t want = frame_samples * 2;      /* stereo int16 count */
            if (want > AUDIO_FRAME_SAMPLES_MAX * 2) want = AUDIO_FRAME_SAMPLES_MAX * 2;
            uint32_t have = (drained > 0) ? (uint32_t)drained : 0;
            if (have > want) have = want;
            if (have && src) memcpy(mix16, src, have * sizeof(int16_t));
            if (have < want) memset(mix16 + have, 0, (want - have) * sizeof(int16_t));
        }
    #elif defined(FRANK_SNES_FAST_MODE)
        // FAST MODE: Mix mono only (half the samples), then duplicate to stereo in packing
        S9xMixSamplesMono((void *)mix16, frame_samples);
    #else
        S9xMixSamples((void *)mix16, frame_samples * 2);
    #endif
#ifdef FRANK_SNES_AUDIO_CAPTURE
        /* Emulator-state fingerprint per frame, so divergence from a host
         * render can be located exactly instead of inferred from audio
         * (which matches trivially while everything is silent). */
        if (diag_statecrc && diag_emu_frame < 1800) {
            uint32_t h = 2166136261u;
            const uint8_t *r = DIAG_APU_RAM;
            for (uint32_t i = 0; i < 0x10000; i++) { h ^= r[i]; h *= 16777619u; }
            uint32_t w = 2166136261u;
            const uint8_t *wr = Memory.RAM;
            for (uint32_t i = 0; i < 0x20000; i++) { w ^= wr[i]; w *= 16777619u; }
            volatile uint32_t *o = &diag_statecrc[diag_emu_frame * 8];
            o[0] = h;
            o[1] = w;
            o[2] = ICPU.Registers.PC | ((uint32_t)ICPU.Registers.A.W << 16);
            o[3] = ICPU.Registers.X.W | ((uint32_t)ICPU.Registers.Y.W << 16);
            o[4] = ICPU.Registers.S.W | ((uint32_t)ICPU.Registers.D.W << 16);
            o[5] = ICPU.Registers.PB | ((uint32_t)ICPU.Registers.DB << 8) |
                   ((uint32_t)(ICPU.Registers.P.W & 0xff) << 16) |
                   ((uint32_t)(CPU.Flags & 0xff) << 24);
            o[1] = (uint32_t)CPU.IRQActive | ((uint32_t)CPU.NMIActive << 8) |
                   ((uint32_t)CPU.WaitingForInterrupt << 16) |
                   ((uint32_t)IPPU.HDMA << 24);
            o[6] = ((uint32_t)CPU.V_Counter & 0xffff) |
                   ((uint32_t)CPU.FastROMSpeed << 16) |
                   ((uint32_t)Memory.FillRAM[0x420d] << 24);
            o[7] = (uint32_t)CPU.Cycles;
            if (diag_emu_frame == 41 && diag_apuram_snap)
                memcpy((void *)diag_apuram_snap, Memory.MapInfo,
                       MEMMAP_NUM_BLOCKS * sizeof(SMapInfo));
        }
        diag_emu_frame++;
        /* Raw mixer output, one chunk per emulated frame, before gain and
         * before the FIFO. This is the sound engine on its own, in a form
         * directly comparable with a host render of the same ROM. */
        /* Window start is settable over SWD before emulation begins, so the
         * whole 5 minutes can be covered in successive deterministic runs
         * without rebuilding. */

#endif
        /* Exactly one mix per emulated frame, straight into the FIFO.
         * The whole frame goes in — 534 samples on NTSC, 641 on PAL — and
         * the fixed-size chunks the DAC wants are resampled out of it
         * below. Pushing a fixed 534 here is what threw away a sixth of
         * every PAL frame. */
#ifdef FRANK_SNES_AUDIO_FINGERPRINT
        {
            /* Fingerprint the MIXER's output, not the DAC's. Everything
             * downstream of here is paced by the wall clock, so hashing
             * delivered frames cannot tell a non-deterministic emulator
             * from a merely adaptive resampler. This can. */
            uint32_t h = audio_hash;
            const uint8_t *hb = (const uint8_t *)mix16;
            for (uint32_t k = 0; k < frame_samples * sizeof(int16_t) * AUDIO_CH; k++)
                { h ^= hb[k]; h *= 16777619u; }
            audio_hash = h;
            if (++audio_hash_chunks >= 50u) {
                audio_hash_chunks = 0;
                if (audio_hash_n < AUDIO_HASH_LOG)
                    audio_hash_log[audio_hash_n++] = h;
            }
        }
#endif
        {
            int32_t peak = 0;
            for (uint32_t k = 0; k < frame_samples * AUDIO_CH; k++) {
                int32_t v = mix16[k]; if (v < 0) v = -v;
                if (v > peak) peak = v;
            }
            if (peak > mix_peak_max) mix_peak_max = peak;
            int32_t post = (peak * (g_settings.volume * 4)) / 100;
            if (post > 24000) mix_limit_frames++;
            if (post > 32767) mix_clip_frames++;
            mix_frames++;
            mix_gain_num = g_settings.volume * 4;
            mix_gain_den = 100;
        }
        sfifo_push(mix16, frame_samples);
        }
    #ifdef FRANK_SNES_PROFILE
        uint32_t t3 = time_us_32();
    #endif

        // Mixer attenuates by >>11 (÷2048) to prevent hard clipping.
        // Boost with soft limiter to restore volume, scaled by volume setting.
        /*
         * The two sound cores hand over audio at very different levels.
         * soundux attenuates by VOL_DIV16*8 and needs boosting; blargg's
         * S-DSP outputs near full scale and must not be. Applying the same
         * gain to both drove the DSP path 61000 past full scale and into
         * the limiter on hundreds of frames.
         */
#ifdef SOUND_CORE_DSP
        const int gain_num = g_settings.volume;
#else
        const int gain_num = g_settings.volume * 4;
#endif
        const int gain_den = 100;
        const bool use_soft_limiter = true;

        /* Rate matching.
         *
         * Emulation produces one emulated frame of audio — AUDIO_FRAME_SAMPLES,
         * which is region-dependent — and the DAC
         * consumes at its own crystal-derived rate, which is never exactly
         * 60 chunks per second. The two must be reconciled without ever
         * creating audio (mixing ahead of emulated time corrupts the DSP and
         * duplicates samples) or destroying it (discarding a mixed chunk
         * punches a hole in the waveform).
         *
         * So the FIFO is drained by a resampler whose ratio is servoed on
         * ring depth — the one signal that reflects the DAC's true rate.
         * Ring filling means the DAC is slower than we assumed, so each
         * output chunk consumes slightly more input; ring draining means the
         * opposite. Audio is neither created nor destroyed, only stretched
         * by a fraction of a percent, which is inaudible.
         */
        for (;;) {
            uint32_t prod = audio_prod_seq;
            uint32_t cons = audio_cons_seq;
            uint32_t depth = prod - cons;
            if (depth >= AUDIO_QUEUE_DEPTH)
                break;                       /* ring full: leave it in the FIFO */

            /*
             * Rate servo.
             *
             * The error signal is the TOTAL buffered audio in frames — the
             * elastic FIFO plus what is already packed in the ring — not the
             * ring depth in chunks. Chunk depth has eight levels, so a servo
             * driven by it moves the playback rate in 12.5% steps: ordinary
             * frame jitter of one or two chunks swung the rate by 12-25% and
             * pinned it against both clamps several times a second. The
             * actual correction required is 32050 Hz produced against 32040
             * consumed — 0.03%. That swing was heard as samples cutting and
             * warbling, and because it is driven by the wall clock it never
             * reproduced the same way twice.
             *
             * Frames give ~4700 levels instead of 8, so a gentle gain can
             * hold the buffer near half full and still cover a genuinely
             * slow producer. Full deflection is 6%, an order of magnitude
             * more than the steady-state error and inaudible.
             */
            int32_t buffered = (int32_t)sfifo_fill +
                               (int32_t)(depth * AUDIO_BUFFER_LENGTH);
            int32_t target   = (int32_t)(SFIFO_FRAMES +
                               AUDIO_QUEUE_DEPTH * AUDIO_BUFFER_LENGTH) / 2;
            int32_t err      = buffered - target;

            /*
             * Nominal rate first, servo second.
             *
             * The producer's rate is known: frame_samples per emulated
             * frame, and the frame period is known too. On a game whose
             * H_Max is stretched by a timing hack (MK3 runs at 110%) that
             * is 705 samples every 20 ms = 35250/s against a 32040 Hz DAC —
             * a 10% mismatch, well outside the servo's 6% authority, so
             * leaving it to the servo just overflows the FIFO. Feed it
             * forward and let the servo trim what is left.
             */
            uint32_t produced = (uint32_t)(((uint64_t)frame_samples * 1000000u)
                                           / (uint32_t)TARGET_FRAME_US);
            int32_t ratio_nom = (int32_t)(((uint64_t)produced << 16)
                                          / AUDIO_SAMPLE_RATE);
            int32_t ratio    = ratio_nom +
                (int32_t)(((int64_t)err * ratio_nom) * 6 / (100 * (int64_t)target));
            /* Authority has to cover the whole rate range the pipeline can
             * see: DAC crystal error (well under 1%) plus emulation running
             * below 60 fps under load. At 53 fps the producer is 12% short,
             * so the stretch limit is set there — beyond that the audio
             * would starve and click instead. */
            /* Authority still has to cover a producer that is genuinely
             * slow, but the servo now reaches these limits only when the
             * emulator really cannot keep up, not on jitter. */
            /* Authority is relative to the nominal rate, not to 1.0. */
            if (ratio < (ratio_nom * 88) / 100) ratio = (ratio_nom * 88) / 100;
            if (ratio > (ratio_nom * 113) / 100) ratio = (ratio_nom * 113) / 100;
            if (ratio < ratio_min) ratio_min = ratio;
            if (ratio > ratio_max) ratio_max = ratio;

            /* Diagnostics for the stretch authority. ratio_floor_hits
             * counts chunks where the servo wanted to stretch further than
             * it is allowed to: past that point the ring drains and the
             * pull holds samples instead of stretching, which is a hole
             * rather than slow sound. The limit was tuned for 53 fps
             * against a 60 fps target, i.e. 88% of nominal — on PAL that
             * same fraction is 44 fps. */
            if (ratio <= 57700) ratio_floor_hits++;
            if (ratio >= 74000) ratio_ceil_hits++;

            uint32_t need = ((AUDIO_BUFFER_LENGTH * (uint32_t)ratio) >> 16) + 2;
            if (sfifo_fill < need) {
                sfifo_short++;
                break;                       /* not a chunk's worth yet */
            }

            sfifo_pull(mix16, AUDIO_BUFFER_LENGTH, ratio);
            uint32_t *dst32 = audio_packed_buffer[prod % AUDIO_QUEUE_DEPTH];
#ifdef FRANK_SNES_FAST_MODE
            audio_pack_mono_to_stereo(dst32, mix16, AUDIO_BUFFER_LENGTH,
                                      gain_num, gain_den, use_soft_limiter);
#else
            audio_pack_opt(dst32, mix16, AUDIO_BUFFER_LENGTH,
                           gain_num, gain_den, use_soft_limiter);
#endif
#ifdef FRANK_SNES_AUDIO_CAPTURE
            /* FINAL output: post gain, post soft-limiter, post resampler —
             * exactly the stereo frames handed to I2S. */
            if (audio_cap_arm && audio_cap_buf && diag_emu_frame >= audio_cap_start &&
                audio_cap_len < audio_cap_max) {
                memcpy((void *)&audio_cap_buf[audio_cap_len * AUDIO_BUFFER_LENGTH],
                       dst32, AUDIO_BUFFER_LENGTH * sizeof(uint32_t));
                if (++audio_cap_len >= audio_cap_max) audio_cap_arm = 0;
            }
#endif
#ifdef FRANK_SNES_AUDIO_FINGERPRINT
            {
                uint32_t h = dac_hash;
                const uint8_t *hb = (const uint8_t *)dst32;
                for (uint32_t k = 0; k < AUDIO_BUFFER_LENGTH * sizeof(uint32_t); k++)
                    { h ^= hb[k]; h *= 16777619u; }
                dac_hash = h;
                if (++dac_hash_chunks >= 60u) {
                    dac_hash_chunks = 0;
                    if (dac_hash_n < AUDIO_HASH_LOG)
                        dac_hash_log[dac_hash_n++] = h;
                }
            }
#endif
            __dmb();
            audio_prod_seq = prod + 1;
            __dmb();
#ifdef FRANK_SNES_HDMI_ALT
            hdmi_alt_audio_write((const int16_t *)dst32, AUDIO_BUFFER_LENGTH);
            __dmb();
            audio_cons_seq = prod + 1;
            __dmb();
#endif
        }


        if (skip_render) {
            consecutive_skipped_frames++;
        } else {
            consecutive_skipped_frames = 0;

            // Swap display buffers only when we rendered
#ifdef FRANK_SNES_CPU_CORE_S9X16
            /* The core drew into Screen16; narrow it into the buffer the
             * driver is about to start showing, then flip. Same ordering as
             * the 1.43 path - the freshly filled buffer is the one that
             * goes on screen. */
            screen16_to_indexed(SCREEN[current_buffer]);
            current_buffer = !current_buffer;
#else
            current_buffer = !current_buffer;
            GFX.Screen = SCREEN[current_buffer];
            GFX.SubScreen = (g_settings.transparency_enabled && SubScreenBuffer)
                            ? SubScreenBuffer : GFX.Screen;
#endif
        }

        // Update palette if brightness changed during frame. Skip when
        // PPU.Brightness is 0 — at end-of-frame, games often sit at
        // force-blank/brt=0 while doing VRAM/CGRAM uploads, and pushing
        // the palette with brightness 0 blacks out the HDMI frame we
        // just displayed (Cybernator regression). The $2100 write
        // handler pushes eagerly on non-zero brightness.
        if (g_palette_needs_update && PPU.Brightness != 0) {
#ifdef FRANK_SNES_CPU_CORE_S9X16
            /* IPPU.Red/Green/Blue are already current - the core rebuilt
             * them when brightness or CGRAM changed. Only the push to the
             * driver is deferred to here, which is a frame boundary. */
            S9xPushPaletteToDisplay();
#else
            S9xFixColourBrightness();
#endif
            g_palette_needs_update = false;
        }

        /*
         * Advance the deadline for the next emulated frame — and slave that
         * period to the audio buffer.
         *
         * The DAC is a fixed-rate sink, so a producer/consumer mismatch has
         * to be absorbed somewhere: by buffer (finite) or by changing the
         * playback rate (audible). Correcting the *producer* removes the
         * mismatch instead of hiding it — the emulator's average frame rate
         * becomes exactly the DAC's rate, the buffer sits at its target, and
         * the resampler downstream is left with nothing to do.
         *
         * Deficit shortens the frame period, surplus lengthens it. The
         * adjustment is bounded at 1/8 of a frame so a stopped DAC (menu,
         * paused audio) can only slow emulation slightly rather than stall
         * it, and so this can never fight the frameskip logic.
         *
         * If the emulator is genuinely CPU-bound this does nothing — it is
         * already running flat out — and the resampler takes over and
         * stretches, which is the honest outcome: slow sound, not holes.
         */
        {
            uint32_t d = audio_prod_seq - audio_cons_seq;
            int32_t buffered = (int32_t)sfifo_fill +
                               (int32_t)(d * AUDIO_BUFFER_LENGTH);
            int32_t target   = (int32_t)(SFIFO_FRAMES +
                               AUDIO_QUEUE_DEPTH * AUDIO_BUFFER_LENGTH) / 2;
            int32_t err      = buffered - target;      /* frames of audio */

            /* One frame period per `target` frames of error: a full swing
             * of the buffer moves the period by one frame, so the loop is
             * critically slow rather than twitchy. */
            int32_t adj = (int32_t)(((int64_t)err * (int32_t)TARGET_FRAME_US)
                                    / target);

            /* Authority is deliberately small — this only has to trim
             * drift between two clocks that are already within 0.03% of
             * each other. A wide limit lets the loop demand a frame period
             * the emulator cannot meet, which pins it permanently "late"
             * and drives constant frameskip. 1/32 of a frame is ~3%. */
            int32_t lim = (int32_t)TARGET_FRAME_US / 32;
            if (adj >  lim) adj =  lim;
            if (adj < -lim) adj = -lim;

            /* And never shorten the period when the emulator has just
             * missed the one it had: it is CPU-bound, not mistimed, and
             * asking for more only starves the video. The resampler
             * absorbs that case as stretch, which is the honest outcome. */
            if (adj < 0 && late_us > 0) adj = 0;

            if (adj < pace_adj_min) pace_adj_min = adj;
            if (adj > pace_adj_max) pace_adj_max = adj;

            next_frame_deadline += (uint32_t)((int32_t)TARGET_FRAME_US + adj);
        }
        frame_num++;

#ifdef FRANK_SNES_SUPERFX_DIAG
        {
            extern volatile uint32_t g_gsu_call_count;
            extern volatile uint32_t g_gsu_inst_count;
            extern volatile uint32_t g_gsu_plot_count;
            extern volatile uint32_t g_gsu_start_fail;
            extern volatile uint32_t g_gsu_stop_count;
            extern volatile uint32_t g_gsu_last_scmr;
            extern volatile uint32_t g_gsu_last_scbr;
            extern volatile uint32_t g_gsu_last_mode;
            extern volatile uint32_t g_gsu_plot_xmin;
            extern volatile uint32_t g_gsu_plot_xmax;
            extern volatile uint32_t g_gsu_plot_ymin;
            extern volatile uint32_t g_gsu_plot_ymax;
            extern volatile uint32_t g_gsu_plot_colors;
            static uint32_t diag_last_us = 0;
            uint32_t diag_now = time_us_32();
            if ((uint32_t)(diag_now - diag_last_us) >= 1000000u) {
                uint32_t sfr = (uint32_t)Memory.FillRAM[0x3030]
                             | ((uint32_t)Memory.FillRAM[0x3031] << 8);
                uint32_t pbr = Memory.FillRAM[0x3034];
                uint8_t r4300 = Memory.FillRAM[0x4300];
                uint8_t r4301 = Memory.FillRAM[0x4301];
                uint16_t r4302 = Memory.FillRAM[0x4302]
                               | (Memory.FillRAM[0x4303] << 8);
                uint8_t r4304 = Memory.FillRAM[0x4304];
                uint16_t r4305 = Memory.FillRAM[0x4305]
                               | (Memory.FillRAM[0x4306] << 8);
                LOG("[gsu] calls=%lu plot=%lu stop=%lu fail=%lu "
                    "sfr=%04lx pbr=%02lx scmr=%02lx scbr=%02lx mode=%lu "
                    "| x=[%lu..%lu] y=[%lu..%lu] colors=%08lx "
                    "| dma0 ctrl=%02x bbus=%02x abus=%04x abnk=%02x cnt=%04x "
                    "| brightness=%u fb=%u\n",
                    (unsigned long)g_gsu_call_count,
                    (unsigned long)g_gsu_plot_count,
                    (unsigned long)g_gsu_stop_count,
                    (unsigned long)g_gsu_start_fail,
                    (unsigned long)sfr, (unsigned long)pbr,
                    (unsigned long)g_gsu_last_scmr,
                    (unsigned long)g_gsu_last_scbr,
                    (unsigned long)g_gsu_last_mode,
                    (unsigned long)g_gsu_plot_xmin, (unsigned long)g_gsu_plot_xmax,
                    (unsigned long)g_gsu_plot_ymin, (unsigned long)g_gsu_plot_ymax,
                    (unsigned long)g_gsu_plot_colors,
                    (unsigned)r4300, (unsigned)r4301, (unsigned)r4302,
                    (unsigned)r4304, (unsigned)r4305,
                    (unsigned)PPU.Brightness,
                    (unsigned)PPU.ForcedBlanking);
                g_gsu_call_count = 0;
                g_gsu_inst_count = 0;
                g_gsu_plot_count = 0;
                g_gsu_start_fail = 0;
                g_gsu_stop_count = 0;
                g_gsu_plot_xmin = 255;
                g_gsu_plot_xmax = 0;
                g_gsu_plot_ymin = 255;
                g_gsu_plot_ymax = 0;
                g_gsu_plot_colors = 0;
                diag_last_us = diag_now;
            }
        }
#endif

        {
            /* Always-on frame counter for the debug probe. One compare and an
               increment per frame; the once-a-second store is negligible.
               This board has no console and cannot be halted, so reading this
               out of RAM over SWD is the only way to see a frame rate. */
            static uint32_t tel_last_us = 0;
            static uint32_t tel_frames  = 0;
            uint32_t tel_now = time_us_32();

            tel_frames++;
            if ((uint32_t)(tel_now - tel_last_us) >= 1000000u) {
                extern volatile uint32_t frank_instr_count, frank_event_count;
                frank_telemetry.seq++;
                frank_telemetry.emu_fps = tel_frames;
                frank_telemetry.instr_per_frame  = frank_instr_count / (tel_frames ? tel_frames : 1);
                /* Divide by FRAMES, not by call count: S9xMainLoop returns on
                   SCAN_KEYS and can be entered more than once per video frame,
                   so sum/calls is a per-call average and reads far too low -
                   it made event_us look larger than total emulation. */
                frank_telemetry.ship_emul_us = (uint32_t)(g_ship_emul_sum /
                                                   (tel_frames ? tel_frames : 1));
                frank_telemetry.ship_mainloop_calls =
                    (uint32_t)((g_ship_emul_n * 100u) / (tel_frames ? tel_frames : 1));
                frank_telemetry.ship_emu_only_us    = g_emu_only_us;
                frank_telemetry.ship_render_cost_us = g_render_cost_us;
                { extern volatile uint32_t frank_upd_us, frank_rs_us, frank_rs_calls;
                  uint32_t d = tel_frames ? tel_frames : 1;
                  frank_telemetry.ship_upd_us   = frank_upd_us / d;
                  frank_telemetry.ship_rs_us    = frank_rs_us / d;
                  frank_telemetry.ship_rs_calls = (frank_rs_calls * 100u) / d;
                  { extern volatile uint32_t frank_rs_sub_us, frank_rs_sub_calls;
                    frank_telemetry.ship_rs_sub_us    = frank_rs_sub_us / d;
                    frank_telemetry.ship_rs_sub_calls = (frank_rs_sub_calls * 100u) / d;
                    frank_rs_sub_us = frank_rs_sub_calls = 0; }
                  { extern volatile uint32_t frank_tile_calls, frank_tile_lines;
                    frank_telemetry.ship_tile_calls = frank_tile_calls / d;
                    frank_telemetry.ship_tile_lines = frank_tile_lines / d;
                    frank_tile_calls = frank_tile_lines = 0; }
                  { extern volatile uint32_t frank_norender;
                    frank_telemetry.ship_norender = frank_norender; }
#ifdef FRANK_SNES_PPU_CAPTURE
                  { extern volatile uint32_t frank_cap_bytes, frank_cap_on,
                                             frank_cap_sum, frank_cap_min,
                                             frank_cap_max, frank_cap_takes;
                    extern volatile uint32_t g_ppu_sent_len, g_ppu_sends;
                    frank_telemetry.ship_cap_bytes = frank_cap_bytes;
                    frank_telemetry.ship_cap_on    = frank_cap_on;
                    frank_telemetry.ship_cap_sum   = frank_cap_sum;
                    frank_telemetry.cap_min      = frank_cap_min;
                    frank_telemetry.cap_max      = frank_cap_max;
                    frank_telemetry.ppu_takes    = frank_cap_takes;
                    frank_telemetry.ppu_sends    = g_ppu_sends;
                    frank_telemetry.ppu_sent_len = g_ppu_sent_len;
                    /* Per sample window, not cumulative: a lifetime min/max
                       stops moving and stops being evidence. */
                    frank_cap_min   = 0xffffffffu;
                    frank_cap_max   = 0;
                    frank_cap_takes = 0;
                    g_ppu_sends     = 0;
                    { extern volatile uint32_t frank_cap_vram_w,
                                               frank_cap_cgram_w,
                                               frank_cap_oam_w;
                      frank_telemetry.cap_vram_w  = frank_cap_vram_w;
                      frank_telemetry.cap_cgram_w = frank_cap_cgram_w;
                      frank_telemetry.cap_oam_w   = frank_cap_oam_w; }
                    if (Memory.FillRAM)
                      frank_telemetry.master_regs =
                            (uint32_t)Memory.FillRAM[0x2100]
                          | ((uint32_t)Memory.FillRAM[0x2105] << 8)
                          | ((uint32_t)Memory.FillRAM[0x212c] << 16)
                          | ((uint32_t)Memory.FillRAM[0x212d] << 24); }
                  { uint32_t ex = 0, fa = 0, lu = 0;
                    link_master_get_stats(&ex, &fa, &lu);
                    frank_telemetry.ship_link_us   = lu;
                    frank_telemetry.ship_link_fail = fa;
                    { extern volatile const char *frank_link_why;
                      frank_telemetry.ship_link_why = (uint32_t)frank_link_why; }
                    { extern volatile link_ppu_stat_t g_ppu_stat;
                      frank_telemetry.slave_render_us  = g_ppu_stat.render_us;
                      frank_telemetry.slave_records    = g_ppu_stat.records;
                      frank_telemetry.slave_oversize   = g_ppu_stat.oversize;
                      frank_telemetry.slave_psram_ok   = g_ppu_stat.psram_ok;
                      frank_telemetry.slave_impossible = g_ppu_stat.impossible;
                      frank_telemetry.slave_want = g_ppu_stat.want;
                      frank_telemetry.slave_pitch_h = g_ppu_stat.dbg_pitch_h;
                      frank_telemetry.slave_flags = g_ppu_stat.dbg_flags;
                      frank_telemetry.slave_stream_len = g_ppu_stat.stream_len;
                      frank_telemetry.slave_stream_sum = g_ppu_stat.stream_sum;
                      frank_telemetry.slave_stop_off   = g_ppu_stat.stop_off;
                      frank_telemetry.slave_stop_ctx   = g_ppu_stat.stop_ctx;
                      frank_telemetry.slave_stop_why   = g_ppu_stat.stop_why;
                      frank_telemetry.ppu_sum_ok  = g_ppu_stat.sum_ok;
                      frank_telemetry.ppu_sum_bad = g_ppu_stat.sum_bad;
                      frank_telemetry.ppu_exp_sum = g_ppu_stat.exp_sum;
                      frank_telemetry.ppu_fb_got = link_master_ppu_got();
                      { /* Is the picture actually a picture? A correct-sized
                           buffer of zeros looks identical to success in every
                           counter measured so far. */
                        const uint8_t *fb = SCREEN[current_buffer];
                        uint32_t h = 2166136261u, nz = 0;
                        for (uint32_t q = 0; q < SNES_WIDTH * SNES_HEIGHT; q += 7) {
                            h ^= fb[q]; h *= 16777619u;
                            if (fb[q]) nz++;
                        }
                        frank_telemetry.fb_hash = h;
                        frank_telemetry.fb_nonzero = nz;
                        frank_telemetry.pal0 = g_ppu_palette[1];
                        frank_telemetry.pal1 = g_ppu_palette[17]; } }
                    { extern volatile uint32_t frank_cap_overflow;
                      frank_telemetry.ship_cap_overflow = frank_cap_overflow; }
                    { extern volatile uint32_t g_ph_sound, g_ph_ppu_tx, g_ph_ack, g_ph_fb;
                      frank_telemetry.ph_sound  = g_ph_sound;
                      frank_telemetry.ph_ppu_tx = g_ph_ppu_tx;
                      frank_telemetry.ph_ack    = g_ph_ack;
                      frank_telemetry.ph_fb     = g_ph_fb;
                      { extern volatile uint32_t g_ph_ev, g_ph_aram;
                        frank_telemetry.ph_ev = g_ph_ev;
                        frank_telemetry.ph_aram = g_ph_aram; } } }
#endif
                  frank_upd_us = frank_rs_us = frank_rs_calls = 0; }
                g_ship_emul_sum = 0; g_ship_emul_n = 0;
                frank_telemetry.events_per_frame = frank_event_count / (tel_frames ? tel_frames : 1);
                { extern volatile uint32_t frank_event_us;
                  frank_telemetry.event_us_per_frame = frank_event_us / (tel_frames ? tel_frames : 1);
                  frank_event_us = 0; }
                { extern volatile uint32_t frank_ev_us_by_type[8], frank_ev_n_by_type[8];
                  uint32_t d = tel_frames ? tel_frames : 1;
                  for (int q = 0; q < 7; q++) {
                      frank_telemetry.ev_us_type[q] = frank_ev_us_by_type[q] / d;
                      frank_telemetry.ev_n_type[q]  = frank_ev_n_by_type[q] / d;
                      frank_ev_us_by_type[q] = 0; frank_ev_n_by_type[q] = 0;
                  } }
                { extern volatile uint32_t frank_hdma_bytes, frank_hdma_chan;
                  uint32_t d = tel_frames ? tel_frames : 1;
                  frank_telemetry.hdma_bytes_per_frame = frank_hdma_bytes / d;
                  frank_telemetry.hdma_chan_per_frame  = frank_hdma_chan / d;
                  frank_hdma_bytes = 0; frank_hdma_chan = 0; }
                { extern volatile uint32_t frank_hdma_calls, frank_getmemptr;
                  uint32_t d = tel_frames ? tel_frames : 1;
                  frank_telemetry.hdma_calls_per_frame = frank_hdma_calls / d;
                  frank_telemetry.getmemptr_per_frame  = frank_getmemptr / d;
                  frank_hdma_calls = 0; frank_getmemptr = 0; }
                frank_instr_count = 0;
                frank_event_count = 0;
                frank_telemetry.seq++;
                tel_frames  = 0;
                tel_last_us = tel_now;
            }
        }

#ifdef FRANK_SNES_PROFILE
        // Update stats (keep overhead tiny; print at most once/sec)
        uint32_t now_us = time_us_32();
        uint32_t emul_us = (uint32_t)(t1 - t0);
        g_perf.frames++;
        if (skip_render) g_perf.skipped++; else g_perf.rendered++;

        g_perf.sum_emul_us += emul_us;
        if (skip_render) {
            g_perf.sum_emul_skip_us += emul_us;
            g_perf.frames_skip++;
            perf_max_u32(&g_perf.max_emul_skip_us, emul_us);
        } else {
            g_perf.sum_emul_render_us += emul_us;
            g_perf.frames_render++;
            perf_max_u32(&g_perf.max_emul_render_us, emul_us);
        }
        /* t4/t5 bracketed the audio pack stage; those markers were removed at
           some point but the accounting below still referenced them, so this
           whole block stopped compiling. Report pack as zero rather than
           attribute garbage to it - the mix and emul figures are the ones
           that matter here. */
        uint32_t t4 = t3, t5 = t3;
        g_perf.sum_mix_us  += (uint32_t)(t3 - t2);
        g_perf.sum_pack_us += (uint32_t)(t5 - t4);
        perf_max_u32(&g_perf.max_emul_us, emul_us);
        perf_max_u32(&g_perf.max_mix_us,  (uint32_t)(t3 - t2));
        perf_max_u32(&g_perf.max_pack_us, (uint32_t)(t5 - t4));
        if (late_us > g_perf.max_late_us) g_perf.max_late_us = late_us;
        perf_min_u32(&g_perf.min_q_fill, q_fill);
        perf_max_u32(&g_perf.max_q_fill, q_fill);

        if ((uint32_t)(now_us - g_perf.last_report_us) >= 1000000u) {
            uint32_t frames = g_perf.frames ? g_perf.frames : 1;
            uint32_t avg_emul = (uint32_t)(g_perf.sum_emul_us / frames);
            uint32_t fr_r = g_perf.frames_render ? g_perf.frames_render : 1;
            uint32_t fr_s = g_perf.frames_skip ? g_perf.frames_skip : 1;
            uint32_t avg_emul_r = (uint32_t)(g_perf.sum_emul_render_us / fr_r);
            uint32_t avg_emul_s = (uint32_t)(g_perf.sum_emul_skip_us / fr_s);
            uint32_t avg_mix  = (uint32_t)(g_perf.sum_mix_us / frames);
            uint32_t avg_pack = (uint32_t)(g_perf.sum_pack_us / frames);

            uint64_t upd_sum = 0;
            uint32_t upd_max = 0;
            uint32_t upd_cnt = 0;
            frank_snes_prof_take_update_screen(&upd_sum, &upd_max, &upd_cnt);
            uint32_t upd_avg = (upd_cnt ? (uint32_t)(upd_sum / upd_cnt) : 0);

            uint64_t uz_sum = 0;
            uint32_t uz_max = 0;
            uint32_t uz_cnt = 0;
            frank_snes_prof_take_upd_zclear(&uz_sum, &uz_max, &uz_cnt);
            uint32_t uz_avg = (uz_cnt ? (uint32_t)(uz_sum / uz_cnt) : 0);

            uint64_t usub_sum = 0;
            uint32_t usub_max = 0;
            uint32_t usub_cnt = 0;
            frank_snes_prof_take_upd_render_sub(&usub_sum, &usub_max, &usub_cnt);
            uint32_t usub_avg = (usub_cnt ? (uint32_t)(usub_sum / usub_cnt) : 0);

            uint64_t umain_sum = 0;
            uint32_t umain_max = 0;
            uint32_t umain_cnt = 0;
            frank_snes_prof_take_upd_render_main(&umain_sum, &umain_max, &umain_cnt);
            uint32_t umain_avg = (umain_cnt ? (uint32_t)(umain_sum / umain_cnt) : 0);

            uint64_t ucm_sum = 0;
            uint32_t ucm_max = 0;
            uint32_t ucm_cnt = 0;
            frank_snes_prof_take_upd_colormath(&ucm_sum, &ucm_max, &ucm_cnt);
            uint32_t ucm_avg = (ucm_cnt ? (uint32_t)(ucm_sum / ucm_cnt) : 0);

            uint64_t ubd_sum = 0;
            uint32_t ubd_max = 0;
            uint32_t ubd_cnt = 0;
            frank_snes_prof_take_upd_backdrop(&ubd_sum, &ubd_max, &ubd_cnt);
            uint32_t ubd_avg = (ubd_cnt ? (uint32_t)(ubd_sum / ubd_cnt) : 0);

            uint64_t usc_sum = 0;
            uint32_t usc_max = 0;
            uint32_t usc_cnt = 0;
            frank_snes_prof_take_upd_scale(&usc_sum, &usc_max, &usc_cnt);
            uint32_t usc_avg = (usc_cnt ? (uint32_t)(usc_sum / usc_cnt) : 0);

            uint64_t rs_sum = 0;
            uint32_t rs_max = 0;
            uint32_t rs_cnt = 0;
            frank_snes_prof_take_render_screen(&rs_sum, &rs_max, &rs_cnt);
            uint32_t rs_avg = (rs_cnt ? (uint32_t)(rs_sum / rs_cnt) : 0);

            uint64_t ro_sum = 0;
            uint32_t ro_max = 0;
            uint32_t ro_cnt = 0;
            frank_snes_prof_take_rs_obj(&ro_sum, &ro_max, &ro_cnt);
            uint32_t ro_avg = (ro_cnt ? (uint32_t)(ro_sum / ro_cnt) : 0);

            uint64_t r0_sum = 0;
            uint32_t r0_max = 0;
            uint32_t r0_cnt = 0;
            frank_snes_prof_take_rs_bg0(&r0_sum, &r0_max, &r0_cnt);
            uint32_t r0_avg = (r0_cnt ? (uint32_t)(r0_sum / r0_cnt) : 0);

            uint64_t r1_sum = 0;
            uint32_t r1_max = 0;
            uint32_t r1_cnt = 0;
            frank_snes_prof_take_rs_bg1(&r1_sum, &r1_max, &r1_cnt);
            uint32_t r1_avg = (r1_cnt ? (uint32_t)(r1_sum / r1_cnt) : 0);

            uint64_t r2_sum = 0;
            uint32_t r2_max = 0;
            uint32_t r2_cnt = 0;
            frank_snes_prof_take_rs_bg2(&r2_sum, &r2_max, &r2_cnt);
            uint32_t r2_avg = (r2_cnt ? (uint32_t)(r2_sum / r2_cnt) : 0);

            uint64_t r3_sum = 0;
            uint32_t r3_max = 0;
            uint32_t r3_cnt = 0;
            frank_snes_prof_take_rs_bg3(&r3_sum, &r3_max, &r3_cnt);
            uint32_t r3_avg = (r3_cnt ? (uint32_t)(r3_sum / r3_cnt) : 0);

            uint64_t r7_sum = 0;
            uint32_t r7_max = 0;
            uint32_t r7_cnt = 0;
            frank_snes_prof_take_rs_mode7(&r7_sum, &r7_max, &r7_cnt);
            uint32_t r7_avg = (r7_cnt ? (uint32_t)(r7_sum / r7_cnt) : 0);

            uint64_t tc_sum = 0;
            uint32_t tc_max = 0;
            uint32_t tc_cnt = 0;
            frank_snes_prof_take_tile_convert(&tc_sum, &tc_max, &tc_cnt);

            // Snapshot a few PPU regs for correlation (cheap: 1 load each)
            uint8_t ppu_bgm = (uint8_t)PPU.BGMode;
            uint8_t r2106 = Memory.FillRAM[0x2106];
            uint8_t r2107 = Memory.FillRAM[0x2107];
            uint8_t r2108 = Memory.FillRAM[0x2108];
            uint8_t r2109 = Memory.FillRAM[0x2109];
            uint8_t r210a = Memory.FillRAM[0x210a];
            uint8_t r210b = Memory.FillRAM[0x210b];
            uint8_t r210c = Memory.FillRAM[0x210c];

            uint8_t r2123 = Memory.FillRAM[0x2123];
            uint8_t r2124 = Memory.FillRAM[0x2124];
            uint8_t r2125 = Memory.FillRAM[0x2125];
            uint8_t r2126 = Memory.FillRAM[0x2126];
            uint8_t r2127 = Memory.FillRAM[0x2127];
            uint8_t r2128 = Memory.FillRAM[0x2128];
            uint8_t r2129 = Memory.FillRAM[0x2129];
            uint8_t r212a = Memory.FillRAM[0x212a];
            uint8_t r212b = Memory.FillRAM[0x212b];

            uint8_t r212c = Memory.FillRAM[0x212c];
            uint8_t r212d = Memory.FillRAM[0x212d];
            uint8_t r212e = Memory.FillRAM[0x212e];
            uint8_t r212f = Memory.FillRAM[0x212f];
            uint8_t r2130 = Memory.FillRAM[0x2130];
            uint8_t r2131 = Memory.FillRAM[0x2131];
            uint8_t r2133 = Memory.FillRAM[0x2133];

            frank_telemetry.seq++;              /* odd: write in progress */
            frank_telemetry.emu_fps      = frames;
            frank_telemetry.rend_fps     = g_perf.rendered;
            frank_telemetry.skip_fps     = g_perf.skipped;
            frank_telemetry.avg_emul_us  = avg_emul;
            frank_telemetry.max_emul_us  = g_perf.max_emul_us;
            frank_telemetry.avg_mix_us   = avg_mix;
            frank_telemetry.avg_pack_us  = avg_pack;
            frank_telemetry.avg_upd_us   = upd_avg;
            frank_telemetry.avg_rs_us    = rs_avg;
            frank_telemetry.avg_zclear_us    = uz_avg;
            frank_telemetry.avg_colormath_us = ucm_avg;
            frank_telemetry.avg_scale_us     = usc_avg;
            frank_telemetry.avg_obj_us       = ro_avg;
            frank_telemetry.avg_bg0_us       = r0_avg;
            /* static, not automatic: this core's stack is 2 KB and this
               function already holds a dozen 64-bit accumulators. 120 bytes
               of extra locals here is not worth the risk. */
            { static uint64_t _s[15];
              _s[0]=upd_sum;  _s[1]=rs_sum;   _s[2]=ro_sum;   _s[3]=r0_sum;
              _s[4]=r1_sum;   _s[5]=r2_sum;   _s[6]=r3_sum;   _s[7]=r7_sum;
              _s[8]=uz_sum;   _s[9]=usub_sum; _s[10]=umain_sum; _s[11]=ucm_sum;
              _s[12]=ubd_sum; _s[13]=usc_sum; _s[14]=tc_sum;
              for (int _i = 0; _i < 15; _i++)
                  frank_telemetry.rend_us[_i] = (uint32_t)(_s[_i] / frames); }
            frank_telemetry.seq++;              /* even: sample is coherent */

            /* The [perf] line is ~1000 characters to a 115200 UART: about
               87 ms of blocking transmit every second, which is far more
               disruptive than the thing being measured. PROF_QUIET keeps the
               measurement and the telemetry block and drops the printf. */
#ifndef PROF_QUIET
            LOG("[perf] emu_fps=%lu rend_fps=%lu skip_fps=%lu late_max=%ldus qmin=%lu qmax=%lu | tilec=%lu | bgm=%u 2106=%02x 2107=%02x 2108=%02x 2109=%02x 210a=%02x 210b=%02x 210c=%02x | 2123=%02x 2124=%02x 2125=%02x 2126=%02x 2127=%02x 2128=%02x 2129=%02x 212a=%02x 212b=%02x | 212c=%02x 212d=%02x 212e=%02x 212f=%02x 2130=%02x 2131=%02x 2133=%02x | emu avg/max=%lu/%lu us | emuR avg/max=%lu/%lu us | emuS avg/max=%lu/%lu us | mix avg/max=%lu/%lu us | pack avg/max=%lu/%lu us | upd avg/max=%lu/%lu us (%lu) | uz avg/max=%lu/%lu us (%lu) | uSub avg/max=%lu/%lu us (%lu) | uMain avg/max=%lu/%lu us (%lu) | uMath avg/max=%lu/%lu us (%lu) | uBack avg/max=%lu/%lu us (%lu) | uScale avg/max=%lu/%lu us (%lu) | rs avg/max=%lu/%lu us (%lu) | ro avg/max=%lu/%lu us (%lu) | r0 avg/max=%lu/%lu us (%lu) | r1 avg/max=%lu/%lu us (%lu) | r2 avg/max=%lu/%lu us (%lu) | r3 avg/max=%lu/%lu us (%lu) | r7 avg/max=%lu/%lu us (%lu)\n",
                (unsigned long)frames,
                (unsigned long)g_perf.rendered,
                (unsigned long)g_perf.skipped,
                (long)g_perf.max_late_us,
                (unsigned long)g_perf.min_q_fill,
                (unsigned long)g_perf.max_q_fill,
                (unsigned long)tc_cnt,
                (unsigned)ppu_bgm,
                (unsigned)r2106,
                (unsigned)r2107,
                (unsigned)r2108,
                (unsigned)r2109,
                (unsigned)r210a,
                (unsigned)r210b,
                (unsigned)r210c,
                (unsigned)r2123,
                (unsigned)r2124,
                (unsigned)r2125,
                (unsigned)r2126,
                (unsigned)r2127,
                (unsigned)r2128,
                (unsigned)r2129,
                (unsigned)r212a,
                (unsigned)r212b,
                (unsigned)r212c,
                (unsigned)r212d,
                (unsigned)r212e,
                (unsigned)r212f,
                (unsigned)r2130,
                (unsigned)r2131,
                (unsigned)r2133,
                (unsigned long)avg_emul, (unsigned long)g_perf.max_emul_us,
                (unsigned long)avg_emul_r, (unsigned long)g_perf.max_emul_render_us,
                (unsigned long)avg_emul_s, (unsigned long)g_perf.max_emul_skip_us,
                (unsigned long)avg_mix,  (unsigned long)g_perf.max_mix_us,
                (unsigned long)avg_pack, (unsigned long)g_perf.max_pack_us,
                (unsigned long)upd_avg, (unsigned long)upd_max, (unsigned long)upd_cnt,
                (unsigned long)uz_avg, (unsigned long)uz_max, (unsigned long)uz_cnt,
                (unsigned long)usub_avg, (unsigned long)usub_max, (unsigned long)usub_cnt,
                (unsigned long)umain_avg, (unsigned long)umain_max, (unsigned long)umain_cnt,
                (unsigned long)ucm_avg, (unsigned long)ucm_max, (unsigned long)ucm_cnt,
                (unsigned long)ubd_avg, (unsigned long)ubd_max, (unsigned long)ubd_cnt,
                (unsigned long)usc_avg, (unsigned long)usc_max, (unsigned long)usc_cnt,
                (unsigned long)rs_avg, (unsigned long)rs_max, (unsigned long)rs_cnt,
                (unsigned long)ro_avg, (unsigned long)ro_max, (unsigned long)ro_cnt,
                (unsigned long)r0_avg, (unsigned long)r0_max, (unsigned long)r0_cnt,
                (unsigned long)r1_avg, (unsigned long)r1_max, (unsigned long)r1_cnt,
                (unsigned long)r2_avg, (unsigned long)r2_max, (unsigned long)r2_cnt,
                (unsigned long)r3_avg, (unsigned long)r3_max, (unsigned long)r3_cnt,
                (unsigned long)r7_avg, (unsigned long)r7_max, (unsigned long)r7_cnt);
#endif
            perf_reset_window(now_us);
        }
#endif

        tight_loop_contents();
    }
}

//=============================================================================
// Main Entry Point
//=============================================================================

int main(void) {
    // Overclock support
#if CPU_CLOCK_MHZ > 252
    vreg_disable_voltage_limit();
    vreg_set_voltage(CPU_VOLTAGE);
    set_flash_timings(CPU_CLOCK_MHZ, 66);
    sleep_ms(100);
#endif
    
    // Set system clock
    if (!set_sys_clock_khz(CPU_CLOCK_MHZ * 1000, false)) {
        set_sys_clock_khz(252 * 1000, true);
    }
    
    stdio_init_all();
#if LIB_PICO_STDIO_USB
    // Give the host a moment to open the CDC port — long enough that
    // most of the boot log lands in the console when a terminal is
    // attached, short enough that we still come up with no host
    // present (so the HDMI display is usable standalone).
    sleep_ms(2000);
#endif
    
    LOG("\n\n");
    LOG("========================================\n");
    LOG("   frank-snes - SNES for RP2350\n");
    LOG("========================================\n");
    LOG("System Clock: %lu MHz\n", clock_get_hz(clk_sys) / 1000000);
    
    // Initialize LED.  C2's LD1 is a WS2812B on GPIO46, not a plain
    // level-driven LED, so the boot indicator is skipped there rather
    // than spending a PIO state machine on it — the link already needs
    // PIO2, and PIO0/PIO1 are HDMI and audio.
#ifdef PICO_DEFAULT_LED_PIN
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
#endif
    
    // Initialize PSRAM
    LOG("Initializing PSRAM...\n");
    uint psram_pin = get_psram_pin();
    LOG("PSRAM pin: %u\n", psram_pin);
    psram_init(psram_pin);
    psram_reset();
    LOG("PSRAM initialized (8 MB)\n");

    // Clear screen buffer BEFORE HDMI init - DMA starts scanning immediately
    // Use palette index 1 instead of 0 to avoid HDMI issues
    memset(SCREEN, 1, sizeof(SCREEN));

    // Initialize PS/2 keyboard + mouse BEFORE HDMI. The PS/2 bit-bang
    // transmissions during mouse init may electrically perturb the HDMI
    // TMDS pairs (GPIO 0/1 vs GPIO 12-19 on M2). Bringing HDMI up after
    // PS/2 init means HDMI is never live during any TX transient, so
    // any coupling hits dark pixels rather than an active video frame.
    LOG("Initializing PS/2...\n");
    ps2kbd_init();
    LOG("PS/2 keyboard initialized\n");

    if (ps2_mouse_init_device()) {
        LOG("PS/2 mouse initialized%s\n",
            ps2_mouse_has_wheel() ? " (IntelliMouse)" : "");
    } else {
        LOG("PS/2 mouse not detected (will remain inactive)\n");
    }

    // Initialize HDMI on Core 0 (like murmgenesis) - critical for ROM selector display.
    // Bring HDMI up BEFORE mounting SD so we can display a visible error
    // screen when no SD card is inserted (otherwise the user just sees black).
    LOG("Initializing HDMI...\n");
    graphics_init(g_out_HDMI);
    graphics_set_buffer(SCREEN[0]);
    graphics_set_res(SCREEN_WIDTH, SCREEN_HEIGHT);
    graphics_set_shift(32, 0);
    graphics_set_mode(GRAPHICSMODE_DEFAULT);
    LOG("HDMI initialized\n");

    // Launch Core 1 (Audio + APU)
    LOG("Starting render core (Audio)...\n");
    multicore_launch_core1(render_core);

#ifndef FRANK_SNES_HDMI_ALT
    /* Hand the scanline interrupt to core 1 - see the note there. */
    while (!g_hdmi_irq_core1_ready) tight_loop_contents();
    graphics_hdmi_irq_release_this_core();
    __dmb();
    g_hdmi_irq_released = true;
#endif


    // Wait for Core 1 to initialize HDMI and audio
    LOG("[Core0] Waiting for Core 1 to initialize...\n");
    while (!core1_ready) {
        tight_loop_contents();
    }
    LOG("[Core0] Render core started (HDMI + Audio on Core 1)\n");

    // Mount SD card (AFTER HDMI so we can show an error screen on failure).
    LOG("Mounting SD card...\n");
    FRESULT res = f_mount(&fs, "", 1);
    if (res != FR_OK) {
        LOG("Failed to mount SD card: %d\n", res);
        // Show a visible error screen and blink the LED while halted.
        rom_selector_show_sd_error(SCREEN[0], (int)res);
        // rom_selector_show_sd_error never returns.
    }
    LOG("SD card mounted\n");

    // Load settings from SD card
    LOG("Loading settings...\n");
    settings_load();
    LOG("Settings loaded (volume=%d, frameskip=%d, p1=%d, p2=%d)\n",
        g_settings.volume, g_settings.frameskip, g_settings.p1_mode, g_settings.p2_mode);

    // Initialize gamepad (GPIO 20/21/26 — far from HDMI, safe to do after HDMI)
    LOG("Initializing input devices...\n");
#ifdef NESPAD_GPIO_CLK
    if (nespad_begin(clock_get_hz(clk_sys) / 1000, NESPAD_GPIO_CLK, NESPAD_GPIO_DATA, NESPAD_GPIO_LATCH)) {
        LOG("NES/SNES gamepad initialized (CLK=%d, DATA=%d, LATCH=%d)\n",
            NESPAD_GPIO_CLK, NESPAD_GPIO_DATA, NESPAD_GPIO_LATCH);
    } else {
        LOG("Failed to initialize NES/SNES gamepad\n");
    }
#else
    LOG("NES/SNES gamepad not configured (NESPAD_GPIO_CLK not defined)\n");
#endif

#ifdef USB_HID_ENABLED
    // Initialize USB HID
    usbhid_init();
    LOG("USB HID initialized\n");
    // Load any previously-learned menu A/B profiles. Done after SD mount
    // so files in /snes/gamepads/ are visible; before tuh_task() fires a
    // mount callback so newly-plugged known pads don't trigger the wizard.
    gamepad_cal_load_all();
#endif

#ifndef FRANK_SNES_AUTOBOOT
    // Show welcome screen on first boot
    welcome_screen_show();

    // Warn the user if any game-affecting video settings are off.
    // No-op when all defaults are intact.
    video_settings_warning_show();
#endif

    // Main loop: ROM selector -> load -> emulate -> repeat
    char rom_path[MAX_ROM_PATH];

    while (true) {
#ifdef FRANK_SNES_AUTOBOOT
        // Autoboot: skip welcome & selector and load one ROM straight
        // away.  AUTOBOOT_PATH overrides the default, which is what
        // makes this usable on a board being driven over SWD with no
        // keyboard attached:
        //   ./build.sh C2  with -DAUTOBOOT_PATH="/snes/Some Game.sfc"
#ifndef AUTOBOOT_PATH
#define AUTOBOOT_PATH "/SNES/Doom (USA).sfc"
#endif
        snprintf(rom_path, sizeof(rom_path), "%s", AUTOBOOT_PATH);
        LOG("AUTOBOOT: %s\n", rom_path);
#else
        // Show ROM selector (sets up its own palette and buffer management)
        LOG("Starting ROM selector...\n");
        bool rom_selected = rom_selector_show(rom_path, sizeof(rom_path), SCREEN[0]);

        if (!rom_selected) {
            LOG("No ROM selected!\n");
            rom_selector_show_sd_error(SCREEN[0], 0);
            // This function never returns
        }

        LOG("ROM selected: %s\n", rom_path);
#endif

        // Clear both screen buffers
        memset(SCREEN[0], 0, sizeof(SCREEN[0]));
        memset(SCREEN[1], 0, sizeof(SCREEN[1]));
        current_buffer = 0;

        // Mark PSRAM so we can restore after emulation
        psram_mark_session();

#ifdef FRANK_SNES_CPU_CORE_S9X16
        /* The 1.6x core owns the ROM buffer, so it has to exist before the
         * file is read into it. The 1.43 order is the other way round only
         * because load_rom_from_sd sets Settings.ForceSuperFX as a hint for
         * S9xInitMemory; this core sizes SRAM from the header it parses and
         * needs no such hint. */
        LOG("Initializing SNES emulator...\n");
        snes9x_init();
#endif

        // Load ROM from SD card
        LOG("Loading ROM...\n");
        bool rom_loaded = load_rom_from_sd(rom_path);

        if (!rom_loaded) {
            LOG("Could not load ROM file!\n");
            psram_restore_session();
            continue;  // Back to ROM selector
        }

#ifndef FRANK_SNES_CPU_CORE_S9X16
        // Initialize SNES emulator
        LOG("Initializing SNES emulator...\n");
        snes9x_init();
#endif

        // Load the ROM into SNES memory map
        LOG("Setting up ROM mapping...\n");
#ifdef FRANK_SNES_CPU_CORE_S9X16
        if (!s9x16_load_rom_inplace(g_rom_content_size, rom_path)) {
#else
        if (!LoadROM(NULL)) {
#endif
            LOG("Failed to initialize ROM!\n");
            psram_restore_session();
            continue;  // Back to ROM selector
        }

        LOG("ROM loaded successfully!\n");
        LOG("ROM Name: %s\n", Memory.ROMName);

        /* memmap.c has just derived Settings.PAL from the ROM's region
         * byte. Everything downstream that is "per frame" — the frame
         * deadline and the number of samples the mixer is asked for —
         * depends on it, so pick it up here rather than assuming 60 Hz. */
        audio_set_region_pacing(Settings.PAL);
        LOG("ROM Size: %lu KB\n", (unsigned long)(Memory.CalculatedSize / 1024));

        /* Set g_rom_name for save state file paths */
        {
            const char *src = Memory.ROMName;
            int j = 0;
            for (int i = 0; src[i] && j < (int)sizeof(g_rom_name) - 1; i++) {
                char c = src[i];
                if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                    (c >= '0' && c <= '9') || c == '-' || c == '_')
                    g_rom_name[j++] = c;
                else if (c == ' ' && j > 0 && g_rom_name[j-1] != '_')
                    g_rom_name[j++] = '_';
            }
            while (j > 0 && g_rom_name[j-1] == '_') j--;
            g_rom_name[j] = '\0';
            if (j == 0) strncpy(g_rom_name, "unknown", sizeof(g_rom_name));
        }

#ifdef PICO_DEFAULT_LED_PIN
        gpio_put(PICO_DEFAULT_LED_PIN, 0);  // LED off = running
#endif

        // Enable CRT effect if configured
        graphics_set_crt_active(g_settings.crt_effect);

        // Run emulation (returns true if user wants ROM selector)
        bool back_to_selector = emulation_loop();

        if (back_to_selector) {
            LOG("Returning to ROM selector...\n");

            // Disable CRT effect for ROM selector
            graphics_set_crt_active(false);

            // Free all PSRAM allocated during this session
            psram_restore_session();

            // Clear emulator state pointers (memory was freed by psram_restore)
            Memory.ROM = NULL;
            Memory.RAM = NULL;
            Memory.VRAM = NULL;
            Memory.SRAM = NULL;
            Memory.FillRAM = NULL;

            // Clear screen buffers
            memset(SCREEN[0], 0, sizeof(SCREEN[0]));
            memset(SCREEN[1], 0, sizeof(SCREEN[1]));
        }
    }

    return 0;
}
