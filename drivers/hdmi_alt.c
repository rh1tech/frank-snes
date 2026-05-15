/*
 * frank-snes - HDMI_ALT driver: libdvi-backed video + audio
 *
 * Drop-in replacement for drivers/HDMI.c when the build is configured
 * with -DHDMI_DRIVER=ALT.  Implements the same graphics_* / startVIDEO
 * API surface against libdvi (vendored under drivers/libdvi/).
 *
 * Pipeline:
 *   - 640x480p 60 Hz output, vertical-doubled to 240 effective lines.
 *   - SNES native frame is 256x224, centred horizontally inside 640px
 *     and vertically inside 240px (8-line letterbox top/bottom).
 *   - Per-line scanbuf converts 8-bit palette indices from
 *     SCREEN[!current_buffer] into RGB565 via palette_rgb565[].
 *   - Audio rides HDMI data-island packets (no I2S required).
 *
 * Threading:
 *   - Core 1 owns DVI: registers DMA IRQ, runs producer+consumer in a
 *     single tight loop (we inline the producer so emulation can stay
 *     on Core 0).
 *   - Core 0 calls hdmi_alt_audio_write() in place of i2s_dma_write()
 *     to push packed stereo frames into dvi0.audio_ring.
 */

#include "board_config.h"
#include "HDMI.h"
#include "hdmi_alt.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sem.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/structs/bus_ctrl.h"

#include "libdvi/dvi.h"
#include "libdvi/dvi_serialiser.h"
#include "libdvi/dvi_config_defs.h"
#include "libdvi/audio_ring.h"

/* ------------------------------------------------------------------ */
/* Pin configuration                                                  */
/* ------------------------------------------------------------------ */

/* The current HDMI_PIO driver wires CLK on HDMI_BASE_PIN+0/1 and three
 * TMDS data lanes on +2..+7.  libdvi's serialiser_cfg expects the n-pin
 * (lower) of each pair, with sm_tmds 0/1/2 for D0/D1/D2.  Map directly
 * from board_config.h so M1 and M2 just work. */

#ifndef DVI_DEFAULT_PIO_INST
#define DVI_DEFAULT_PIO_INST pio0
#endif

/* M1/M2 wire each TMDS pair with the N (negative) on the lower-numbered
 * GPIO and P on the next higher GPIO (CLKN=12/CLKP=13, D0N=14/D0P=15,
 * etc).  libdvi's PIO writes the lower side-set bit to the lower pin,
 * so when it asserts a logical "1" it drives N=0/P=1 — the spec-
 * correct polarity, no inversion required. */
static const struct dvi_serialiser_cfg frank_snes_dvi_cfg = {
    .pio              = DVI_DEFAULT_PIO_INST,
    .sm_tmds          = { 0, 1, 2 },
    .pins_tmds        = { HDMI_PIN_D0N, HDMI_PIN_D1N, HDMI_PIN_D2N },
    .pins_clk         = HDMI_PIN_CLKN,
    .invert_diffpairs = true,
};

/* ------------------------------------------------------------------ */
/* Mode constants                                                     */
/* ------------------------------------------------------------------ */

#define DVI_TIMING_PRESET     dvi_timing_640x480p_60hz
#define HDMI_FRAME_WIDTH      640
#define HDMI_FRAME_HEIGHT     480
/* DVI_VERTICAL_REPEAT is 2 in libdvi; one logical scanline drives two
 * raster lines, so we generate 240 unique scanlines per frame. */
#define HDMI_LOGICAL_LINES    (HDMI_FRAME_HEIGHT / DVI_VERTICAL_REPEAT)

/* libdvi's 16bpp scanline encoder is pixel-doubling: scanline buffers
 * carry h_active_pixels / 2 source pixels and the encoder repeats each
 * to produce h_active_pixels output TMDS symbols.  So we draw into a
 * 320-pixel-wide buffer (RGB565), not 640. */
#define HDMI_LOGICAL_WIDTH    (HDMI_FRAME_WIDTH / 2)

#define SNES_NATIVE_W         256
#define SNES_NATIVE_H         224
/* Centre 256-wide source content within the 320 logical pixels:
 * 32 left / 32 right pillarbox. */
#define HDMI_X_OFFSET         ((HDMI_LOGICAL_WIDTH - SNES_NATIVE_W) / 2)
/* Centre 224 lines within 240 logical lines: 8 top / 8 bottom. */
#define HDMI_Y_OFFSET         ((HDMI_LOGICAL_LINES - SNES_NATIVE_H) / 2)

/* Number of scanline buffers held in q_colour_*.  2 gives one frame
 * of slack between producer and consumer — important when Core 0 is
 * doing heavy SRAM/PSRAM work (rom-selector carousel) and stalls
 * Core 1 momentarily. */
#define HDMI_N_SCANLINE_BUFS  2

/* ------------------------------------------------------------------ */
/* Audio constants                                                    */
/* ------------------------------------------------------------------ */

/* SNES audio output is 32040 Hz stereo (see main.c).  Use 32 kHz HDMI
 * audio class — the 40 Hz drift is well within receiver tolerance and
 * keeps clocks tidy. */
#define HDMI_AUDIO_RATE   32000
#define HDMI_AUDIO_N      4096    /* CEA-861 N for 32 kHz */

/* Power-of-two size for the data-island ring.  Restored to 512 (= 2 KB)
 * once audio is re-enabled; while the audio data-island path is
 * disabled for video bring-up the ring storage is not allocated. */
#define HDMI_AUDIO_RING_SAMPLES (512)

/* ------------------------------------------------------------------ */
/* libdvi state and buffers                                           */
/* ------------------------------------------------------------------ */

struct dvi_inst dvi0;

/* Scanline buffers (16bpp = uint16_t per pixel) — sized for the
 * pixel-doubling encoder, so HDMI_LOGICAL_WIDTH (= h_active/2). */
static uint16_t __attribute__((aligned(4)))
    scanline_buf[HDMI_N_SCANLINE_BUFS][HDMI_LOGICAL_WIDTH];

static audio_sample_t __attribute__((aligned(4)))
    audio_ring_storage[HDMI_AUDIO_RING_SAMPLES];

/* ------------------------------------------------------------------ */
/* Frame buffer hookup with main.c                                    */
/* ------------------------------------------------------------------ */

extern uint8_t SCREEN[2][256 * 224];
extern volatile uint32_t current_buffer;

static uint8_t *graphics_buffer       = NULL;
static int      graphics_buffer_w     = SNES_NATIVE_W;
static int      graphics_buffer_h     = SNES_NATIVE_H;
static int      graphics_shift_x      = 0;
static int      graphics_shift_y      = 0;
static enum graphics_mode_t hdmi_graphics_mode = GRAPHICSMODE_DEFAULT;

/* Effects flags (parity with HDMI.c). */
static bool crt_active       = false;
static bool greyscale_active = false;

/* ------------------------------------------------------------------ */
/* Palette: 256-entry RGB565 LUT and original 0xRRGGBB cache          */
/* ------------------------------------------------------------------ */

static uint32_t palette_rgb888[256];
/* Hot-path LUT — placed in scratch_y so Core 1's fill_scanline reads
 * are not contending with Core 0 emulation traffic on the main SRAM
 * banks. */
static uint16_t __scratch_y("hdmi_alt_pal565") palette_rgb565[256];

static inline uint16_t rgb888_to_rgb565(uint32_t rgb) {
    uint8_t r = (rgb >> 16) & 0xff;
    uint8_t g = (rgb >> 8)  & 0xff;
    uint8_t b = (rgb >> 0)  & 0xff;
    return (uint16_t)(((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3));
}

static inline uint32_t rgb888_to_grey(uint32_t rgb) {
    uint8_t r = (rgb >> 16) & 0xff;
    uint8_t g = (rgb >> 8)  & 0xff;
    uint8_t b = (rgb >> 0)  & 0xff;
    uint8_t y = (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
    return ((uint32_t)y << 16) | ((uint32_t)y << 8) | y;
}

static void rebuild_palette_rgb565(void) {
    for (int i = 0; i < 256; ++i) {
        uint32_t c = palette_rgb888[i];
        if (greyscale_active) c = rgb888_to_grey(c);
        palette_rgb565[i] = rgb888_to_rgb565(c);
    }
}

/* ------------------------------------------------------------------ */
/* Public graphics_* API (matches drivers/HDMI.h)                     */
/* ------------------------------------------------------------------ */

void graphics_set_buffer(uint8_t *buffer) {
    graphics_buffer = buffer;
}

uint8_t* graphics_get_buffer(void) {
    return graphics_buffer;
}

uint32_t graphics_get_width(void) {
    return (uint32_t)graphics_buffer_w;
}

uint32_t graphics_get_height(void) {
    return (uint32_t)graphics_buffer_h;
}

void graphics_set_res(int w, int h) {
    graphics_buffer_w = w;
    graphics_buffer_h = h;
}

void graphics_set_shift(int x, int y) {
    graphics_shift_x = x;
    graphics_shift_y = y;
}

void graphics_set_mode(enum graphics_mode_t mode) {
    hdmi_graphics_mode = mode;
}

uint32_t graphics_get_palette(uint8_t i) {
    return palette_rgb888[i];
}

void graphics_set_palette(uint8_t i, uint32_t color888) {
    color888 &= 0x00ffffff;
    palette_rgb888[i] = color888;
    uint32_t c = greyscale_active ? rgb888_to_grey(color888) : color888;
    palette_rgb565[i] = rgb888_to_rgb565(c);
}

void graphics_set_bgcolor(uint32_t color888) {
    /* HDMI_PIO uses palette[255] as background sentinel.  Keep the same
     * convention so callers don't care which driver is active. */
    graphics_set_palette(255, color888);
}

void graphics_restore_sync_colors(void) {
    /* libdvi handles HDMI sync internally — no reserved palette slots
     * to repair, so this is a no-op. */
}

void graphics_set_crt_active(bool active) { crt_active = active; }
bool graphics_get_crt_active(void)         { return crt_active; }

void graphics_set_greyscale(bool active) {
    if (greyscale_active == active) return;
    greyscale_active = active;
    rebuild_palette_rgb565();
}
bool graphics_get_greyscale(void) { return greyscale_active; }

struct video_mode_t graphics_get_video_mode(int mode) {
    (void)mode;
    /* Match HDMI.c's table for callers that read these fields. */
    struct video_mode_t v = { .h_total = 524, .h_width = 480, .freq = 60, .vgaPxClk = 25175000 };
    return v;
}

void set_palette(uint8_t n) { (void)n; }   /* Stub — parity with HDMI.c. */

void startVIDEO(uint8_t vol) { (void)vol; }

/* Called by ppu.c::S9xFixColourBrightness().  HDMI.c also leaves this
 * as a stub since palette updates are immediate via graphics_set_palette. */
void graphics_request_palette_update(void) {}

/* ------------------------------------------------------------------ */
/* Producer + consumer loop on Core 1                                 */
/* ------------------------------------------------------------------ */

/* Forward decl: defined further down, replicates the body of
 * libdvi's static _dvi_prepare_scanline_16bpp so we can drive one
 * iteration at a time. */
static void encode_one_scanline_16bpp(struct dvi_inst *inst);

/* Fill a single HDMI_LOGICAL_WIDTH-pixel RGB565 scanline from the SNES
 * framebuffer.  libdvi's 16bpp encoder is pixel-doubling, so this 320-
 * wide buffer becomes 640 output pixels on the wire.
 *
 * Placed in scratch_y so the per-scanline 256-pixel palette lookup
 * runs out of Core 1's local SRAM bank, free of Core 0 contention
 * during heavy emulator/UI work (e.g. the rom-selector carousel
 * sprite scaling).  The pillarbox columns are written ONCE at init
 * time and never re-zeroed per frame, eliminating two memset() calls
 * per scanline that were the dominant source of red "late-scanline"
 * flashes during heavy Core 0 SRAM traffic.
 *
 * For top/bottom letterbox lines we also avoid the full 320-wide
 * memset — the scanline buffer is only ever HDMI_LOGICAL_WIDTH wide
 * and the pillarbox columns are already black, so we just zero the
 * middle SNES_NATIVE_W slice. */
static void __scratch_y("fill_scanline") fill_scanline(uint16_t *dst, int logical_y) {
    int snes_y = logical_y - HDMI_Y_OFFSET;
    uint16_t *out = dst + HDMI_X_OFFSET;

    /* Top/bottom letterbox or no source: zero only the active SNES
     * slice, leaving the static pillarbox black columns untouched. */
    if (snes_y < 0 || snes_y >= graphics_buffer_h || graphics_buffer == NULL ||
        (crt_active && (logical_y & 1))) {
        memset(out, 0, SNES_NATIVE_W * sizeof(uint16_t));
        return;
    }

    /* main.c double-buffers SCREEN; renderer writes current_buffer,
     * we display !current_buffer.  Honour graphics_set_buffer() if
     * the caller has pinned it (used by menus and ROM selector). */
    const uint8_t *src;
    if (graphics_buffer == SCREEN[0] || graphics_buffer == SCREEN[1]) {
        src = SCREEN[!current_buffer];
    } else {
        src = graphics_buffer;
    }
    src += (size_t)snes_y * (size_t)graphics_buffer_w;

    int w = graphics_buffer_w;
    if (w > HDMI_LOGICAL_WIDTH - HDMI_X_OFFSET) w = HDMI_LOGICAL_WIDTH - HDMI_X_OFFSET;
    for (int x = 0; x < w; ++x) {
        out[x] = palette_rgb565[src[x]];
    }
}

/* TMDS encoder is exported by libdvi. */
extern void tmds_encode_data_channel_16bpp(const uint32_t *pixbuf,
                                           uint32_t *symbuf, size_t n_pix,
                                           uint channel_msb, uint channel_lsb);

/* Single-iteration version of dvi_scanbuf_main_16bpp's loop body —
 * libdvi's own copy is an infinite loop, so we replicate the body here
 * to keep producer + consumer interleaved on Core 1. */
static void __not_in_flash_func(encode_one_scanline_16bpp)(struct dvi_inst *inst) {
    uint32_t *scanbuf = NULL;
    queue_remove_blocking(&inst->q_colour_valid, &scanbuf);

    uint32_t *tmdsbuf = NULL;
    queue_remove_blocking(&inst->q_tmds_free, &tmdsbuf);
    uint pixwidth       = inst->timing->h_active_pixels;
    uint words_per_chan = pixwidth / DVI_SYMBOLS_PER_WORD;
    tmds_encode_data_channel_16bpp(scanbuf, tmdsbuf + 0 * words_per_chan,
                                   pixwidth / 2,
                                   DVI_16BPP_BLUE_MSB,  DVI_16BPP_BLUE_LSB);
    tmds_encode_data_channel_16bpp(scanbuf, tmdsbuf + 1 * words_per_chan,
                                   pixwidth / 2,
                                   DVI_16BPP_GREEN_MSB, DVI_16BPP_GREEN_LSB);
    tmds_encode_data_channel_16bpp(scanbuf, tmdsbuf + 2 * words_per_chan,
                                   pixwidth / 2,
                                   DVI_16BPP_RED_MSB,   DVI_16BPP_RED_LSB);
    queue_add_blocking(&inst->q_tmds_valid, &tmdsbuf);

    queue_add_blocking(&inst->q_colour_free, &scanbuf);
}

/* Core 1 entry: producer + consumer in one tight loop. */
static void __not_in_flash_func(hdmi_alt_core1_main)(void) {
    /* Use DMA_IRQ_1 to avoid contention with anything on Core 0 that
     * uses DMA_IRQ_0.  pico-zxspectrum's libdvi integration does the
     * same. */
    dvi_register_irqs_this_core(&dvi0, DMA_IRQ_1);

    /* Prime the TMDS pipeline before dvi_start: the receiver will not
     * lock until at least one fully encoded scanline is in q_tmds_valid,
     * since dvi_start's first DMA chain otherwise reads a placeholder
     * (SRAM_BASE) and emits garbage TMDS while sync rolls. */
    for (int i = 0; i < HDMI_N_SCANLINE_BUFS; ++i) {
        uint16_t *scanbuf = NULL;
        queue_remove_blocking(&dvi0.q_colour_free, &scanbuf);
        fill_scanline(scanbuf, i);
        queue_add_blocking(&dvi0.q_colour_valid, &scanbuf);
        encode_one_scanline_16bpp(&dvi0);
    }

    dvi_start(&dvi0);

    int logical_y = 0;
    while (1) {
        /* Producer: pop free, fill, push valid. */
        uint16_t *scanbuf = NULL;
        queue_remove_blocking(&dvi0.q_colour_free, &scanbuf);
        fill_scanline(scanbuf, logical_y);
        queue_add_blocking(&dvi0.q_colour_valid, &scanbuf);

        /* Consumer: pop valid, TMDS-encode, push tmds_valid. */
        encode_one_scanline_16bpp(&dvi0);

        ++logical_y;
        if (logical_y >= HDMI_LOGICAL_LINES) logical_y = 0;
    }
}

/* ------------------------------------------------------------------ */
/* Initialisation                                                     */
/* ------------------------------------------------------------------ */

void graphics_init(g_out g_out) {
    (void)g_out;

    /* libdvi runs the TMDS serialiser straight off sys_clock with a
     * fixed PIO program — the system clock MUST equal the TMDS bit
     * clock (252 MHz for 640x480p60).  main.c will already have
     * overclocked to e.g. 504 MHz for emulation, but at 504 MHz the
     * TMDS line rate is 2x spec and no display will lock.  Force the
     * clock down here.  This costs SNES emulation throughput but is
     * the only way to get a valid HDMI signal without a redesign of
     * libdvi's PIO program. */
    uint target_khz = DVI_TIMING_PRESET.bit_clk_khz;
    if (clock_get_hz(clk_sys) / 1000 != target_khz) {
        if (!set_sys_clock_khz(target_khz, false)) {
            set_sys_clock_khz(252000, true);
        }
    }

    /* Give Core 1 (which is going to drive DVI) bus priority so its
     * encoded scanlines reach SRAM in time, and ALSO promote DMA bus
     * priority so the TMDS DMA channels aren't starved by Core 0
     * emulation work — the carousel UI in particular hammers PSRAM
     * and the AHB fabric, which without this caused libdvi's
     * late_scanline_ctr to fire and the screen to flash a red line on
     * every dropped scanline. */
    hw_set_bits(&bus_ctrl_hw->priority,
                BUSCTRL_BUS_PRIORITY_PROC1_BITS |
                BUSCTRL_BUS_PRIORITY_DMA_R_BITS |
                BUSCTRL_BUS_PRIORITY_DMA_W_BITS);

    /* Initialise libdvi. */
    dvi0.timing  = &DVI_TIMING_PRESET;
    dvi0.ser_cfg = frank_snes_dvi_cfg;
    /* All M1/M2 HDMI pins are below 32, so PIO default GPIO base of 0
     * is correct.  Set explicitly anyway for clarity. */
    pio_set_gpio_base(frank_snes_dvi_cfg.pio, 0);
    dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());


    /* Audio data-island setup.  CTS = pixel_clk * N / (128 * fs). */
    dvi_get_blank_settings(&dvi0)->top    = 0;
    dvi_get_blank_settings(&dvi0)->bottom = 0;
    dvi_audio_sample_buffer_set(&dvi0, audio_ring_storage, HDMI_AUDIO_RING_SAMPLES);
    int cts = DVI_TIMING_PRESET.bit_clk_khz * HDMI_AUDIO_N / (HDMI_AUDIO_RATE / 100) / 128;
    dvi_set_audio_freq(&dvi0, HDMI_AUDIO_RATE, cts, HDMI_AUDIO_N);

    /* Pre-fill the static pillarbox columns of every scanline buffer
     * with black RGB565.  fill_scanline() never rewrites these
     * columns again, which saves two memset() calls per scanline and
     * removes the dominant source of TMDS underruns during heavy UI
     * work on Core 0. */
    for (int i = 0; i < HDMI_N_SCANLINE_BUFS; ++i) {
        memset(scanline_buf[i], 0, HDMI_LOGICAL_WIDTH * sizeof(uint16_t));
    }

    /* Pre-feed scanline buffers into the colour-free queue so the
     * Core 1 loop has something to fill on first iteration. */
    for (int i = 0; i < HDMI_N_SCANLINE_BUFS; ++i) {
        void *p = scanline_buf[i];
        queue_add_blocking(&dvi0.q_colour_free, &p);
    }

    /* Default palette: black until caller fills it. */
    memset(palette_rgb888, 0, sizeof(palette_rgb888));
    memset(palette_rgb565, 0, sizeof(palette_rgb565));

    /* Core 1 is launched by main.c's render_core, which calls
     * hdmi_alt_run_core1() under FRANK_SNES_HDMI_ALT.  No Core 1 work
     * here. */
}

/* Called from main.c's render_core when FRANK_SNES_HDMI_ALT is set,
 * after audio bring-up so DSP can flow.  Never returns. */
void __not_in_flash_func(hdmi_alt_run_core1)(void) {
    hdmi_alt_core1_main();
    __builtin_unreachable();
}

/* ------------------------------------------------------------------ */
/* Audio output: Core 0 writes packed L<<16|R frames into audio_ring  */
/* ------------------------------------------------------------------ */

uint32_t hdmi_alt_audio_free(void) {
    return get_write_size(&dvi0.audio_ring, false);
}

/* Caller passes a pointer to packed uint32_t frames where each word is
 * (left << 16) | (right & 0xffff), as produced by audio_pack_opt /
 * audio_pack_mono_to_stereo / audio_pack_asm.  We unpack into the
 * libdvi audio_sample_t (channels[0] = left, channels[1] = right).
 *
 * The argument is typed int16_t* to keep the call-site identical in
 * shape to i2s_dma_write(); the actual layout is uint32_t per frame. */
uint32_t __not_in_flash_func(hdmi_alt_audio_write)(const int16_t *frames_lr,
                                                   uint32_t num_frames) {
    if (!dvi_is_started(&dvi0)) return 0;

    uint32_t free_frames = get_write_size(&dvi0.audio_ring, false);
    if (free_frames == 0) {
        /* Ring is full — drop and return 0 so caller doesn't spin. */
        return 0;
    }
    uint32_t to_write = num_frames < free_frames ? num_frames : free_frames;

    const uint32_t *src = (const uint32_t *)frames_lr;
    audio_sample_t *base = get_buffer_top(&dvi0.audio_ring);
    uint32_t mask  = get_buffer_size(&dvi0.audio_ring) - 1;
    uint32_t wpos  = get_write_offset(&dvi0.audio_ring);
    for (uint32_t i = 0; i < to_write; ++i) {
        uint32_t w = src[i];
        base[wpos].channels[0] = (int16_t)(w >> 16);    /* left  (high 16) */
        base[wpos].channels[1] = (int16_t)(w & 0xffff); /* right (low 16) */
        wpos = (wpos + 1) & mask;
    }
    set_write_offset(&dvi0.audio_ring, wpos);
    return to_write;
}
