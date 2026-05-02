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

#include "board_config.h"
#include "HDMI.h"
// USB CDC serial used for debug output (dev builds only)
#include "psram_init.h"
#include "psram_allocator.h"
#include "ff.h"

// Snes9x includes
#include "snes9x/snes9x.h"
#include "snes9x/soundux.h"
#include "snes9x/memmap.h"
#include "snes9x/apu.h"
#include "snes9x/display.h"
#include "snes9x/gfx.h"
#include "snes9x/cpuexec.h"
#include "snes9x/srtc.h"

// APU on Core 1
#include "snes9x/apu_core1.h"

// Audio driver (exact copy from pico-snes-master)
#include "audio.h"

// Audio optimizations
#include "audio_opt.h"

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

//=============================================================================
// Configuration
//=============================================================================

#define SCREEN_WIDTH     SNES_WIDTH    // 256
#define SCREEN_HEIGHT    SNES_HEIGHT   // 224

// Audio sample rate - real SNES is 32000 Hz but we use 32040 which
// divides evenly by 60 (534 samples/frame), avoiding the 0.33 sample/frame
// deficit that causes periodic audio pops from buffer underruns.
#define AUDIO_SAMPLE_RATE   (32040)
#define AUDIO_BUFFER_LENGTH (AUDIO_SAMPLE_RATE / 60)

//=============================================================================
// Screen Buffers
//=============================================================================

// Screen buffers - 256x224 8-bit palette-indexed (HDMI driver maps index to color)
uint8_t __attribute__((aligned(4))) SCREEN[2][SNES_WIDTH * SNES_HEIGHT];
static uint8_t __attribute__((aligned(4))) ZBuffer[SNES_WIDTH * SNES_HEIGHT];
static uint8_t __attribute__((aligned(4))) SubZBuffer[SNES_WIDTH * SNES_HEIGHT];

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

// Current display buffer (double buffering) - accessed by HDMI driver
volatile uint32_t current_buffer = 0;

//=============================================================================
// Audio - Core 0 mixes into buffer, Core 1 plays
//=============================================================================

// Audio handoff: Core 0 mixes int16 stereo, then applies gain/limiting and packs
// into 32-bit stereo frames for I2S (L in high 16, R in low 16). Core 1 then
// streams these packed frames to pico_audio_i2s.
//
// Key goal: keep Core 1 work minimal so HDMI activity doesn't starve audio.
// 16 frames (~267ms) - absorbs CPU spikes during scene transitions
#define AUDIO_QUEUE_DEPTH 8
// NOTE: With fixed 60Hz emulation producing exactly one audio chunk per frame,
// the producer cannot stay "ahead" of the consumer by >1 chunk in steady state.
// Using queue-fill watermarks to decide frame skipping will therefore
// permanently starve video (e.g. render ~12fps with MAX_FRAME_SKIP=4).
// We keep this watermark only for choosing the cheaper limiter path.
#define AUDIO_LOW_WATERMARK 4
static uint32_t __attribute__((aligned(32))) audio_packed_buffer[AUDIO_QUEUE_DEPTH][AUDIO_BUFFER_LENGTH];
static uint32_t __attribute__((aligned(32))) audio_packed_discard[AUDIO_BUFFER_LENGTH];
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
    GFX.Pitch = SNES_WIDTH;  // 8-bit pixels: 1 byte per pixel
    GFX.ZPitch = SNES_WIDTH;
    GFX.Screen = SCREEN[current_buffer];

    // SubScreenBuffer lives in the PSRAM scratch region (see declaration
    // above) — always valid, no allocation needed.  Zero it so a stale
    // sub-screen from the previous ROM doesn't leak into color math.
    memset(SubScreenBuffer, 0, SNES_WIDTH * SNES_HEIGHT);
    GFX.SubScreen = g_settings.transparency_enabled ? SubScreenBuffer : GFX.Screen;

    GFX.ZBuffer = (uint8_t *)ZBuffer;
    GFX.SubZBuffer = (uint8_t *)SubZBuffer;
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
            S9xNotifyButtonPress();
        prev_joypad = joypad;
    }

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

    S9xInitDisplay();
    S9xInitMemory();
    S9xInitAPU();
    S9xInitSound(0, 0);
    if (!S9xInitGFX()) {
        LOG("FATAL: S9xInitGFX failed (out of memory)\n");
    }
    S9xSetPlaybackRate(Settings.SoundPlaybackRate);
    IPPU.RenderThisFrame = 1;
}

//=============================================================================
// ROM Loading from SD Card
//=============================================================================

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
    
    // Read ROM into buffer
    res = f_read(&file, Memory.ROM, file_size, &bytes_read);
    f_close(&file);
    
    if (res != FR_OK || bytes_read != file_size) {
        LOG("Failed to read ROM: res=%d, read=%lu\n", res, (unsigned long)bytes_read);
        return false;
    }
    
    LOG("ROM loaded: %lu bytes\n", (unsigned long)bytes_read);
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
}

//=============================================================================
// Main Emulation Loop
//=============================================================================

// Deferred palette update flag from PPU
extern volatile bool g_palette_needs_update;
extern void S9xFixColourBrightness(void);

// Auto frame skip - target ~60fps (16.67ms per frame)
#define TARGET_FRAME_US 16667

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
            emu_overrun_us -= (int32_t)TARGET_FRAME_US;  // recover one frame worth
        }

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
            GFX.Screen = SCREEN[0];
            GFX.SubScreen = (g_settings.transparency_enabled && SubScreenBuffer)
                            ? SubScreenBuffer : GFX.Screen;

            // Restore emulation palette
            S9xFixColourBrightness();
            g_palette_needs_update = false;

            // Clear stale joypad state so the game doesn't see buttons
            // from before the menu on the first frame of resumed emulation
            // (game can read $4218/$4016 before VBlank updates them)
            for (int j = 0; j < 5; j++)
                IPPU.Joypads[j] = 0;
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

        { extern volatile uint32_t dsp_log_frame; dsp_log_frame++; }

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

        // Mix audio on Core 0 (always, even when skipping render), then apply
        // gain/limiting and pack to 32-bit stereo frames.
        static int16_t __attribute__((aligned(32))) mix16[AUDIO_BUFFER_LENGTH * 2];
    #ifdef FRANK_SNES_PROFILE
        uint32_t t2 = time_us_32();
    #endif
    #ifdef FRANK_SNES_FAST_MODE
        // FAST MODE: Mix mono only (half the samples), then duplicate to stereo in packing
        S9xMixSamplesMono((void *)mix16, AUDIO_BUFFER_LENGTH);
    #else
        S9xMixSamples((void *)mix16, AUDIO_BUFFER_LENGTH * 2);
    #endif
    #ifdef FRANK_SNES_PROFILE
        uint32_t t3 = time_us_32();
    #endif

        uint32_t prod = audio_prod_seq;
        uint32_t cons = audio_cons_seq;
        bool ring_full = (prod - cons) >= AUDIO_QUEUE_DEPTH;
        uint32_t *dst32 = ring_full ? audio_packed_discard : audio_packed_buffer[prod % AUDIO_QUEUE_DEPTH];

        // Mixer attenuates by >>11 (÷2048) to prevent hard clipping.
        // Boost with soft limiter to restore volume, scaled by volume setting.
        // At volume=100: gain = 400/100 = 4x (full). At volume=50: 2x. At volume=10: 0.4x.
        const int gain_num = g_settings.volume * 4;
        const int gain_den = 100;
        const bool use_soft_limiter = true;
#ifdef FRANK_SNES_PROFILE
        uint32_t t4 = time_us_32();
#endif
        // Use optimized audio packing
#ifdef FRANK_SNES_FAST_MODE
        // FAST MODE: Pack mono to stereo (duplicate each sample)
        audio_pack_mono_to_stereo(dst32, mix16, AUDIO_BUFFER_LENGTH, gain_num, gain_den, use_soft_limiter);
#else
        audio_pack_opt(dst32, mix16, AUDIO_BUFFER_LENGTH, gain_num, gain_den, use_soft_limiter);
#endif
#ifdef FRANK_SNES_PROFILE
        uint32_t t5 = time_us_32();
#endif

        if (!ring_full) {
            // Publish the new chunk after the samples are fully written.
            __dmb();
            audio_prod_seq = prod + 1;
            __dmb();
        }

        // Wall-clock audio catch-up: produce extra chunks so I2S never starves.
        // Accumulate real elapsed time; each TARGET_FRAME_US owes one chunk.
        // The normal mix above already covered one chunk worth of time.
        #define AUDIO_CATCHUP_MAX 6  // cap burst after long stall
        {
            uint32_t now_ac = time_us_32();
            audio_acc_us += (now_ac - audio_last_us);
            audio_last_us = now_ac;

            // The normal mix covered one chunk's worth of time
            if (audio_acc_us >= TARGET_FRAME_US)
                audio_acc_us -= TARGET_FRAME_US;
            else
                audio_acc_us = 0;

            // Soft-cap: after a very long stall (loading, pause) don't burst
            // dozens of chunks — just refill a modest amount.
            if (audio_acc_us > TARGET_FRAME_US * AUDIO_CATCHUP_MAX)
                audio_acc_us = TARGET_FRAME_US * 2;

            // Produce extra chunks for the remaining accumulated time
            uint32_t extra = 0;
            while (audio_acc_us >= TARGET_FRAME_US && extra < AUDIO_CATCHUP_MAX) {
                uint32_t p2 = audio_prod_seq;
                if ((p2 - audio_cons_seq) >= AUDIO_QUEUE_DEPTH)
                    break;
#ifdef FRANK_SNES_FAST_MODE
                S9xMixSamplesMono((void *)mix16, AUDIO_BUFFER_LENGTH);
#else
                S9xMixSamples((void *)mix16, AUDIO_BUFFER_LENGTH * 2);
#endif
                uint32_t *edst = audio_packed_buffer[p2 % AUDIO_QUEUE_DEPTH];
#ifdef FRANK_SNES_FAST_MODE
                audio_pack_mono_to_stereo(edst, mix16, AUDIO_BUFFER_LENGTH, gain_num, gain_den, use_soft_limiter);
#else
                audio_pack_opt(edst, mix16, AUDIO_BUFFER_LENGTH, gain_num, gain_den, use_soft_limiter);
#endif
                __dmb();
                audio_prod_seq = p2 + 1;
                __dmb();
                audio_acc_us -= TARGET_FRAME_US;
                extra++;
            }
        }

        if (skip_render) {
            consecutive_skipped_frames++;
        } else {
            consecutive_skipped_frames = 0;

            // Swap display buffers only when we rendered
            current_buffer = !current_buffer;
            GFX.Screen = SCREEN[current_buffer];
            GFX.SubScreen = (g_settings.transparency_enabled && SubScreenBuffer)
                            ? SubScreenBuffer : GFX.Screen;
        }

        // Update palette if brightness changed during frame
        if (g_palette_needs_update) {
            S9xFixColourBrightness();
            g_palette_needs_update = false;
        }

        // Advance deadline and frame counter for the next emulated frame.
        next_frame_deadline += TARGET_FRAME_US;
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
    // Dev builds (USB_HID=0): USB CDC is the stdio sink. Wait until the
    // host terminal opens the port, THEN sleep 3 more seconds before any
    // LOG() fires, so the whole boot log lands in the console.
    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }
    sleep_ms(3000);
#endif
    
    LOG("\n\n");
    LOG("========================================\n");
    LOG("   frank-snes - SNES for RP2350\n");
    LOG("========================================\n");
    LOG("System Clock: %lu MHz\n", clock_get_hz(clk_sys) / 1000000);
    
    // Initialize LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    
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
        // Autoboot: skip welcome & selector, hardcode Doom path
        snprintf(rom_path, sizeof(rom_path), "%s",
                 "/SNES/Doom (USA).sfc");
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

        // Load ROM from SD card
        LOG("Loading ROM...\n");
        bool rom_loaded = load_rom_from_sd(rom_path);

        if (!rom_loaded) {
            LOG("Could not load ROM file!\n");
            psram_restore_session();
            continue;  // Back to ROM selector
        }

        // Initialize SNES emulator
        LOG("Initializing SNES emulator...\n");
        snes9x_init();

        // Load the ROM into SNES memory map
        LOG("Setting up ROM mapping...\n");
        if (!LoadROM(NULL)) {
            LOG("Failed to initialize ROM!\n");
            psram_restore_session();
            continue;  // Back to ROM selector
        }

        LOG("ROM loaded successfully!\n");
        LOG("ROM Name: %s\n", Memory.ROMName);
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

        gpio_put(PICO_DEFAULT_LED_PIN, 0);  // LED off = running

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
