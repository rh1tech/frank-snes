/*
 * MurmSNES - Runtime Settings
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>
#include <stdbool.h>

// Input mode values
#define INPUT_MODE_ANY      0
#define INPUT_MODE_NES1     1
#define INPUT_MODE_NES2     2
#define INPUT_MODE_USB1     3
#define INPUT_MODE_USB2     4
#define INPUT_MODE_KEYBOARD 5
#define INPUT_MODE_DISABLED 6
#define INPUT_MODE_COUNT    7

// Volume range (0=OFF, 10-100 step 10)
#define VOLUME_MIN  0
#define VOLUME_MAX  100
#define VOLUME_STEP 10

// ROM selector view mode (persisted across boots)
#define SELECTOR_MODE_CAROUSEL 0
#define SELECTOR_MODE_BROWSER  1

// Button mapping: 8 remappable SNES buttons (D-pad is always fixed)
// Each entry maps a SNES button to a physical button index (0-7)
#define BTNMAP_COUNT 8
#define BTNMAP_A      0
#define BTNMAP_B      1
#define BTNMAP_X      2
#define BTNMAP_Y      3
#define BTNMAP_L      4
#define BTNMAP_R      5
#define BTNMAP_START  6
#define BTNMAP_SELECT 7

typedef struct {
    uint8_t map[BTNMAP_COUNT];  // map[snes_btn] = physical_btn index
} button_map_t;

// Default identity mapping
#define BTNMAP_DEFAULT { {0, 1, 2, 3, 4, 5, 6, 7} }

typedef struct {
    uint8_t p1_mode;              // Player 1 input mode (INPUT_MODE_*)
    uint8_t p2_mode;              // Player 2 input mode (INPUT_MODE_*)
    uint8_t volume;               // Master volume 0-100 (0=OFF)
    bool    crt_effect;           // CRT scanline effect ON/OFF
    bool    greyscale;            // Black & white palette mode
    uint8_t frameskip;            // 0=none, 1=low, 2=medium, 3=high, 4=extreme

    // Video settings
    uint8_t bg_enabled;           // BG1-4 enable bits (bit 0=BG1, ..., bit 3=BG4)
    bool    sprites_enabled;
    bool    transparency_enabled;
    bool    hdma_enabled;
    bool    crt_overscan;         // CRT overscan simulation: blank edge columns to hide scroll edge artifacts

    // Audio settings
    bool    echo_enabled;         // Sound echo (reverb)
    bool    interpolation;        // Sound interpolation

    // Button mappings per input device
    button_map_t btnmap_kbd;      // Keyboard
    button_map_t btnmap_nes;      // NES/SNES gamepad
    button_map_t btnmap_usb;      // USB gamepad

    // ROM selector state
    uint8_t selector_mode;        // SELECTOR_MODE_CAROUSEL or SELECTOR_MODE_BROWSER
    char    browser_path[280];    // Last file browser directory
    char    browser_file[256];    // Last launched file name (in browser_path)
} settings_t;

extern settings_t g_settings;

/* Current ROM name (no path/extension) - set by main.c during ROM load */
extern char g_rom_name[64];

typedef enum {
    SETTINGS_RESULT_EXIT,         // Back to game / back to ROM selector
    SETTINGS_RESULT_ROM_SELECT,   // Return to ROM selector (no reboot)
} settings_result_t;

/**
 * Load settings from SD card (/snes/settings.ini)
 * Uses defaults if file doesn't exist.
 */
void settings_load(void);

/**
 * Save current settings to SD card
 */
bool settings_save(void);

/**
 * Apply settings that affect the emulator at runtime
 * (frameskip, echo, interpolation, BG/sprite/transparency/HDMA enables)
 */
void settings_apply_runtime(void);

/**
 * Check if menu hotkey is pressed (Start+Select, ESC, or F12)
 * Call during emulation loop or ROM selector.
 */
bool settings_check_hotkey(void);

/**
 * Display settings menu and block until user exits.
 * @param screen_buffer 256x224 byte buffer for menu rendering
 * @param in_game true when called from emulation, false from ROM selector
 * @return SETTINGS_RESULT_EXIT or SETTINGS_RESULT_ROM_SELECT
 */
settings_result_t settings_menu_show(uint8_t *screen_buffer, bool in_game);

#endif // SETTINGS_H
