/*
 * MurmSNES - Settings Menu
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "settings.h"
#include "HDMI.h"
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "nespad/nespad.h"
#include "ps2kbd/ps2kbd_wrapper.h"
#include "ff.h"
#include "snes9x/snapshot.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef USB_HID_ENABLED
#include "usbhid/usbhid.h"
#endif

/* Screen dimensions */
#define SCREEN_WIDTH  256
#define SCREEN_HEIGHT 224

/* Font constants (5x7 bitmap font) */
#define FONT_WIDTH  6    /* 5px glyph + 1px spacing */
#define FONT_HEIGHT 7
#define LINE_HEIGHT 12

/* UI layout */
#define MENU_TITLE_Y  16
#define MENU_START_Y  40
#define MENU_X        20
#define VALUE_X       132

/* Palette indices for menu rendering */
#define PAL_BG          1
#define PAL_WHITE       2
#define PAL_YELLOW      3
#define PAL_GRAY        4
#define PAL_RED         5
#define PAL_DARKGRAY    6   /* Used by scrollbar track */

/* ─── Menu pages ──────────────────────────────────────────────────── */

typedef enum {
    PAGE_MAIN,
    PAGE_VIDEO,
    PAGE_AUDIO,
    PAGE_BUTTONS,      // Device selector
    PAGE_BTNMAP_KBD,   // Keyboard mapping
    PAGE_BTNMAP_NES,   // NES/SNES pad mapping
    PAGE_BTNMAP_USB,   // USB gamepad mapping
} menu_page_t;

/* Main menu items */
typedef enum {
    MAIN_VOLUME,
    MAIN_CRT,
    MAIN_BW,
    MAIN_FRAMESKIP,
    MAIN_SEP1,
    MAIN_PLAYER1,
    MAIN_PLAYER2,
    MAIN_BUTTONS,
    MAIN_SEP2,
    MAIN_SAVE_GAME,
    MAIN_LOAD_GAME,
    MAIN_SEP3,
    MAIN_VIDEO,
    MAIN_AUDIO,
    MAIN_SEP4,
    MAIN_RESTART,
    MAIN_BACK_ROM,
    MAIN_BACK_GAME,
    MAIN_ITEM_COUNT
} main_item_t;

/* Video submenu items */
typedef enum {
    VIDEO_BG1,
    VIDEO_BG2,
    VIDEO_BG3,
    VIDEO_BG4,
    VIDEO_SPRITES,
    VIDEO_TRANSPARENCY,
    VIDEO_HDMA,
    VIDEO_OVERSCAN,
    VIDEO_SEP,
    VIDEO_BACK,
    VIDEO_ITEM_COUNT
} video_item_t;

/* Audio submenu items */
typedef enum {
    AUDIO_ECHO,
    AUDIO_INTERP,
    AUDIO_SEP,
    AUDIO_BACK,
    AUDIO_ITEM_COUNT
} audio_item_t;

/* Global settings with defaults */
settings_t g_settings = {
    .p1_mode = INPUT_MODE_ANY,
    .p2_mode = INPUT_MODE_DISABLED,
    .volume = 100,
    .crt_effect = false,
    .greyscale = false,
    .frameskip = 2,  /* medium */
    .bg_enabled = 0x0F,  /* all BGs on */
    .sprites_enabled = true,
    .transparency_enabled = true,
    .hdma_enabled = true,
    .crt_overscan = false,
    .echo_enabled = false,
    .interpolation = true,
    .btnmap_kbd = BTNMAP_DEFAULT,
    .btnmap_nes = BTNMAP_DEFAULT,
    .btnmap_usb = BTNMAP_DEFAULT,
    .selector_mode = SELECTOR_MODE_CAROUSEL,
    .browser_path = "",
    .browser_file = "",
};

/* Edit copy */
static settings_t edit;

/* Current page and selection */
static menu_page_t current_page;
static int selected;
static bool menu_in_game;  /* true = called from emulation */

/* Input mode names */
static const char *input_mode_names[] = {
    "ANY", "NES 1", "NES 2", "USB 1", "USB 2", "KEYBOARD", "DISABLED"
};

/* Frameskip names */
static const char *frameskip_names[] = {
    "NONE", "LOW", "MEDIUM", "HIGH", "EXTREME"
};

/* ─── 5x7 bitmap font ────────────────────────────────────────────── */

static const uint8_t glyphs_5x7[][7] = {
    [' '-' '] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    ['!'-' '] = {0x04, 0x04, 0x04, 0x04, 0x00, 0x04, 0x00},
    ['"'-' '] = {0x0A, 0x0A, 0x0A, 0x00, 0x00, 0x00, 0x00},
    ['#'-' '] = {0x0A, 0x0A, 0x1F, 0x0A, 0x1F, 0x0A, 0x0A},
    ['$'-' '] = {0x04, 0x0F, 0x14, 0x0E, 0x05, 0x1E, 0x04},
    ['%'-' '] = {0x19, 0x1A, 0x04, 0x08, 0x0B, 0x13, 0x00},
    ['&'-' '] = {0x0C, 0x12, 0x14, 0x08, 0x15, 0x12, 0x0D},
    ['\''-' '] = {0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00},
    ['('-' '] = {0x02, 0x04, 0x08, 0x08, 0x08, 0x04, 0x02},
    [')'-' '] = {0x08, 0x04, 0x02, 0x02, 0x02, 0x04, 0x08},
    ['*'-' '] = {0x00, 0x04, 0x15, 0x0E, 0x15, 0x04, 0x00},
    ['+'-' '] = {0x00, 0x04, 0x04, 0x1F, 0x04, 0x04, 0x00},
    [','-' '] = {0x00, 0x00, 0x00, 0x00, 0x0C, 0x04, 0x08},
    ['-'-' '] = {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00},
    ['.'-' '] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C},
    ['/'-' '] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x00, 0x00},
    ['0'-' '] = {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E},
    ['1'-' '] = {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},
    ['2'-' '] = {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F},
    ['3'-' '] = {0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E},
    ['4'-' '] = {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02},
    ['5'-' '] = {0x1F, 0x10, 0x10, 0x1E, 0x01, 0x01, 0x1E},
    ['6'-' '] = {0x0E, 0x10, 0x10, 0x1E, 0x11, 0x11, 0x0E},
    ['7'-' '] = {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},
    ['8'-' '] = {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E},
    ['9'-' '] = {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x0E},
    [':'-' '] = {0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00},
    [';'-' '] = {0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x04, 0x08},
    ['<'-' '] = {0x02, 0x04, 0x08, 0x10, 0x08, 0x04, 0x02},
    ['='-' '] = {0x00, 0x00, 0x1F, 0x00, 0x1F, 0x00, 0x00},
    ['>'-' '] = {0x08, 0x04, 0x02, 0x01, 0x02, 0x04, 0x08},
    ['?'-' '] = {0x0E, 0x11, 0x01, 0x02, 0x04, 0x00, 0x04},
    ['@'-' '] = {0x0E, 0x11, 0x17, 0x15, 0x17, 0x10, 0x0E},
    ['A'-' '] = {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},
    ['B'-' '] = {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E},
    ['C'-' '] = {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E},
    ['D'-' '] = {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E},
    ['E'-' '] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F},
    ['F'-' '] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10},
    ['G'-' '] = {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0E},
    ['H'-' '] = {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},
    ['I'-' '] = {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1F},
    ['J'-' '] = {0x07, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0C},
    ['K'-' '] = {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11},
    ['L'-' '] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F},
    ['M'-' '] = {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11},
    ['N'-' '] = {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11},
    ['O'-' '] = {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E},
    ['P'-' '] = {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10},
    ['Q'-' '] = {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D},
    ['R'-' '] = {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11},
    ['S'-' '] = {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E},
    ['T'-' '] = {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
    ['U'-' '] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E},
    ['V'-' '] = {0x11, 0x11, 0x11, 0x11, 0x0A, 0x0A, 0x04},
    ['W'-' '] = {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A},
    ['X'-' '] = {0x11, 0x0A, 0x04, 0x04, 0x04, 0x0A, 0x11},
    ['Y'-' '] = {0x11, 0x0A, 0x04, 0x04, 0x04, 0x04, 0x04},
    ['Z'-' '] = {0x1F, 0x02, 0x04, 0x08, 0x10, 0x10, 0x1F},
    ['['-' '] = {0x0E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0E},
    ['\\'-' '] = {0x10, 0x08, 0x04, 0x02, 0x01, 0x00, 0x00},
    [']'-' '] = {0x0E, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0E},
    ['^'-' '] = {0x04, 0x0A, 0x11, 0x00, 0x00, 0x00, 0x00},
    ['_'-' '] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F},
    ['{'-' '] = {0x02, 0x04, 0x04, 0x08, 0x04, 0x04, 0x02},
    ['|'-' '] = {0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
    ['}'-' '] = {0x08, 0x04, 0x04, 0x02, 0x04, 0x04, 0x08},
    ['~'-' '] = {0x00, 0x00, 0x08, 0x15, 0x02, 0x00, 0x00},
};

static const uint8_t *glyph_5x7(char ch) {
    static const uint8_t glyph_space[7] = {0};
    int c = (unsigned char)ch;
    if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';
    int idx = c - ' ';
    if (idx >= 0 && idx < (int)(sizeof(glyphs_5x7)/sizeof(glyphs_5x7[0])))
        return glyphs_5x7[idx];
    return glyph_space;
}

/* ─── Drawing primitives ──────────────────────────────────────────── */

static void draw_char(uint8_t *screen, int x, int y, char ch, uint8_t color) {
    const uint8_t *rows = glyph_5x7(ch);
    for (int row = 0; row < FONT_HEIGHT; ++row) {
        int yy = y + row;
        if (yy < 0 || yy >= SCREEN_HEIGHT) continue;
        uint8_t bits = rows[row];
        for (int col = 0; col < 5; ++col) {
            int xx = x + col;
            if (xx < 0 || xx >= SCREEN_WIDTH) continue;
            if (bits & (1u << (4 - col)))
                screen[yy * SCREEN_WIDTH + xx] = color;
        }
    }
}

static void draw_text(uint8_t *screen, int x, int y, const char *text, uint8_t color) {
    for (const char *p = text; *p; ++p) {
        draw_char(screen, x, y, *p, color);
        x += FONT_WIDTH;
    }
}

static int text_width(const char *text) {
    return (int)strlen(text) * FONT_WIDTH;
}

static void draw_text_centered(uint8_t *screen, int y, const char *text, uint8_t color) {
    int x = (SCREEN_WIDTH - text_width(text)) / 2;
    if (x < 0) x = 0;
    draw_text(screen, x, y, text, color);
}

static void draw_hline(uint8_t *screen, int x, int y, int w, uint8_t color) {
    if (y < 0 || y >= SCREEN_HEIGHT) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > SCREEN_WIDTH) w = SCREEN_WIDTH - x;
    if (w <= 0) return;
    memset(&screen[y * SCREEN_WIDTH + x], color, (size_t)w);
}

static void draw_rect(uint8_t *screen, int x, int y, int w, int h, uint8_t color) {
    for (int yy = y; yy < y + h && yy < SCREEN_HEIGHT; yy++) {
        if (yy < 0) continue;
        int x0 = x < 0 ? 0 : x;
        int x1 = (x + w) > SCREEN_WIDTH ? SCREEN_WIDTH : (x + w);
        if (x0 < x1) memset(&screen[yy * SCREEN_WIDTH + x0], color, (size_t)(x1 - x0));
    }
}

/* ─── Menu palette setup ──────────────────────────────────────────── */

static void setup_menu_palette(void) {
    graphics_set_palette(0, 0x000000);
    graphics_set_palette(PAL_BG,       0x080810);    /* dark blue-gray */
    graphics_set_palette(PAL_WHITE,    0xFFFFFF);
    graphics_set_palette(PAL_YELLOW,   0xFFFF00);
    graphics_set_palette(PAL_GRAY,     0x808080);
    graphics_set_palette(PAL_RED,      0xFF4444);
    graphics_set_palette(PAL_DARKGRAY, 0x404048);
    graphics_restore_sync_colors();
}

/* ─── Save state (SD card) ────────────────────────────────────────── */

static bool save_exists = false;
static const char *status_msg = NULL;
static int status_frames = 0;
static const char *save_error = NULL;

static void get_save_path(char *path, size_t path_size) {
    snprintf(path, path_size, "/snes/.save/%s.sav", g_rom_name);
}

static bool check_save_exists(void) {
    if (g_rom_name[0] == '\0') return false;
    char path[128];
    get_save_path(path, sizeof(path));
    FILINFO fno;
    return (f_stat(path, &fno) == FR_OK);
}

/* Reuse the FatFS FATFS object from main.c (already mounted during emulation) */
extern FATFS fs;

static bool do_save_game(void) {
    save_error = NULL;
    if (g_rom_name[0] == '\0') { save_error = "NO ROM NAME"; return false; }

    char path[128];
    get_save_path(path, sizeof(path));
    printf("do_save: saving to %s\n", path);

    /* Try open first; only mkdir if it fails (avoids extra LFN mallocs) */
    FIL file;
    FRESULT fr = f_open(&file, path, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        f_mkdir("/snes");
        f_mkdir("/snes/.save");
        fr = f_open(&file, path, FA_WRITE | FA_CREATE_ALWAYS);
    }
    if (fr != FR_OK) {
        save_error = "FILE OPEN FAIL";
        printf("do_save: f_open failed (%d)\n", fr);
        return false;
    }

    bool ok = S9xSaveState(&file);
    f_close(&file);

    if (!ok) { save_error = "STATE FAILED"; return false; }
    printf("do_save: OK\n");
    return true;
}

static bool do_load_game(void) {
    if (g_rom_name[0] == '\0') return false;

    char path[128];
    get_save_path(path, sizeof(path));
    printf("do_load: loading from %s\n", path);

    FIL file;
    if (f_open(&file, path, FA_READ) != FR_OK) return false;

    bool ok = S9xLoadState(&file);
    f_close(&file);
    printf("do_load: %s\n", ok ? "OK" : "FAILED");
    return ok;
}

/* ─── Input reading (merge all sources) ───────────────────────────── */

#define BTN_UP    0x10
#define BTN_DOWN  0x20
#define BTN_LEFT  0x40
#define BTN_RIGHT 0x80
#define BTN_A     0x01
#define BTN_B     0x02
#define BTN_START 0x08
#define BTN_SEL   0x04

static int read_menu_buttons(void) {
    nespad_read();
    ps2kbd_tick();

    int buttons = 0;

    /* NES/SNES gamepad (either player) */
    uint32_t pad = nespad_state | nespad_state2;
    if (pad & DPAD_A)      buttons |= BTN_A;
    if (pad & DPAD_B)      buttons |= BTN_B;
    if (pad & DPAD_SELECT) buttons |= BTN_SEL;
    if (pad & DPAD_START)  buttons |= BTN_START;
    if (pad & DPAD_UP)     buttons |= BTN_UP;
    if (pad & DPAD_DOWN)   buttons |= BTN_DOWN;
    if (pad & DPAD_LEFT)   buttons |= BTN_LEFT;
    if (pad & DPAD_RIGHT)  buttons |= BTN_RIGHT;

    /* PS/2 keyboard */
    uint16_t kbd = ps2kbd_get_state();
#ifdef USB_HID_ENABLED
    kbd |= usbhid_get_kbd_state();
#endif
    if (kbd & KBD_STATE_UP)     buttons |= BTN_UP;
    if (kbd & KBD_STATE_DOWN)   buttons |= BTN_DOWN;
    if (kbd & KBD_STATE_LEFT)   buttons |= BTN_LEFT;
    if (kbd & KBD_STATE_RIGHT)  buttons |= BTN_RIGHT;
    if (kbd & KBD_STATE_A)      buttons |= BTN_A;
    if (kbd & KBD_STATE_B)      buttons |= BTN_B;
    if (kbd & KBD_STATE_SELECT) buttons |= BTN_SEL;
    if (kbd & KBD_STATE_START)  buttons |= BTN_START;
    if (kbd & KBD_STATE_ESC)    buttons |= BTN_B;  /* ESC = back */

#ifdef USB_HID_ENABLED
    usbhid_task();
    if (usbhid_gamepad_connected()) {
        usbhid_gamepad_state_t gp;
        usbhid_get_gamepad_state(&gp);
        if (gp.dpad & 0x01) buttons |= BTN_UP;
        if (gp.dpad & 0x02) buttons |= BTN_DOWN;
        if (gp.dpad & 0x04) buttons |= BTN_LEFT;
        if (gp.dpad & 0x08) buttons |= BTN_RIGHT;
        if (gp.buttons & 0x0001) buttons |= BTN_A;
        if (gp.buttons & 0x0002) buttons |= BTN_B;
        if (gp.buttons & 0x0040) buttons |= BTN_START;
        if (gp.buttons & 0x0080) buttons |= BTN_SEL;
    }
#endif

    return buttons;
}

/* ─── Item helpers (main menu) ────────────────────────────────────── */

static bool is_separator_main(int item) {
    return item == MAIN_SEP1 || item == MAIN_SEP2 || item == MAIN_SEP3 || item == MAIN_SEP4;
}

static bool is_hidden_main(int item) {
    /* Hide "Back to Game" when not in a game */
    if (!menu_in_game && item == MAIN_BACK_GAME) return true;
    /* Hide save/load and restart when not in a game */
    if (!menu_in_game && (item == MAIN_SAVE_GAME || item == MAIN_LOAD_GAME ||
                          item == MAIN_SEP3 || item == MAIN_RESTART)) return true;
    return false;
}

static bool is_selectable_main(int item) {
    if (is_separator_main(item)) return false;
    if (is_hidden_main(item)) return false;
    if (item == MAIN_LOAD_GAME && !save_exists) return false;
    return true;
}

static const char *main_label(int item) {
    switch (item) {
        case MAIN_VOLUME:    return "VOLUME";
        case MAIN_CRT:       return "CRT EFFECT";
        case MAIN_BW:        return "BLACK & WHITE";
        case MAIN_FRAMESKIP: return "FRAMESKIP";
        case MAIN_PLAYER1:   return "GAMEPAD 1";
        case MAIN_PLAYER2:   return "GAMEPAD 2";
        case MAIN_SAVE_GAME: return (status_frames > 0) ? status_msg : "SAVE GAME";
        case MAIN_LOAD_GAME: return save_exists ? "LOAD GAME" : "LOAD GAME (-)";
        case MAIN_BUTTONS:   return "BUTTON MAPPING...";
        case MAIN_VIDEO:     return "VIDEO SETTINGS...";
        case MAIN_AUDIO:     return "AUDIO SETTINGS...";
        case MAIN_RESTART:   return "RESTART GAME";
        case MAIN_BACK_GAME: return "BACK TO GAME";
        case MAIN_BACK_ROM:  return menu_in_game ? "CHANGE ROM" : "BACK";
        default:             return "";
    }
}

static char value_buf[24];

static const char *main_value(int item) {
    switch (item) {
        case MAIN_VOLUME:
            if (edit.volume == 0) return "OFF";
            snprintf(value_buf, sizeof(value_buf), "%d%%", edit.volume);
            return value_buf;
        case MAIN_CRT:
            return edit.crt_effect ? "ON" : "OFF";
        case MAIN_BW:
            return edit.greyscale ? "ON" : "OFF";
        case MAIN_FRAMESKIP:
            return frameskip_names[edit.frameskip];
        case MAIN_PLAYER1:
            return input_mode_names[edit.p1_mode];
        case MAIN_PLAYER2:
            return input_mode_names[edit.p2_mode];
        default:
            return NULL;
    }
}

/* Check if a specific mode is already taken by the other player */
static bool mode_taken_by_other(uint8_t mode, uint8_t other_mode) {
    if (mode == INPUT_MODE_ANY || mode == INPUT_MODE_DISABLED) return false;
    return mode == other_mode;
}

static void main_change_value(int item, int dir) {
    switch (item) {
        case MAIN_VOLUME: {
            int v = (int)edit.volume + dir * VOLUME_STEP;
            if (v < VOLUME_MIN) v = VOLUME_MIN;
            if (v > VOLUME_MAX) v = VOLUME_MAX;
            edit.volume = (uint8_t)v;
            break;
        }
        case MAIN_CRT:
            edit.crt_effect = !edit.crt_effect;
            break;
        case MAIN_BW:
            edit.greyscale = !edit.greyscale;
            break;
        case MAIN_FRAMESKIP:
            if (dir > 0) { if (edit.frameskip < 4) edit.frameskip++; else edit.frameskip = 0; }
            else { if (edit.frameskip > 0) edit.frameskip--; else edit.frameskip = 4; }
            break;
        case MAIN_PLAYER1: {
            uint8_t m = edit.p1_mode;
            for (int i = 0; i < INPUT_MODE_COUNT; i++) {
                m = (uint8_t)((m + INPUT_MODE_COUNT + dir) % INPUT_MODE_COUNT);
                if (m == INPUT_MODE_DISABLED) continue;
                if (m == INPUT_MODE_ANY && edit.p2_mode != INPUT_MODE_DISABLED) continue;
                if (mode_taken_by_other(m, edit.p2_mode)) continue;
                break;
            }
            edit.p1_mode = m;
            break;
        }
        case MAIN_PLAYER2: {
            uint8_t m = edit.p2_mode;
            for (int i = 0; i < INPUT_MODE_COUNT; i++) {
                m = (uint8_t)((m + INPUT_MODE_COUNT + dir) % INPUT_MODE_COUNT);
                if (m == INPUT_MODE_ANY) continue;
                if (mode_taken_by_other(m, edit.p1_mode)) continue;
                break;
            }
            edit.p2_mode = m;
            if (edit.p2_mode != INPUT_MODE_DISABLED && edit.p1_mode == INPUT_MODE_ANY) {
                for (uint8_t c = INPUT_MODE_NES1; c <= INPUT_MODE_KEYBOARD; c++) {
                    if (c != edit.p2_mode) { edit.p1_mode = c; break; }
                }
            }
            break;
        }
        default:
            break;
    }
}

/* ─── Item helpers (video submenu) ────────────────────────────────── */

static bool is_separator_video(int item) { return item == VIDEO_SEP; }
static bool is_selectable_video(int item) { return !is_separator_video(item); }

static const char *video_label(int item) {
    switch (item) {
        case VIDEO_BG1:          return "BACKGROUND 1";
        case VIDEO_BG2:          return "BACKGROUND 2";
        case VIDEO_BG3:          return "BACKGROUND 3";
        case VIDEO_BG4:          return "BACKGROUND 4";
        case VIDEO_SPRITES:      return "SPRITES";
        case VIDEO_TRANSPARENCY: return "TRANSPARENCY";
        case VIDEO_HDMA:         return "HDMA";
        case VIDEO_OVERSCAN:     return "CRT OVERSCAN";
        case VIDEO_BACK:         return "BACK";
        default:                 return "";
    }
}

static const char *video_value(int item) {
    switch (item) {
        case VIDEO_BG1: return (edit.bg_enabled & 0x01) ? "ON" : "OFF";
        case VIDEO_BG2: return (edit.bg_enabled & 0x02) ? "ON" : "OFF";
        case VIDEO_BG3: return (edit.bg_enabled & 0x04) ? "ON" : "OFF";
        case VIDEO_BG4: return (edit.bg_enabled & 0x08) ? "ON" : "OFF";
        case VIDEO_SPRITES:      return edit.sprites_enabled ? "ON" : "OFF";
        case VIDEO_TRANSPARENCY: return edit.transparency_enabled ? "ON" : "OFF";
        case VIDEO_HDMA:         return edit.hdma_enabled ? "ON" : "OFF";
        case VIDEO_OVERSCAN:     return edit.crt_overscan ? "ON" : "OFF";
        default: return NULL;
    }
}

static void video_change_value(int item, int dir) {
    (void)dir;
    switch (item) {
        case VIDEO_BG1: edit.bg_enabled ^= 0x01; break;
        case VIDEO_BG2: edit.bg_enabled ^= 0x02; break;
        case VIDEO_BG3: edit.bg_enabled ^= 0x04; break;
        case VIDEO_BG4: edit.bg_enabled ^= 0x08; break;
        case VIDEO_SPRITES:      edit.sprites_enabled = !edit.sprites_enabled; break;
        case VIDEO_TRANSPARENCY: edit.transparency_enabled = !edit.transparency_enabled; break;
        case VIDEO_HDMA:         edit.hdma_enabled = !edit.hdma_enabled; break;
        case VIDEO_OVERSCAN:     edit.crt_overscan = !edit.crt_overscan; break;
        default: break;
    }
}

/* ─── Item helpers (audio submenu) ────────────────────────────────── */

static bool is_separator_audio(int item) { return item == AUDIO_SEP; }
static bool is_selectable_audio(int item) { return !is_separator_audio(item); }

static const char *audio_label(int item) {
    switch (item) {
        case AUDIO_ECHO:   return "SOUND ECHO";
        case AUDIO_INTERP: return "INTERPOLATION";
        case AUDIO_BACK:   return "BACK";
        default:           return "";
    }
}

static const char *audio_value(int item) {
    switch (item) {
        case AUDIO_ECHO:   return edit.echo_enabled ? "ON" : "OFF";
        case AUDIO_INTERP: return edit.interpolation ? "ON" : "OFF";
        default: return NULL;
    }
}

static void audio_change_value(int item, int dir) {
    (void)dir;
    switch (item) {
        case AUDIO_ECHO:   edit.echo_enabled = !edit.echo_enabled; break;
        case AUDIO_INTERP: edit.interpolation = !edit.interpolation; break;
        default: break;
    }
}

/* ─── Item helpers (button mapping device selector) ───────────────── */

typedef enum {
    BTN_DEV_KEYBOARD,
    BTN_DEV_NES,
    BTN_DEV_USB,
    BTN_DEV_SEP,
    BTN_DEV_BACK,
    BTN_DEV_ITEM_COUNT
} btn_dev_item_t;

static bool is_separator_btndev(int item) { return item == BTN_DEV_SEP; }
static bool is_selectable_btndev(int item) { return !is_separator_btndev(item); }

static const char *btndev_label(int item) {
    switch (item) {
        case BTN_DEV_KEYBOARD: return "KEYBOARD";
        case BTN_DEV_NES:      return "NES/SNES GAMEPAD";
        case BTN_DEV_USB:      return "USB GAMEPAD";
        case BTN_DEV_BACK:     return "BACK";
        default:               return "";
    }
}

static const char *btndev_value(int item) { (void)item; return NULL; }

/* ─── Item helpers (per-device button mapping) ────────────────────── */

typedef enum {
    BMAP_A,
    BMAP_B,
    BMAP_X,
    BMAP_Y,
    BMAP_L,
    BMAP_R,
    BMAP_START,
    BMAP_SELECT,
    BMAP_SEP,
    BMAP_RESET,
    BMAP_BACK,
    BMAP_ITEM_COUNT
} bmap_item_t;

static const char *phys_btn_names[] = {
    "A", "B", "X", "Y", "L", "R", "START", "SELECT"
};

static const char *snes_btn_labels[] = {
    "SNES A", "SNES B", "SNES X", "SNES Y",
    "SNES L", "SNES R", "SNES START", "SNES SELECT"
};

/* Which device's map are we editing? Set when entering a mapping page */
static button_map_t *edit_btnmap = NULL;

static bool is_separator_bmap(int item) { return item == BMAP_SEP; }
static bool is_selectable_bmap(int item) { return !is_separator_bmap(item); }

static const char *bmap_label(int item) {
    if (item >= BMAP_A && item <= BMAP_SELECT) return snes_btn_labels[item];
    if (item == BMAP_RESET) return "RESET TO DEFAULT";
    if (item == BMAP_BACK)  return "BACK";
    return "";
}

/* Scan mode: -1 = not scanning, 0-7 = scanning for SNES button index */
static int scan_active = -1;
static bool scan_wait_release = false;  /* wait for device buttons to be released first */

static const char *bmap_value(int item) {
    if (item >= BMAP_A && item <= BMAP_SELECT && edit_btnmap) {
        if (scan_active == item) return "PRESS...";
        return phys_btn_names[edit_btnmap->map[item]];
    }
    return NULL;
}

/* NES pad physical button bits indexed by BTNMAP_* (for scan detection)
 * Physical SNES A = DPAD_Y, B = DPAD_A, X = DPAD_X, Y = DPAD_B */
static const uint32_t scan_nes_bits[BTNMAP_COUNT] = {
    DPAD_Y, DPAD_A, DPAD_X, DPAD_B,
    DPAD_LT, DPAD_RT, DPAD_START, DPAD_SELECT
};

/* Keyboard state bits indexed by BTNMAP_* (for scan detection) */
static const uint16_t scan_kbd_bits[BTNMAP_COUNT] = {
    KBD_STATE_A, KBD_STATE_B, KBD_STATE_X, KBD_STATE_Y,
    KBD_STATE_L, KBD_STATE_R, KBD_STATE_START, KBD_STATE_SELECT
};

/* USB gamepad button bits indexed by BTNMAP_* (for scan detection) */
static const uint16_t scan_usb_bits[BTNMAP_COUNT] = {
    0x0001, 0x0002, 0x0004, 0x0008,
    0x0010, 0x0020, 0x0040, 0x0080
};

/* Poll device-specific raw input and return detected physical button index, or -1.
 * Caller must have already called nespad_read()/ps2kbd_tick()/usbhid_task(). */
static int scan_detect_button(menu_page_t page) {
    switch (page) {
        case PAGE_BTNMAP_NES: {
            uint32_t pad = nespad_state | nespad_state2;
            for (int i = 0; i < BTNMAP_COUNT; i++)
                if (pad & scan_nes_bits[i]) return i;
            break;
        }
        case PAGE_BTNMAP_KBD: {
            uint16_t kbd = ps2kbd_get_state();
#ifdef USB_HID_ENABLED
            kbd |= usbhid_get_kbd_state();
#endif
            for (int i = 0; i < BTNMAP_COUNT; i++)
                if (kbd & scan_kbd_bits[i]) return i;
            break;
        }
#ifdef USB_HID_ENABLED
        case PAGE_BTNMAP_USB: {
            if (usbhid_gamepad_connected()) {
                usbhid_gamepad_state_t gp;
                usbhid_get_gamepad_state(&gp);
                for (int i = 0; i < BTNMAP_COUNT; i++)
                    if (gp.buttons & scan_usb_bits[i]) return i;
            }
            break;
        }
#endif
        default:
            break;
    }
    return -1;
}

/* ─── Generic navigation ──────────────────────────────────────────── */

typedef bool (*is_selectable_fn)(int);

static int next_selectable(int sel, int dir, int count, is_selectable_fn fn) {
    int s = sel;
    for (int i = 0; i < count; i++) {
        s += dir;
        if (s < 0) s = count - 1;
        if (s >= count) s = 0;
        if (fn(s)) return s;
    }
    return sel;
}

/* ─── Menu drawing ────────────────────────────────────────────────── */

static void draw_menu(uint8_t *screen, const char *title, int item_count,
                      const char *(*get_label)(int), const char *(*get_value)(int),
                      bool (*is_sep)(int), bool (*is_hidden)(int),
                      is_selectable_fn is_sel, int sel)
{
    /* Clear screen */
    memset(screen, PAL_BG, SCREEN_WIDTH * SCREEN_HEIGHT);

    /* Title */
    draw_text_centered(screen, MENU_TITLE_Y, title, PAL_WHITE);
    draw_hline(screen, MENU_X, MENU_TITLE_Y + FONT_HEIGHT + 3, SCREEN_WIDTH - 2 * MENU_X, PAL_GRAY);

    /* Help text (always at bottom) */
    const int help_y = SCREEN_HEIGHT - 14;
    draw_text_centered(screen, help_y, "D-PAD:NAV  L/R:CHANGE  B:BACK", PAL_GRAY);

    /* Compute visible area for menu items */
    const int vis_top = MENU_START_Y;
    const int vis_bottom = help_y - 4;  /* leave gap above help */
    const int vis_height = vis_bottom - vis_top;

    /* Count total height and find selected item's position */
    int total_height = 0;
    int sel_y_start = 0;
    for (int i = 0; i < item_count; i++) {
        if (is_hidden && is_hidden(i)) continue;
        if (i == sel) sel_y_start = total_height;
        total_height += LINE_HEIGHT;
    }

    /* Compute scroll offset so selected item is visible */
    static int scroll_offset = 0;
    if (sel_y_start < scroll_offset)
        scroll_offset = sel_y_start;
    if (sel_y_start + LINE_HEIGHT > scroll_offset + vis_height)
        scroll_offset = sel_y_start + LINE_HEIGHT - vis_height;
    if (scroll_offset < 0) scroll_offset = 0;
    if (total_height <= vis_height) scroll_offset = 0;

    /* Draw menu items with scroll offset */
    int y = vis_top - scroll_offset;
    for (int i = 0; i < item_count; i++) {
        if (is_hidden && is_hidden(i)) continue;

        /* Skip items fully outside visible area */
        if (y + LINE_HEIGHT <= vis_top) { y += LINE_HEIGHT; continue; }
        if (y >= vis_bottom) break;

        if (is_sep(i)) {
            draw_hline(screen, MENU_X, y + LINE_HEIGHT / 2, SCREEN_WIDTH - 2 * MENU_X, PAL_GRAY);
            y += LINE_HEIGHT;
            continue;
        }

        uint8_t color;
        if (is_sel && !is_sel(i))
            color = PAL_GRAY;
        else
            color = (i == sel) ? PAL_YELLOW : PAL_WHITE;

        /* Selection indicator */
        if (i == sel)
            draw_char(screen, MENU_X - 10, y, '>', color);

        /* Label */
        draw_text(screen, MENU_X, y, get_label(i), color);

        /* Value with arrows */
        const char *val = get_value(i);
        if (val) {
            char buf[32];
            snprintf(buf, sizeof(buf), "< %s >", val);
            draw_text(screen, VALUE_X, y, buf, color);
        }

        y += LINE_HEIGHT;
    }

    /* Scrollbar — only drawn when the list overflows */
    if (total_height > vis_height) {
        int sb_x = SCREEN_WIDTH - 8;
        int sb_w = 3;
        int track_y = vis_top;
        int track_h = vis_height;
        int thumb_h = track_h * vis_height / total_height;
        if (thumb_h < 6) thumb_h = 6;
        int max_scroll = total_height - vis_height;
        int thumb_y = track_y;
        if (max_scroll > 0)
            thumb_y += (track_h - thumb_h) * scroll_offset / max_scroll;
        draw_rect(screen, sb_x, track_y, sb_w, track_h, PAL_DARKGRAY);
        draw_rect(screen, sb_x, thumb_y, sb_w, thumb_h, PAL_WHITE);
    }
}

/* ─── Settings persistence ────────────────────────────────────────── */

#define SETTINGS_PATH "/snes/settings.ini"

void settings_load(void) {
    FIL file;
    FRESULT res = f_open(&file, SETTINGS_PATH, FA_READ);
    if (res != FR_OK) {
        res = f_open(&file, "/SNES/settings.ini", FA_READ);
        if (res != FR_OK) return;
    }

    char line[128];
    while (f_gets(line, sizeof(line), &file)) {
        char *eq = strchr(line, '=');
        if (!eq) continue;
        *eq = '\0';
        char *key = line;
        char *value = eq + 1;
        while (*key == ' ') key++;
        while (*value == ' ') value++;
        char *end = value + strlen(value) - 1;
        while (end > value && (*end == '\n' || *end == '\r' || *end == ' ')) *end-- = '\0';

        if (strcmp(key, "p1_mode") == 0) {
            int v = atoi(value);
            if (v >= 0 && v < INPUT_MODE_COUNT) g_settings.p1_mode = (uint8_t)v;
        } else if (strcmp(key, "p2_mode") == 0) {
            int v = atoi(value);
            if (v >= 0 && v < INPUT_MODE_COUNT) g_settings.p2_mode = (uint8_t)v;
        } else if (strcmp(key, "volume") == 0) {
            int v = atoi(value);
            if (v >= VOLUME_MIN && v <= VOLUME_MAX) g_settings.volume = (uint8_t)v;
        } else if (strcmp(key, "crt_effect") == 0) {
            g_settings.crt_effect = (atoi(value) != 0);
        } else if (strcmp(key, "greyscale") == 0) {
            g_settings.greyscale = (atoi(value) != 0);
        } else if (strcmp(key, "frameskip") == 0) {
            int v = atoi(value);
            if (v >= 0 && v <= 4) g_settings.frameskip = (uint8_t)v;
        } else if (strcmp(key, "bg_enabled") == 0) {
            g_settings.bg_enabled = (uint8_t)(atoi(value) & 0x0F);
        } else if (strcmp(key, "sprites") == 0) {
            g_settings.sprites_enabled = (atoi(value) != 0);
        } else if (strcmp(key, "transparency") == 0) {
            g_settings.transparency_enabled = (atoi(value) != 0);
        } else if (strcmp(key, "hdma") == 0) {
            g_settings.hdma_enabled = (atoi(value) != 0);
        } else if (strcmp(key, "crt_overscan") == 0) {
            g_settings.crt_overscan = (atoi(value) != 0);
        } else if (strcmp(key, "echo") == 0) {
            g_settings.echo_enabled = (atoi(value) != 0);
        } else if (strcmp(key, "interpolation") == 0) {
            g_settings.interpolation = (atoi(value) != 0);
        } else if (strcmp(key, "btnmap_kbd") == 0) {
            sscanf(value, "%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu",
                   &g_settings.btnmap_kbd.map[0], &g_settings.btnmap_kbd.map[1],
                   &g_settings.btnmap_kbd.map[2], &g_settings.btnmap_kbd.map[3],
                   &g_settings.btnmap_kbd.map[4], &g_settings.btnmap_kbd.map[5],
                   &g_settings.btnmap_kbd.map[6], &g_settings.btnmap_kbd.map[7]);
            for (int i = 0; i < BTNMAP_COUNT; i++)
                if (g_settings.btnmap_kbd.map[i] >= BTNMAP_COUNT) g_settings.btnmap_kbd.map[i] = (uint8_t)i;
        } else if (strcmp(key, "btnmap_nes") == 0) {
            sscanf(value, "%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu",
                   &g_settings.btnmap_nes.map[0], &g_settings.btnmap_nes.map[1],
                   &g_settings.btnmap_nes.map[2], &g_settings.btnmap_nes.map[3],
                   &g_settings.btnmap_nes.map[4], &g_settings.btnmap_nes.map[5],
                   &g_settings.btnmap_nes.map[6], &g_settings.btnmap_nes.map[7]);
            for (int i = 0; i < BTNMAP_COUNT; i++)
                if (g_settings.btnmap_nes.map[i] >= BTNMAP_COUNT) g_settings.btnmap_nes.map[i] = (uint8_t)i;
        } else if (strcmp(key, "btnmap_usb") == 0) {
            sscanf(value, "%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu",
                   &g_settings.btnmap_usb.map[0], &g_settings.btnmap_usb.map[1],
                   &g_settings.btnmap_usb.map[2], &g_settings.btnmap_usb.map[3],
                   &g_settings.btnmap_usb.map[4], &g_settings.btnmap_usb.map[5],
                   &g_settings.btnmap_usb.map[6], &g_settings.btnmap_usb.map[7]);
            for (int i = 0; i < BTNMAP_COUNT; i++)
                if (g_settings.btnmap_usb.map[i] >= BTNMAP_COUNT) g_settings.btnmap_usb.map[i] = (uint8_t)i;
        } else if (strcmp(key, "selector_mode") == 0) {
            if (strcmp(value, "browser") == 0)
                g_settings.selector_mode = SELECTOR_MODE_BROWSER;
            else
                g_settings.selector_mode = SELECTOR_MODE_CAROUSEL;
        } else if (strcmp(key, "browser_path") == 0) {
            strncpy(g_settings.browser_path, value, sizeof(g_settings.browser_path) - 1);
            g_settings.browser_path[sizeof(g_settings.browser_path) - 1] = '\0';
        } else if (strcmp(key, "browser_file") == 0) {
            strncpy(g_settings.browser_file, value, sizeof(g_settings.browser_file) - 1);
            g_settings.browser_file[sizeof(g_settings.browser_file) - 1] = '\0';
        }
    }

    f_close(&file);
    printf("Settings loaded from %s\n", SETTINGS_PATH);
}

bool settings_save(void) {
    FIL file;
    FRESULT res = f_open(&file, SETTINGS_PATH, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) {
        f_mkdir("/snes");
        res = f_open(&file, SETTINGS_PATH, FA_WRITE | FA_CREATE_ALWAYS);
        if (res != FR_OK) return false;
    }

    f_printf(&file, "; frank-snes settings\n");
    f_printf(&file, "p1_mode=%d\n", g_settings.p1_mode);
    f_printf(&file, "p2_mode=%d\n", g_settings.p2_mode);
    f_printf(&file, "volume=%d\n", g_settings.volume);
    f_printf(&file, "crt_effect=%d\n", g_settings.crt_effect ? 1 : 0);
    f_printf(&file, "greyscale=%d\n", g_settings.greyscale ? 1 : 0);
    f_printf(&file, "frameskip=%d\n", g_settings.frameskip);
    f_printf(&file, "bg_enabled=%d\n", g_settings.bg_enabled);
    f_printf(&file, "sprites=%d\n", g_settings.sprites_enabled ? 1 : 0);
    f_printf(&file, "transparency=%d\n", g_settings.transparency_enabled ? 1 : 0);
    f_printf(&file, "hdma=%d\n", g_settings.hdma_enabled ? 1 : 0);
    f_printf(&file, "crt_overscan=%d\n", g_settings.crt_overscan ? 1 : 0);
    f_printf(&file, "echo=%d\n", g_settings.echo_enabled ? 1 : 0);
    f_printf(&file, "interpolation=%d\n", g_settings.interpolation ? 1 : 0);
    f_printf(&file, "btnmap_kbd=%d,%d,%d,%d,%d,%d,%d,%d\n",
             g_settings.btnmap_kbd.map[0], g_settings.btnmap_kbd.map[1],
             g_settings.btnmap_kbd.map[2], g_settings.btnmap_kbd.map[3],
             g_settings.btnmap_kbd.map[4], g_settings.btnmap_kbd.map[5],
             g_settings.btnmap_kbd.map[6], g_settings.btnmap_kbd.map[7]);
    f_printf(&file, "btnmap_nes=%d,%d,%d,%d,%d,%d,%d,%d\n",
             g_settings.btnmap_nes.map[0], g_settings.btnmap_nes.map[1],
             g_settings.btnmap_nes.map[2], g_settings.btnmap_nes.map[3],
             g_settings.btnmap_nes.map[4], g_settings.btnmap_nes.map[5],
             g_settings.btnmap_nes.map[6], g_settings.btnmap_nes.map[7]);
    f_printf(&file, "btnmap_usb=%d,%d,%d,%d,%d,%d,%d,%d\n",
             g_settings.btnmap_usb.map[0], g_settings.btnmap_usb.map[1],
             g_settings.btnmap_usb.map[2], g_settings.btnmap_usb.map[3],
             g_settings.btnmap_usb.map[4], g_settings.btnmap_usb.map[5],
             g_settings.btnmap_usb.map[6], g_settings.btnmap_usb.map[7]);
    f_printf(&file, "selector_mode=%s\n",
             g_settings.selector_mode == SELECTOR_MODE_BROWSER ? "browser" : "carousel");
    f_printf(&file, "browser_path=%s\n", g_settings.browser_path);
    f_printf(&file, "browser_file=%s\n", g_settings.browser_file);

    f_close(&file);
    printf("Settings saved to %s\n", SETTINGS_PATH);
    return true;
}

/* ─── Runtime application ─────────────────────────────────────────── */

/* Defined in main.c */
extern void set_frameskip_level(uint8_t level);

/* snes9x globals for audio settings */
#include "snes9x/snes9x.h"

void settings_apply_runtime(void) {
    set_frameskip_level(g_settings.frameskip);

    /* Audio settings */
    Settings.DisableSoundEcho = !g_settings.echo_enabled;
    Settings.InterpolatedSound = g_settings.interpolation;
    Settings.Mute = (g_settings.volume == 0);

    /* CRT effect */
    graphics_set_crt_active(g_settings.crt_effect);

    /* Black & white mode */
    graphics_set_greyscale(g_settings.greyscale);
}

/* ─── Hotkey detection ────────────────────────────────────────────── */

bool settings_check_hotkey(void) {
    /* NES/SNES gamepad: Start + Select */
    bool triggered = (nespad_state & DPAD_SELECT) && (nespad_state & DPAD_START);

    /* PS/2 / USB keyboard: F12 */
    uint16_t kbd = ps2kbd_get_state();
#ifdef USB_HID_ENABLED
    kbd |= usbhid_get_kbd_state();
#endif
    if (kbd & KBD_STATE_F12) triggered = true;

#ifdef USB_HID_ENABLED
    /* USB gamepad: Start + Select */
    if (usbhid_gamepad_connected()) {
        usbhid_gamepad_state_t gp;
        usbhid_get_gamepad_state(&gp);
        if ((gp.buttons & 0x0040) && (gp.buttons & 0x0080)) triggered = true;
    }
#endif

    return triggered;
}

/* ─── Menu main loop ──────────────────────────────────────────────── */

/* Double-buffered screen arrays from main.c */
extern uint8_t SCREEN[2][256 * 224];
extern volatile uint32_t current_buffer;

settings_result_t settings_menu_show(uint8_t *screen_buffer, bool in_game) {
    (void)screen_buffer;  /* We use SCREEN[0]/SCREEN[1] directly for double-buffering */

    menu_in_game = in_game;

    /* Copy settings for editing */
    edit = g_settings;

    /* Check if a save file exists for this ROM */
    save_exists = in_game ? check_save_exists() : false;
    status_frames = 0;

    /* Set up palette */
    setup_menu_palette();

    current_page = PAGE_MAIN;
    selected = MAIN_VOLUME;

    /* Auto-repeat state */
    uint32_t hold_counter = 0;
    const uint32_t REPEAT_DELAY = 10;
    const uint32_t REPEAT_RATE = 3;

    /* Double-buffered menu: draw into back buffer, then swap to front.
     * This prevents HDMI from reading a half-drawn (memset'd) frame. */
    int draw_buf = 0;  /* index of buffer we draw into */

    /* Draw initial frame into buf 0, display it.
     * DMA displays SCREEN[!current_buffer], so set current_buffer=1 to show SCREEN[0]. */
    draw_menu(SCREEN[0], "SETTINGS", MAIN_ITEM_COUNT,
              main_label, main_value, is_separator_main, is_hidden_main,
              is_selectable_main, selected);
    current_buffer = 1;
    draw_buf = 1;  /* next draw goes into buf 1 */

    /* Wait for all buttons to be released */
    for (int i = 0; i < 30; i++) {
        if (read_menu_buttons() == 0) break;
        sleep_ms(16);
    }
    int prev_buttons = read_menu_buttons();

    settings_result_t result = SETTINGS_RESULT_EXIT;

    while (1) {
        int buttons = read_menu_buttons();

        /* Edge detection + repeat */
        int pressed = buttons & ~prev_buttons;
        if (buttons != 0 && buttons == prev_buttons) {
            hold_counter++;
            if (hold_counter > REPEAT_DELAY && (hold_counter % REPEAT_RATE) == 0)
                pressed = buttons;
        } else {
            hold_counter = 0;
        }
        prev_buttons = buttons;

        /* Button scan mode: skip normal input, poll target device */
        if (scan_active >= 0) {
            /* Poll hardware once (shared by cancel check and scan detection) */
            nespad_read();
            ps2kbd_tick();
#ifdef USB_HID_ENABLED
            usbhid_task();
#endif
            /* Cancel: NES Start+Select combo, or ESC/F12 on keyboard */
            bool cancel = false;
            uint32_t pad = nespad_state | nespad_state2;
            if ((pad & DPAD_START) && (pad & DPAD_SELECT)) cancel = true;
            uint16_t kbd = ps2kbd_get_state();
#ifdef USB_HID_ENABLED
            kbd |= usbhid_get_kbd_state();
#endif
            if (kbd & (KBD_STATE_ESC | KBD_STATE_F12)) cancel = true;
#ifdef USB_HID_ENABLED
            if (usbhid_gamepad_connected()) {
                usbhid_gamepad_state_t gp;
                usbhid_get_gamepad_state(&gp);
                if ((gp.buttons & 0x0040) && (gp.buttons & 0x0080)) cancel = true;
            }
#endif
            if (cancel) {
                scan_active = -1;
                for (int i = 0; i < 30; i++) { if (read_menu_buttons() == 0) break; sleep_ms(16); }
                prev_buttons = read_menu_buttons();
                goto draw_frame;
            }
            /* Detect single button press on the target device */
            int detected = scan_detect_button(current_page);
            if (scan_wait_release) {
                if (detected < 0)
                    scan_wait_release = false;
            } else if (detected >= 0 && edit_btnmap) {
                edit_btnmap->map[scan_active] = (uint8_t)detected;
                scan_active = -1;
                for (int i = 0; i < 30; i++) { if (read_menu_buttons() == 0) break; sleep_ms(16); }
                prev_buttons = read_menu_buttons();
            }
            goto draw_frame;
        }

        /* Dispatch based on current page */
        int item_count;
        is_selectable_fn is_sel;

        switch (current_page) {
            case PAGE_VIDEO:
                item_count = VIDEO_ITEM_COUNT;
                is_sel = is_selectable_video;
                break;
            case PAGE_AUDIO:
                item_count = AUDIO_ITEM_COUNT;
                is_sel = is_selectable_audio;
                break;
            case PAGE_BUTTONS:
                item_count = BTN_DEV_ITEM_COUNT;
                is_sel = is_selectable_btndev;
                break;
            case PAGE_BTNMAP_KBD:
            case PAGE_BTNMAP_NES:
            case PAGE_BTNMAP_USB:
                item_count = BMAP_ITEM_COUNT;
                is_sel = is_selectable_bmap;
                break;
            default:
                item_count = MAIN_ITEM_COUNT;
                is_sel = is_selectable_main;
                break;
        }

        /* Navigation */
        if (pressed & BTN_UP)
            selected = next_selectable(selected, -1, item_count, is_sel);
        if (pressed & BTN_DOWN)
            selected = next_selectable(selected, 1, item_count, is_sel);

        /* Value change */
        if (pressed & BTN_LEFT) {
            switch (current_page) {
                case PAGE_MAIN:  main_change_value(selected, -1); break;
                case PAGE_VIDEO: video_change_value(selected, -1); break;
                case PAGE_AUDIO: audio_change_value(selected, -1); break;
                default: break;
            }
        }
        if (pressed & BTN_RIGHT) {
            switch (current_page) {
                case PAGE_MAIN:  main_change_value(selected, 1); break;
                case PAGE_VIDEO: video_change_value(selected, 1); break;
                case PAGE_AUDIO: audio_change_value(selected, 1); break;
                default: break;
            }
        }

        /* Confirm (A / Start) */
        if (pressed & (BTN_A | BTN_START)) {
            if (current_page == PAGE_MAIN) {
                if (selected == MAIN_VIDEO) {
                    current_page = PAGE_VIDEO;
                    selected = VIDEO_BG1;
                } else if (selected == MAIN_AUDIO) {
                    current_page = PAGE_AUDIO;
                    selected = AUDIO_ECHO;
                } else if (selected == MAIN_BUTTONS) {
                    current_page = PAGE_BUTTONS;
                    selected = BTN_DEV_KEYBOARD;
                } else if (selected == MAIN_SAVE_GAME) {
                    bool ok = do_save_game();
                    if (ok) {
                        save_exists = true;
                        status_msg = "SAVED.";
                    } else {
                        status_msg = save_error ? save_error : "SAVE FAILED.";
                    }
                    status_frames = 120;
                } else if (selected == MAIN_LOAD_GAME && save_exists) {
                    do_load_game();
                    g_settings = edit;
                    settings_save();
                    result = SETTINGS_RESULT_EXIT;
                    break;
                } else if (selected == MAIN_BACK_GAME) {
                    g_settings = edit;
                    settings_save();
                    result = SETTINGS_RESULT_EXIT;
                    break;
                } else if (selected == MAIN_RESTART) {
                    g_settings = edit;
                    settings_save();
                    result = SETTINGS_RESULT_RESTART;
                    break;
                } else if (selected == MAIN_BACK_ROM) {
                    g_settings = edit;
                    settings_save();
                    result = SETTINGS_RESULT_ROM_SELECT;
                    break;
                }
            } else if (current_page == PAGE_VIDEO) {
                if (selected == VIDEO_BACK) {
                    current_page = PAGE_MAIN;
                    selected = MAIN_VIDEO;
                }
            } else if (current_page == PAGE_AUDIO) {
                if (selected == AUDIO_BACK) {
                    current_page = PAGE_MAIN;
                    selected = MAIN_AUDIO;
                }
            } else if (current_page == PAGE_BUTTONS) {
                if (selected == BTN_DEV_KEYBOARD) {
                    edit_btnmap = &edit.btnmap_kbd;
                    current_page = PAGE_BTNMAP_KBD;
                    selected = BMAP_A; scan_active = -1;
                } else if (selected == BTN_DEV_NES) {
                    edit_btnmap = &edit.btnmap_nes;
                    current_page = PAGE_BTNMAP_NES;
                    selected = BMAP_A; scan_active = -1;
                } else if (selected == BTN_DEV_USB) {
                    edit_btnmap = &edit.btnmap_usb;
                    current_page = PAGE_BTNMAP_USB;
                    selected = BMAP_A; scan_active = -1;
                } else if (selected == BTN_DEV_BACK) {
                    current_page = PAGE_MAIN;
                    selected = MAIN_BUTTONS;
                }
            } else if (current_page == PAGE_BTNMAP_KBD ||
                       current_page == PAGE_BTNMAP_NES ||
                       current_page == PAGE_BTNMAP_USB) {
                if (selected >= BMAP_A && selected <= BMAP_SELECT) {
                    scan_active = selected;
                    scan_wait_release = true;
                } else if (selected == BMAP_RESET) {
                    for (int i = 0; i < BTNMAP_COUNT; i++)
                        edit_btnmap->map[i] = (uint8_t)i;
                } else if (selected == BMAP_BACK) {
                    current_page = PAGE_BUTTONS;
                    selected = BTN_DEV_KEYBOARD;
                }
            }
        }

        /* Back (B) — not used on bmap pages (B is a scannable button there) */
        if (pressed & BTN_B) {
            if (current_page == PAGE_VIDEO) {
                current_page = PAGE_MAIN;
                selected = MAIN_VIDEO;
            } else if (current_page == PAGE_AUDIO) {
                current_page = PAGE_MAIN;
                selected = MAIN_AUDIO;
            } else if (current_page == PAGE_BUTTONS) {
                current_page = PAGE_MAIN;
                selected = MAIN_BUTTONS;
            } else if (current_page == PAGE_BTNMAP_KBD ||
                       current_page == PAGE_BTNMAP_NES ||
                       current_page == PAGE_BTNMAP_USB) {
                /* ignore B on mapping pages */
            } else {
                /* Main page: save and exit */
                g_settings = edit;
                settings_save();
                result = SETTINGS_RESULT_EXIT;
                break;
            }
        }

        /* Start+Select = back on button mapping pages */
        if ((buttons & BTN_START) && (buttons & BTN_SEL)) {
            if (current_page == PAGE_BTNMAP_KBD ||
                current_page == PAGE_BTNMAP_NES ||
                current_page == PAGE_BTNMAP_USB) {
                scan_active = -1;
                current_page = PAGE_BUTTONS;
                selected = BTN_DEV_KEYBOARD;
                for (int i = 0; i < 30; i++) { if (read_menu_buttons() == 0) break; sleep_ms(16); }
                prev_buttons = read_menu_buttons();
            }
        }

        /* Tick status message countdown */
        if (status_frames > 0) status_frames--;

draw_frame:
        /* Draw into back buffer, then swap to front */
        uint8_t *back = SCREEN[draw_buf];
        switch (current_page) {
            case PAGE_VIDEO:
                draw_menu(back, "VIDEO SETTINGS", VIDEO_ITEM_COUNT,
                          video_label, video_value, is_separator_video, NULL, NULL, selected);
                break;
            case PAGE_AUDIO:
                draw_menu(back, "AUDIO SETTINGS", AUDIO_ITEM_COUNT,
                          audio_label, audio_value, is_separator_audio, NULL, NULL, selected);
                break;
            case PAGE_BUTTONS:
                draw_menu(back, "BUTTON MAPPING", BTN_DEV_ITEM_COUNT,
                          btndev_label, btndev_value, is_separator_btndev, NULL, NULL, selected);
                break;
            case PAGE_BTNMAP_KBD:
                draw_menu(back, "KEYBOARD MAPPING", BMAP_ITEM_COUNT,
                          bmap_label, bmap_value, is_separator_bmap, NULL, NULL, selected);
                goto bmap_help;
            case PAGE_BTNMAP_NES:
                draw_menu(back, "NES/SNES PAD MAPPING", BMAP_ITEM_COUNT,
                          bmap_label, bmap_value, is_separator_bmap, NULL, NULL, selected);
                goto bmap_help;
            case PAGE_BTNMAP_USB:
                draw_menu(back, "USB GAMEPAD MAPPING", BMAP_ITEM_COUNT,
                          bmap_label, bmap_value, is_separator_bmap, NULL, NULL, selected);
            bmap_help: {
                /* Override help text for scan-based mapping */
                int hy = SCREEN_HEIGHT - 14;
                draw_hline(back, 0, hy, SCREEN_WIDTH, PAL_BG);
                draw_hline(back, 0, hy + 1, SCREEN_WIDTH, PAL_BG);
                draw_hline(back, 0, hy + 2, SCREEN_WIDTH, PAL_BG);
                draw_hline(back, 0, hy + 3, SCREEN_WIDTH, PAL_BG);
                draw_hline(back, 0, hy + 4, SCREEN_WIDTH, PAL_BG);
                draw_hline(back, 0, hy + 5, SCREEN_WIDTH, PAL_BG);
                draw_hline(back, 0, hy + 6, SCREEN_WIDTH, PAL_BG);
                const char *help = scan_active >= 0
                    ? "PRESS BUTTON  ST+SEL:CANCEL"
                    : "A:SCAN  ST+SEL:BACK";
                draw_text_centered(back, hy, help, PAL_GRAY);
                break;
            }
            default:
                draw_menu(back, "SETTINGS", MAIN_ITEM_COUNT,
                          main_label, main_value, is_separator_main, is_hidden_main,
                          is_selectable_main, selected);
                break;
        }

        /* Swap: tell HDMI to display the just-drawn buffer.
         * DMA displays SCREEN[!current_buffer], so set current_buffer = !draw_buf. */
        current_buffer = !draw_buf;
        draw_buf ^= 1;  /* next draw goes into the other buffer */
        sleep_ms(33);  /* ~30fps menu refresh */
    }

    /* Wait for buttons to be released */
    for (int i = 0; i < 60; i++) {
        if (read_menu_buttons() == 0) break;
        sleep_ms(16);
    }

    return result;
}
