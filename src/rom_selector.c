/*
 * MurmSNES - ROM Selector with SNES cartridge display and cover art
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "rom_selector.h"
#include "menu_ui.h"
#include "settings.h"
#include "ff.h"
#include "pico/stdlib.h"
#include "HDMI.h"
#include "board_config.h"
#include "psram_allocator.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "nespad/nespad.h"
#include "ps2kbd/ps2kbd_wrapper.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef USB_HID_ENABLED
#include "usbhid/usbhid.h"
#endif

extern volatile uint32_t current_buffer;
extern uint8_t SCREEN[2][256 * 224];

#ifndef FRANK_SNES_VERSION
#define FRANK_SNES_VERSION "?"
#endif

#define SCREEN_W 256
#define SCREEN_H 224

/* ─── Fixed 6x6x6 RGB color cube palette (216 colors) ────────────── */

#define PAL_BLACK      0
#define PAL_CUBE_BASE  1
#define PAL_CART_BODY  217
#define PAL_CART_LIGHT 218
#define PAL_CART_DARK  219
#define PAL_CART_LABEL 220
#define PAL_CART_RIDGE 221
#define PAL_CART_SLOT  222
#define PAL_WHITE      223
#define PAL_GRAY       224
#define PAL_BG         225

static const uint8_t cube_levels[6] = {0, 51, 102, 153, 204, 255};

static uint8_t rgb555_to_pal(uint16_t p) {
    uint8_t r5 = (p >> 10) & 0x1F;
    uint8_t g5 = (p >> 5) & 0x1F;
    uint8_t b5 = p & 0x1F;
    int ri = (r5 * 5 + 15) / 31;
    int gi = (g5 * 5 + 15) / 31;
    int bi = (b5 * 5 + 15) / 31;
    return (uint8_t)(PAL_CUBE_BASE + ri * 36 + gi * 6 + bi);
}

static void setup_selector_palette(void) {
    graphics_set_palette(PAL_BLACK, 0x000000);

    for (int r = 0; r < 6; r++)
        for (int g = 0; g < 6; g++)
            for (int b = 0; b < 6; b++) {
                int idx = PAL_CUBE_BASE + r * 36 + g * 6 + b;
                uint32_t rgb = ((uint32_t)cube_levels[r] << 16) |
                               ((uint32_t)cube_levels[g] << 8) |
                               (uint32_t)cube_levels[b];
                graphics_set_palette(idx, rgb);
            }

    graphics_set_palette(PAL_CART_BODY,  0xB0B0B8);
    graphics_set_palette(PAL_CART_LIGHT, 0xC8C8D0);
    graphics_set_palette(PAL_CART_DARK,  0x808088);
    graphics_set_palette(PAL_CART_LABEL, 0x202028);
    graphics_set_palette(PAL_CART_RIDGE, 0x989898);
    graphics_set_palette(PAL_CART_SLOT,  0x505058);
    graphics_set_palette(PAL_WHITE,      0xFFFFFF);
    graphics_set_palette(PAL_GRAY,       0x808080);
    graphics_set_palette(PAL_BG,         0x1A1A22);

    graphics_restore_sync_colors();
}

/* ─── Framebuffer helpers ─────────────────────────────────────────── */

static uint8_t *fb;
static int draw_buf;

static inline void fb_pixel(int x, int y, uint8_t color) {
    if (x >= 0 && x < SCREEN_W && y >= 0 && y < SCREEN_H)
        fb[y * SCREEN_W + x] = color;
}

static void fb_fill(uint8_t color) {
    memset(fb, color, SCREEN_W * SCREEN_H);
}

static void fb_rect(int x, int y, int w, int h, uint8_t color) {
    for (int yy = y; yy < y + h && yy < SCREEN_H; yy++) {
        if (yy < 0) continue;
        int x0 = x < 0 ? 0 : x;
        int x1 = (x + w) > SCREEN_W ? SCREEN_W : (x + w);
        if (x0 < x1) memset(&fb[yy * SCREEN_W + x0], color, x1 - x0);
    }
}

static void fb_hline(int x, int y, int w, uint8_t color) {
    if (y < 0 || y >= SCREEN_H) return;
    int x0 = x < 0 ? 0 : x;
    int x1 = (x + w) > SCREEN_W ? SCREEN_W : (x + w);
    if (x0 < x1) memset(&fb[y * SCREEN_W + x0], color, x1 - x0);
}

static void fb_vline(int x, int y, int h, uint8_t color) {
    if (x < 0 || x >= SCREEN_W) return;
    for (int yy = y; yy < y + h && yy < SCREEN_H; yy++)
        if (yy >= 0) fb[yy * SCREEN_W + x] = color;
}

static void present(void) {
    current_buffer = !draw_buf;
    draw_buf ^= 1;
    fb = SCREEN[draw_buf];
}

/* ─── 5x7 font ───────────────────────────────────────────────────── */

static const uint8_t glyphs[][7] = {
    [' '-' '] = {0,0,0,0,0,0,0},
    ['!'-' '] = {0x04,0x04,0x04,0x04,0x00,0x04,0x00},
    ['"'-' '] = {0x0A,0x0A,0x0A,0x00,0x00,0x00,0x00},
    ['#'-' '] = {0x0A,0x0A,0x1F,0x0A,0x1F,0x0A,0x0A},
    ['$'-' '] = {0x04,0x0F,0x14,0x0E,0x05,0x1E,0x04},
    ['%'-' '] = {0x19,0x1A,0x04,0x08,0x0B,0x13,0x00},
    ['&'-' '] = {0x0C,0x12,0x14,0x08,0x15,0x12,0x0D},
    ['\''-' '] = {0x04,0x04,0x00,0x00,0x00,0x00,0x00},
    ['('-' '] = {0x02,0x04,0x08,0x08,0x08,0x04,0x02},
    [')'-' '] = {0x08,0x04,0x02,0x02,0x02,0x04,0x08},
    ['*'-' '] = {0x00,0x04,0x15,0x0E,0x15,0x04,0x00},
    ['+'-' '] = {0x00,0x04,0x04,0x1F,0x04,0x04,0x00},
    [','-' '] = {0x00,0x00,0x00,0x00,0x0C,0x04,0x08},
    ['-'-' '] = {0x00,0x00,0x00,0x1F,0x00,0x00,0x00},
    ['.'-' '] = {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C},
    ['/'-' '] = {0x01,0x02,0x04,0x08,0x10,0x00,0x00},
    ['0'-' '] = {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},
    ['1'-' '] = {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
    ['2'-' '] = {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F},
    ['3'-' '] = {0x1E,0x01,0x01,0x0E,0x01,0x01,0x1E},
    ['4'-' '] = {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},
    ['5'-' '] = {0x1F,0x10,0x10,0x1E,0x01,0x01,0x1E},
    ['6'-' '] = {0x0E,0x10,0x10,0x1E,0x11,0x11,0x0E},
    ['7'-' '] = {0x1F,0x01,0x02,0x04,0x08,0x08,0x08},
    ['8'-' '] = {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},
    ['9'-' '] = {0x0E,0x11,0x11,0x0F,0x01,0x01,0x0E},
    [':'-' '] = {0x00,0x0C,0x0C,0x00,0x0C,0x0C,0x00},
    [';'-' '] = {0x00,0x0C,0x0C,0x00,0x0C,0x04,0x08},
    ['<'-' '] = {0x02,0x04,0x08,0x10,0x08,0x04,0x02},
    ['='-' '] = {0x00,0x00,0x1F,0x00,0x1F,0x00,0x00},
    ['>'-' '] = {0x08,0x04,0x02,0x01,0x02,0x04,0x08},
    ['?'-' '] = {0x0E,0x11,0x01,0x02,0x04,0x00,0x04},
    ['@'-' '] = {0x0E,0x11,0x17,0x15,0x17,0x10,0x0E},
    ['A'-' '] = {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11},
    ['B'-' '] = {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E},
    ['C'-' '] = {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E},
    ['D'-' '] = {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E},
    ['E'-' '] = {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F},
    ['F'-' '] = {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10},
    ['G'-' '] = {0x0E,0x11,0x10,0x17,0x11,0x11,0x0E},
    ['H'-' '] = {0x11,0x11,0x11,0x1F,0x11,0x11,0x11},
    ['I'-' '] = {0x1F,0x04,0x04,0x04,0x04,0x04,0x1F},
    ['J'-' '] = {0x07,0x02,0x02,0x02,0x12,0x12,0x0C},
    ['K'-' '] = {0x11,0x12,0x14,0x18,0x14,0x12,0x11},
    ['L'-' '] = {0x10,0x10,0x10,0x10,0x10,0x10,0x1F},
    ['M'-' '] = {0x11,0x1B,0x15,0x15,0x11,0x11,0x11},
    ['N'-' '] = {0x11,0x19,0x15,0x13,0x11,0x11,0x11},
    ['O'-' '] = {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},
    ['P'-' '] = {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10},
    ['Q'-' '] = {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D},
    ['R'-' '] = {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11},
    ['S'-' '] = {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E},
    ['T'-' '] = {0x1F,0x04,0x04,0x04,0x04,0x04,0x04},
    ['U'-' '] = {0x11,0x11,0x11,0x11,0x11,0x11,0x0E},
    ['V'-' '] = {0x11,0x11,0x11,0x11,0x0A,0x0A,0x04},
    ['W'-' '] = {0x11,0x11,0x11,0x15,0x15,0x15,0x0A},
    ['X'-' '] = {0x11,0x0A,0x04,0x04,0x04,0x0A,0x11},
    ['Y'-' '] = {0x11,0x0A,0x04,0x04,0x04,0x04,0x04},
    ['Z'-' '] = {0x1F,0x02,0x04,0x08,0x10,0x10,0x1F},
    ['['-' '] = {0x0E,0x08,0x08,0x08,0x08,0x08,0x0E},
    ['\\'-' '] = {0x10,0x08,0x04,0x02,0x01,0x00,0x00},
    [']'-' '] = {0x0E,0x02,0x02,0x02,0x02,0x02,0x0E},
    ['^'-' '] = {0x04,0x0A,0x11,0x00,0x00,0x00,0x00},
    ['_'-' '] = {0x00,0x00,0x00,0x00,0x00,0x00,0x1F},
    ['{'-' '] = {0x02,0x04,0x04,0x08,0x04,0x04,0x02},
    ['|'-' '] = {0x04,0x04,0x04,0x04,0x04,0x04,0x04},
    ['}'-' '] = {0x08,0x04,0x04,0x02,0x04,0x04,0x08},
    ['~'-' '] = {0x00,0x00,0x08,0x15,0x02,0x00,0x00},
};

static void fb_char(int x, int y, char ch, uint8_t color) {
    int c = (unsigned char)ch;
    if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';
    int idx = c - ' ';
    if (idx < 0 || idx >= (int)(sizeof(glyphs)/sizeof(glyphs[0]))) return;
    const uint8_t *g = glyphs[idx];
    for (int row = 0; row < 7; row++) {
        uint8_t bits = g[row];
        for (int col = 0; col < 5; col++) {
            if (bits & (1u << (4 - col)))
                fb_pixel(x + col, y + row, color);
        }
    }
}

static void fb_text(int x, int y, const char *s, uint8_t color) {
    for (; *s; s++) { fb_char(x, y, *s, color); x += 6; }
}

static void fb_text_center(int y, const char *s, uint8_t color) {
    int x = (SCREEN_W - (int)strlen(s) * 6) / 2;
    fb_text(x, y, s, color);
}

/* Render text with a 1-pixel offset shadow so it stays legible over
 * animated backgrounds without darkening the whole region. */
static void fb_text_center_shadow(int y, const char *s, uint8_t fg_color, uint8_t shadow_color) {
    int x = (SCREEN_W - (int)strlen(s) * 6) / 2;
    fb_text(x + 1, y + 1, s, shadow_color);
    fb_text(x,     y,     s, fg_color);
}

/* Map 3-bit r/g/b (0..7) into the 6x6x6 cube palette (0..5 each channel). */
static inline uint8_t cube_rgb(int r, int g, int b) {
    r = (r * 5) / 7;
    g = (g * 5) / 7;
    b = (b * 5) / 7;
    return (uint8_t)(PAL_CUBE_BASE + r * 36 + g * 6 + b);
}

/* ─── CRC32 ───────────────────────────────────────────────────────── */

static uint32_t crc32_table[256];
static bool crc32_ready = false;

static void crc32_init(void) {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int j = 0; j < 8; j++)
            c = (c >> 1) ^ (0xEDB88320 & (-(c & 1)));
        crc32_table[i] = c;
    }
    crc32_ready = true;
}

static uint32_t crc32_file(FIL *fil, int skip) {
    if (!crc32_ready) crc32_init();
    uint32_t crc = 0xFFFFFFFF;
    uint8_t buf[512];
    f_lseek(fil, skip);
    while (1) {
        UINT br;
        if (f_read(fil, buf, sizeof(buf), &br) != FR_OK || br == 0) break;
        for (UINT i = 0; i < br; i++)
            crc = crc32_table[(crc ^ buf[i]) & 0xFF] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFF;
}

/* ─── ROM list ────────────────────────────────────────────────────── */

#define MAX_ROMS 48

typedef struct {
    char filename[48];
    uint32_t crc;
    bool crc_valid;
} rom_entry_t;

static rom_entry_t *rom_list;  /* allocated in PSRAM */
#define IMG_BUF_BYTES (40 * 1024)
static uint8_t *img_buf;      /* allocated in PSRAM */
static int rom_count = 0;

static bool is_snes_ext(const char *fname) {
    const char *ext = strrchr(fname, '.');
    if (!ext) return false;
    return (strcasecmp(ext, ".smc") == 0 ||
            strcasecmp(ext, ".sfc") == 0 ||
            strcasecmp(ext, ".fig") == 0);
}

static int strcasecmp_rom(const void *a, const void *b) {
    const rom_entry_t *ra = (const rom_entry_t *)a;
    const rom_entry_t *rb = (const rom_entry_t *)b;
    const char *sa = ra->filename, *sb = rb->filename;
    for (;; sa++, sb++) {
        int ca = (*sa >= 'a' && *sa <= 'z') ? *sa - 32 : *sa;
        int cb = (*sb >= 'a' && *sb <= 'z') ? *sb - 32 : *sb;
        if (ca != cb) return ca - cb;
        if (ca == 0) return 0;
    }
}

/* Scan result — distinguishes "SD not mounted" / "no /snes" / "no roms" */
typedef enum {
    ROM_SCAN_OK,
    ROM_SCAN_NO_SD,
    ROM_SCAN_NO_SNES_DIR,
    ROM_SCAN_NO_ROMS,
} rom_scan_result_t;

static rom_scan_result_t scan_result = ROM_SCAN_NO_SD;

static int scan_roms(bool *out_dir_ok) {
    rom_count = 0;
    if (out_dir_ok) *out_dir_ok = false;
    static DIR dir;
    if (f_opendir(&dir, "/snes") != FR_OK) {
        if (f_opendir(&dir, "/SNES") != FR_OK) return 0;
    }
    if (out_dir_ok) *out_dir_ok = true;
    static FILINFO fno;
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0' && rom_count < MAX_ROMS) {
        if (fno.fattrib & AM_DIR) continue;
        if (!is_snes_ext(fno.fname)) continue;
        strncpy(rom_list[rom_count].filename, fno.fname, sizeof(rom_list[0].filename) - 1);
        rom_list[rom_count].filename[sizeof(rom_list[0].filename) - 1] = '\0';
        rom_list[rom_count].crc_valid = false;
        rom_count++;
    }
    f_closedir(&dir);
    if (rom_count > 1)
        qsort(rom_list, rom_count, sizeof(rom_entry_t), strcasecmp_rom);
    return rom_count;
}

static void ensure_crc(int idx) {
    if (rom_list[idx].crc_valid) return;
    char path[MAX_ROM_PATH];
    snprintf(path, sizeof(path), "/snes/%s", rom_list[idx].filename);
    static FIL fil;
    if (f_open(&fil, path, FA_READ) == FR_OK) {
        /* SNES ROMs may have a 512-byte copier header */
        FSIZE_t sz = f_size(&fil);
        int skip = (sz % 1024 == 512) ? 512 : 0;
        rom_list[idx].crc = crc32_file(&fil, skip);
        rom_list[idx].crc_valid = true;
        f_close(&fil);
        printf("CRC32(%s) = %08lX\n", rom_list[idx].filename, (unsigned long)rom_list[idx].crc);
    }
}

/* ─── CRC cache ───────────────────────────────────────────────────── */

#define CRC_CACHE_PATH "/snes/.crc_cache"
#define LAST_ROM_PATH  "/snes/.last_rom"

static void load_crc_cache(void) {
    static FIL fil;
    if (f_open(&fil, CRC_CACHE_PATH, FA_READ) != FR_OK) return;
    char line[128];
    while (f_gets(line, sizeof(line), &fil)) {
        char *eq = strchr(line, '=');
        if (!eq) continue;
        *eq = '\0';
        uint32_t crc = 0;
        for (const char *p = eq + 1; *p && *p != '\n' && *p != '\r'; p++) {
            crc <<= 4;
            if (*p >= '0' && *p <= '9') crc |= (*p - '0');
            else if (*p >= 'A' && *p <= 'F') crc |= (*p - 'A' + 10);
            else if (*p >= 'a' && *p <= 'f') crc |= (*p - 'a' + 10);
        }
        for (int i = 0; i < rom_count; i++) {
            if (strcmp(rom_list[i].filename, line) == 0) {
                rom_list[i].crc = crc;
                rom_list[i].crc_valid = true;
                break;
            }
        }
    }
    f_close(&fil);
}

static void save_crc_cache(void) {
    static FIL fil;
    if (f_open(&fil, CRC_CACHE_PATH, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return;
    for (int i = 0; i < rom_count; i++) {
        if (!rom_list[i].crc_valid) continue;
        char line[128];
        snprintf(line, sizeof(line), "%s=%08lX\n",
                 rom_list[i].filename, (unsigned long)rom_list[i].crc);
        UINT bw;
        f_write(&fil, line, strlen(line), &bw);
    }
    f_close(&fil);
}

/* ─── Last selected ROM ───────────────────────────────────────────── */

static int last_selected_rom = 0;

static void load_last_rom(void) {
    static FIL fil;
    if (f_open(&fil, LAST_ROM_PATH, FA_READ) != FR_OK) return;
    char name[64];
    if (f_gets(name, sizeof(name), &fil)) {
        size_t len = strlen(name);
        while (len > 0 && (name[len-1] == '\n' || name[len-1] == '\r'))
            name[--len] = '\0';
        for (int i = 0; i < rom_count; i++) {
            if (strcmp(rom_list[i].filename, name) == 0) {
                last_selected_rom = i;
                break;
            }
        }
    }
    f_close(&fil);
}

static void save_last_rom(int selected) {
    static FIL fil;
    if (f_open(&fil, LAST_ROM_PATH, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return;
    f_puts(rom_list[selected].filename, &fil);
    f_puts("\n", &fil);
    f_close(&fil);
}

/* ─── Metadata ────────────────────────────────────────────────────── */

typedef struct {
    char title[64];
    char desc[256];
    char year[8];
    char genre[48];
    char players[8];
} rom_meta_t;

static rom_meta_t *rom_meta;  /* allocated in PSRAM */

static void extract_xml_tag(const char *buf, const char *tag, char *dst, int dst_size) {
    dst[0] = '\0';
    char open[32], close[32];
    snprintf(open, sizeof(open), "<%s>", tag);
    snprintf(close, sizeof(close), "</%s>", tag);
    const char *start = strstr(buf, open);
    if (!start) return;
    start += strlen(open);
    const char *end = strstr(start, close);
    if (!end) return;
    int src_len = end - start;
    int di = 0;
    bool prev_space = false;
    int max_out = dst_size - 4;
    int si;
    for (si = 0; si < src_len && di < max_out; si++) {
        char c = start[si];
        if (c == '\n' || c == '\r' || c == '\t') c = ' ';
        if (c == ' ') {
            if (!prev_space && di > 0) { dst[di++] = ' '; prev_space = true; }
        } else {
            dst[di++] = c;
            prev_space = false;
        }
    }
    if (si < src_len) {
        while (di > 0 && dst[di - 1] == ' ') di--;
        dst[di++] = '.'; dst[di++] = '.'; dst[di++] = '.';
    }
    dst[di] = '\0';
}

static void load_rom_title(int idx) {
    memset(&rom_meta[idx], 0, sizeof(rom_meta[idx]));
    if (!rom_list[idx].crc_valid) return;
    uint32_t crc = rom_list[idx].crc;
    char hex_char = "0123456789ABCDEF"[(crc >> 28) & 0xF];

    char path[128];
    snprintf(path, sizeof(path), "/snes/metadata/descr/%c/%08lX.txt", hex_char, (unsigned long)crc);
    static FIL fil;
    if (f_open(&fil, path, FA_READ) == FR_OK) {
        /* Reuse img_buf (not in use during metadata loading) */
        char *buf = (char *)img_buf;
        int buf_size = 1536;
        UINT br;
        if (f_read(&fil, buf, buf_size - 1, &br) == FR_OK) {
            buf[br] = '\0';
            extract_xml_tag(buf, "name", rom_meta[idx].title, sizeof(rom_meta[idx].title));
            extract_xml_tag(buf, "desc", rom_meta[idx].desc, sizeof(rom_meta[idx].desc));
            extract_xml_tag(buf, "genre", rom_meta[idx].genre, sizeof(rom_meta[idx].genre));
            extract_xml_tag(buf, "players", rom_meta[idx].players, sizeof(rom_meta[idx].players));
            char datestr[32];
            extract_xml_tag(buf, "releasedate", datestr, sizeof(datestr));
            if (datestr[0] && strlen(datestr) >= 4) {
                memcpy(rom_meta[idx].year, datestr, 4);
                rom_meta[idx].year[4] = '\0';
            }
        }
        f_close(&fil);
    }
}

/* ─── Cover art image ─────────────────────────────────────────────── */

static uint16_t cur_img_w, cur_img_h;
static uint16_t *cur_img_pixels;
static int cur_img_idx = -1;

static void load_rom_image(int idx) {
    cur_img_pixels = NULL;
    cur_img_w = 0;
    cur_img_h = 0;
    cur_img_idx = idx;

    if (!rom_list[idx].crc_valid) return;
    uint32_t crc = rom_list[idx].crc;
    char hex_char = "0123456789ABCDEF"[(crc >> 28) & 0xF];

    char path[128];
    snprintf(path, sizeof(path), "/snes/metadata/images/%c/%08lX.555", hex_char, (unsigned long)crc);

    static FIL fil;
    if (f_open(&fil, path, FA_READ) != FR_OK) {
        return;
    }

    uint8_t hdr[4];
    UINT br;
    if (f_read(&fil, hdr, 4, &br) != FR_OK || br != 4) { f_close(&fil); return; }

    uint16_t w = hdr[0] | (hdr[1] << 8);
    uint16_t h = hdr[2] | (hdr[3] << 8);
    if (w == 0 || w > 320 || h == 0 || h > 240) { f_close(&fil); return; }

    uint32_t data_size = (uint32_t)w * h * 2;
    if (data_size > IMG_BUF_BYTES) { f_close(&fil); return; }

    if (f_read(&fil, img_buf, data_size, &br) == FR_OK && br == data_size) {
        cur_img_pixels = (uint16_t *)img_buf;
        cur_img_w = w;
        cur_img_h = h;
    }
    f_close(&fil);
}

/* ─── SNES cartridge rendering ────────────────────────────────────── */
/*
 * SNES cart (landscape, wider than tall):
 *   ┌──────────────────────────────┐  ← rounded top corners
 *   │  ┌────────────────────────┐  │
 *   │  │                        │  │  ← label / cover art
 *   │  │       LABEL AREA       │  │
 *   │  │                        │  │
 *   │  └────────────────────────┘  │
 *   │                              │
 *   │      ┌────────────────┐      │  ← connector grooves
 *   │      │ ══════════════ │      │
 *   │      └────────────────┘      │
 *   └──────────────────────────────┘
 */

#define CART_W    170
#define CART_H    130
#define CART_X    ((SCREEN_W - CART_W) / 2)
#define CART_Y    8

/* Label area */
#define LABEL_MARGIN_X 12
#define LABEL_MARGIN_Y 7
#define LABEL_H   78

/* Connector groove (bottom center) */
#define GROOVE_W  100
#define GROOVE_H  16

static void draw_cart_at(int cx, int cy, int rom_idx) {
    int label_x = cx + LABEL_MARGIN_X;
    int label_y = cy + LABEL_MARGIN_Y;
    int label_w = CART_W - LABEL_MARGIN_X * 2;
    int label_h = LABEL_H;

    int groove_x = cx + (CART_W - GROOVE_W) / 2;
    int groove_y = cy + CART_H - GROOVE_H - 6;

    /* Main body */
    fb_rect(cx, cy, CART_W, CART_H, PAL_CART_BODY);

    /* Rounded top corners */
    fb_rect(cx, cy, 2, 1, PAL_BG);
    fb_rect(cx, cy, 1, 2, PAL_BG);
    fb_rect(cx + CART_W - 2, cy, 2, 1, PAL_BG);
    fb_rect(cx + CART_W - 1, cy, 1, 2, PAL_BG);

    /* 3D edges — light top/left, dark bottom/right */
    fb_hline(cx + 2, cy, CART_W - 4, PAL_CART_LIGHT);
    fb_vline(cx, cy + 2, CART_H - 2, PAL_CART_LIGHT);
    fb_hline(cx + 1, cy + CART_H - 1, CART_W - 2, PAL_CART_DARK);
    fb_vline(cx + CART_W - 1, cy + 2, CART_H - 2, PAL_CART_DARK);

    /* Label border (inset) */
    fb_rect(label_x - 1, label_y - 1, label_w + 2, label_h + 2, PAL_CART_DARK);

    /* Label fill */
    bool has_img = (cur_img_pixels && cur_img_idx == rom_idx);
    fb_rect(label_x, label_y, label_w, label_h, has_img ? PAL_CART_LABEL : PAL_CART_SLOT);

    /* Cover art */
    if (has_img) {
        int iw = cur_img_w;
        int ih = cur_img_h;
        /* Scale to fit label, preserving aspect ratio */
        if (iw * label_h > ih * label_w) {
            ih = ih * label_w / iw;
            iw = label_w;
        } else {
            iw = iw * label_h / ih;
            ih = label_h;
        }
        int ix = label_x + (label_w - iw) / 2;
        int iy = label_y + (label_h - ih) / 2;
        for (int y = 0; y < ih; y++) {
            int sy = y * cur_img_h / ih;
            for (int x = 0; x < iw; x++) {
                int sx = x * cur_img_w / iw;
                uint16_t px = cur_img_pixels[sy * cur_img_w + sx];
                fb_pixel(ix + x, iy + y, rgb555_to_pal(px));
            }
        }
    } else {
        /* No image: subtle inset border */
        int m = 6;
        fb_hline(label_x + m, label_y + m, label_w - m * 2, PAL_CART_DARK);
        fb_hline(label_x + m, label_y + label_h - m - 1, label_w - m * 2, PAL_CART_DARK);
        fb_vline(label_x + m, label_y + m, label_h - m * 2, PAL_CART_DARK);
        fb_vline(label_x + label_w - m - 1, label_y + m, label_h - m * 2, PAL_CART_DARK);
    }

    /* Top notches (2 small lines on top edge) */
    int notch1_x = cx + CART_W / 2 - 16;
    int notch2_x = cx + CART_W / 2 + 14;
    fb_vline(notch1_x, cy, 3, PAL_CART_DARK);
    fb_vline(notch2_x, cy, 3, PAL_CART_DARK);

    /* Corner ridges — 5 small horizontal lines on top-left and top-right */
    for (int i = 0; i < 5; i++) {
        int ry = cy + 3 + i * 3;
        fb_hline(cx + 2, ry, 5, PAL_CART_DARK);
        fb_hline(cx + CART_W - 7, ry, 5, PAL_CART_DARK);
    }

    /* Screws on bottom-left and bottom-right */
    int screw_y = cy + CART_H - 10;
    int screw_lx = cx + 10;
    int screw_rx = cx + CART_W - 13;
    /* Left screw */
    fb_rect(screw_lx, screw_y, 3, 3, PAL_CART_DARK);
    fb_pixel(screw_lx + 1, screw_y + 1, PAL_CART_LIGHT);
    /* Right screw */
    fb_rect(screw_rx, screw_y, 3, 3, PAL_CART_DARK);
    fb_pixel(screw_rx + 1, screw_y + 1, PAL_CART_LIGHT);

    /* Connector groove area */
    fb_rect(groove_x, groove_y, GROOVE_W, GROOVE_H, PAL_CART_RIDGE);
    /* Groove border */
    fb_hline(groove_x, groove_y, GROOVE_W, PAL_CART_DARK);
    fb_hline(groove_x, groove_y + GROOVE_H - 1, GROOVE_W, PAL_CART_LIGHT);
    fb_vline(groove_x, groove_y, GROOVE_H, PAL_CART_DARK);
    fb_vline(groove_x + GROOVE_W - 1, groove_y, GROOVE_H, PAL_CART_LIGHT);
    /* Horizontal ridges inside groove */
    for (int i = 0; i < 3; i++) {
        int ry = groove_y + 3 + i * 4;
        fb_hline(groove_x + 4, ry,     GROOVE_W - 8, PAL_CART_DARK);
        fb_hline(groove_x + 4, ry + 1, GROOVE_W - 8, PAL_CART_LIGHT);
    }
}

/* ─── Info panel ──────────────────────────────────────────────────── */

typedef enum {
    INFO_HIDDEN, INFO_SLIDING_IN, INFO_SHOWN, INFO_SLIDING_OUT,
} info_state_t;

#define INFO_ANIM_FRAMES 10
static info_state_t info_state = INFO_HIDDEN;
static int info_anim_frame = 0;
#define INFO_CART_X (-(CART_W * 70 / 100))

static int info_ease(int frame) {
    int t = frame * 256 / INFO_ANIM_FRAMES;
    return t * (512 - t) / 256;
}

static int info_cart_x(void) {
    switch (info_state) {
    case INFO_HIDDEN: return CART_X;
    case INFO_SLIDING_IN: return CART_X + (INFO_CART_X - CART_X) * info_ease(info_anim_frame) / 256;
    case INFO_SHOWN: return INFO_CART_X;
    case INFO_SLIDING_OUT: return INFO_CART_X + (CART_X - INFO_CART_X) * info_ease(info_anim_frame) / 256;
    }
    return CART_X;
}

static int fb_text_wrap(int x, int y, int max_w, const char *s, uint8_t color, int max_lines) {
    int max_chars = max_w / 6;
    if (max_chars < 1) max_chars = 1;
    int line = 0;
    const char *p = s;
    while (*p && line < max_lines) {
        int len = (int)strlen(p);
        if (len <= max_chars) { fb_text(x, y, p, color); break; }
        int brk = max_chars;
        for (int i = max_chars; i > 0; i--) {
            if (p[i] == ' ') { brk = i; break; }
        }
        char tmp[64];
        int copy = brk > 63 ? 63 : brk;
        memcpy(tmp, p, copy);
        tmp[copy] = '\0';
        if (line == max_lines - 1 && (int)strlen(p) > brk) {
            if (copy > 3) copy -= 3;
            tmp[copy] = '.'; tmp[copy+1] = '.'; tmp[copy+2] = '.'; tmp[copy+3] = '\0';
        }
        fb_text(x, y, tmp, color);
        y += 9;
        line++;
        p += brk;
        while (*p == ' ') p++;
    }
    return y;
}

static void draw_info_panel(int selected) {
    int text_x = INFO_CART_X + CART_W + 8;
    int text_w = SCREEN_W - text_x - 6;
    int ty = CART_Y + 4;

    const char *title = rom_meta[selected].title[0] ? rom_meta[selected].title : rom_list[selected].filename;
    char dt[40];
    int max_c = text_w / 6;
    if (max_c > 39) max_c = 39;
    int tlen = (int)strlen(title);
    if (tlen > max_c) {
        int cut = max_c > 3 ? max_c - 3 : 0;
        memcpy(dt, title, cut);
        dt[cut] = '.'; dt[cut+1] = '.'; dt[cut+2] = '.'; dt[cut+3] = '\0';
    } else {
        memcpy(dt, title, tlen); dt[tlen] = '\0';
    }
    fb_text(text_x, ty, dt, PAL_WHITE);
    ty += 14;

    fb_hline(text_x, ty, text_w, PAL_CART_RIDGE);
    ty += 6;

    if (rom_meta[selected].year[0]) {
        char line[48];
        snprintf(line, sizeof(line), "YEAR: %s", rom_meta[selected].year);
        fb_text(text_x, ty, line, PAL_GRAY);
        ty += 12;
    }
    if (rom_meta[selected].players[0]) {
        char line[48];
        snprintf(line, sizeof(line), "PLAYERS: %s", rom_meta[selected].players);
        fb_text(text_x, ty, line, PAL_GRAY);
        ty += 12;
    }
    if (rom_meta[selected].genre[0]) {
        char gline[48];
        int glen = (int)strlen(rom_meta[selected].genre);
        int gmax = text_w / 6;
        if (gmax > 47) gmax = 47;
        if (glen > gmax) {
            memcpy(gline, rom_meta[selected].genre, gmax - 3);
            gline[gmax-3] = '.'; gline[gmax-2] = '.'; gline[gmax-1] = '.';
            gline[gmax] = '\0';
        } else {
            strncpy(gline, rom_meta[selected].genre, 47);
            gline[47] = '\0';
        }
        fb_text(text_x, ty, gline, PAL_GRAY);
        ty += 12;
    }
    if (rom_meta[selected].desc[0]) {
        ty += 4;
        int max_desc_lines = (CART_Y + CART_H + 10 - ty) / 9;
        if (max_desc_lines > 12) max_desc_lines = 12;
        if (max_desc_lines > 0)
            fb_text_wrap(text_x, ty, text_w, rom_meta[selected].desc, PAL_GRAY, max_desc_lines);
    }
}

/* ─── Animation ───────────────────────────────────────────────────── */

static int bounce_offset(uint32_t frame) {
    int t = (int)(frame % 36);
    if (t < 6) return 0;
    if (t < 9) return 1;
    if (t < 15) return 2;
    if (t < 18) return 1;
    if (t < 24) return 0;
    if (t < 27) return -1;
    if (t < 33) return -2;
    return -1;
}

#define SCROLL_FRAMES 8
static int scroll_dir = 0;
static int scroll_frame = 0;
static int scroll_from = 0;

static int ease_out(int frame) {
    int t = frame * 256 / SCROLL_FRAMES;
    return t * (512 - t) / 256;
}

static void draw_selector_text(int selected) {
    bool info_visible = (info_state != INFO_HIDDEN);

    if (!info_visible) {
        const char *title = rom_meta[selected].title[0] ? rom_meta[selected].title : rom_list[selected].filename;
        char dt[40];
        int max_c = (SCREEN_W - 20) / 6;
        if (max_c > 39) max_c = 39;
        int tlen = (int)strlen(title);
        if (tlen > max_c) {
            int cut = max_c > 3 ? max_c - 3 : 0;
            memcpy(dt, title, cut);
            dt[cut] = '.'; dt[cut+1] = '.'; dt[cut+2] = '.'; dt[cut+3] = '\0';
        } else {
            memcpy(dt, title, tlen); dt[tlen] = '\0';
        }
        fb_text_center(CART_Y + CART_H + 10, dt, PAL_WHITE);

        char counter[16];
        snprintf(counter, sizeof(counter), "%d / %d", selected + 1, rom_count);
        fb_text_center(CART_Y + CART_H + 24, counter, PAL_GRAY);
    }

    if (info_state == INFO_SHOWN)
        fb_text_center(SCREEN_H - 14, "DOWN   A:START", PAL_GRAY);
    else if (info_state == INFO_HIDDEN)
        fb_text_center(SCREEN_H - 14, "A:START SEL+A:FIND SEL+START:BROWSE", PAL_GRAY);
}

static void draw_scene(int selected, uint32_t frame_count) {
    fb_fill(PAL_BG);

    if (scroll_dir != 0 && scroll_frame < SCROLL_FRAMES) {
        int progress = ease_out(scroll_frame);
        int travel = (SCREEN_W / 2 + CART_W);
        int out_x = CART_X + (-scroll_dir * travel * progress / 256);
        int in_x  = CART_X + (scroll_dir * travel * (256 - progress) / 256);
        draw_cart_at(out_x, CART_Y, scroll_from);
        draw_cart_at(in_x, CART_Y, selected);
    } else if (info_state != INFO_HIDDEN) {
        int cx = info_cart_x();
        draw_cart_at(cx, CART_Y, selected);
        if (info_state == INFO_SHOWN)
            draw_info_panel(selected);
    } else {
        int by = bounce_offset(frame_count);
        draw_cart_at(CART_X, CART_Y + by, selected);
    }

    draw_selector_text(selected);
}

/* ─── Input ───────────────────────────────────────────────────────── */

#define BTN_LEFT  0x0001
#define BTN_RIGHT 0x0002
#define BTN_A     0x0004
#define BTN_START 0x0008
#define BTN_UP    0x0010
#define BTN_DOWN  0x0020
#define BTN_SEL   0x0040
#define BTN_F12   0x0080
#define BTN_B     0x0100
#define BTN_ESC   0x0200   /* ESC (distinct from F12 so file browser can tell them apart) */

static int read_selector_buttons(void) {
    nespad_read();
    ps2kbd_tick();
    int buttons = 0;
    uint32_t pad = nespad_state | nespad_state2;
    if (pad & DPAD_LEFT)   buttons |= BTN_LEFT;
    if (pad & DPAD_RIGHT)  buttons |= BTN_RIGHT;
    if (pad & DPAD_UP)     buttons |= BTN_UP;
    if (pad & DPAD_DOWN)   buttons |= BTN_DOWN;
    if (pad & DPAD_A)      buttons |= BTN_A;
    if (pad & DPAD_B)      buttons |= BTN_B;
    if (pad & DPAD_START)  buttons |= BTN_START;
    if (pad & DPAD_SELECT) buttons |= BTN_SEL;
    uint16_t kbd = ps2kbd_get_state();
#ifdef USB_HID_ENABLED
    kbd |= usbhid_get_kbd_state();
#endif
    if (kbd & KBD_STATE_LEFT)  buttons |= BTN_LEFT;
    if (kbd & KBD_STATE_RIGHT) buttons |= BTN_RIGHT;
    if (kbd & KBD_STATE_UP)    buttons |= BTN_UP;
    if (kbd & KBD_STATE_DOWN)  buttons |= BTN_DOWN;
    if (kbd & KBD_STATE_A)     buttons |= BTN_A;
    if (kbd & KBD_STATE_B)     buttons |= BTN_B;
    if (kbd & KBD_STATE_START) buttons |= BTN_START;
    if (kbd & KBD_STATE_SELECT) buttons |= BTN_SEL;
    if (kbd & KBD_STATE_F12)   buttons |= BTN_F12;
    if (kbd & KBD_STATE_ESC)   buttons |= BTN_ESC;
#ifdef USB_HID_ENABLED
    usbhid_task();
    if (usbhid_gamepad_connected()) {
        usbhid_gamepad_state_t gp;
        usbhid_get_gamepad_state(&gp);
        if (gp.dpad & 0x01) buttons |= BTN_UP;
        if (gp.dpad & 0x02) buttons |= BTN_DOWN;
        if (gp.dpad & 0x04) buttons |= BTN_LEFT;
        if (gp.dpad & 0x08) buttons |= BTN_RIGHT;
        if (gp.buttons & 0x01) buttons |= BTN_A;
        if (gp.buttons & 0x02) buttons |= BTN_B;
        if (gp.buttons & 0x40) buttons |= BTN_START;
        if (gp.buttons & 0x80) buttons |= BTN_SEL;
    }
#endif
    return buttons;
}

/* ─── File browser mode ──────────────────────────────────────────── */

#define FB_MAX_ENTRIES  256
#define FB_VISIBLE_LINES 20
#define FB_LIST_Y        31
#define FB_LINE_H         9
#define FB_NAME_X         4

typedef struct {
    char name[256];
    bool is_dir;
    uint32_t size;
} fb_entry_t;

/* Allocated in PSRAM alongside rom_list/rom_meta/img_buf */
static fb_entry_t *fb_entries;
static int fb_entry_count;

static bool is_snes_file_name(const char *name) {
    size_t len = strlen(name);
    if (len < 5) return false;
    const char *ext = name + len - 4;
    return (strcasecmp(ext, ".smc") == 0 ||
            strcasecmp(ext, ".sfc") == 0 ||
            strcasecmp(ext, ".fig") == 0);
}

static int strcasecmp_fb(const void *a, const void *b) {
    const fb_entry_t *ea = (const fb_entry_t *)a;
    const fb_entry_t *eb = (const fb_entry_t *)b;
    /* Directories before files */
    if (ea->is_dir != eb->is_dir) return ea->is_dir ? -1 : 1;
    const char *sa = ea->name, *sb = eb->name;
    for (;; sa++, sb++) {
        int ca = (*sa >= 'a' && *sa <= 'z') ? *sa - 32 : *sa;
        int cb = (*sb >= 'a' && *sb <= 'z') ? *sb - 32 : *sb;
        if (ca != cb) return ca - cb;
        if (ca == 0) return 0;
    }
}

static int fb_scan_dir(const char *path) {
    fb_entry_count = 0;
    DIR dir;
    if (f_opendir(&dir, path) != FR_OK) return 0;

    /* ".." entry to go up unless at root */
    int sort_start = 0;
    if (strlen(path) > 1) {
        strcpy(fb_entries[0].name, "..");
        fb_entries[0].is_dir = true;
        fb_entries[0].size = 0;
        fb_entry_count = 1;
        sort_start = 1;
    }

    FILINFO fno;
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0'
           && fb_entry_count < FB_MAX_ENTRIES) {
        if (fno.fname[0] == '.') continue;
        bool is_dir = (fno.fattrib & AM_DIR) != 0;
        if (!is_dir && !is_snes_file_name(fno.fname)) continue;
        strncpy(fb_entries[fb_entry_count].name, fno.fname,
                sizeof(fb_entries[0].name) - 1);
        fb_entries[fb_entry_count].name[sizeof(fb_entries[0].name) - 1] = '\0';
        fb_entries[fb_entry_count].is_dir = is_dir;
        fb_entries[fb_entry_count].size = (uint32_t)fno.fsize;
        fb_entry_count++;
    }
    f_closedir(&dir);
    if (fb_entry_count - sort_start > 1)
        qsort(&fb_entries[sort_start], fb_entry_count - sort_start,
              sizeof(fb_entry_t), strcasecmp_fb);
    return fb_entry_count;
}

static void fb_text_trunc(int x, int y, const char *s, uint8_t color, int max_chars) {
    int len = (int)strlen(s);
    if (len <= max_chars) {
        fb_text(x, y, s, color);
    } else {
        int cut = max_chars - 3;
        if (cut < 0) cut = 0;
        for (int i = 0; i < cut && s[i]; i++)
            fb_char(x + i * 6, y, s[i], color);
        for (int i = 0; i < 3 && cut + i < max_chars; i++)
            fb_char(x + (cut + i) * 6, y, '.', color);
    }
}

static void fb_draw_browser(const char *path, int selected, int scroll) {
    fb_fill(PAL_BG);

    /* Header: current path */
    fb_text_trunc(10, 13, path, PAL_WHITE, (SCREEN_W - 20) / 6);
    fb_hline(0, 22, SCREEN_W, PAL_GRAY);

    /* File list */
    int list_bottom = SCREEN_H - 24;
    bool has_scrollbar = fb_entry_count > FB_VISIBLE_LINES;
    int sb_x = SCREEN_W - 12;
    int text_right = has_scrollbar ? sb_x - 2 : SCREEN_W - 4;

    for (int i = 0; i < FB_VISIBLE_LINES && (scroll + i) < fb_entry_count; i++) {
        int idx = scroll + i;
        fb_entry_t *e = &fb_entries[idx];
        int y = FB_LIST_Y + i * FB_LINE_H;
        if (y + 7 > list_bottom) break;
        uint8_t color = PAL_GRAY;

        if (idx == selected) {
            fb_rect(0, y - 1, text_right, FB_LINE_H, PAL_CART_DARK);
            color = PAL_WHITE;
        }

        int name_x;
        if (e->is_dir) {
            fb_text(FB_NAME_X, y, "<DIR>", PAL_CART_LIGHT);
            name_x = FB_NAME_X + 36;
        } else {
            char sz[6];
            uint32_t kb = (e->size + 1023) / 1024;
            if (kb < 1000)
                snprintf(sz, sizeof(sz), "%4luK", (unsigned long)kb);
            else
                snprintf(sz, sizeof(sz), "%4luM", (unsigned long)(kb / 1024));
            fb_text(FB_NAME_X, y, sz, PAL_CART_LIGHT);
            name_x = FB_NAME_X + 36;
        }
        int name_max = (text_right - name_x) / 6;
        fb_text_trunc(name_x, y, e->name, color, name_max);
    }

    /* Scrollbar */
    if (has_scrollbar) {
        int bar_h = list_bottom - FB_LIST_Y;
        int thumb_h = bar_h * FB_VISIBLE_LINES / fb_entry_count;
        if (thumb_h < 8) thumb_h = 8;
        int max_scroll = fb_entry_count - FB_VISIBLE_LINES;
        int thumb_y = FB_LIST_Y;
        if (max_scroll > 0)
            thumb_y += (bar_h - thumb_h) * scroll / max_scroll;
        fb_rect(sb_x, FB_LIST_Y, 4, bar_h, PAL_CART_SLOT);
        fb_rect(sb_x, thumb_y, 4, thumb_h, PAL_WHITE);
    }

    /* Footer */
    fb_hline(0, SCREEN_H - 22, SCREEN_W, PAL_GRAY);
    fb_text_center(SCREEN_H - 19, "A:OPEN B:BACK SEL+A:FIND", PAL_GRAY);
}

/* Persistent state across browser calls */
static char fb_persist_path[MAX_ROM_PATH];
static int  fb_persist_selected = 0;
static int  fb_persist_scroll = 0;
static bool fb_persist_valid = false;

static void fb_save_path(const char *path) {
    strncpy(fb_persist_path, path, sizeof(fb_persist_path) - 1);
    fb_persist_path[sizeof(fb_persist_path) - 1] = '\0';
    strncpy(g_settings.browser_path, path, sizeof(g_settings.browser_path) - 1);
    g_settings.browser_path[sizeof(g_settings.browser_path) - 1] = '\0';
}

typedef enum {
    FB_RESULT_BACK,          /* user pressed B/Select+Start → back to carousel */
    FB_RESULT_SELECTED,      /* user picked a ROM → path written to out buffer */
    FB_RESULT_CAROUSEL_IDX,  /* search hit in library → open carousel at *out_idx */
} fb_result_t;

static int search_dialog_show(void);

static fb_result_t file_browser_show(char *out_path, size_t out_sz, int *out_idx) {
    setup_selector_palette();
    draw_buf = 0;
    fb = SCREEN[draw_buf];

    char cur_path[MAX_ROM_PATH];
    bool cold_boot_restore = false;
    bool persist_usable = false;

    /* Seed from SD-saved path on first call (cold boot) */
    if (!fb_persist_valid && g_settings.browser_path[0] != '\0') {
        DIR check_dir;
        if (f_opendir(&check_dir, g_settings.browser_path) == FR_OK) {
            f_closedir(&check_dir);
            strncpy(cur_path, g_settings.browser_path, sizeof(cur_path) - 1);
            cur_path[sizeof(cur_path) - 1] = '\0';
            cold_boot_restore = true;
            persist_usable = true;
        }
    }

    /* Subsequent calls: restore last in-session directory */
    if (!persist_usable && fb_persist_valid) {
        DIR check_dir;
        if (f_opendir(&check_dir, fb_persist_path) == FR_OK) {
            f_closedir(&check_dir);
            strncpy(cur_path, fb_persist_path, sizeof(cur_path) - 1);
            cur_path[sizeof(cur_path) - 1] = '\0';
            persist_usable = true;
        } else {
            fb_persist_valid = false;
        }
    }
    if (!persist_usable) {
        strcpy(cur_path, "/snes");
    }
    /* Fall back to root if the chosen directory is missing */
    {
        DIR probe;
        if (f_opendir(&probe, cur_path) != FR_OK) {
            strcpy(cur_path, "/");
        } else {
            f_closedir(&probe);
        }
    }

    fb_scan_dir(cur_path);

    int selected = 0, scroll = 0;
    if (cold_boot_restore && g_settings.browser_file[0] != '\0') {
        for (int i = 0; i < fb_entry_count; i++) {
            if (strcmp(fb_entries[i].name, g_settings.browser_file) == 0) {
                selected = i;
                break;
            }
        }
        scroll = selected - FB_VISIBLE_LINES / 2;
        if (scroll < 0) scroll = 0;
        if (fb_entry_count > FB_VISIBLE_LINES && scroll > fb_entry_count - FB_VISIBLE_LINES)
            scroll = fb_entry_count - FB_VISIBLE_LINES;
    } else if (persist_usable && !cold_boot_restore) {
        selected = fb_persist_selected;
        scroll = fb_persist_scroll;
        if (selected >= fb_entry_count) selected = fb_entry_count > 0 ? fb_entry_count - 1 : 0;
        if (scroll > selected) scroll = selected;
        if (fb_entry_count > FB_VISIBLE_LINES && scroll > fb_entry_count - FB_VISIBLE_LINES)
            scroll = fb_entry_count - FB_VISIBLE_LINES;
        if (scroll < 0) scroll = 0;
    }
    fb_persist_valid = true;

    int prev_buttons = read_selector_buttons();
    uint32_t hold_counter = 0;

    while (1) {
        fb_draw_browser(cur_path, selected, scroll);
        present();
        sleep_ms(16);

        int buttons = read_selector_buttons();
        int pressed = buttons & ~prev_buttons;
        if (buttons != 0 && buttons == prev_buttons) {
            hold_counter++;
            if (hold_counter > 20 && (hold_counter % 3) == 0)
                pressed = buttons & (BTN_UP | BTN_DOWN | BTN_LEFT | BTN_RIGHT);
        } else {
            hold_counter = 0;
        }
        prev_buttons = buttons;

        /* Select+Start: back to carousel (checked FIRST so it never leaks) */
        bool sel_start = ((pressed & BTN_SEL) && (buttons & BTN_START)) ||
                         ((pressed & BTN_START) && (buttons & BTN_SEL));
        if (sel_start) {
            g_settings.selector_mode = SELECTOR_MODE_CAROUSEL;
            fb_save_path(cur_path);
            fb_persist_selected = selected;
            fb_persist_scroll = scroll;
            settings_save();
            return FB_RESULT_BACK;
        }

        /* B / ESC / F12: back to carousel */
        if (pressed & (BTN_B | BTN_ESC | BTN_F12)) {
            g_settings.selector_mode = SELECTOR_MODE_CAROUSEL;
            fb_save_path(cur_path);
            fb_persist_selected = selected;
            fb_persist_scroll = scroll;
            settings_save();
            return FB_RESULT_BACK;
        }

        /* Select+A: search over indexed library. A hit switches back
         * to the carousel at that index. */
        bool sel_a = ((pressed & BTN_SEL) && (buttons & BTN_A)) ||
                     ((pressed & BTN_A) && (buttons & BTN_SEL));
        if (sel_a && rom_count > 0) {
            int found = search_dialog_show();
            if (found >= 0 && found < rom_count) {
                g_settings.selector_mode = SELECTOR_MODE_CAROUSEL;
                fb_save_path(cur_path);
                fb_persist_selected = selected;
                fb_persist_scroll = scroll;
                settings_save();
                if (out_idx) *out_idx = found;
                return FB_RESULT_CAROUSEL_IDX;
            }
            setup_selector_palette();
            prev_buttons = read_selector_buttons();
            continue;
        }

        /* Navigation (with wrap on single-step Up/Down) */
        if (pressed & BTN_UP) {
            if (selected > 0) selected--;
            else selected = fb_entry_count - 1;
        }
        if (pressed & BTN_DOWN) {
            if (selected < fb_entry_count - 1) selected++;
            else selected = 0;
        }
        if (pressed & BTN_LEFT) {
            selected -= FB_VISIBLE_LINES;
            if (selected < 0) selected = 0;
        }
        if (pressed & BTN_RIGHT) {
            selected += FB_VISIBLE_LINES;
            if (selected >= fb_entry_count) selected = fb_entry_count - 1;
        }

        /* Keep selection visible */
        if (selected < scroll) scroll = selected;
        if (selected >= scroll + FB_VISIBLE_LINES)
            scroll = selected - FB_VISIBLE_LINES + 1;

        /* A / Start: open directory or pick ROM */
        if ((pressed & (BTN_A | BTN_START)) && fb_entry_count > 0) {
            fb_entry_t *e = &fb_entries[selected];

            if (e->is_dir) {
                char new_path[MAX_ROM_PATH];
                if (strcmp(e->name, "..") == 0) {
                    strncpy(new_path, cur_path, sizeof(new_path) - 1);
                    new_path[sizeof(new_path) - 1] = '\0';
                    char *last_slash = strrchr(new_path, '/');
                    if (last_slash && last_slash != new_path)
                        *last_slash = '\0';
                    else
                        strcpy(new_path, "/");
                } else {
                    size_t plen = strlen(cur_path);
                    if (plen == 1 && cur_path[0] == '/')
                        snprintf(new_path, sizeof(new_path), "/%s", e->name);
                    else
                        snprintf(new_path, sizeof(new_path), "%s/%s", cur_path, e->name);
                }
                if (strlen(new_path) < sizeof(cur_path)) {
                    strcpy(cur_path, new_path);
                    fb_scan_dir(cur_path);
                    selected = 0;
                    scroll = 0;
                }
            } else if (is_snes_file_name(e->name)) {
                size_t plen = strlen(cur_path);
                if (plen == 1 && cur_path[0] == '/')
                    snprintf(out_path, out_sz, "/%s", e->name);
                else
                    snprintf(out_path, out_sz, "%s/%s", cur_path, e->name);

                /* Persist path + filename so cold boot restores the cursor */
                strncpy(g_settings.browser_file, e->name, sizeof(g_settings.browser_file) - 1);
                g_settings.browser_file[sizeof(g_settings.browser_file) - 1] = '\0';
                fb_save_path(cur_path);
                fb_persist_selected = selected;
                fb_persist_scroll = scroll;
                g_settings.selector_mode = SELECTOR_MODE_BROWSER;
                settings_save();

                /* Wait for button release so held Start/Select doesn't
                 * trigger settings hotkey in gameplay. */
                for (int i = 0; i < 60; i++) {
                    if (read_selector_buttons() == 0) break;
                    sleep_ms(16);
                }
                return FB_RESULT_SELECTED;
            }
        }
    }
}

/* ─── Search dialog ──────────────────────────────────────────────── */

#define SEARCH_MAX_QUERY   16
#define SEARCH_MAX_RESULTS  5
#define SEARCH_RESULT_NONE -1

static const char *osk_rows[] = {
    "0123456789",
    "ABCDEFGHIJ",
    "KLMNOPQRST",
    "UVWXYZ <--",
};
#define OSK_ROWS      4
#define OSK_CELL_W   14
#define OSK_CELL_H   14
#define OSK_PAD_X    ((SCREEN_W - 10 * OSK_CELL_W) / 2)

static bool osk_is_backspace(int row, int col) {
    return row == 3 && col >= 7;
}
static bool osk_is_space(int row, int col) {
    return row == 3 && col == 6;
}

static bool str_contains_ci(const char *haystack, const char *needle) {
    if (!needle[0]) return true;
    int nlen = (int)strlen(needle);
    int hlen = (int)strlen(haystack);
    for (int i = 0; i <= hlen - nlen; i++) {
        bool match = true;
        for (int j = 0; j < nlen; j++) {
            char hc = haystack[i + j];
            char nc = needle[j];
            if (hc >= 'a' && hc <= 'z') hc -= 32;
            if (nc >= 'a' && nc <= 'z') nc -= 32;
            if (hc != nc) { match = false; break; }
        }
        if (match) return true;
    }
    return false;
}

static void search_find_results(const char *query, int *results, int *result_count) {
    *result_count = 0;
    if (!query[0]) return;
    for (int i = 0; i < rom_count && *result_count < SEARCH_MAX_RESULTS; i++) {
        const char *title = rom_meta[i].title[0] ? rom_meta[i].title : rom_list[i].filename;
        if (str_contains_ci(title, query))
            results[(*result_count)++] = i;
    }
}

#define SAFE_X 10
#define SAFE_Y 10
#define SAFE_W (SCREEN_W - 2 * SAFE_X)

static void draw_search_screen(const char *query, int osk_row, int osk_col,
                               int *results, int result_count, int result_sel,
                               bool in_results) {
    fb_fill(PAL_BG);

    fb_text_center(SAFE_Y, "SEARCH", PAL_WHITE);
    fb_hline(SAFE_X, SAFE_Y + 10, SAFE_W, PAL_GRAY);

    int qx = OSK_PAD_X;
    int qy = SAFE_Y + 15;
    int qw = 10 * OSK_CELL_W;
    fb_rect(qx - 1, qy - 1, qw + 2, 11, PAL_BLACK);
    fb_hline(qx - 1, qy - 1, qw + 2, PAL_GRAY);
    fb_hline(qx - 1, qy + 9, qw + 2, PAL_GRAY);
    fb_vline(qx - 1, qy - 1, 11, PAL_GRAY);
    fb_vline(qx + qw, qy - 1, 11, PAL_GRAY);
    if (query[0])
        fb_text(qx + 2, qy + 1, query, PAL_WHITE);

    int osk_y0 = qy + 14;
    for (int r = 0; r < OSK_ROWS; r++) {
        const char *row_str = osk_rows[r];
        int row_len = (int)strlen(row_str);
        for (int c = 0; c < row_len; c++) {
            int cx = OSK_PAD_X + c * OSK_CELL_W;
            int cy = osk_y0 + r * OSK_CELL_H;

            if (osk_is_backspace(r, c)) {
                if (c == 7) {
                    int bx = cx;
                    int bw = 3 * OSK_CELL_W;
                    bool bs_sel = !in_results && (r == osk_row && osk_col >= 7);
                    if (bs_sel) {
                        fb_hline(bx, cy + 1, bw - 1, PAL_WHITE);
                        fb_hline(bx, cy + OSK_CELL_H - 1, bw - 1, PAL_WHITE);
                        fb_vline(bx, cy + 1, OSK_CELL_H - 1, PAL_WHITE);
                        fb_vline(bx + bw - 2, cy + 1, OSK_CELL_H - 1, PAL_WHITE);
                    }
                    fb_text(bx + (bw - 3 * 6) / 2, cy + 4, "DEL",
                            bs_sel ? PAL_WHITE : PAL_GRAY);
                }
                continue;
            }

            bool selected = !in_results && (r == osk_row && c == osk_col);

            if (osk_is_space(r, c)) {
                if (selected) {
                    fb_hline(cx, cy + 1, OSK_CELL_W - 1, PAL_WHITE);
                    fb_hline(cx, cy + OSK_CELL_H - 1, OSK_CELL_W - 1, PAL_WHITE);
                    fb_vline(cx, cy + 1, OSK_CELL_H - 1, PAL_WHITE);
                    fb_vline(cx + OSK_CELL_W - 2, cy + 1, OSK_CELL_H - 1, PAL_WHITE);
                }
                fb_char(cx + (OSK_CELL_W - 5) / 2, cy + 4, '_',
                        selected ? PAL_WHITE : PAL_GRAY);
                continue;
            }

            if (selected) {
                fb_hline(cx, cy + 1, OSK_CELL_W - 1, PAL_WHITE);
                fb_hline(cx, cy + OSK_CELL_H - 1, OSK_CELL_W - 1, PAL_WHITE);
                fb_vline(cx, cy + 1, OSK_CELL_H - 1, PAL_WHITE);
                fb_vline(cx + OSK_CELL_W - 2, cy + 1, OSK_CELL_H - 1, PAL_WHITE);
            }

            fb_char(cx + (OSK_CELL_W - 5) / 2, cy + 4, row_str[c],
                    selected ? PAL_WHITE : PAL_GRAY);
        }
    }

    int sep_y = osk_y0 + OSK_ROWS * OSK_CELL_H + 4;
    fb_hline(SAFE_X, sep_y, SAFE_W, PAL_GRAY);

    int max_result_chars = (SAFE_W - 4) / 6;
    int ry = sep_y + 4;
    if (result_count == 0 && query[0]) {
        fb_text_center(ry + 10, "NO RESULTS", PAL_GRAY);
    } else {
        for (int i = 0; i < result_count; i++) {
            int idx = results[i];
            const char *title = rom_meta[idx].title[0] ? rom_meta[idx].title : rom_list[idx].filename;
            bool rsel = in_results && (i == result_sel);
            if (rsel)
                fb_rect(SAFE_X, ry - 1, SAFE_W, 11, PAL_CART_DARK);
            fb_text_trunc(SAFE_X + 2, ry, title, rsel ? PAL_WHITE : PAL_GRAY, max_result_chars);
            ry += 12;
        }
    }

    int foot_y = SCREEN_H - SAFE_Y - 10;
    fb_hline(SAFE_X, foot_y, SAFE_W, PAL_GRAY);
    fb_text_center(foot_y + 3, "A:SEL  B:BACK  DOWN:RESULTS", PAL_GRAY);
}

static int search_dialog_show(void) {
    char query[SEARCH_MAX_QUERY + 1];
    query[0] = '\0';
    int qlen = 0;

    int osk_row = 1, osk_col = 0;
    int results[SEARCH_MAX_RESULTS];
    int result_count = 0;
    int result_sel = 0;
    bool in_results = false;

    int prev_buttons = read_selector_buttons();
    uint32_t hold_counter = 0;

    while (1) {
        draw_search_screen(query, osk_row, osk_col, results, result_count,
                           result_sel, in_results);
        present();
        sleep_ms(16);

        /* Consume raw ASCII from real keyboards (PS/2 + USB). Typing
         * updates the query directly without touching the on-screen
         * keyboard state. */
        int raw;
        bool query_changed = false;
        while ((raw = ps2kbd_get_raw_char()) >= 0) {
            if (raw == '\b') {
                if (qlen > 0) query[--qlen] = '\0';
                query_changed = true;
            } else if (raw == ' ' || (raw >= '0' && raw <= '9') ||
                       (raw >= 'a' && raw <= 'z') || (raw >= 'A' && raw <= 'Z')) {
                if (qlen < SEARCH_MAX_QUERY) {
                    char c = (char)raw;
                    if (c >= 'a' && c <= 'z') c -= 32;
                    query[qlen++] = c;
                    query[qlen] = '\0';
                    query_changed = true;
                }
            }
        }
#ifdef USB_HID_ENABLED
        while ((raw = usbhid_get_raw_char()) >= 0) {
            if (raw == '\b') {
                if (qlen > 0) query[--qlen] = '\0';
                query_changed = true;
            } else if (raw == ' ' || (raw >= '0' && raw <= '9') ||
                       (raw >= 'a' && raw <= 'z') || (raw >= 'A' && raw <= 'Z')) {
                if (qlen < SEARCH_MAX_QUERY) {
                    char c = (char)raw;
                    if (c >= 'a' && c <= 'z') c -= 32;
                    query[qlen++] = c;
                    query[qlen] = '\0';
                    query_changed = true;
                }
            }
        }
#endif
        if (query_changed) {
            search_find_results(query, results, &result_count);
            result_sel = 0;
            in_results = false;
        }

        int buttons = read_selector_buttons();
        int pressed = buttons & ~prev_buttons;
        if (buttons != 0 && buttons == prev_buttons) {
            hold_counter++;
            if (hold_counter > 20 && (hold_counter % 3) == 0)
                pressed = buttons & (BTN_UP | BTN_DOWN | BTN_LEFT | BTN_RIGHT);
        } else {
            hold_counter = 0;
        }
        prev_buttons = buttons;

        if (pressed & (BTN_B | BTN_ESC | BTN_F12))
            return SEARCH_RESULT_NONE;

        if (in_results) {
            if (pressed & BTN_UP) {
                if (result_sel > 0) result_sel--;
                else in_results = false;
            }
            if (pressed & BTN_DOWN) {
                if (result_sel < result_count - 1) result_sel++;
            }
            if (pressed & (BTN_A | BTN_START)) {
                if (result_count > 0) return results[result_sel];
            }
        } else {
            if (pressed & BTN_UP) {
                if (osk_row > 0) osk_row--;
            }
            if (pressed & BTN_DOWN) {
                if (osk_row < OSK_ROWS - 1) {
                    osk_row++;
                } else if (result_count > 0) {
                    in_results = true;
                    result_sel = 0;
                }
            }
            if (pressed & BTN_LEFT) {
                if (osk_col > 0) osk_col--;
            }
            if (pressed & BTN_RIGHT) {
                int row_len = (int)strlen(osk_rows[osk_row]);
                if (osk_col < row_len - 1) osk_col++;
            }

            int row_len = (int)strlen(osk_rows[osk_row]);
            if (osk_col >= row_len) osk_col = row_len - 1;

            if (pressed & (BTN_A | BTN_START)) {
                if (osk_is_backspace(osk_row, osk_col)) {
                    if (qlen > 0) query[--qlen] = '\0';
                } else if (osk_is_space(osk_row, osk_col)) {
                    if (qlen < SEARCH_MAX_QUERY) {
                        query[qlen++] = ' ';
                        query[qlen] = '\0';
                    }
                } else {
                    char ch = osk_rows[osk_row][osk_col];
                    if (qlen < SEARCH_MAX_QUERY) {
                        query[qlen++] = ch;
                        query[qlen] = '\0';
                    }
                }
                search_find_results(query, results, &result_count);
                result_sel = 0;
            }
        }
    }
}

/* ─── Progress bar ────────────────────────────────────────────────── */

static void show_indexing_progress(const char *label, int current, int total) {
    fb_fill(PAL_BG);
    fb_text_center(104, label, PAL_WHITE);

    int bar_w = 180;
    int bar_h = 10;
    int bar_x = (SCREEN_W - bar_w) / 2;
    int bar_y = 122;
    fb_rect(bar_x - 1, bar_y - 1, bar_w + 2, bar_h + 2, PAL_GRAY);
    fb_rect(bar_x, bar_y, bar_w, bar_h, PAL_BLACK);
    int fill_w = (total > 0) ? (current * bar_w / total) : 0;
    if (fill_w > 0)
        fb_rect(bar_x, bar_y, fill_w, bar_h, PAL_WHITE);

    char count_str[32];
    snprintf(count_str, sizeof(count_str), "%d / %d", current, total);
    fb_text_center(140, count_str, PAL_GRAY);

    present();
}

/* ─── Main selector ───────────────────────────────────────────────── */

/* ─── No-ROMs notice screen ───────────────────────────────────────── */

/* Shown when /snes is missing or empty. Tells the user how to seed the SD
 * card and waits for a button press before handing off to the file browser
 * (which starts at root when /snes is missing). */
static void rom_selector_no_roms_notice(void) {
    setup_selector_palette();
    draw_buf = 0;
    fb = SCREEN[draw_buf];

    const char *line1;
    const char *line2;
    if (scan_result == ROM_SCAN_NO_SNES_DIR) {
        line1 = "NO /SNES DIRECTORY ON SD CARD";
        line2 = "CREATE A /SNES FOLDER";
    } else {
        line1 = "NO .SMC/.SFC ROMS FOUND IN /SNES";
        line2 = "COPY ROMS TO /SNES";
    }

    int prev_buttons = read_selector_buttons();
    uint32_t frame = 0;

    while (1) {
        fb_fill(PAL_BG);

        fb_text_center(40, "ROM LIBRARY EMPTY", PAL_WHITE);
        fb_hline(10, 52, SCREEN_W - 20, PAL_GRAY);

        fb_text_center(70, line1, PAL_GRAY);
        fb_text_center(84, line2, PAL_GRAY);

        fb_text_center(108, "OPTIONAL: COPY METADATA/", PAL_GRAY);
        fb_text_center(120, "FOR COVER ART AND TITLES", PAL_GRAY);

        fb_text_center(150, "OPENING FILE BROWSER...", PAL_CART_LIGHT);

        if (((frame / 30) & 1) == 0)
            fb_text_center(SCREEN_H - 40, "PRESS ANY BUTTON TO CONTINUE", PAL_WHITE);

        present();
        sleep_ms(16);

        int buttons = read_selector_buttons();
        int pressed = buttons & ~prev_buttons;
        prev_buttons = buttons;
        if (pressed) break;

        frame++;
        if (frame >= 600) break;  /* auto-continue after ~10s */
    }

    /* Wait for button release so it doesn't leak into the browser */
    for (int i = 0; i < 60; i++) {
        if (read_selector_buttons() == 0) break;
        sleep_ms(16);
    }
}

/* ─── Main selector ───────────────────────────────────────────────── */

bool rom_selector_show(char *selected_rom_path, size_t buffer_size, uint8_t *screen_buffer) {
    (void)screen_buffer;

    /* Allocate large buffers in PSRAM (reset to reclaim any prior session) */
    psram_reset();
    rom_list = (rom_entry_t *)psram_malloc(MAX_ROMS * sizeof(rom_entry_t));
    rom_meta = (rom_meta_t *)psram_malloc(MAX_ROMS * sizeof(rom_meta_t));
    fb_entries = (fb_entry_t *)psram_malloc(FB_MAX_ENTRIES * sizeof(fb_entry_t));
    img_buf = (uint8_t *)psram_malloc(IMG_BUF_BYTES);
    if (!rom_list || !rom_meta || !fb_entries || !img_buf) return false;
    memset(rom_list, 0, MAX_ROMS * sizeof(rom_entry_t));
    memset(rom_meta, 0, MAX_ROMS * sizeof(rom_meta_t));

    /* Set up palette and show loading screen immediately */
    setup_selector_palette();
    draw_buf = 0;
    fb = SCREEN[draw_buf];
    fb_fill(PAL_BG);
    fb_text_center(SCREEN_H / 2 - 4, "LOADING...", PAL_WHITE);
    present();           /* show SCREEN[0], set draw_buf=1 */
    sleep_ms(100);       /* let TV sync */

    /* Scan ROMs (SD access may take time) */
    bool snes_dir_ok = false;
    scan_roms(&snes_dir_ok);

    /* No ROMs in /snes: show notice, then drop into the file browser as the
     * only way forward. Keep reopening it until the user loads a ROM. */
    if (rom_count == 0) {
        scan_result = snes_dir_ok ? ROM_SCAN_NO_ROMS : ROM_SCAN_NO_SNES_DIR;
        rom_selector_no_roms_notice();
        while (1) {
            /* Search returns FB_RESULT_CAROUSEL_IDX only when library has
             * ROMs — here rom_count==0, so only SELECTED / BACK. */
            if (file_browser_show(selected_rom_path, buffer_size, NULL) == FB_RESULT_SELECTED)
                return true;
            /* No carousel to show. Just reopen the browser. */
        }
    }
    scan_result = ROM_SCAN_OK;

    /* CRC cache — compute missing CRCs with progress bar */
    load_crc_cache();
    for (int i = 0; i < rom_count; i++) {
        if (!rom_list[i].crc_valid) ensure_crc(i);
        show_indexing_progress("INDEXING ROMS...", i + 1, rom_count);
    }
    save_crc_cache();

    /* Load metadata with progress bar */
    for (int i = 0; i < rom_count; i++) {
        load_rom_title(i);
        show_indexing_progress("LOADING METADATA...", i + 1, rom_count);
    }

    load_last_rom();

    int selected = last_selected_rom;
    if (selected >= rom_count) selected = 0;

    /* Start directly in file browser if that was the last used mode */
    if (g_settings.selector_mode == SELECTOR_MODE_BROWSER) {
        int fb_idx = -1;
        fb_result_t fr = file_browser_show(selected_rom_path, buffer_size, &fb_idx);
        if (fr == FB_RESULT_SELECTED) return true;
        if (fr == FB_RESULT_CAROUSEL_IDX && fb_idx >= 0 && fb_idx < rom_count)
            selected = fb_idx;
        /* Otherwise fall through to carousel */
    }

    int prev_buttons = read_selector_buttons();
    uint32_t hold_counter = 0;
    uint32_t frame_count = 0;
    cur_img_idx = -1;
    scroll_dir = 0;
    scroll_frame = 0;
    info_state = INFO_HIDDEN;
    info_anim_frame = 0;

    load_rom_image(selected);

    while (1) {
        draw_scene(selected, frame_count);
        present();
        frame_count++;
        sleep_ms(16);

        /* Advance scroll animation */
        if (scroll_dir != 0) {
            scroll_frame++;
            if (scroll_frame >= SCROLL_FRAMES) {
                scroll_dir = 0;
                scroll_frame = 0;
            }
        }

        /* Advance info panel animation */
        if (info_state == INFO_SLIDING_IN) {
            info_anim_frame++;
            if (info_anim_frame >= INFO_ANIM_FRAMES) {
                info_state = INFO_SHOWN;
                info_anim_frame = 0;
            }
        } else if (info_state == INFO_SLIDING_OUT) {
            info_anim_frame++;
            if (info_anim_frame >= INFO_ANIM_FRAMES) {
                info_state = INFO_HIDDEN;
                info_anim_frame = 0;
            }
        }

        /* Input */
        int buttons = read_selector_buttons();
        int pressed = buttons & ~prev_buttons;
        if (buttons != 0 && buttons == prev_buttons) {
            hold_counter++;
            if (hold_counter > 20 && (hold_counter % 5) == 0)
                pressed = buttons;
        } else {
            hold_counter = 0;
        }
        prev_buttons = buttons;

        /* Select+Start: switch to file browser. Checked BEFORE anything
         * else so the combo never leaks to per-button handlers. Settings
         * is intentionally NOT available from the carousel — it only
         * opens while a ROM is running. */
        bool sel_start = ((pressed & BTN_SEL) && (buttons & BTN_START)) ||
                         ((pressed & BTN_START) && (buttons & BTN_SEL));
        if (sel_start || (pressed & BTN_F12)) {
            g_settings.selector_mode = SELECTOR_MODE_BROWSER;
            int fb_idx = -1;
            fb_result_t fr = file_browser_show(selected_rom_path, buffer_size, &fb_idx);
            if (fr == FB_RESULT_SELECTED) return true;
            if (fr == FB_RESULT_CAROUSEL_IDX && fb_idx >= 0 && fb_idx < rom_count)
                selected = fb_idx;
            /* Browser returned without picking — restore carousel */
            setup_selector_palette();
            draw_buf = 0;
            fb = SCREEN[draw_buf];
            cur_img_idx = -1;
            load_rom_image(selected);
            prev_buttons = read_selector_buttons();
            info_state = INFO_HIDDEN;
            info_anim_frame = 0;
            scroll_dir = 0;
            scroll_frame = 0;
            continue;
        }

        /* Select+A: search dialog */
        bool sel_a = ((pressed & BTN_SEL) && (buttons & BTN_A)) ||
                     ((pressed & BTN_A) && (buttons & BTN_SEL));
        if (sel_a) {
            int found = search_dialog_show();
            if (found >= 0 && found < rom_count) {
                selected = found;
                load_rom_image(selected);
            }
            setup_selector_palette();
            cur_img_idx = -1;
            load_rom_image(selected);
            prev_buttons = read_selector_buttons();
            info_state = INFO_HIDDEN;
            info_anim_frame = 0;
            scroll_dir = 0;
            scroll_frame = 0;
            continue;
        }

        /* Info panel: UP opens, DOWN closes */
        if (info_state == INFO_HIDDEN && scroll_dir == 0 && (pressed & BTN_UP)) {
            info_state = INFO_SLIDING_IN;
            info_anim_frame = 0;
        }
        if (info_state == INFO_SHOWN && (pressed & BTN_DOWN)) {
            info_state = INFO_SLIDING_OUT;
            info_anim_frame = 0;
        }

        /* Navigation (only when idle) */
        bool can_navigate = (scroll_dir == 0 && info_state == INFO_HIDDEN);
        if (can_navigate) {
            if (pressed & BTN_LEFT) {
                scroll_from = selected;
                selected = (selected - 1 + rom_count) % rom_count;
                scroll_dir = -1;
                scroll_frame = 0;
                load_rom_image(selected);
            }
            if (pressed & BTN_RIGHT) {
                scroll_from = selected;
                selected = (selected + 1) % rom_count;
                scroll_dir = 1;
                scroll_frame = 0;
                load_rom_image(selected);
            }
        }

        /* Select ROM */
        if (pressed & (BTN_A | BTN_START)) {
            if (!(buttons & BTN_SEL)) {
                /* Close info panel first */
                if (info_state != INFO_HIDDEN) {
                    info_state = INFO_SLIDING_OUT;
                    info_anim_frame = 0;
                    while (info_state == INFO_SLIDING_OUT) {
                        draw_scene(selected, frame_count);
                        present();
                        frame_count++;
                        sleep_ms(16);
                        info_anim_frame++;
                        if (info_anim_frame >= INFO_ANIM_FRAMES) {
                            info_state = INFO_HIDDEN;
                            info_anim_frame = 0;
                        }
                    }
                }
                if (scroll_dir != 0) continue;

                /* Wait for button release */
                for (int w = 0; w < 60; w++) {
                    if (read_selector_buttons() == 0) break;
                    sleep_ms(16);
                }

                save_last_rom(selected);
                g_settings.selector_mode = SELECTOR_MODE_CAROUSEL;
                settings_save();
                snprintf(selected_rom_path, buffer_size, "/snes/%s", rom_list[selected].filename);
                return true;
            }
        }
    }

    return false;
}

/* ─── SNES controller pixel art (37x17) ───────────────────────────── */

/* Palette:  0=transparent  1=black  2=white  3=light_gray  4=body_gray
 *           5=dark_gray  6=blue  7=green  8=red  9=yellow */
#define LOGO_W 37
#define LOGO_H 17

static const uint8_t snes_logo[LOGO_H][LOGO_W] = {
    {0,0,0,0,0,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,0,0,0,0,0},
    {0,0,0,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,0,0,0},
    {0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0},
    {0,1,4,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,4,1,0},
    {0,1,4,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,4,1,0},
    {1,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,1},
    {1,4,4,4,4,4,4,1,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,6,6,4,4,4,4,4,4,1},
    {1,4,4,4,4,4,1,1,1,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,7,7,4,4,4,4,8,8,4,4,4,1},
    {1,4,4,4,1,1,1,1,1,1,1,4,4,4,1,1,1,4,4,1,1,1,4,4,4,7,7,4,4,4,4,8,8,4,4,4,1},
    {1,4,4,4,4,4,1,1,1,4,4,4,4,4,1,1,1,4,4,1,1,1,4,4,4,4,4,4,9,9,4,4,4,4,4,4,1},
    {1,4,4,4,4,4,4,1,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,9,9,4,4,4,4,4,4,1},
    {1,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,1},
    {1,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,1},
    {1,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,1},
    {0,1,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,1,0},
    {0,0,1,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,1,0,0},
    {0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0},
};

/* Map logo pixel values to framebuffer palette indices */
#define PAL_LOGO_BODY   226
#define PAL_LOGO_LGRAY  227
#define PAL_LOGO_DGRAY  228
#define PAL_LOGO_BLUE   229
#define PAL_LOGO_GREEN  230
#define PAL_LOGO_RED    231
#define PAL_LOGO_YELLOW 232
#define PAL_LOGO_CIRCLE 233
#define PAL_LOGO_SHADOW 234

static const uint8_t logo_pal_map[10] = {
    0,              /* 0 = transparent */
    PAL_BLACK,      /* 1 = black */
    PAL_WHITE,      /* 2 = white */
    PAL_LOGO_LGRAY, /* 3 = light gray */
    PAL_LOGO_BODY,  /* 4 = body gray */
    PAL_LOGO_DGRAY, /* 5 = dark gray */
    PAL_LOGO_BLUE,  /* 6 = blue */
    PAL_LOGO_GREEN, /* 7 = green */
    PAL_LOGO_RED,   /* 8 = red */
    PAL_LOGO_YELLOW,/* 9 = yellow */
};

static void setup_welcome_palette(void) {
    setup_selector_palette();
    graphics_set_palette(PAL_LOGO_BODY,   0xB0B0B8);
    graphics_set_palette(PAL_LOGO_LGRAY,  0xD0D0D8);
    graphics_set_palette(PAL_LOGO_DGRAY,  0x606068);
    graphics_set_palette(PAL_LOGO_BLUE,   0x4040E0);
    graphics_set_palette(PAL_LOGO_GREEN,  0x30C030);
    graphics_set_palette(PAL_LOGO_RED,    0xE03030);
    graphics_set_palette(PAL_LOGO_YELLOW, 0xE0D020);
    graphics_set_palette(PAL_LOGO_CIRCLE, 0x909090);
    graphics_set_palette(PAL_LOGO_SHADOW, 0x606060);
    graphics_restore_sync_colors();
}

static void draw_filled_circle(int cx, int cy, int r, uint8_t color) {
    /* Use the half-pixel test (2dx+1)^2 <= 4*(r^2 - dy^2) so each row's
     * half-width matches a continuous-circle sample at the pixel centre.
     * The naive integer test ((dx+1)^2 <= r^2-dy^2) produced four
     * single-pixel stubs at the cardinal extremes because one row was
     * allowed to poke past the reach of its neighbours. */
    int four_r2 = 4 * r * r;
    for (int y = cy - r; y <= cy + r; y++) {
        if (y < 0 || y >= SCREEN_H) continue;
        int dy = y - cy;
        int limit = four_r2 - 4 * dy * dy;
        if (limit <= 0) continue; /* drop the collapsed top/bottom row */
        int dx = 0;
        while ((2 * dx + 3) * (2 * dx + 3) <= limit) dx++;
        int x0 = cx - dx;
        int x1 = cx + dx;
        if (x0 < 0) x0 = 0;
        if (x1 >= SCREEN_W) x1 = SCREEN_W - 1;
        if (x0 <= x1) memset(&fb[y * SCREEN_W + x0], color, x1 - x0 + 1);
    }
}

/* ─── Starfield backdrop (welcome screen) ─────────────────────────── */

#define STAR_COUNT 64

typedef struct {
    int32_t x, y;       /* Q8.8 sub-pixel position */
    int16_t vx;         /* Q8.8 per-frame horizontal velocity (depth tier) */
    int16_t prev_x, prev_y;
    uint8_t color;
    uint8_t _pad;
} star_t;

static star_t stars[STAR_COUNT];
static bool stars_ready = false;

static uint32_t star_rng_state = 0xC0FFEE17u;
static uint32_t star_rng(void) {
    star_rng_state = star_rng_state * 1664525u + 1013904223u;
    return star_rng_state;
}

static void init_starfield(void) {
    const uint8_t tier_color[3] = {
        cube_rgb(2, 2, 3),   /* far: dim blue-grey */
        cube_rgb(4, 4, 5),   /* mid: light blue */
        cube_rgb(7, 7, 7),   /* near: white */
    };
    const int16_t tier_vx[3] = { -32, -80, -160 };

    for (int i = 0; i < STAR_COUNT; i++) {
        int tier = (int)(star_rng() % 3);
        stars[i].x = (int32_t)(star_rng() % (SCREEN_W << 8));
        stars[i].y = (int32_t)(star_rng() % (SCREEN_H << 8));
        stars[i].vx = tier_vx[tier];
        stars[i].color = tier_color[tier];
        stars[i].prev_x = -1;
        stars[i].prev_y = -1;
    }
    stars_ready = true;
}

static void draw_starfield(void) {
    if (!stars_ready) init_starfield();
    for (int i = 0; i < STAR_COUNT; i++) {
        stars[i].x += stars[i].vx;
        if (stars[i].x < 0) stars[i].x += (SCREEN_W << 8);
        else if (stars[i].x >= (SCREEN_W << 8)) stars[i].x -= (SCREEN_W << 8);

        int sx = stars[i].x >> 8;
        int sy = stars[i].y >> 8;
        stars[i].prev_x = (int16_t)sx;
        stars[i].prev_y = (int16_t)sy;
        fb_pixel(sx, sy, stars[i].color);
    }
}

static void draw_logo_3x(int ox, int oy) {
    for (int y = 0; y < LOGO_H; y++) {
        for (int x = 0; x < LOGO_W; x++) {
            uint8_t px = snes_logo[y][x];
            if (px == 0) continue;
            uint8_t c = logo_pal_map[px];
            int dx = ox + x * 3;
            int dy = oy + y * 3;
            for (int sy = 0; sy < 3; sy++)
                for (int sx = 0; sx < 3; sx++)
                    fb_pixel(dx + sx, dy + sy, c);
        }
    }
}

static void draw_logo_shadow(int cx, int cy, int bounce, int circle_r) {
    int logo_h_half = (LOGO_H * 3 / 2);
    int shadow_cy = cy + logo_h_half + 4 - bounce;
    int base_rx = (LOGO_W * 3 / 2) - 4;
    int base_ry = 3;
    int rx = base_rx + (-bounce);
    int ry = base_ry + (-bounce) / 2;
    if (rx < 4) rx = 4;
    if (ry < 2) ry = 2;
    int cr2 = circle_r * circle_r;
    for (int y = shadow_cy - ry; y <= shadow_cy + ry; y++) {
        if (y < 0 || y >= SCREEN_H) continue;
        int dy_s = y - shadow_cy;
        int dy_c = y - cy;
        if (dy_c * dy_c > cr2) continue;
        for (int x = cx - rx; x <= cx + rx; x++) {
            if (x < 0 || x >= SCREEN_W) continue;
            int dx_s = x - cx;
            int dx_c = x - cx;
            if (dx_s * dx_s * ry * ry + dy_s * dy_s * rx * rx < rx * rx * ry * ry) {
                if (dx_c * dx_c + dy_c * dy_c <= cr2) {
                    fb_pixel(x, y, PAL_LOGO_SHADOW);
                }
            }
        }
    }
}

static int welcome_bounce(uint32_t frame) {
    int t = (int)(frame % 36);
    if (t < 6) return 0;
    if (t < 9) return 1;
    if (t < 15) return 2;
    if (t < 18) return 1;
    if (t < 24) return 0;
    if (t < 27) return -1;
    if (t < 33) return -2;
    return -1;
}

/* ─── Welcome screen ──────────────────────────────────────────────── */

void welcome_screen_show(void) {
    setup_welcome_palette();
    draw_buf = 0;
    fb = SCREEN[draw_buf];

#ifdef FRANK_SNES_VERSION
    char version_str[16];
    snprintf(version_str, sizeof(version_str), "V%s", FRANK_SNES_VERSION);
#else
    const char *version_str = "V?";
#endif

    uint32_t frame = 0;
    int prev_buttons = 0xFF;  /* ignore initial held buttons */

    while (1) {
        fb_fill(PAL_BG);
        draw_starfield();

        /* Circle behind the logo */
        int circle_cx = SCREEN_W / 2;
        int circle_cy = 68;
        int circle_r = 44;
        draw_filled_circle(circle_cx, circle_cy, circle_r, PAL_LOGO_CIRCLE);

        /* SNES controller with bounce and shadow */
        int bounce = (frame >= 60) ? welcome_bounce(frame) : 0;
        int logo_x = circle_cx - (LOGO_W * 3 / 2);
        int logo_y = circle_cy - (LOGO_H * 3 / 2) + bounce;
        draw_logo_shadow(circle_cx, circle_cy, bounce, circle_r);
        draw_logo_3x(logo_x, logo_y);

        /* Text — drop-shadowed so the starfield stays readable */
        fb_text_center_shadow(118, "FRANK SNES", PAL_WHITE, PAL_BLACK);
        fb_text_center_shadow(132, version_str, PAL_LOGO_LGRAY, PAL_BLACK);
        fb_text_center_shadow(152, "BY MIKHAIL MATVEEV", PAL_LOGO_LGRAY, PAL_BLACK);
        fb_text_center_shadow(164, "GITHUB.COM/RH1TECH/MURMSNES", PAL_LOGO_LGRAY, PAL_BLACK);
        fb_text_center_shadow(184, "RH1.TECH", PAL_LOGO_LGRAY, PAL_BLACK);

        /* Blinking "PRESS START" after 2 seconds */
        if (frame >= 120 && ((frame / 30) & 1) == 0) {
            fb_text_center_shadow(SCREEN_H - 16, "PRESS START", PAL_WHITE, PAL_BLACK);
        }

        present();
        frame++;
        sleep_ms(16);

        /* Read input every frame so edge detection works across the
         * settle-to-active transition. */
        int buttons = read_selector_buttons();
        if (frame >= 120) {
            int pressed = buttons & ~prev_buttons;
            if (pressed) break;   /* any button advances */
        }
        prev_buttons = buttons;

        /* Auto-continue after 10 seconds */
        if (frame >= 600) break;
    }

    /* Wait for buttons to be released */
    for (int i = 0; i < 60; i++) {
        if (read_selector_buttons() == 0) break;
        sleep_ms(16);
    }
}

/* ─── SD error screen ─────────────────────────────────────────────── */

void rom_selector_show_sd_error(uint8_t *screen_buffer, int error_code) {
    (void)screen_buffer;
    setup_selector_palette();

    draw_buf = 0;
    fb = SCREEN[draw_buf];
    fb_fill(PAL_BG);
    fb_text_center(SCREEN_H / 2 - 20, "SD CARD ERROR", PAL_WHITE);
    fb_text_center(SCREEN_H / 2, "NO SD CARD DETECTED", PAL_GRAY);

    char code_str[32];
    snprintf(code_str, sizeof(code_str), "ERROR CODE: %d", error_code);
    fb_text_center(SCREEN_H / 2 + 16, code_str, PAL_GRAY);
    fb_text_center(SCREEN_H / 2 + 36, "INSERT FAT32 SD CARD", PAL_WHITE);
    fb_text_center(SCREEN_H / 2 + 48, "AND RESET THE DEVICE", PAL_WHITE);
    present();

    while (1) { tight_loop_contents(); }
}
