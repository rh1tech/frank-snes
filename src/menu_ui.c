/*
 * FRANK SNES - Menu UI rendering primitives
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "menu_ui.h"
#include "HDMI.h"
#include <string.h>

// Initialize palette entries for menu colors
// Uses same indices as murmgenesis for compatibility
void menu_ui_init_palette(void) {
    // Set up palette entries for menu colors
    // Format: RGB888 (0x00RRGGBB)
    graphics_set_palette(0, 0x020202);      // Very dark (not pure black - HDMI issue)
    graphics_set_palette(1, 0x020202);      // Near-black (COLOR_BLACK)
    graphics_set_palette(16, 0x404040);     // Dark gray (COLOR_DARK)
    graphics_set_palette(32, 0xFF0000);     // Red (COLOR_RED)
    graphics_set_palette(42, 0x808080);     // Medium gray (COLOR_GRAY)
    graphics_set_palette(48, 0xFFFF00);     // Yellow (COLOR_YELLOW)
    graphics_set_palette(63, 0xFFFFFF);     // White (COLOR_WHITE)
    graphics_restore_sync_colors();         // Ensure HDMI reserved sync symbols are intact
}

// 5x7 font glyphs
static const uint8_t *glyph_5x7(char ch) {
    static const uint8_t glyph_space[7] = {0, 0, 0, 0, 0, 0, 0};
    static const uint8_t glyph_dot[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C};
    static const uint8_t glyph_hyphen[7] = {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00};
    static const uint8_t glyph_underscore[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F};
    static const uint8_t glyph_colon[7] = {0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00};
    static const uint8_t glyph_slash[7] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x00, 0x00};
    static const uint8_t glyph_comma[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x04};
    static const uint8_t glyph_lparen[7] = {0x02, 0x04, 0x08, 0x08, 0x08, 0x04, 0x02};
    static const uint8_t glyph_rparen[7] = {0x08, 0x04, 0x02, 0x02, 0x02, 0x04, 0x08};

    static const uint8_t glyph_0[7] = {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E};
    static const uint8_t glyph_1[7] = {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E};
    static const uint8_t glyph_2[7] = {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F};
    static const uint8_t glyph_3[7] = {0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E};
    static const uint8_t glyph_4[7] = {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02};
    static const uint8_t glyph_5[7] = {0x1F, 0x10, 0x10, 0x1E, 0x01, 0x01, 0x1E};
    static const uint8_t glyph_6[7] = {0x0E, 0x10, 0x10, 0x1E, 0x11, 0x11, 0x0E};
    static const uint8_t glyph_7[7] = {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08};
    static const uint8_t glyph_8[7] = {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E};
    static const uint8_t glyph_9[7] = {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x0E};

    static const uint8_t glyph_A[7] = {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11};
    static const uint8_t glyph_B[7] = {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E};
    static const uint8_t glyph_C[7] = {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E};
    static const uint8_t glyph_D[7] = {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E};
    static const uint8_t glyph_E[7] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F};
    static const uint8_t glyph_F[7] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10};
    static const uint8_t glyph_G[7] = {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0E};
    static const uint8_t glyph_H[7] = {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11};
    static const uint8_t glyph_I[7] = {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1F};
    static const uint8_t glyph_J[7] = {0x07, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0C};
    static const uint8_t glyph_K[7] = {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11};
    static const uint8_t glyph_L[7] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F};
    static const uint8_t glyph_M[7] = {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11};
    static const uint8_t glyph_N[7] = {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11};
    static const uint8_t glyph_O[7] = {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E};
    static const uint8_t glyph_P[7] = {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10};
    static const uint8_t glyph_Q[7] = {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D};
    static const uint8_t glyph_R[7] = {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11};
    static const uint8_t glyph_S[7] = {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E};
    static const uint8_t glyph_T[7] = {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04};
    static const uint8_t glyph_U[7] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E};
    static const uint8_t glyph_V[7] = {0x11, 0x11, 0x11, 0x11, 0x0A, 0x0A, 0x04};
    static const uint8_t glyph_W[7] = {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A};
    static const uint8_t glyph_X[7] = {0x11, 0x0A, 0x04, 0x04, 0x04, 0x0A, 0x11};
    static const uint8_t glyph_Y[7] = {0x11, 0x0A, 0x04, 0x04, 0x04, 0x04, 0x04};
    static const uint8_t glyph_Z[7] = {0x1F, 0x02, 0x04, 0x08, 0x10, 0x10, 0x1F};

    // Convert lowercase to uppercase
    int c = (unsigned char)ch;
    if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';

    switch (c) {
        case ' ': return glyph_space;
        case '.': return glyph_dot;
        case '-': return glyph_hyphen;
        case '_': return glyph_underscore;
        case ':': return glyph_colon;
        case '/': return glyph_slash;
        case ',': return glyph_comma;
        case '(': return glyph_lparen;
        case ')': return glyph_rparen;
        case '0': return glyph_0;
        case '1': return glyph_1;
        case '2': return glyph_2;
        case '3': return glyph_3;
        case '4': return glyph_4;
        case '5': return glyph_5;
        case '6': return glyph_6;
        case '7': return glyph_7;
        case '8': return glyph_8;
        case '9': return glyph_9;
        case 'A': return glyph_A;
        case 'B': return glyph_B;
        case 'C': return glyph_C;
        case 'D': return glyph_D;
        case 'E': return glyph_E;
        case 'F': return glyph_F;
        case 'G': return glyph_G;
        case 'H': return glyph_H;
        case 'I': return glyph_I;
        case 'J': return glyph_J;
        case 'K': return glyph_K;
        case 'L': return glyph_L;
        case 'M': return glyph_M;
        case 'N': return glyph_N;
        case 'O': return glyph_O;
        case 'P': return glyph_P;
        case 'Q': return glyph_Q;
        case 'R': return glyph_R;
        case 'S': return glyph_S;
        case 'T': return glyph_T;
        case 'U': return glyph_U;
        case 'V': return glyph_V;
        case 'W': return glyph_W;
        case 'X': return glyph_X;
        case 'Y': return glyph_Y;
        case 'Z': return glyph_Z;
        default: return glyph_space;
    }
}

// Bold 7x9 font for title
static const uint8_t *glyph_bold(char ch) {
    static const uint8_t glyph_space[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t glyph_M[9] = {0x63, 0x77, 0x7F, 0x6B, 0x63, 0x63, 0x63, 0x63, 0x63};
    static const uint8_t glyph_U[9] = {0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x77, 0x3E};
    static const uint8_t glyph_R[9] = {0x7E, 0x63, 0x63, 0x63, 0x7E, 0x6C, 0x66, 0x63, 0x63};
    static const uint8_t glyph_S[9] = {0x3E, 0x63, 0x60, 0x70, 0x3E, 0x07, 0x03, 0x63, 0x3E};
    static const uint8_t glyph_N[9] = {0x63, 0x73, 0x7B, 0x7F, 0x6F, 0x67, 0x63, 0x63, 0x63};
    static const uint8_t glyph_E[9] = {0x7F, 0x60, 0x60, 0x60, 0x7E, 0x60, 0x60, 0x60, 0x7F};

    int c = (unsigned char)ch;
    if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';

    switch (c) {
        case 'M': return glyph_M;
        case 'U': return glyph_U;
        case 'R': return glyph_R;
        case 'S': return glyph_S;
        case 'N': return glyph_N;
        case 'E': return glyph_E;
        default: return glyph_space;
    }
}

void menu_draw_char(uint8_t *screen, int x, int y, char ch, uint8_t color) {
    const uint8_t *rows = glyph_5x7(ch);
    for (int row = 0; row < FONT_HEIGHT; ++row) {
        int yy = y + row;
        if (yy < 0 || yy >= MENU_SCREEN_HEIGHT) continue;
        uint8_t bits = rows[row];
        for (int col = 0; col < 5; ++col) {
            int xx = x + col;
            if (xx < 0 || xx >= MENU_SCREEN_WIDTH) continue;
            if (bits & (1u << (4 - col))) {
                screen[yy * MENU_SCREEN_WIDTH + xx] = color;
            }
        }
    }
}

void menu_draw_text(uint8_t *screen, int x, int y, const char *text, uint8_t color) {
    for (const char *p = text; *p; ++p) {
        menu_draw_char(screen, x, y, *p, color);
        x += FONT_WIDTH;
    }
}

void menu_draw_char_bold(uint8_t *screen, int x, int y, char ch, uint8_t color) {
    const uint8_t *rows = glyph_bold(ch);
    for (int row = 0; row < BOLD_FONT_HEIGHT; ++row) {
        int yy = y + row;
        if (yy < 0 || yy >= MENU_SCREEN_HEIGHT) continue;
        uint8_t bits = rows[row];
        for (int col = 0; col < 7; ++col) {
            int xx = x + col;
            if (xx < 0 || xx >= MENU_SCREEN_WIDTH) continue;
            if (bits & (1u << (6 - col))) {
                screen[yy * MENU_SCREEN_WIDTH + xx] = color;
            }
        }
    }
}

void menu_draw_text_bold(uint8_t *screen, int x, int y, const char *text, uint8_t color) {
    for (const char *p = text; *p; ++p) {
        menu_draw_char_bold(screen, x, y, *p, color);
        x += BOLD_FONT_WIDTH;
    }
}

void menu_fill_rect(uint8_t *screen, int x, int y, int w, int h, uint8_t color) {
    if (w <= 0 || h <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > MENU_SCREEN_WIDTH) w = MENU_SCREEN_WIDTH - x;
    if (y + h > MENU_SCREEN_HEIGHT) h = MENU_SCREEN_HEIGHT - y;
    if (w <= 0 || h <= 0) return;

    for (int yy = y; yy < y + h; ++yy) {
        uint8_t *row = &screen[yy * MENU_SCREEN_WIDTH + x];
        for (int xx = 0; xx < w; ++xx) {
            row[xx] = color;
        }
    }
}

void menu_clear_screen(uint8_t *screen, uint8_t color) {
    for (int i = 0; i < MENU_SCREEN_WIDTH * MENU_SCREEN_HEIGHT; ++i) {
        screen[i] = color;
    }
}

int menu_text_width(const char *text) {
    return (int)strlen(text) * FONT_WIDTH;
}

int menu_text_width_bold(const char *text) {
    return (int)strlen(text) * BOLD_FONT_WIDTH;
}

void menu_draw_text_centered(uint8_t *screen, int y, const char *text, uint8_t color) {
    int w = menu_text_width(text);
    int x = (MENU_SCREEN_WIDTH - w) / 2;
    if (x < 0) x = 0;
    menu_draw_text(screen, x, y, text, color);
}

void menu_draw_text_bold_centered(uint8_t *screen, int y, const char *text, uint8_t color) {
    int w = menu_text_width_bold(text);
    int x = (MENU_SCREEN_WIDTH - w) / 2;
    if (x < 0) x = 0;
    menu_draw_text_bold(screen, x, y, text, color);
}
