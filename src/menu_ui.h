/*
 * FRANK SNES - Menu UI rendering primitives
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef MENU_UI_H
#define MENU_UI_H

#include <stdint.h>
#include <stdbool.h>

// Screen dimensions for SNES
#define MENU_SCREEN_WIDTH  256
#define MENU_SCREEN_HEIGHT 224

// Palette index definitions - must match palette setup in main.c
// Use same indices as murmgenesis for compatibility
#define COLOR_BLACK   1    // Near-black (not 0 for HDMI compatibility)
#define COLOR_WHITE   63   // 0b00111111 - bright white
#define COLOR_GRAY    42   // 0b00101010 - medium gray
#define COLOR_YELLOW  48   // Yellow
#define COLOR_RED     32   // Red
#define COLOR_DARK    16   // Dark gray for scrollbar background

// Font dimensions
#define FONT_WIDTH 6    // 5px glyph + 1px spacing
#define FONT_HEIGHT 7
#define LINE_HEIGHT 10

// Bold font dimensions (for title)
#define BOLD_FONT_WIDTH 8   // 7px glyph + 1px spacing
#define BOLD_FONT_HEIGHT 9

/**
 * Initialize the menu palette colors in the HDMI driver
 * Must be called before drawing any menu content
 */
void menu_ui_init_palette(void);

/**
 * Draw a single character at (x, y) using the 5x7 font
 */
void menu_draw_char(uint8_t *screen, int x, int y, char ch, uint8_t color);

/**
 * Draw text string at (x, y)
 */
void menu_draw_text(uint8_t *screen, int x, int y, const char *text, uint8_t color);

/**
 * Draw bold character at (x, y) using the 7x9 font
 */
void menu_draw_char_bold(uint8_t *screen, int x, int y, char ch, uint8_t color);

/**
 * Draw bold text string at (x, y)
 */
void menu_draw_text_bold(uint8_t *screen, int x, int y, const char *text, uint8_t color);

/**
 * Fill rectangle with color
 */
void menu_fill_rect(uint8_t *screen, int x, int y, int w, int h, uint8_t color);

/**
 * Clear entire screen to a color
 */
void menu_clear_screen(uint8_t *screen, uint8_t color);

/**
 * Get text width in pixels
 */
int menu_text_width(const char *text);

/**
 * Get bold text width in pixels
 */
int menu_text_width_bold(const char *text);

/**
 * Draw centered text
 */
void menu_draw_text_centered(uint8_t *screen, int y, const char *text, uint8_t color);

/**
 * Draw centered bold text
 */
void menu_draw_text_bold_centered(uint8_t *screen, int y, const char *text, uint8_t color);

#endif // MENU_UI_H
