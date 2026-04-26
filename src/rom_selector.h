/*
 * MurmSNES - ROM Selector
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef ROM_SELECTOR_H
#define ROM_SELECTOR_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Maximum length of ROM filename (including path)
#define MAX_ROM_PATH 128

/**
 * Display ROM selection screen and wait for user to select a ROM
 * @param selected_rom_path Buffer to store the selected ROM path
 * @param buffer_size Size of the buffer
 * @param screen_buffer Pointer to the screen buffer (256x224 8-bit palette-indexed)
 * @return true if ROM was selected, false if user canceled or no ROMs found
 */
bool rom_selector_show(char *selected_rom_path, size_t buffer_size, uint8_t *screen_buffer);

/**
 * Display SD card error screen (blocks forever)
 * @param screen_buffer Pointer to the screen buffer (256x224 8-bit palette-indexed)
 * @param error_code The FRESULT error code from f_mount
 */
void rom_selector_show_sd_error(uint8_t *screen_buffer, int error_code);

/**
 * Show welcome/splash screen with SNES controller logo.
 * Waits for user input or auto-continues after timeout.
 */
void welcome_screen_show(void);

/**
 * Show a warning screen if any game-affecting video settings are off
 * (BGs, sprites, transparency, HDMA).  No-op when all are default.
 * CRT overscan is excluded because it does not affect game content.
 * Waits for user input or auto-continues after timeout.
 */
void video_settings_warning_show(void);

#endif // ROM_SELECTOR_H
