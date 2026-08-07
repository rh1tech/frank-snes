/*
 * FRANK SNES — C2 input stubs
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * The FRANK Core 2 board has neither a NES pad header nor a PS/2
 * header: GPIO20..43 are the inter-processor link to the sound slave,
 * and the PS/2 keyboard driver would want PIO2, which the link needs
 * because bus B reaches GPIO39 and only PIO2 can be given the upper
 * GPIO window. USB HID is the only input path on this board.
 *
 * Rather than thread `#ifdef BOARD_C2` through every input call site in
 * main.c and rom_selector.c — there are two dozen — the drivers are left
 * out of the build and their API is answered here with "nothing
 * connected". The state variables read as zero, the tick functions do
 * nothing, and the mouse reports itself uninitialised, which is exactly
 * what the existing code already handles for a board with no pad
 * plugged in.
 *
 * Only compiled for BOARD_VARIANT=C2.
 */

#include <stdbool.h>
#include <stdint.h>

/* ---- NES/SNES pad (drivers/nespad) ---- */

uint32_t nespad_state;
uint32_t nespad_state2;
bool     nespad_is_snes;
bool     nespad2_is_snes;

bool nespad_begin(uint32_t cpu_khz, uint8_t clkPin, uint8_t dataPin,
                  uint8_t latPin)
{
    (void)cpu_khz; (void)clkPin; (void)dataPin; (void)latPin;
    return false;
}

void nespad_read(void) { }

/* ---- PS/2 keyboard (src/ps2kbd) ---- */

void ps2kbd_init(void) { }
void ps2kbd_tick(void) { }

int ps2kbd_get_key(int *pressed, unsigned char *key)
{
    (void)pressed; (void)key;
    return 0;
}

uint16_t ps2kbd_get_state(void)       { return 0; }
int      ps2kbd_get_raw_char(void)    { return -1; }
int      ps2kbd_ctrl_alt_del_pressed(void) { return 0; }

/* ---- PS/2 mouse (drivers/ps2) ---- */

bool ps2_mouse_init_device(void)   { return false; }
void ps2_mouse_poll(void)          { }
bool ps2_mouse_is_initialized(void){ return false; }
bool ps2_mouse_has_wheel(void)     { return false; }

bool ps2_mouse_get_state(int16_t *dx, int16_t *dy, int8_t *wheel,
                         uint8_t *buttons)
{
    (void)dx; (void)dy; (void)wheel; (void)buttons;
    return false;
}

void ps2_mouse_get_errors(uint32_t *frame_err, uint32_t *parity_err,
                          uint32_t *sync_err)
{
    if (frame_err)  *frame_err  = 0;
    if (parity_err) *parity_err = 0;
    if (sync_err)   *sync_err   = 0;
}

void ps2_mouse_get_counters(uint32_t *raw_bytes, uint32_t *packets,
                            uint32_t *ring_drops)
{
    if (raw_bytes)  *raw_bytes  = 0;
    if (packets)    *packets    = 0;
    if (ring_drops) *ring_drops = 0;
}

uint32_t ps2_mouse_pio_fifo_level(void) { return 0; }
