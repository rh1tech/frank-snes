#pragma once

#include <stdint.h>
#include <stdbool.h>

// NES/SNES button bit masks
#define DPAD_LEFT   0x001000
#define DPAD_RIGHT  0x004000
#define DPAD_DOWN   0x000400
#define DPAD_UP     0x000100
#define DPAD_START  0x000040
#define DPAD_SELECT 0x000010
#define DPAD_B      0x000004   // Y on SNES
#define DPAD_A      0x000001   // B on SNES

#define DPAD_Y      0x010000   // A on SNES
#define DPAD_X      0x040000
#define DPAD_LT     0x100000
#define DPAD_RT     0x400000

extern uint32_t nespad_state;  // (S)NES Joystick1
extern uint32_t nespad_state2; // (S)NES Joystick2

/* Sticky per-port controller-type latches.
 * NES pads only drive bits 0-7 of the shift register (A, B, Select, Start,
 * D-pad). SNES pads additionally drive bits 8-11 (A, X, L, R). We latch
 * `true` the first time any SNES-only bit appears, which stays set until
 * power cycle and is used to pick the right button map at the call site.
 * Starts `false` (NES assumed) so an NES-only pad works before it has had
 * a chance to emit any SNES-only bit. */
extern bool nespad_is_snes;    // port 1
extern bool nespad2_is_snes;   // port 2

/* Mask of SNES-only shift-register bits (SNES A/X/L/R). */
#define NESPAD_SNES_ONLY_MASK (DPAD_Y | DPAD_X | DPAD_LT | DPAD_RT)

extern bool nespad_begin(uint32_t cpu_khz, uint8_t clkPin, uint8_t dataPin,
                         uint8_t latPin);

extern void nespad_read();
