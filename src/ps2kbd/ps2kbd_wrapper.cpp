// PS/2 Keyboard Wrapper for SNES
// SPDX-License-Identifier: GPL-2.0-or-later

#include "board_config.h"
#include "ps2kbd_wrapper.h"
#include "ps2kbd_mrmltr.h"
#include "ps2/ps2.h"

// Simple ring buffer for key events (avoids std::queue to save RAM)
#define EVENT_QUEUE_SIZE 16

struct KeyEvent {
    uint8_t pressed;
    uint8_t key;
};

static KeyEvent event_queue[EVENT_QUEUE_SIZE];
static volatile uint8_t queue_head = 0;  // Next write position
static volatile uint8_t queue_tail = 0;  // Next read position

static inline bool queue_empty(void) {
    return queue_head == queue_tail;
}

static inline bool queue_full(void) {
    return ((queue_head + 1) & (EVENT_QUEUE_SIZE - 1)) == queue_tail;
}

static void queue_push(uint8_t pressed, uint8_t key) {
    if (!queue_full()) {
        event_queue[queue_head].pressed = pressed;
        event_queue[queue_head].key = key;
        queue_head = (queue_head + 1) & (EVENT_QUEUE_SIZE - 1);
    }
}

static bool queue_pop(uint8_t* pressed, uint8_t* key) {
    if (queue_empty()) return false;
    *pressed = event_queue[queue_tail].pressed;
    *key = event_queue[queue_tail].key;
    queue_tail = (queue_tail + 1) & (EVENT_QUEUE_SIZE - 1);
    return true;
}

/* Raw character ring buffer for text input (search dialog typing) */
#define RAW_CHAR_QUEUE_SIZE 16
static uint8_t raw_char_queue[RAW_CHAR_QUEUE_SIZE];
static volatile uint8_t raw_char_head = 0;
static volatile uint8_t raw_char_tail = 0;

static void raw_char_push(uint8_t ch) {
    uint8_t next = (raw_char_head + 1) & (RAW_CHAR_QUEUE_SIZE - 1);
    if (next != raw_char_tail) {
        raw_char_queue[raw_char_head] = ch;
        raw_char_head = next;
    }
}

static int raw_char_pop(void) {
    if (raw_char_head == raw_char_tail) return -1;
    uint8_t ch = raw_char_queue[raw_char_tail];
    raw_char_tail = (raw_char_tail + 1) & (RAW_CHAR_QUEUE_SIZE - 1);
    return ch;
}

/* Convert HID keycode + modifier into a printable ASCII character, or 0
 * for keys that don't represent text input. Backspace (0x2A) is folded
 * into '\b' so the search dialog can handle it uniformly. */
static uint8_t hid_to_ascii(uint8_t code, uint8_t modifier) {
    bool shift = (modifier & 0x22) != 0;
    if (code >= 0x04 && code <= 0x1D)
        return shift ? ('A' + (code - 0x04)) : ('a' + (code - 0x04));
    if (code >= 0x1E && code <= 0x26)
        return '1' + (code - 0x1E);
    if (code == 0x27) return '0';
    if (code == 0x2C) return ' ';
    if (code == 0x2A) return '\b';
    return 0;
}

// HID to SNES key mapping
// Key mapping:
//   Arrow keys -> D-pad (Up/Down/Left/Right)
//   X, Z       -> SNES A, B buttons
//   S, A       -> SNES X, Y buttons
//   Q, W       -> SNES L, R buttons (shoulder)
//   Enter      -> Start
//   Space      -> Select
//   ESC        -> Settings menu
// Returns 0 if no mapping
static unsigned char hid_to_snes(uint8_t code) {
    switch (code) {
        // Arrow keys -> D-pad
        case 0x52: return SNES_KEY_UP;     // Up arrow
        case 0x51: return SNES_KEY_DOWN;   // Down arrow
        case 0x50: return SNES_KEY_LEFT;   // Left arrow
        case 0x4F: return SNES_KEY_RIGHT;  // Right arrow

        // X, Z -> SNES A, B (right side face buttons)
        case 0x1B: return SNES_KEY_A;      // X key -> A
        case 0x1D: return SNES_KEY_B;      // Z key -> B

        // S, A -> SNES X, Y (left side face buttons)
        case 0x16: return SNES_KEY_X;      // S key -> X
        case 0x04: return SNES_KEY_Y;      // A key -> Y

        // Q, W -> SNES L, R (shoulder buttons)
        case 0x14: return SNES_KEY_L;      // Q key -> L
        case 0x1A: return SNES_KEY_R;      // W key -> R

        // Start = Enter or Keypad Enter
        case 0x28: return SNES_KEY_START;  // Enter
        case 0x58: return SNES_KEY_START;  // Keypad Enter

        // Select = Space
        case 0x2C: return SNES_KEY_SELECT; // Space

        // ESC = Settings menu / Back
        case 0x29: return SNES_KEY_ESC;    // Escape

        // F12 = Settings menu (alternative)
        case 0x45: return SNES_KEY_F12;    // F12

        // F11 = back to ROM selector during gameplay
        case 0x44: return SNES_KEY_F11;    // F11

        // Tab = toggle carousel / file browser
        case 0x2B: return SNES_KEY_TAB;    // Tab

        default: return 0;
    }
}

/* Raw modifier byte + Del key tracking for the Ctrl+Alt+Del chord.
 * Kept outside the KBD_STATE_ bitmask because Del has no dedicated bit
 * and the chord is only consulted from the gameplay loop. */
static volatile uint8_t  g_kbd_modifier  = 0;
static volatile bool     g_kbd_del_held  = false;

static void key_handler(hid_keyboard_report_t *curr, hid_keyboard_report_t *prev) {
    /* Mirror modifier byte + Del state for Ctrl+Alt+Del detection. */
    g_kbd_modifier = curr->modifier;
    bool del_held = false;
    for (int i = 0; i < 6; i++) {
        if (curr->keycode[i] == 0x4C) { del_held = true; break; }
    }
    g_kbd_del_held = del_held;

    // Check keys - new key presses
    for (int i = 0; i < 6; i++) {
        if (curr->keycode[i] != 0) {
            bool found = false;
            for (int j = 0; j < 6; j++) {
                if (prev->keycode[j] == curr->keycode[i]) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                unsigned char k = hid_to_snes(curr->keycode[i]);
                if (k) queue_push(1, k);
                uint8_t ch = hid_to_ascii(curr->keycode[i], curr->modifier);
                if (ch) raw_char_push(ch);
            }
        }
    }

    // Check keys - key releases
    for (int i = 0; i < 6; i++) {
        if (prev->keycode[i] != 0) {
            bool found = false;
            for (int j = 0; j < 6; j++) {
                if (curr->keycode[j] == prev->keycode[i]) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                unsigned char k = hid_to_snes(prev->keycode[i]);
                if (k) queue_push(0, k);
            }
        }
    }
}

static Ps2Kbd_Mrmltr* kbd = nullptr;
static volatile uint16_t g_kbd_state = 0;  // Global keyboard state bitmask

// Convert key code to state bit
static uint16_t key_to_state_bit(uint8_t key) {
    switch (key) {
        case SNES_KEY_UP:     return KBD_STATE_UP;
        case SNES_KEY_DOWN:   return KBD_STATE_DOWN;
        case SNES_KEY_LEFT:   return KBD_STATE_LEFT;
        case SNES_KEY_RIGHT:  return KBD_STATE_RIGHT;
        case SNES_KEY_A:      return KBD_STATE_A;
        case SNES_KEY_B:      return KBD_STATE_B;
        case SNES_KEY_X:      return KBD_STATE_X;
        case SNES_KEY_Y:      return KBD_STATE_Y;
        case SNES_KEY_L:      return KBD_STATE_L;
        case SNES_KEY_R:      return KBD_STATE_R;
        case SNES_KEY_START:  return KBD_STATE_START;
        case SNES_KEY_SELECT: return KBD_STATE_SELECT;
        case SNES_KEY_ESC:    return KBD_STATE_ESC;
        case SNES_KEY_F12:   return KBD_STATE_F12;
        case SNES_KEY_F11:   return KBD_STATE_F11;
        case SNES_KEY_TAB:   return KBD_STATE_TAB;
        default: return 0;
    }
}

extern "C" void ps2kbd_init(void) {
    // Topology matches frank-wolf (proven working with this mouse):
    //   pio0 = HDMI
    //   pio1 = audio + nespad + PS/2 mouse
    //   pio2 = PS/2 keyboard
    ps2_mouse_pio_init(pio1, PS2_MOUSE_CLK);
    ps2_kbd_pio_init(pio2, PS2_PIN_CLK);
    kbd = new Ps2Kbd_Mrmltr(key_handler);
    g_kbd_state = 0;
}

extern "C" void ps2kbd_tick(void) {
    if (kbd) kbd->tick();

    // Update global state from queued events (peek, don't consume)
    // Process all events and update g_kbd_state
    uint8_t p, k;
    while (queue_pop(&p, &k)) {
        uint16_t bit = key_to_state_bit(k);
        if (bit) {
            if (p) {
                g_kbd_state |= bit;
            } else {
                g_kbd_state &= ~bit;
            }
        }
    }
}

extern "C" int ps2kbd_get_key(int* pressed, unsigned char* key) {
    // This function is now deprecated - use ps2kbd_get_state() instead
    // But keep it for compatibility - always returns 0 since we consume in tick()
    (void)pressed;
    (void)key;
    return 0;
}

extern "C" uint16_t ps2kbd_get_state(void) {
    return g_kbd_state;
}

extern "C" int ps2kbd_get_raw_char(void) {
    return raw_char_pop();
}

extern "C" int ps2kbd_ctrl_alt_del_pressed(void) {
    /* tinyusb modifier masks:
     *   LEFTCTRL=0x01, RIGHTCTRL=0x10, LEFTALT=0x04, RIGHTALT=0x40. */
    uint8_t m = g_kbd_modifier;
    bool ctrl = (m & 0x11) != 0;
    bool alt  = (m & 0x44) != 0;
    return (ctrl && alt && g_kbd_del_held) ? 1 : 0;
}
