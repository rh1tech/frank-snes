#ifndef PS2KBD_WRAPPER_H
#define PS2KBD_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

// SNES key codes returned by ps2kbd_get_key()
#define SNES_KEY_UP     0x01
#define SNES_KEY_DOWN   0x02
#define SNES_KEY_LEFT   0x03
#define SNES_KEY_RIGHT  0x04
#define SNES_KEY_A      0x05
#define SNES_KEY_B      0x06
#define SNES_KEY_X      0x07
#define SNES_KEY_Y      0x08
#define SNES_KEY_L      0x09
#define SNES_KEY_R      0x0A
#define SNES_KEY_START  0x0B
#define SNES_KEY_SELECT 0x0C
#define SNES_KEY_ESC    0x0D
#define SNES_KEY_F12    0x0E
#define SNES_KEY_F11    0x0F

// Keyboard state bits for ps2kbd_get_state()
#define KBD_STATE_UP     (1 << 0)
#define KBD_STATE_DOWN   (1 << 1)
#define KBD_STATE_LEFT   (1 << 2)
#define KBD_STATE_RIGHT  (1 << 3)
#define KBD_STATE_A      (1 << 4)
#define KBD_STATE_B      (1 << 5)
#define KBD_STATE_X      (1 << 6)
#define KBD_STATE_Y      (1 << 7)
#define KBD_STATE_L      (1 << 8)
#define KBD_STATE_R      (1 << 9)
#define KBD_STATE_START  (1 << 10)
#define KBD_STATE_SELECT (1 << 11)
#define KBD_STATE_ESC    (1 << 12)
#define KBD_STATE_F12    (1 << 13)
#define KBD_STATE_F11    (1 << 14)

void ps2kbd_init(void);
void ps2kbd_tick(void);
int ps2kbd_get_key(int* pressed, unsigned char* key);
uint16_t ps2kbd_get_state(void);  // Get current keyboard state bitmask

/* Pop the next queued raw ASCII character from keyboard input.
 * Returns a-z (plus shifted A-Z), 0-9, space, or '\b' for Backspace.
 * Returns -1 when the queue is empty. Used by the search dialog so
 * users with a real keyboard can type instead of using the OSK. */
int ps2kbd_get_raw_char(void);

/* Returns non-zero while Ctrl+Alt+Del are all currently held.
 * Del is not exposed in the KBD_STATE bitmask; this chord is only used
 * to soft-reset a running ROM. */
int ps2kbd_ctrl_alt_del_pressed(void);

#ifdef __cplusplus
}
#endif

#endif
