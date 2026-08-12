/* Pure-C façade over the snes9x 1.6x core.
 *
 * The 1.43 core in src/snes9x is C, so main.c reaches straight into
 * Settings, IPPU, GFX and Memory. The 1.6x core is C++ - memmap.h alone
 * pulls in <string> - so those headers cannot be included from any of this
 * port's C files. Rather than convert the front end to C++, everything the
 * front end needs crosses here, and only this file's .cpp side sees the
 * core's headers.
 *
 * The API is deliberately narrower than what main.c does today against the
 * 1.43 core. Direct struct access is not reproduced field for field; each
 * entry point below exists because something in main.c genuinely needs it,
 * and the ones that were only reachable through a struct member are named
 * for what they mean instead of where they lived.
 *
 * Audio: s9x16_drain_audio hands back the core's own resampler output. It
 * is a borrowed pointer, valid until the next s9x16_run_frame, and the
 * count is in int16 samples (stereo interleaved), not frames.
 */

#ifndef FRANK_SNES_S9X16_API_H
#define FRANK_SNES_S9X16_API_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- lifecycle ------------------------------------------------------- */

/* Allocates the core's buffers (WRAM/VRAM/tile caches go to PSRAM via the
   port's malloc). False means an allocation failed; nothing else may be
   called after that. */
bool s9x16_init(void);

/* rom points at the whole file image, already in PSRAM. Copies it into the
   core's own storage, so the caller still owns rom afterwards. */
bool s9x16_load_rom(const uint8_t *rom, size_t size, const char *name);

/* The core's ROM buffer, so a loader can read a file straight into it and
   avoid holding two ROM-sized allocations in PSRAM at once. Read the image
   in, then call s9x16_load_rom_inplace with the number of bytes read. */
/* Allocate the core's ROM buffer for a ROM of this many bytes. Must be
   called before s9x16_rom_storage(); the core no longer reserves a
   worst-case buffer up front. */
bool s9x16_alloc_rom(uint32_t rom_bytes);

uint8_t *s9x16_rom_storage(uint32_t *capacity);
bool     s9x16_load_rom_inplace(size_t size, const char *name);

void s9x16_reset(void);
void s9x16_soft_reset(void);

/* --- per frame ------------------------------------------------------- */

void s9x16_run_frame(void);

/* Skip the PPU for this frame. The CPU and APU still run, so audio is
   unaffected - this is the frameskip lever, not a pause. */
void s9x16_set_render(bool render);

uint32_t s9x16_frame_count(void);

/* --- audio ----------------------------------------------------------- */

/* Borrowed pointer into the core's resampler output; *samples is a count of
   int16 values (stereo interleaved). Returns NULL with *samples 0 when the
   core produced nothing this frame - that is a real answer, not an error,
   and the caller must not treat it as silence to be invented. */
const int16_t *s9x16_drain_audio(int *samples);

/* The rate the S-DSP emits at. This core does not resample - it hands over
   native 32040 Hz output - so the caller cannot ask for a different rate,
   only find out what it is. That happens to be exactly the port's I2S rate,
   which is why the elastic FIFO in main.c has so little work to do here. */
uint32_t s9x16_audio_rate(void);

/* --- video ----------------------------------------------------------- */

/* 16-bit RGB565 framebuffer owned by the core. pitch is in pixels. */
uint16_t *s9x16_screen(uint32_t *pitch_pixels, uint32_t *width, uint32_t *height);

/* --- input ----------------------------------------------------------- */

/* Button mask in the core's own bit order for one of the five ports. */
void s9x16_set_joypad(int port, uint32_t buttons);

/* SNES Mouse on port 0 or 1, or off. 1.6x has no Settings.Mouse flag - the
   controller type is what decides, so this reconfigures the port. */
void s9x16_set_mouse(bool enabled, int port);

void s9x16_set_mute(bool mute);

/* --- region and timing ----------------------------------------------- */

bool     s9x16_is_pal(void);
uint32_t s9x16_frame_time_us(void);

/* --- ROM and save data ------------------------------------------------ */

const char *s9x16_rom_name(void);
uint8_t    *s9x16_sram(uint32_t *size);
uint32_t    s9x16_rom_size(void);

#ifdef __cplusplus
}
#endif

#endif /* FRANK_SNES_S9X16_API_H */
