/* Port-side globals and entry points the 1.43 core used to provide.
 *
 * These are not part of upstream snes9x on either side - they are this
 * port's own, and they lived in src/snes9x/ppu.c and apu.c. The 1.6x core
 * does not build those files, so they live here instead.
 */

#include "snes9x.h"
#include "ppu.h"

extern "C" {

/* Set whenever CGRAM changes. main.c pushes the new palette to the HDMI
 * driver on the next frame boundary. With FRANK_SNES_INDEXED_SCREEN the
 * framebuffer holds CGRAM indices, so this flag is the only thing that
 * carries actual colour to the display. */
volatile bool g_palette_needs_update = false;

/* Diagnostic frame counter for the DSP log build. */
volatile uint32_t dsp_log_frame = 0;

/* Save states are not wired to this core yet.
 *
 * 1.6x's snapshot layer is S9xFreezeGameMem/S9xUnfreezeGameMem - a whole
 * different shape from the 1.43 FIL* API the front end calls, and it wants
 * a state-sized staging buffer in PSRAM. Returning false means "no state
 * was written/read", which the caller already handles; inventing a success
 * here would silently produce save files that cannot be loaded back. */
bool S9xSaveState(void *fp)
{
   (void)fp;
   return false;
}

bool S9xLoadState(void *fp)
{
   (void)fp;
   return false;
}

}  /* extern "C" */

/* --------------------------------------------------------------------- */
/* Coprocessors left out of this build. memmap.cpp wires these up
 * unconditionally, so the symbols must exist; a game that needs one will
 * read open bus rather than silently get plausible-looking rubbish. */

#if SNES9X_NO_DSP3
extern "C" {
struct SDSP3_stub { int unused; };
uint8_t DSP3GetByte(uint16_t address) { (void)address; return 0; }
void    DSP3SetByte(uint8_t byte, uint16_t address) { (void)byte; (void)address; }
}
#endif
