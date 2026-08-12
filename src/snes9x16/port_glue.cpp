/* Port glue for the snes9x 1.6x core on FRANK SNES.
 *
 * Upstream expects the front end to supply a handful of callbacks. On a
 * desktop front end these do real work - open windows, write files, print
 * to a console. This device has none of those, so the honest implementation
 * of most of them is "do nothing", and saying so once here is better than
 * scattering #ifdefs through the core.
 *
 * Split into three groups:
 *
 *   1. Display   - S9xInitUpdate/ContinueUpdate/DeinitUpdate. The core calls
 *                  these around each rendered frame. This port draws straight
 *                  out of GFX.Screen after S9xMainLoop returns, so there is
 *                  nothing to do per frame. Present for the ABI only.
 *   2. Pacing    - S9xSyncSpeed. Frame pacing on this device is owned by the
 *                  audio path (main.c holds the deadline PLL against the I2S
 *                  clock); the core must not also try to sleep.
 *   3. Disabled  - cheats, path resolution, zip loading, SRAM autosave.
 *                  Nothing in this build reaches them; they exist because
 *                  memmap.cpp and snapshot.cpp reference them unconditionally.
 *
 * Every function here that returns a value returns the "nothing happened"
 * answer, never a fabricated success that would let a caller proceed on a
 * false premise.
 */

#include <string>

#include "snes9x.h"
#include "memmap.h"
#include "display.h"

/* ---------------------------------------------------------------- display */

bool8 S9xInitUpdate(void)
{
   return TRUE;
}

bool8 S9xContinueUpdate(int width, int height)
{
   (void)width;
   (void)height;
   return TRUE;
}

bool8 S9xDeinitUpdate(int width, int height)
{
   (void)width;
   (void)height;
   return TRUE;
}

/* ----------------------------------------------------------------- pacing */

void S9xSyncSpeed(void)
{
   /* Deliberately empty. main.c paces frames against the I2S sample clock;
      a second regulator here would fight it. */
}

/* --------------------------------------------------------------- messages */

void S9xMessage(int type, int number, const char *message)
{
   (void)type;
   (void)number;
   (void)message;
}

/* ---------------------------------------------------------------- unbuilt */

/* S9xGetCrosshair and both S9xGetFilename overloads are NOT here:
   crosshairs.cpp and fscompat.cpp are in this build and already define
   them. S9xGetDirectory is the front end's job, so it is. */

/* There is no writable filesystem behind the core on this device - saves
   and SRAM go through the front end's own SD path. An empty directory is
   the truthful answer, and every caller here is in a code path this build
   does not reach (BS-X satellite data, cheat files). */
std::string S9xGetDirectory(enum s9x_getdirtype dirtype)
{
   (void)dirtype;
   return std::string();
}

void S9xAutoSaveSRAM(void)
{
   /* SRAM is flushed by the front end on its own schedule, not by the core. */
}

void S9xInitCheatData(void) {}
void S9xDeleteCheats(void) {}
void S9xUpdateCheatsInMemory(void) {}

bool8 S9xLoadCheatFile(const std::string &filename)
{
   (void)filename;
   return FALSE;
}

/* ROMs arrive as a flat buffer from the SD card reader, never as a zip. */
bool8 LoadZip(const char *zipname, uint32 *headers, uint8 *buffer)
{
   (void)zipname;
   (void)headers;
   (void)buffer;
   return FALSE;
}
