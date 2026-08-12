/* MSU-1 stub for the device build.
 *
 * Upstream's msu1.c streams PCM tracks off a filesystem through libretro's
 * VFS. There is no VFS here and no game in this port's library uses MSU-1,
 * so the real implementation is left out of the build entirely and these
 * no-ops stand in for the handful of entry points memmap/ppu reference.
 *
 * S9xMSU1ROMExists() returning 0 is what keeps the rest of the core from
 * ever routing anything through here.
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "msu1.h"

/* MSU1 itself is defined in globals.cpp. */

void S9xResetMSU1(void)
{
   memset(&MSU1, 0, sizeof(MSU1));
}

void S9xMSU1Init(void)
{
   S9xResetMSU1();
}

void S9xMSU1DeInit(void)
{
}

uint8_t S9xMSU1ROMExists(void)
{
   return 0;
}

void S9xMSU1SetROMPath(const char *rom_path)
{
   (void)rom_path;
}

uint8_t S9xMSU1ReadPort(uint8_t port)
{
   (void)port;
   return 0;
}

void S9xMSU1WritePort(uint8_t port, uint8_t byte)
{
   (void)port;
   (void)byte;
}

void S9xMSU1Mix(int16_t *buffer, size_t sample_count, uint32_t output_rate)
{
   (void)buffer;
   (void)sample_count;
   (void)output_rate;
}

void S9xMSU1PreSaveState(void)
{
}

void S9xMSU1PostLoadState(void)
{
}
