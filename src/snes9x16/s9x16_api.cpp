/* C façade over the snes9x 1.6x core - implementation.
 *
 * This is the only file in the port that sees both the core's C++ headers
 * and the front end. See s9x16_api.h for why the boundary exists.
 *
 * The init order below is the one 1.6x's own libretro front end uses
 * (Memory.Init -> S9xInitAPU -> S9xInitSound -> S9xGraphicsInit ->
 * controls), and it is order-sensitive: S9xGraphicsInit reads geometry the
 * memory map has already established, and the APU must exist before the
 * first S9xReset.
 */

#include "snes9x.h"
#include "memmap.h"
#include "apu/apu.h"
#include "gfx.h"
#include "ppu.h"
#include "controls.h"
#include "sa1.h"
#include "tile.h"
#include "apu/bapu/snes/snes.hpp"
#include "display.h"

#include "s9xbridge.h"

#include "s9x16_api.h"

/* After every standard header: the C++ runtime declares malloc itself, and
   redefining it earlier makes <cstdlib> fail to compile. Same reason
   memmap.cpp puts this line where it does. */
#include "../snes_alloc.h"
#define malloc snes_malloc

static bool s_initialised = false;

bool s9x16_init(void)
{
   /* Deliberately NOT idempotent. The front end calls psram_restore_session()
      when it returns to the ROM selector, which rolls the PSRAM allocator
      back past everything Memory.Init() allocated. Returning early on a
      second launch would leave every pointer in the core dangling. */

   /* Settings is zeroed first, so every field below has to be one the core
      genuinely needs non-zero. The cycle counts and the sprite-tile limit
      are the SNES's real numbers - they are what makes this core charge
      cycles correctly, which is the entire reason for the port. Leaving
      them at zero does not merely mistime the machine, it stops it. */
   memset(&Settings, 0, sizeof(Settings));

   Settings.MouseMaster            = FALSE;
   Settings.SuperScopeMaster       = FALSE;
   Settings.JustifierMaster        = FALSE;
   Settings.MultiPlayer5Master     = FALSE;
   Settings.MacsRifleMaster        = FALSE;

   Settings.FrameTimePAL           = 20000;
   Settings.FrameTimeNTSC          = 16667;

   Settings.Stereo                 = TRUE;
   Settings.Mute                   = FALSE;
   Settings.InterpolationMethod    = DSP_INTERPOLATION_GAUSSIAN;
   for (int i = 0; i < 9; i++)
      Settings.ChannelsVolumePercent[i] = 100;

   Settings.Transparency           = TRUE;
   Settings.BlockInvalidVRAMAccess = TRUE;
   Settings.HDMATimingHack         = 100;
   Settings.MaxSpriteTilesPerLine  = 34;
   Settings.SuperFXClockMultiplier = 100;

   /* Master-cycle costs of a fast access, a slow access and a long access.
      Upstream's defaults for accurate timing. */
   Settings.OneClockCycle          = 6;
   Settings.OneSlowClockCycle      = 8;
   Settings.TwoClockCycles         = 12;

   Settings.InitialInfoStringTimeout = 0;
   Settings.UpAndDown              = FALSE;
   Settings.DisableGameSpecificHacks = FALSE;

   /* APU RAM and Mode 7's caches are lazily allocated from PSRAM and cached
      in statics; after psram_restore_session() those pointers are stale. */
   SNES::smp.apuram = NULL;
   S9xForgetMode7Cache();

#ifdef FRANK_SNES_CPU_CORE_S9X16
   /* SA-1's 32 KB of map, in PSRAM. Allocated unconditionally because
      memmap.cpp writes it while building the map for every cartridge, not
      only for SA-1 ones. */
   SA1.Map      = (uint8 **) malloc(MEMMAP_NUM_BLOCKS * sizeof(uint8 *));
   SA1.WriteMap = (uint8 **) malloc(MEMMAP_NUM_BLOCKS * sizeof(uint8 *));
   if (!SA1.Map || !SA1.WriteMap)
      return false;
#endif

   if (!Memory.Init() || !S9xInitAPU())
      return false;

   if (!S9xInitSound(0))
      return false;

   S9xSetSoundMute(FALSE);
   S9xGraphicsInit();

   S9xUnmapAllControls();
   for (int port = 0; port < 2; port++)
      S9xSetController(port, CTL_JOYPAD, port, 0, 0, 0);

   s_initialised = true;
   return true;
}

bool s9x16_load_rom(const uint8_t *rom, size_t size, const char *name)
{
   if (!s_initialised || !rom || !size)
      return false;

   if (!Memory.LoadROMMem(rom, (uint32)size, name))
      return false;

   /* LoadROMMem has just set Settings.PAL from the ROM's destination code,
      so the frame time is only correct after it, never before. */
   Settings.FrameTime = Settings.PAL ? Settings.FrameTimePAL
                                     : Settings.FrameTimeNTSC;
   return true;
}

bool s9x16_alloc_rom(uint32_t rom_bytes)
{
   if (!s_initialised || !rom_bytes || rom_bytes > (uint32_t)CMemory::MAX_ROM_SIZE)
      return false;

   /* 0x8000 of headroom in front is FillRAM, and stops SuperFX code from
      walking off the start of the image; 0x200 behind covers a copier
      header. Sized to this ROM rather than to MAX_ROM_SIZE, because the
      difference is megabytes of PSRAM the tile caches and framebuffers
      need. */
   const uint32_t want = 0x8000u + rom_bytes + 0x200u;

   if (Memory.ROMStorage && Memory.ROMStorageSize >= want)
   {
      memset(Memory.ROMStorage, 0, Memory.ROMStorageSize);
   }
   else
   {
      if (Memory.ROMStorage)
         free(Memory.ROMStorage);

      Memory.ROMStorage = (uint8_t *) malloc(want);
      if (!Memory.ROMStorage)
      {
         Memory.ROMStorageSize = 0;
         return false;
      }
      Memory.ROMStorageSize = want;
      memset(Memory.ROMStorage, 0, want);
   }

   Memory.FillRAM = &Memory.ROMStorage[0];
   Memory.ROM     = &Memory.ROMStorage[0x8000];

   /* Init() derived all of these from ROM before the buffer existed, so
      they are re-derived now. There are fourteen of them - doing it by
      hand is how BSXROMBase stayed null and faulted on ROM[0xFFDA]. */
   Memory.RebindROMPointers();

   return true;
}

uint8_t *s9x16_rom_storage(uint32_t *capacity)
{
   if (capacity)
      *capacity = Memory.ROMStorageSize > 0x8200u
                     ? (Memory.ROMStorageSize - 0x8200u) : 0u;

   return Memory.ROM;
}

bool s9x16_load_rom_inplace(size_t size, const char *name)
{
   if (!s_initialised || !size || size > (size_t)CMemory::MAX_ROM_SIZE)
      return false;

   /* LoadROMMem would memset ROM and then memcpy the image into it, which
      cannot work when the caller has already read the file straight into
      that buffer. Doing it in place is not an optimisation here: the front
      end's own copy plus ROMStorage would be two ROM-sized allocations in
      8 MB of PSRAM. */
   Memory.ROMFilename = name ? name : "MemoryROM";
   memset(&Multi, 0, sizeof(Multi));

   if (!Memory.LoadROMInt((int32)size))
      return false;

   Settings.FrameTime = Settings.PAL ? Settings.FrameTimePAL
                                     : Settings.FrameTimeNTSC;
   return true;
}

void s9x16_reset(void)
{
   S9xReset();
}

void s9x16_soft_reset(void)
{
   S9xSoftReset();
}

void s9x16_run_frame(void)
{
   S9xMainLoop();
}

void s9x16_set_render(bool render)
{
   IPPU.RenderThisFrame = render ? TRUE : FALSE;
}

uint32_t s9x16_frame_count(void)
{
   return (uint32_t)ICPU.Frame;
}

const int16_t *s9x16_drain_audio(int *samples)
{
   int count = 0;
   const int16 *src = S9xDrainAudio(&count);

   if (samples)
      *samples = count;

   return (count > 0) ? (const int16_t *)src : NULL;
}

uint32_t s9x16_audio_rate(void)
{
   return (uint32_t)S9xGetAudioSampleRate();
}

uint16_t *s9x16_screen(uint32_t *pitch_pixels, uint32_t *width, uint32_t *height)
{
   if (pitch_pixels)
      *pitch_pixels = GFX.Pitch >> 1;
   if (width)
      *width = IPPU.RenderedScreenWidth;
   if (height)
      *height = IPPU.RenderedScreenHeight;

   return (uint16_t *)GFX.Screen;
}

void s9x16_set_joypad(int port, uint32_t buttons)
{
   S9xSetJoypadButtons(port, (uint16)buttons);
}

void s9x16_set_mouse(bool enabled, int port)
{
   if (port < 0 || port > 1)
      return;

   Settings.MouseMaster = enabled ? TRUE : FALSE;

   /* Reassert both ports: turning the mouse off has to put a joypad back,
      not just clear a flag, or the port keeps answering as a mouse. */
   for (int p = 0; p < 2; p++)
   {
      if (enabled && p == port)
         S9xSetController(p, CTL_MOUSE, p, 0, 0, 0);
      else
         S9xSetController(p, CTL_JOYPAD, p, 0, 0, 0);
   }

   S9xVerifyControllers();
}

void s9x16_set_mute(bool mute)
{
   S9xSetSoundMute(mute ? TRUE : FALSE);
}

bool s9x16_is_pal(void)
{
   return Settings.PAL ? true : false;
}

uint32_t s9x16_frame_time_us(void)
{
   return (uint32_t)Settings.FrameTime;
}

const char *s9x16_rom_name(void)
{
   return Memory.ROMName;
}

uint8_t *s9x16_sram(uint32_t *size)
{
   if (size)
      *size = (uint32_t)Memory.SRAMStorageSize;

   return Memory.SRAM;
}

uint32_t s9x16_rom_size(void)
{
   return (uint32_t)Memory.CalculatedSize;
}
