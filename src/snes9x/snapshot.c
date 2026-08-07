/* This file is part of Snes9x. See LICENSE file. */

#include "snes9x.h"
#include "memmap.h"
#include "ppu.h"
#include "cpuexec.h"
#include "apu.h"
#include "dma.h"
#include "display.h"
#include "srtc.h"
#include "soundux.h"

#include "ff.h"
#include <stdio.h>
#include <string.h>

static const char header[16] = "SNES9X_000000002";

/*
 * PSRAM-safe write: Memory.VRAM/RAM/SRAM/FillRAM and IAPU.RAM live in PSRAM
 * (XIP-mapped). The SD card's PIO SPI uses DMA which cannot read from PSRAM
 * XIP space, causing hard faults. We copy through a small SRAM bounce buffer.
 */
#define BOUNCE_SIZE 512

static bool write_chunk(FIL *fp, const void *data, UINT size) {
   uint8_t bounce[BOUNCE_SIZE];
   const uint8_t *src = (const uint8_t *)data;
   UINT remaining = size;

   while (remaining > 0) {
      UINT chunk = (remaining > BOUNCE_SIZE) ? BOUNCE_SIZE : remaining;
      memcpy(bounce, src, chunk);
      UINT bw;
      FRESULT fr = f_write(fp, bounce, chunk, &bw);
      if (fr != FR_OK || bw != chunk)
         return false;
      src += chunk;
      remaining -= chunk;
   }
   return true;
}

static bool read_chunk(FIL *fp, void *data, UINT size) {
   uint8_t bounce[BOUNCE_SIZE];
   uint8_t *dst = (uint8_t *)data;
   UINT remaining = size;

   while (remaining > 0) {
      UINT chunk = (remaining > BOUNCE_SIZE) ? BOUNCE_SIZE : remaining;
      UINT br;
      FRESULT fr = f_read(fp, bounce, chunk, &br);
      if (fr != FR_OK || br != chunk)
         return false;
      memcpy(dst, bounce, chunk);
      dst += chunk;
      remaining -= chunk;
   }
   return true;
}

bool S9xSaveState(FIL *fp)
{
   int chunks = 0;

   chunks += write_chunk(fp, header, sizeof(header));
   chunks += write_chunk(fp, &CPU, sizeof(CPU));
   chunks += write_chunk(fp, &ICPU, sizeof(ICPU));
   chunks += write_chunk(fp, &PPU, sizeof(PPU));
   chunks += write_chunk(fp, &DMA, sizeof(DMA));
   chunks += write_chunk(fp, Memory.VRAM, VRAM_SIZE);
   chunks += write_chunk(fp, Memory.RAM, RAM_SIZE);
   chunks += write_chunk(fp, Memory.SRAM, SRAM_SIZE);
   chunks += write_chunk(fp, Memory.FillRAM, FILLRAM_SIZE);
#ifdef USE_BLARGG_APU
   /* The accurate core keeps the SPC700, its RAM and the DSP in two
    * self-contained blocks with no internal pointers, so it saves as
    * two chunks where the legacy core needed three plus a fix-up pass.
    * Kept at three chunks so the chunk count below is unchanged. */
   chunks += write_chunk(fp, S9xAPUStateBlock(), S9xAPUStateSize());
   chunks += write_chunk(fp, spc_apuram(), 0x10000);
   chunks += write_chunk(fp, spc_dsp_state(), spc_dsp_state_size());
   chunks += 1;   /* the legacy build's fourth chunk */
#else
   chunks += write_chunk(fp, &APU, sizeof(APU));
   chunks += write_chunk(fp, &IAPU, sizeof(IAPU));
   chunks += write_chunk(fp, IAPU.RAM, 0x10000);
   chunks += write_chunk(fp, &SoundData, sizeof(SoundData));
#endif

   printf("Saved chunks = %d\n", chunks);

   return chunks == 13;
}

bool S9xLoadState(FIL *fp)
{
   uint8_t buf[16];
   int chunks = 0;

   if (!read_chunk(fp, buf, 16) || memcmp(header, buf, sizeof(header)) != 0)
   {
      printf("Wrong header found\n");
      return false;
   }

   /* At this point we can't go back and a failure will corrupt the state anyway */
   S9xReset();

#ifndef USE_BLARGG_APU
   uint8_t *IAPU_RAM = IAPU.RAM;
#endif

   chunks += read_chunk(fp, &CPU, sizeof(CPU));
   chunks += read_chunk(fp, &ICPU, sizeof(ICPU));
   chunks += read_chunk(fp, &PPU, sizeof(PPU));
   chunks += read_chunk(fp, &DMA, sizeof(DMA));
   chunks += read_chunk(fp, Memory.VRAM, VRAM_SIZE);
   chunks += read_chunk(fp, Memory.RAM, RAM_SIZE);
   chunks += read_chunk(fp, Memory.SRAM, SRAM_SIZE);
   chunks += read_chunk(fp, Memory.FillRAM, FILLRAM_SIZE);
#ifdef USE_BLARGG_APU
   chunks += read_chunk(fp, S9xAPUStateBlock(), S9xAPUStateSize());
   chunks += read_chunk(fp, spc_apuram(), 0x10000);
   chunks += read_chunk(fp, spc_dsp_state(), spc_dsp_state_size());
   chunks += 1;
#else
   chunks += read_chunk(fp, &APU, sizeof(APU));
   chunks += read_chunk(fp, &IAPU, sizeof(IAPU));
   chunks += read_chunk(fp, IAPU.RAM, 0x10000);
   chunks += read_chunk(fp, &SoundData, sizeof(SoundData));
#endif

   printf("Loaded chunks = %d\n", chunks);

   /* Fixing up registers and pointers: */

#ifndef USE_BLARGG_APU
   IAPU.PC = IAPU.PC - IAPU.RAM + IAPU_RAM;
   IAPU.DirectPage = IAPU.DirectPage - IAPU.RAM + IAPU_RAM;
   IAPU.WaitAddress1 = IAPU.WaitAddress1 - IAPU.RAM + IAPU_RAM;
   IAPU.WaitAddress2 = IAPU.WaitAddress2 - IAPU.RAM + IAPU_RAM;
   IAPU.RAM = IAPU_RAM;
#endif

   FixROMSpeed();
   IPPU.ColorsChanged = true;
   IPPU.OBJChanged = true;
   CPU.InDMA = false;
   S9xFixColourBrightness();
#ifndef USE_BLARGG_APU
   S9xAPUUnpackStatus();
#endif
   S9xFixSoundAfterSnapshotLoad();
   ICPU.ShiftedPB = ICPU.Registers.PB << 16;
   ICPU.ShiftedDB = ICPU.Registers.DB << 16;
   S9xSetPCBase(ICPU.ShiftedPB + ICPU.Registers.PC);
   S9xUnpackStatus();
   S9xFixCycles();
   S9xReschedule();

   return true;
}
