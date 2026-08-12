/* This file is part of Snes9x. See LICENSE file. */

#include "snes9x.h"
#include "memmap.h"
#include "ppu.h"
#include "dsp.h"
#include "cpuexec.h"
#include "apu.h"
#include "dma.h"
#include "srtc.h"
#include "obc1.h"
#include "fxemu.h"

extern FxInit_s SuperFX;

static void S9xResetSuperFX(void)
{
   FxReset(&SuperFX);
}

/* Populate the scanline geometry from the loaded ROM's region and 5A22
 * revision. Called from S9xInitROM, after the per-game H_Max stretches. */
void S9xInitTimings(void)
{
   Timings.H_Max_Master = Settings.H_Max;
   Timings.H_Max        = Timings.H_Max_Master;
   Timings.V_Max_Master = Settings.PAL ? SNES_MAX_PAL_VCOUNTER : SNES_MAX_NTSC_VCOUNTER;
   Timings.V_Max        = Timings.V_Max_Master;

   /* H=274, a fixed dot position - not 256 dots scaled by H_Max.
    * Settings.HBlankStart came out of (256 * H_Max) / 341 = 1024, so the
    * HBlank flag in $4212 was raised 72 cycles early on every scanline and
    * the HBLANK_START event fired 18 dots ahead of the hardware. */
   Timings.HBlankStart  = SNES_HBLANK_START_HC;
   Timings.HBlankEnd    = SNES_HBLANK_END_HC;
   Timings.HDMAInit     = SNES_HDMA_INIT_HC;
   Timings.HDMAStart    = SNES_HDMA_START_HC;
   Timings.RenderPos    = SNES_RENDER_START_HC;

   /* Revision 2 of the 5A22 alternates the refresh position between two dots.
      SNES_5A22 here is the version nibble reported through $4210, not the
      revision, so there is nothing to branch on: assume rev 2, which is what
      almost every console is and what this was validated against offline. */
   Timings.WRAMRefreshPos = SNES_WRAM_REFRESH_HC_v2;

   Timings.DMACPUSync       = 18;
   Timings.IRQTriggerCycles = 14;
   Timings.NextIRQTimer     = 0x0fffffff;
   Timings.InterlaceField   = false;
   S9xVTimerPosition        = 0;
}

void S9xResetCPU()
{
   /* The event state must be valid before the first bus access below.
    *
    * Scanline 0 enters the ring at its FIRST event. Entering at HBLANK_START
    * skips HDMA_INIT, RENDER and WRAM_REFRESH on line 0, so the line loses its
    * 40-cycle refresh stall - and CPU.Cycles never recovers it. */
   CPU.InDMA = false;
   CPU.WhichEvent = HC_RENDER_EVENT;
   CPU.Cycles = 182;
   CPU.NextEvent = Timings.RenderPos;
   CPU.V_Counter = 0;

   ICPU.Registers.PB = 0;
   ICPU.Registers.PC = S9xGetWord(0xfffc);
   ICPU.Registers.D.W = 0;
   ICPU.Registers.DB = 0;
   ICPU.Registers.SH = 1;
   ICPU.Registers.SL = 0xff - 3; /* reset pushes three bytes before the vector fetch */
   ICPU.Registers.XH = 0;
   ICPU.Registers.YH = 0;
   ICPU.Registers.P.W = 0;

   ICPU.ShiftedPB = 0;
   ICPU.ShiftedDB = 0;
   SetFlags(MemoryFlag | IndexFlag | IRQ | Emulation);
   ClearFlags(Decimal);

   CPU.Flags = CPU.Flags & (DEBUG_MODE_FLAG | TRACE_FLAG);
   CPU.BranchSkip = false;
   CPU.NMIActive = false;
   CPU.IRQActive = false;
   CPU.WaitingForInterrupt = false;
   CPU.InDMA = false;
   CPU.PC = NULL;
   CPU.PCBase = NULL;
   CPU.PCAtOpcodeStart = NULL;
   CPU.WaitAddress = NULL;
   CPU.WaitCounter = 1;
   /* CPU.Cycles / NextEvent / V_Counter are NOT re-initialised here: they were
    * set above, before the reset-vector fetch, and that fetch charges its own
    * bus cycles. Assigning 182 again would discard them. */
   CPU.MemSpeed = SLOW_ONE_CYCLE;
   CPU.MemSpeedx2 = SLOW_ONE_CYCLE * 2;
   CPU.SRAMModified = false;
   CPU.NMICycleCount = 0;
   CPU.IRQCycleCount = 0;
   S9xSetPCBase(ICPU.Registers.PC);

   ICPU.S9xOpcodes = S9xOpcodesE1;
   ICPU.CPUExecuting = true;

   S9xUnpackStatus();
}

static void CommonS9xReset()
{
   memset(Memory.FillRAM, 0, FILLRAM_SIZE);
   memset(Memory.VRAM, 0x00, VRAM_SIZE);

   S9xResetCPU();
   S9xResetSRTC();

   S9xResetDMA();
   S9xResetAPU();
   if (Settings.DSP)
      S9xResetDSP();
   if (Settings.OBC1)
      ResetOBC1();
   if (Settings.C4)
      S9xInitC4();
   if (Settings.SuperFX)
      S9xResetSuperFX();
}

void S9xReset()
{
   CommonS9xReset();
   S9xResetPPU();
   memset(Memory.RAM, 0x55, RAM_SIZE);
}

void S9xSoftReset()
{
   CommonS9xReset();
   S9xSoftResetPPU();
}
