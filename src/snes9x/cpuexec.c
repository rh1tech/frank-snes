/* This file is part of Snes9x. See LICENSE file. */

#ifdef PICO_ON_DEVICE
#include "pico/time.h"
#endif
#include "snes9x.h"
#include "memmap.h"
#include "cpuops.h"
#include "ppu.h"
#include "cpuexec.h"
#include "gfx.h"
#include "apu.h"
#include "dma.h"
#include <stdio.h>
#include "../settings.h"

/* Mark main loop as hot for RAM execution on Pico */
#ifdef PICO_ON_DEVICE
#define CPU_HOT __attribute__((hot, section(".time_critical.cpu_loop")))
#else
#define CPU_HOT
#endif

static inline bool S9xNMIDue(void)
{
   return ((uint32_t) CPU.Cycles - CPU.NMICycleCount) < 0x80000000u;
}

static inline void S9xWakeFromWAI(void)
{
   CPU.WaitingForInterrupt = false;
   CPU.PC++;
   CPU.Cycles += TWO_CYCLES + ONE_DOT_CYCLE_DIV_2;
   while (CPU.Cycles >= CPU.NextEvent)
      S9xDoHEventProcessing();
}

volatile uint32_t frank_instr_count;   /* read over SWD; ~1 cycle per instruction */
volatile uint32_t frank_event_us;      /* total us inside S9xDoHEventProcessing */
volatile uint32_t frank_ev_us_by_type[8];  /* 0 hbl 1 hdma_s 2 hcmax 3 hdma_i 4 render 5 wram 6 irq */
volatile uint32_t frank_ev_n_by_type[8];
volatile uint32_t frank_event_count;

CPU_HOT void S9xMainLoop()
{
   /* Accumulate in a local, flush once per frame. `frank_instr_count` is
      volatile (the debug probe reads it out of RAM while the board runs), so
      incrementing it here cost a load, an add and a store on EVERY emulated
      instruction - 3 of the 22 instructions on the loop's minimum path, and 2
      of its 13 memory operations. A local lives in a callee-saved register
      across the opcode's indirect call, leaving just the add. The value the
      probe reads is identical; it simply lands once per frame. */
   uint32_t instr_this_frame = 0;

   do
   {
      APU_EXECUTE();
      if (CPU.Flags)
      {
         if (CPU.Flags & NMI_FLAG)
         {
            if (S9xNMIDue())
            {
               CPU.Flags &= ~NMI_FLAG;
               if (CPU.WaitingForInterrupt)
                  S9xWakeFromWAI();
               S9xOpcode_NMI();
            }
         }

         if (CPU.Cycles >= Timings.NextIRQTimer)
         {
            /* The H/V timer is an absolute cycle deadline tested at every
               opcode boundary, not a slot in the scanline event ring. A ring
               slot can only fire where an event already exists, so a V-only
               timer was rounded up to HC=20 while hardware raises it at
               HC=10, and the IRQ was taken one instruction late every line. */
            S9xUpdateIRQPositions(false);
            S9xSetIRQ(PPU.HTimerEnabled ? PPU_H_BEAM_IRQ_SOURCE
                                        : PPU_V_BEAM_IRQ_SOURCE);
         }

         if (CPU.Flags & IRQ_PENDING_FLAG)
         {
            /* Retire a stale pending flag BEFORE anything else. WAI is
               released by an asserted interrupt line, so waking first and
               asking whether the line was actually asserted afterwards let a
               spent IRQ pull the CPU out of WAI early - and the real NMI then
               landed an instruction late. */
            if (!CPU.IRQActive || Settings.DisableIRQ)
               CPU.Flags &= ~IRQ_PENDING_FLAG;
            else
            {
               if (CPU.WaitingForInterrupt)
                  S9xWakeFromWAI();
               /* Level-triggered: while the I flag masks it the line stays
                  asserted and we retry, as the accurate core does. */
               if (!CheckFlag(IRQ))
                  S9xOpcode_IRQ();
            }
         }

         /* Waking from WAI charges the pipeline restart and drains events,
            which can carry the scanline over and make the NMI due. The NMI
            test above already ran, so without this re-check the CPU executes
            one instruction before servicing it. */
         if ((CPU.Flags & NMI_FLAG) && S9xNMIDue())
         {
            CPU.Flags &= ~NMI_FLAG;
            S9xOpcode_NMI();
         }
         if (CPU.Flags & SCAN_KEYS_FLAG)
            break;
      }

      CPU.PCAtOpcodeStart = CPU.PC;
      instr_this_frame++;
      CPU.Cycles += CPU.MemSpeed;
      (*ICPU.S9xOpcodes [*CPU.PC++].S9xOpcode)();
      /* A while, not an if: an instruction can cross more than one event
         position, and a single test silently drops all but the first. */
      while (CPU.Cycles >= CPU.NextEvent)
         S9xDoHEventProcessing();
   } while(true);

   frank_instr_count += instr_this_frame;

   ICPU.Registers.PC = CPU.PC - CPU.PCBase;
#ifndef USE_BLARGG_APU
   IAPU.Registers.PC = IAPU.PC - IAPU.RAM;
#endif

   S9xPackStatus();
#ifndef USE_BLARGG_APU
   S9xAPUPackStatus();
#endif
   CPU.Flags &= ~SCAN_KEYS_FLAG;
}

void S9xSetIRQ(uint32_t source)
{
   CPU.IRQActive |= source;
   CPU.Flags |= IRQ_PENDING_FLAG;
   /* Taken at this cycle: the recognition delay is already baked into the
      timer's position by S9xUpdateIRQPositions. The old three-INSTRUCTION
      countdown made the IRQ's phase depend on what code was running. */
   /* Waking from WAI is NOT done here. The main loop does it where the
      interrupt is actually taken, and charges the pipeline restart
      (TWO_CYCLES + ONE_DOT_CYCLE/2) for it. */
}

void S9xClearIRQ(uint32_t source)
{
   CLEAR_IRQ_SOURCE(source);
}

/* The timer position can coincide exactly with a scheduled event, in which
 * case no IRQ_x_y slot is created for it and the base event raises it. */
static void S9xCheckMissingHTimerPosition(void)
{
   /* Raised by the main loop from Timings.NextIRQTimer instead. */
}

/* A timer falling inside the refresh window cannot be taken while the CPU is
 * off the bus, so hold the IRQ back by one instruction. */
static void S9xCheckMissingHTimerHalt(void)
{
   /* The timer deadline is absolute now, so the refresh stall it has to sit
      behind is already accounted for by CPU.Cycles itself. */
}

/* Dispatch one scanline event.
 *
 * The work that used to happen all at once at the end of the line is now
 * distributed to the point in the line where the hardware does it, and the
 * WRAM refresh stall - absent from this core entirely - is charged. */
/* Deliberately NOT CPU_HOT. That attribute puts the function in
 * .time_critical, which the linker places in RAM, and the six-event handler is
 * far larger than the two-event one it replaced - enough to overflow RAM on
 * the single-chip boards. It runs about 94k times a second, not once per
 * instruction, so executing it from XIP-cached flash is the right trade. */
static void S9xDoHEventProcessingInner(void);

CPU_HOT void S9xDoHEventProcessing()
{
   uint32_t _t = time_us_32();
   uint32_t _w = CPU.WhichEvent;
   uint32_t _slot = (_w == HC_HBLANK_START_EVENT) ? 0 :
                    (_w == HC_HDMA_START_EVENT)   ? 1 :
                    (_w == HC_HCOUNTER_MAX_EVENT) ? 2 :
                    (_w == HC_HDMA_INIT_EVENT)    ? 3 :
                    (_w == HC_RENDER_EVENT)       ? 4 :
                    (_w == HC_WRAM_REFRESH_EVENT) ? 5 : 6;
   frank_event_count++;
   S9xDoHEventProcessingInner();
   { uint32_t _d = time_us_32() - _t;
     frank_event_us += _d;
     frank_ev_us_by_type[_slot] += _d;
     frank_ev_n_by_type[_slot]++; }
}

static void S9xDoHEventProcessingInner()
{
   CPU.WaitCounter++;

   switch (CPU.WhichEvent)
   {
   case HC_HBLANK_START_EVENT:
      if (PPU.HTimerPosition == Timings.HBlankStart)
         S9xCheckMissingHTimerPosition();
      S9xReschedule();
      break;

   case HC_HDMA_START_EVENT:
      if (PPU.HTimerPosition == Timings.HDMAStart)
         S9xCheckMissingHTimerPosition();
      S9xReschedule();

      /* HDMA transfers at HC=1106, not at the start of HBlank. */
      if (g_settings.hdma_enabled && IPPU.HDMA && CPU.V_Counter <= PPU.ScreenHeight)
         IPPU.HDMA = S9xDoHDMA(IPPU.HDMA);
      break;

   case HC_HCOUNTER_MAX_EVENT:
      if (Settings.SuperFX)
         S9xSuperFXExec();

#ifndef USE_BLARGG_APU
      CPU.Cycles -= Timings.H_Max;
#if defined(PICO_ON_DEVICE) && defined(APU_ON_CORE1) && APU_ON_CORE1
      /* Don't touch APU.Cycles from Core 0 — Core 1 owns it.
       * Accumulate debt that Core 1 applies via atomic exchange. */
      { extern volatile int32_t apu_cycle_debt;
        __atomic_fetch_add(&apu_cycle_debt, Timings.H_Max, __ATOMIC_RELAXED); }
#else
      APU.Cycles -= Timings.H_Max;
#endif
#else
      S9xAPUExecute();
      CPU.Cycles -= Timings.H_Max;
      S9xAPUSetReferenceTime(CPU.Cycles);
#endif
      if (CPU.Flags & NMI_FLAG)
         CPU.NMICycleCount -= Timings.H_Max;
      /* Without this the timer deadline drifts a whole scanline further away
         every line, and the IRQ fires at roughly half its true rate. */
      if (Timings.NextIRQTimer != 0x0fffffff)
         Timings.NextIRQTimer -= Timings.H_Max;

      if (++CPU.V_Counter >= Timings.V_Max)
      {
         CPU.V_Counter = 0;
         Timings.InterlaceField = !Timings.InterlaceField;
         Memory.FillRAM[0x213F] ^= 0x80;
         PPU.RangeTimeOver = 0;
         CPU.NMIActive = false;
         ICPU.Frame++;
         CPU.Flags |= SCAN_KEYS_FLAG;
      }

      /* Scanline 240 of a non-interlaced odd field is one dot short. */
      Timings.H_Max = Timings.H_Max_Master;
      if (CPU.V_Counter == 240 && !IPPU.Interlace && Timings.InterlaceField)
         Timings.H_Max -= ONE_DOT_CYCLE;

      if (CPU.V_Counter != 240 || IPPU.Interlace || !Timings.InterlaceField)
      {
         if (Timings.WRAMRefreshPos == SNES_WRAM_REFRESH_HC_v2_MIN_ONE_DOT_CYCLE)
            Timings.WRAMRefreshPos = SNES_WRAM_REFRESH_HC_v2;
         else
            Timings.WRAMRefreshPos = SNES_WRAM_REFRESH_HC_v2_MIN_ONE_DOT_CYCLE;
      }

      if (CPU.V_Counter == PPU.ScreenHeight + FIRST_VISIBLE_LINE)
      {
         /* Start of V-blank */
         S9xEndScreenRefresh();
         IPPU.HDMA = 0;
         /* Bits 7 and 6 of $4212 are computed when read in S9xGetPPU. */
         PPU.ForcedBlanking = (Memory.FillRAM [0x2100] >> 7) & 1;

         if (!PPU.ForcedBlanking)
         {
            uint8_t tmp = 0;
            PPU.OAMAddr = PPU.SavedOAMAddr;

            if (PPU.OAMPriorityRotation)
               tmp = (PPU.OAMAddr & 0xFE) >> 1;
            if ((PPU.OAMFlip & 1) || PPU.FirstSprite != tmp)
            {
               PPU.FirstSprite = tmp;
               IPPU.OBJChanged = true;
            }

            PPU.OAMFlip = 0;
         }

         Memory.FillRAM[0x4210] = 0x80 | SNES_5A22;
         if (Memory.FillRAM[0x4200] & 0x80)
         {
            CPU.NMIActive = true;
            CPU.Flags |= NMI_FLAG;
            CPU.NMICycleCount = TWO_CYCLES;
         }
      }

      if (CPU.V_Counter == PPU.ScreenHeight + 3)
         S9xUpdateJoypads();

      if (CPU.V_Counter == FIRST_VISIBLE_LINE)
      {
         Memory.FillRAM[0x4210] = SNES_5A22;
         CPU.Flags &= ~NMI_FLAG;
         S9xStartScreenRefresh();
      }

#ifndef USE_BLARGG_APU
      if (APU.TimerEnabled [2])
      {
         APU.Timer [2] += 4;
         while (APU.Timer [2] >= APU.TimerTarget [2])
         {
            IAPU.RAM [0xff] = (IAPU.RAM [0xff] + 1) & 0xf;
            APU.Timer [2] -= APU.TimerTarget [2];
            IAPU.WaitCounter++;
            IAPU.APUExecuting = true;
         }
      }
      if (CPU.V_Counter & 1)
      {
         if (APU.TimerEnabled [0])
         {
            APU.Timer [0]++;
            if (APU.Timer [0] >= APU.TimerTarget [0])
            {
               IAPU.RAM [0xfd] = (IAPU.RAM [0xfd] + 1) & 0xf;
               APU.Timer [0] = 0;
               IAPU.WaitCounter++;
               IAPU.APUExecuting = true;
            }
         }
         if (APU.TimerEnabled [1])
         {
            APU.Timer [1]++;
            if (APU.Timer [1] >= APU.TimerTarget [1])
            {
               IAPU.RAM [0xfe] = (IAPU.RAM [0xfe] + 1) & 0xf;
               APU.Timer [1] = 0;
               IAPU.WaitCounter++;
               IAPU.APUExecuting = true;
            }
         }
      }
#endif

      if (PPU.HTimerPosition == 0)
         S9xCheckMissingHTimerPosition();

      /* Only now: NextEvent is the guard against re-entry, and everything
         above can touch the bus. */
      CPU.NextEvent = -1;
      S9xReschedule();
      break;

   case HC_HDMA_INIT_EVENT:
      if (PPU.HTimerPosition == Timings.HDMAInit)
         S9xCheckMissingHTimerPosition();
      S9xReschedule();

      /* HDMA is armed near the top of the first line, not at the end of the
         last one. */
      if (CPU.V_Counter == 0 && g_settings.hdma_enabled)
         S9xStartHDMA();
      break;

   case HC_RENDER_EVENT:
      if (CPU.V_Counter >= FIRST_VISIBLE_LINE &&
          CPU.V_Counter < PPU.ScreenHeight + FIRST_VISIBLE_LINE)
      {
         /* SuperFX games keep forced blanking on while the GSU renders
          * across multiple frames. Override so the PPU displays VRAM data. */
         if (Settings.SuperFX && PPU.ForcedBlanking) {
            PPU.ForcedBlanking = 0;
            if (PPU.Brightness == 0) {
               PPU.Brightness = 0xF;
               S9xFixColourBrightness();
            }
         }
         RenderLine(CPU.V_Counter - FIRST_VISIBLE_LINE);
      }

      if (PPU.HTimerPosition == Timings.RenderPos)
         S9xCheckMissingHTimerPosition();
      S9xReschedule();
      break;

   case HC_WRAM_REFRESH_EVENT:
      /* The CPU is held off the bus once per scanline while WRAM is
         refreshed. This core never modelled it, so it gave the 65816 2.9%
         more time per line than the hardware does - time the SPC700, clocked
         independently, does not get. That was the whole rate error. */
      if (PPU.HTimerPosition >= Timings.WRAMRefreshPos &&
          PPU.HTimerPosition < Timings.WRAMRefreshPos + SNES_WRAM_REFRESH_CYCLES)
         S9xCheckMissingHTimerHalt();

      CPU.Cycles += SNES_WRAM_REFRESH_CYCLES;

      if (PPU.HTimerPosition == Timings.WRAMRefreshPos)
         S9xCheckMissingHTimerPosition();
      S9xReschedule();
      break;

   case HC_IRQ_1_3_EVENT:
   case HC_IRQ_3_5_EVENT:
   case HC_IRQ_5_7_EVENT:
   case HC_IRQ_7_9_EVENT:
   case HC_IRQ_9_A_EVENT:
   case HC_IRQ_A_1_EVENT:
      if ((PPU.HTimerEnabled && (!PPU.VTimerEnabled || CPU.V_Counter == S9xVTimerPosition)) ||
          (PPU.VTimerEnabled && CPU.V_Counter == S9xVTimerPosition))
         S9xSetIRQ(PPU_H_BEAM_IRQ_SOURCE);

      S9xReschedule();
      break;
   }
}
