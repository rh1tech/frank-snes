/* This file is part of Snes9x. See LICENSE file. */

#ifndef _CPUEXEC_H_
#define _CPUEXEC_H_

typedef struct
{
   void (*S9xOpcode)(void);
} SOpcodes;

#include "ppu.h"
#include "memmap.h"
#include "65c816.h"

typedef struct
{
   uint8_t*   UNUSED1;
   const SOpcodes*  S9xOpcodes;
   SRegisters Registers;
   uint8_t    _Carry;
   uint8_t    _Zero;
   uint8_t    _Negative;
   uint8_t    _Overflow;
   bool       CPUExecuting;
   uint32_t   ShiftedPB;
   uint32_t   ShiftedDB;
   uint32_t   Frame;
   uint32_t   UNUSED2;
   uint32_t   UNUSED3;
} SICPU;

void S9xMainLoop(void);
void S9xReset(void);
void S9xSoftReset(void);
void S9xDoHEventProcessing(void);

/* Drain every scanline event whose position CPU.Cycles has now passed.
 *
 * A while, not an if: one instruction or DMA burst can cross several event
 * positions. Not during DMA - the transfer charges and drains itself, and
 * re-entering the handler mid-transfer would let HDMA recurse. */
/* The common case is "no event due": one load and one compare. Testing
   CPU.InDMA first cost a second load and branch on every single bus access
   and every addressing-mode charge - millions per frame - to guard a case
   that almost never applies. Moving it inside the loop is behaviourally
   identical because the loop body is only reached when an event is due. */
#ifdef FRANK_SNES_NODRAIN
/* MEASUREMENT ONLY - compiles out the sub-instruction event drain so its cost
   on this hardware can be read off. Emulation is WRONG in this build: events
   then only fire at instruction boundaries. Never ship it. */
#define S9xDrainEvents() do { } while (0)
#else
#define S9xDrainEvents() \
   do { \
      while (CPU.Cycles >= CPU.NextEvent) \
      { \
         if (CPU.InDMA) \
            break; \
         S9xDoHEventProcessing(); \
      } \
   } while (0)
#endif
void S9xClearIRQ(uint32_t source);
void S9xSetIRQ(uint32_t source);

extern const SOpcodes S9xOpcodesE1   [256];
extern const SOpcodes S9xOpcodesM1X1 [256];
extern const SOpcodes S9xOpcodesM1X0 [256];
extern const SOpcodes S9xOpcodesM0X1 [256];
extern const SOpcodes S9xOpcodesM0X0 [256];

extern SICPU ICPU;

static INLINE void S9xUnpackStatus(void)
{
   ICPU._Zero = (ICPU.Registers.PL & Zero) == 0;
   ICPU._Negative = (ICPU.Registers.PL & Negative);
   ICPU._Carry = (ICPU.Registers.PL & Carry);
   ICPU._Overflow = (ICPU.Registers.PL & Overflow) >> 6;
}

static INLINE void S9xPackStatus(void)
{
   ICPU.Registers.PL &= ~(Zero | Negative | Carry | Overflow);
   ICPU.Registers.PL |= ICPU._Carry | ((ICPU._Zero == 0) << 1) | (ICPU._Negative & 0x80) | (ICPU._Overflow << 6);
}

static INLINE void CLEAR_IRQ_SOURCE(uint32_t M)
{
   CPU.IRQActive &= ~M;
   if (!CPU.IRQActive)
      CPU.Flags &= ~IRQ_PENDING_FLAG;
}

static INLINE void S9xFixCycles(void)
{
   if (CheckEmulation())
      ICPU.S9xOpcodes = S9xOpcodesE1;
   else if (CheckMemory())
   {
      if (CheckIndex())
         ICPU.S9xOpcodes = S9xOpcodesM1X1;
      else
         ICPU.S9xOpcodes = S9xOpcodesM1X0;
   }
   else
   {
      if (CheckIndex())
         ICPU.S9xOpcodes = S9xOpcodesM0X1;
      else
         ICPU.S9xOpcodes = S9xOpcodesM0X0;
   }
}

/* Advance to the next event in the ring.
 *
 * A pure state machine: each event knows its successor and that successor's
 * position, so there are no comparisons against the current cycle count here.
 * The old version chose between two events by testing which half of the line
 * it was in, and consulted HTimerEnabled/VTimerEnabled on every call. */
static INLINE void S9xReschedule(void)
{
   uint8_t which = HC_HBLANK_START_EVENT;
   int32_t hpos  = Timings.HBlankStart;

   switch (CPU.WhichEvent)
   {
   case HC_HBLANK_START_EVENT:
   case HC_IRQ_1_3_EVENT:
      which = HC_HDMA_START_EVENT;   hpos = Timings.HDMAStart;      break;
   case HC_HDMA_START_EVENT:
   case HC_IRQ_3_5_EVENT:
      which = HC_HCOUNTER_MAX_EVENT; hpos = Timings.H_Max;          break;
   case HC_HCOUNTER_MAX_EVENT:
   case HC_IRQ_5_7_EVENT:
      which = HC_HDMA_INIT_EVENT;    hpos = Timings.HDMAInit;       break;
   case HC_HDMA_INIT_EVENT:
   case HC_IRQ_7_9_EVENT:
      which = HC_RENDER_EVENT;       hpos = Timings.RenderPos;      break;
   case HC_RENDER_EVENT:
   case HC_IRQ_9_A_EVENT:
      which = HC_WRAM_REFRESH_EVENT; hpos = Timings.WRAMRefreshPos; break;
   case HC_WRAM_REFRESH_EVENT:
   case HC_IRQ_A_1_EVENT:
      which = HC_HBLANK_START_EVENT; hpos = Timings.HBlankStart;    break;
   }

   /* Timer due before the event just chosen? Take its slot. */
   if (((int32_t) PPU.HTimerPosition > CPU.NextEvent) && ((int32_t) PPU.HTimerPosition < hpos))
   {
      hpos = (int32_t) PPU.HTimerPosition;

      switch (which)
      {
      case HC_HDMA_START_EVENT:   which = HC_IRQ_1_3_EVENT; break;
      case HC_HCOUNTER_MAX_EVENT: which = HC_IRQ_3_5_EVENT; break;
      case HC_HDMA_INIT_EVENT:    which = HC_IRQ_5_7_EVENT; break;
      case HC_RENDER_EVENT:       which = HC_IRQ_7_9_EVENT; break;
      case HC_WRAM_REFRESH_EVENT: which = HC_IRQ_9_A_EVENT; break;
      case HC_HBLANK_START_EVENT: which = HC_IRQ_A_1_EVENT; break;
      }
   }

   CPU.NextEvent  = hpos;
   CPU.WhichEvent = which;
}
#endif
