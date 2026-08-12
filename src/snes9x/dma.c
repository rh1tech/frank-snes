/* This file is part of Snes9x. See LICENSE file. */

#include "snes9x.h"
#include "memmap.h"
#include "ppu.h"
#include "cpuexec.h"
#include "dma.h"
#include "apu.h"
#include <stdio.h>

/*modified per anomie Mode 5 findings */
static const int32_t HDMA_ModeByteCounts [8] =
{
   1, 2, 2, 4, 4, 4, 2, 4
};
extern uint8_t* HDMAMemPointers [8];
extern uint8_t* HDMABasePointers [8];

/**********************************************************************************************/
/* S9xDoDMA()                                                                                 */
/* This function preforms the general dma transfer                                            */
/**********************************************************************************************/
#ifdef PICO_ON_DEVICE
__attribute__((hot, section(".time_critical.dma")))
#endif
void S9xDoDMA(uint8_t Channel)
{
   uint8_t Work;
   int32_t count;
   int32_t inc;
   SDMA* d;

   if (Channel > 7 || CPU.InDMA)
      return;

   CPU.InDMA = true;
   d = &DMA[Channel];
   count = d->TransferBytes;


   /* Prepare for custom chip DMA */
   if (count == 0)
      count = 0x10000;

   inc = d->AAddressFixed ? 0 : (!d->AAddressDecrement ? 1 : -1);

   if ((d->ABank == 0x7E || d->ABank == 0x7F) && d->BAddress == 0x80 && !d->TransferDirection)
   {
      d->AAddress += d->TransferBytes;
      /* Does an invalid DMA actually take time?
       * I'd say yes, since 'invalid' is probably just the WRAM chip
       * not being able to read and write itself at the same time */
      CPU.Cycles += (d->TransferBytes + 1) * SLOW_ONE_CYCLE;
      goto update_address;
   }
   switch (d->BAddress)
   {
      case 0x18:
      case 0x19:
         if (IPPU.RenderThisFrame)
            FLUSH_REDRAW();
         break;
   }

   if (!d->TransferDirection)
   {
      uint8_t* base;
      uint16_t p;
      /* XXX: DMA is potentially broken here for cases where we DMA across
       * XXX: memmap boundries. A possible solution would be to re-call
       * XXX: GetBasePointer whenever we cross a boundry, and when
       * XXX: GetBasePointer returns (0) to take the 'slow path' and use
       * XXX: S9xGetByte instead of *base. GetBasePointer() would want to
       * XXX: return 0 for MAP_PPU and whatever else is a register range
       * XXX: rather than a RAM/ROM block, and we'd want to detect MAP_PPU
       * XXX: (or specifically, Address Bus B addresses $2100-$21FF in
       * XXX: banks $00-$3F) specially and treat it as MAP_NONE (since
       * XXX: PPU->PPU transfers don't work).
       */

      /* reflects extra cycle used by DMA */
      CPU.Cycles += SLOW_ONE_CYCLE * (count + 1);

      base = GetBasePointer((d->ABank << 16) + d->AAddress);
      p    = d->AAddress;

      if (!base)
         base = Memory.ROM;

      if (inc > 0)
         d->AAddress += count;
      else if (inc < 0)
         d->AAddress -= count;

      if (d->TransferMode == 0 || d->TransferMode == 2 || d->TransferMode == 6)
      {
         switch (d->BAddress)
         {
            case 0x04:
               do
               {
                  Work = *(base + p);
                  REGISTER_2104(Work);
                  p += inc;
               } while (--count > 0);
               break;
            case 0x18:
               IPPU.FirstVRAMRead = true;
               if (!PPU.VMA.FullGraphicCount)
               {
                  do
                  {
                     Work = *(base + p);
                     REGISTER_2118_linear(Work);
                     p += inc;
                  } while (--count > 0);
               }
               else
               {
                  do
                  {
                     Work = *(base + p);
                     REGISTER_2118_tile(Work);
                     p += inc;
                  } while (--count > 0);
               }
               break;
            case 0x19:
               IPPU.FirstVRAMRead = true;
               if (!PPU.VMA.FullGraphicCount)
               {
                  do
                  {
                     Work = *(base + p);
                     REGISTER_2119_linear(Work);
                     p += inc;
                  } while (--count > 0);
               }
               else
               {
                  do
                  {
                     Work = *(base + p);
                     REGISTER_2119_tile(Work);
                     p += inc;
                  } while (--count > 0);
               }
               break;
            case 0x22:
               do
               {
                  Work = *(base + p);
                  REGISTER_2122(Work);
                  p += inc;
               } while (--count > 0);
               break;
            case 0x80:
               do
               {
                  Work = *(base + p);
                  REGISTER_2180(Work);
                  p += inc;
               } while (--count > 0);
               break;
            default:
               do
               {
                  Work = *(base + p);
                  S9xSetPPU(Work, 0x2100 + d->BAddress);
                  p += inc;
               } while (--count > 0);
               break;
         }
      }
      else if (d->TransferMode == 1 || d->TransferMode == 5)
      {
         if (d->BAddress == 0x18)
         {
            /* Write to V-RAM */
            IPPU.FirstVRAMRead = true;
            if (!PPU.VMA.FullGraphicCount)
            {
               while (count > 1)
               {
                  Work = *(base + p);
                  REGISTER_2118_linear(Work);
                  p += inc;

                  Work = *(base + p);
                  REGISTER_2119_linear(Work);
                  p += inc;
                  count -= 2;
               }
               if (count == 1)
               {
                  Work = *(base + p);
                  REGISTER_2118_linear(Work);
               }
            }
            else
            {
               while (count > 1)
               {
                  Work = *(base + p);
                  REGISTER_2118_tile(Work);
                  p += inc;

                  Work = *(base + p);
                  REGISTER_2119_tile(Work);
                  p += inc;
                  count -= 2;
               }
               if (count == 1)
               {
                  Work = *(base + p);
                  REGISTER_2118_tile(Work);
               }
            }
         }
         else
         {
            /* DMA mode 1 general case */
            while (count > 1)
            {
               Work = *(base + p);
               S9xSetPPU(Work, 0x2100 + d->BAddress);
               p += inc;

               Work = *(base + p);
               S9xSetPPU(Work, 0x2101 + d->BAddress);
               p += inc;
               count -= 2;
            }
            if (count == 1)
            {
               Work = *(base + p);
               S9xSetPPU(Work, 0x2100 + d->BAddress);
            }
         }
      }
      else if (d->TransferMode == 3 || d->TransferMode == 7)
      {
         do
         {
            Work = *(base + p);
            S9xSetPPU(Work, 0x2100 + d->BAddress);
            p += inc;
            if (count <= 1)
               break;

            Work = *(base + p);
            S9xSetPPU(Work, 0x2100 + d->BAddress);
            p += inc;
            if (count <= 2)
               break;

            Work = *(base + p);
            S9xSetPPU(Work, 0x2101 + d->BAddress);
            p += inc;
            if (count <= 3)
               break;

            Work = *(base + p);
            S9xSetPPU(Work, 0x2101 + d->BAddress);
            p += inc;
            count -= 4;
         } while (count > 0);
      }
      else if (d->TransferMode == 4)
      {
         do
         {
            Work = *(base + p);
            S9xSetPPU(Work, 0x2100 + d->BAddress);
            p += inc;
            if (count <= 1)
               break;

            Work = *(base + p);
            S9xSetPPU(Work, 0x2101 + d->BAddress);
            p += inc;
            if (count <= 2)
               break;

            Work = *(base + p);
            S9xSetPPU(Work, 0x2102 + d->BAddress);
            p += inc;
            if (count <= 3)
               break;

            Work = *(base + p);
            S9xSetPPU(Work, 0x2103 + d->BAddress);
            p += inc;
            count -= 4;
         } while (count > 0);
      }
   }
   else
   {
      /* XXX: DMA is potentially broken here for cases where the dest is
       * XXX: in the Address Bus B range. Note that this bad dest may not
       * XXX: cover the whole range of the DMA though, if we transfer
       * XXX: 65536 bytes only 256 of them may be Address Bus B.
       */
      do
      {
         switch (d->TransferMode)
         {
            case 0:
            case 2:
            case 6:
               Work = S9xGetPPU(0x2100 + d->BAddress);
               S9xSetByte(Work, (d->ABank << 16) + d->AAddress);
               d->AAddress += inc;
               --count;
               break;
            case 1:
            case 5:
               Work = S9xGetPPU(0x2100 + d->BAddress);
               S9xSetByte(Work, (d->ABank << 16) + d->AAddress);
               d->AAddress += inc;
               if (!--count)
                  break;

               Work = S9xGetPPU(0x2101 + d->BAddress);
               S9xSetByte(Work, (d->ABank << 16) + d->AAddress);
               d->AAddress += inc;
               count--;
               break;
            case 3:
            case 7:
               Work = S9xGetPPU(0x2100 + d->BAddress);
               S9xSetByte(Work, (d->ABank << 16) + d->AAddress);
               d->AAddress += inc;
               if (!--count)
                  break;

               Work = S9xGetPPU(0x2100 + d->BAddress);
               S9xSetByte(Work, (d->ABank << 16) + d->AAddress);
               d->AAddress += inc;
               if (!--count)
                  break;

               Work = S9xGetPPU(0x2101 + d->BAddress);
               S9xSetByte(Work, (d->ABank << 16) + d->AAddress);
               d->AAddress += inc;
               if (!--count)
                  break;

               Work = S9xGetPPU(0x2101 + d->BAddress);
               S9xSetByte(Work, (d->ABank << 16) + d->AAddress);
               d->AAddress += inc;
               count--;
               break;
            case 4:
               Work = S9xGetPPU(0x2100 + d->BAddress);
               S9xSetByte(Work, (d->ABank << 16) + d->AAddress);
               d->AAddress += inc;
               if (!--count)
                  break;

               Work = S9xGetPPU(0x2101 + d->BAddress);
               S9xSetByte(Work, (d->ABank << 16) + d->AAddress);
               d->AAddress += inc;
               if (!--count)
                  break;

               Work = S9xGetPPU(0x2102 + d->BAddress);
               S9xSetByte(Work, (d->ABank << 16) + d->AAddress);
               d->AAddress += inc;
               if (!--count)
                  break;

               Work = S9xGetPPU(0x2103 + d->BAddress);
               S9xSetByte(Work, (d->ABank << 16) + d->AAddress);
               d->AAddress += inc;
               count--;
               break;
            default:
               count = 0;
               break;
         }
      } while (count);
   }
#ifndef USE_BLARGG_APU
   IAPU.APUExecuting = Settings.APUEnabled;
   APU_EXECUTE();
#endif
   while (CPU.Cycles >= CPU.NextEvent)
      S9xDoHEventProcessing();

update_address:
   if (CPU.Flags & NMI_FLAG)
      CPU.NMICycleCount = CPU.Cycles + 24;

   /* Super Punch-Out requires that the A-BUS address be updated after the DMA transfer. */
   Memory.FillRAM[0x4302 + (Channel << 4)] = (uint8_t) d->AAddress;
   Memory.FillRAM[0x4303 + (Channel << 4)] = d->AAddress >> 8;

   /* Secret of Mana requires that the DMA bytes transfer count be set to zero when DMA has completed. */
   Memory.FillRAM [0x4305 + (Channel << 4)] = 0;
   Memory.FillRAM [0x4306 + (Channel << 4)] = 0;

   DMA[Channel].IndirectAddress = 0;
   d->TransferBytes = 0;

   CPU.InDMA = false;
}

/* Load a channel's line count (and, when indirect, its data pointer) and
 * charge the bus cycles that costs.
 *
 * This used to happen at the START of the line that needed it - one scanline
 * after the hardware reads it - while S9xStartHDMA separately charged for a
 * read it never performed, so the frame's first HDMA was billed twice.
 * Returns false when the table has ended and the channel should stop. */
static bool HDMAReadLineCount(int32_t d)
{
   SDMA*   p = &DMA [d];
   uint8_t line;

   /* InDMA is set, so the accessors charge nothing of their own. */
   line = S9xGetByte((p->ABank << 16) + p->Address);
   CPU.Cycles += SLOW_ONE_CYCLE;

   if (!line)
   {
      p->Repeat    = false;
      p->LineCount = 128;

      if (p->HDMAIndirectAddressing)
      {
         if (IPPU.HDMA & (0xfe << d))
         {
            p->Address++;
            CPU.Cycles += SLOW_ONE_CYCLE << 1;
         }
         else
            CPU.Cycles += SLOW_ONE_CYCLE;

         p->IndirectAddress = S9xGetWord((p->ABank << 16) + p->Address);
         p->Address++;
      }

      p->Address++;
      HDMAMemPointers [d] = NULL;
      return false;
   }

   if (line == 0x80)
   {
      p->Repeat    = true;
      p->LineCount = 128;
   }
   else
   {
      p->Repeat    = !(line & 0x80);
      p->LineCount = line & 0x7f;
   }

   p->Address++;
   p->FirstLine  = true;
   p->DoTransfer = true;

   if (p->HDMAIndirectAddressing)
   {
      /* A word fetch is two bus cycles, not four. */
      CPU.Cycles += SLOW_ONE_CYCLE << 1;
      p->IndirectBank    = Memory.FillRAM [0x4307 + (d << 4)];
      p->IndirectAddress = S9xGetWord((p->ABank << 16) + p->Address);
      p->Address += 2;
   }
   else
   {
      p->IndirectBank    = p->ABank;
      p->IndirectAddress = p->Address;
   }

   HDMABasePointers [d] = HDMAMemPointers [d] =
      S9xGetMemPointer((p->IndirectBank << 16) + p->IndirectAddress);
   return true;
}

void S9xStartHDMA(void)
{
   uint8_t i;
   IPPU.HDMA = Memory.FillRAM [0x420c];

   if (IPPU.HDMA != 0)
      CPU.Cycles += Timings.DMACPUSync;

   for (i = 0; i < 8; i++)
      HDMAMemPointers [i] = NULL;

   {
      /* HDMA can fire from inside a general DMA's event drain, so the flag has
         to be restored, not forced false - clearing it would put the enclosing
         transfer back on the charged path mid-flight. */
      bool prev_in_dma = CPU.InDMA;

      CPU.InDMA = true;
      for (i = 0; i < 8; i++)
      {
         if (IPPU.HDMA & (1 << i))
         {
            DMA [i].Address = DMA [i].AAddress;
            if (!HDMAReadLineCount(i))
               IPPU.HDMA &= ~(1 << i);
         }
         else
            DMA [i].DoTransfer = false;
      }
      CPU.InDMA = prev_in_dma;
   }
}

volatile uint32_t frank_hdma_bytes;    /* transferred bytes, read over SWD */
volatile uint32_t frank_hdma_chan;     /* armed-channel iterations */
volatile uint32_t frank_hdma_calls;    /* S9xDoHDMA entries */
volatile uint32_t frank_getmemptr;     /* S9xGetMemPointer calls from HDMA */

/* Index of the lowest armed channel. Both passes below visit exactly the
   channels named in the mask, in ascending order - the same channels the old
   test-all-eight loops reached via `continue`, at a fraction of the cost.
   HDMA is typically armed on two or three channels and most scanlines
   transfer nothing, so the skipped iterations were the bulk of the work:
   measured on MK3 in a fight, 3,172 iterations per frame become 1,128
   (2.81x). Verified a behavioural no-op offline - MK3 scores 1.000 on the
   announcer gate and SMW, Axelay and Batman Forever render byte-identical to
   the previous build. */
#define LOWEST_CHANNEL(m) (__builtin_ctz((unsigned) (m)))

uint8_t S9xDoHDMA(uint8_t byte)
{
   uint8_t mask, rem;
   SDMA*   p = &DMA [0];
   int32_t d = 0;

   bool prev_in_dma = CPU.InDMA;

   frank_hdma_calls++;
   CPU.InDMA = true;
   CPU.Cycles += Timings.DMACPUSync;

   /* Pass 1: every armed channel that owes a transfer does it now. */
   for (rem = byte; rem; rem &= rem - 1)
   {
      d    = LOWEST_CHANNEL(rem);
      mask = 1 << d;
      p    = &DMA [d];

      if (!HDMAMemPointers [d])
      {
         uint32_t bank = p->HDMAIndirectAddressing ? p->IndirectBank : p->ABank;
         uint16_t addr = p->HDMAIndirectAddressing ? p->IndirectAddress : p->Address;

         frank_getmemptr++;
         if (!(HDMABasePointers [d] = HDMAMemPointers [d] =
                  S9xGetMemPointer((bank << 16) + addr)))
         {
            byte &= ~mask;
            continue;
         }
      }

      if (!p->DoTransfer)
         continue;

         frank_hdma_bytes += HDMA_ModeByteCounts[p->TransferMode];
         switch (p->TransferMode)
         {
            case 0:
               CPU.Cycles += SLOW_ONE_CYCLE;
               S9xSetPPU(*HDMAMemPointers [d]++, 0x2100 + p->BAddress);
               break;
            case 5:
               CPU.Cycles += 2 * SLOW_ONE_CYCLE;
               S9xSetPPU(*(HDMAMemPointers [d] + 0), 0x2100 + p->BAddress);
               S9xSetPPU(*(HDMAMemPointers [d] + 1), 0x2101 + p->BAddress);
               HDMAMemPointers [d] += 2;
               /* fall through */
            case 1:
               CPU.Cycles += 2 * SLOW_ONE_CYCLE;
               S9xSetPPU(*(HDMAMemPointers [d] + 0), 0x2100 + p->BAddress);
               S9xSetPPU(*(HDMAMemPointers [d] + 1), 0x2101 + p->BAddress);
               HDMAMemPointers [d] += 2;
               break;
            case 2:
            case 6:
               CPU.Cycles += 2 * SLOW_ONE_CYCLE;
               S9xSetPPU(*(HDMAMemPointers [d] + 0), 0x2100 + p->BAddress);
               S9xSetPPU(*(HDMAMemPointers [d] + 1), 0x2100 + p->BAddress);
               HDMAMemPointers [d] += 2;
               break;
            case 3:
            case 7:
               CPU.Cycles += 4 * SLOW_ONE_CYCLE;
               S9xSetPPU(*(HDMAMemPointers [d] + 0), 0x2100 + p->BAddress);
               S9xSetPPU(*(HDMAMemPointers [d] + 1), 0x2100 + p->BAddress);
               S9xSetPPU(*(HDMAMemPointers [d] + 2), 0x2101 + p->BAddress);
               S9xSetPPU(*(HDMAMemPointers [d] + 3), 0x2101 + p->BAddress);
               HDMAMemPointers [d] += 4;
               break;
            case 4:
               CPU.Cycles += 4 * SLOW_ONE_CYCLE;
               S9xSetPPU(*(HDMAMemPointers [d] + 0), 0x2100 + p->BAddress);
               S9xSetPPU(*(HDMAMemPointers [d] + 1), 0x2101 + p->BAddress);
               S9xSetPPU(*(HDMAMemPointers [d] + 2), 0x2102 + p->BAddress);
               S9xSetPPU(*(HDMAMemPointers [d] + 3), 0x2103 + p->BAddress);
               HDMAMemPointers [d] += 4;
               break;
         }

   }

   /* Pass 2: advance the tables and reload any channel whose count ran out.
      The reload belongs to the line that consumed the last entry, not to the
      line that follows it. */
   for (rem = byte; rem; rem &= rem - 1)
   {
      d    = LOWEST_CHANNEL(rem);
      mask = 1 << d;
      p    = &DMA [d];

      if (p->DoTransfer)
      {
         if (p->HDMAIndirectAddressing)
            p->IndirectAddress += HDMA_ModeByteCounts [p->TransferMode];
         else
            p->Address += HDMA_ModeByteCounts [p->TransferMode];
      }

      p->FirstLine  = false;
      p->DoTransfer = !p->Repeat;

      if (!--p->LineCount)
      {
         if (!HDMAReadLineCount(d))
         {
            byte &= ~mask;
            p->DoTransfer = false;
         }
      }
      else
         CPU.Cycles += SLOW_ONE_CYCLE;
   }

   CPU.InDMA = prev_in_dma;
   return byte;
}

void S9xResetDMA(void)
{
   int32_t c, d;
   for (d = 0; d < 8; d++)
   {
      DMA [d].TransferDirection = false;
      DMA [d].HDMAIndirectAddressing = false;
      DMA [d].AAddressFixed = true;
      DMA [d].AAddressDecrement = false;
      DMA [d].TransferMode = 7;
      DMA [d].ABank = 0xff;
      DMA [d].AAddress = 0xffff;
      DMA [d].Address = 0xffff;
      DMA [d].BAddress = 0xff;
      DMA [d].TransferBytes = 0xffff;
      DMA [d].IndirectAddress = 0xffff;
   }
}
