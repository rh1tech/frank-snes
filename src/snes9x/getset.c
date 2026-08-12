/* This file is part of Snes9x. See LICENSE file. */

#include "ppu.h"

/* The bus accessors run on every single memory access the 65816 makes, so on
   the Pico they belong in RAM rather than XIP flash. */
#ifdef PICO_ON_DEVICE
#define BUS_HOT __attribute__((hot, section(".time_critical.bus")))
#else
#define BUS_HOT
#endif

#ifdef __GNUC__
#define NO_INLINE __attribute__((noinline))
#else
#define NO_INLINE
#endif
#include "dsp.h"
#include "cpuexec.h"
#include "obc1.h"

/* Undefine assembly redirects so we can define the C versions */
#undef S9xGetByte
#undef S9xGetWord

extern uint8_t OpenBus;

/* The bus access happens BEFORE its cycles are charged.
 *
 * Charging first timestamps a read of a cycle-sensitive register - $2137's
 * beam latch, $213c-f, $4212 - as if the beam had already advanced by the
 * width of the access itself. */
/* The fast path (a real pointer in the memory map) is 91.6% of byte reads,
   measured on MK3 in a fight. Keeping the rare switch in the SAME function
   made the compiler reserve every register both paths need, so every call
   pushed and popped SIX registers - twelve words of stack traffic to service
   one load. Splitting the cold half out lets the hot half keep a small frame.
   Pure code motion, verified byte-exact offline: MK3 gate 1.000, full-frame
   video hash unchanged over 2,500 frames, SMW/Axelay/Batman Forever
   identical. */
static NO_INLINE uint8_t S9xGetByteSlow(uint32_t Address, uint8_t* GetAddress)
{
   switch ((intptr_t) GetAddress)
   {
   case MAP_PPU:
      return S9xGetPPU(Address & 0xffff);
   case MAP_CPU:
      return S9xGetCPU(Address & 0xffff);
   case MAP_DSP:
      return S9xGetDSP(Address & 0xffff);
   case MAP_SA1RAM:
   case MAP_LOROM_SRAM:
      /*Address & 0x7FFF - offset into bank
       *Address & 0xFF0000 - bank
       *bank >> 1 | offset = s-ram address, unbound
       *unbound & SRAMMask = Sram offset */
      return Memory.SRAM[(((Address & 0xFF0000) >> 1) | (Address & 0x7FFF)) &Memory.SRAMMask];
   case MAP_RONLY_SRAM:
   case MAP_HIROM_SRAM:
      return Memory.SRAM[((Address & 0x7fff) - 0x6000 + ((Address & 0xf0000) >> 3)) & Memory.SRAMMask];
   case MAP_C4:
      return S9xGetC4(Address & 0xffff);
   case MAP_BWRAM:
   case MAP_SPC7110_ROM:
   case MAP_SPC7110_DRAM:
   case MAP_OBC_RAM:
   case MAP_SETA_DSP:
   case MAP_SETA_RISC:
   default:
      return OpenBus;
   }
}

static INLINE uint8_t S9xGetByteBody(uint32_t Address, int32_t block, uint8_t* GetAddress)
{
   if (GetAddress >= (uint8_t*) MAP_LAST)
   {
      if (Memory.MapInfo[block].Type == MAP_TYPE_RAM)
         CPU.WaitAddress = CPU.PCAtOpcodeStart;
      return GetAddress[Address & 0xffff];
   }

   return S9xGetByteSlow(Address, GetAddress);
}

BUS_HOT uint8_t S9xGetByte(uint32_t Address)
{
   int32_t  block = (Address >> MEMMAP_SHIFT) & MEMMAP_MASK;
   uint8_t* GetAddress = Memory.Map [block];
   uint8_t  byte;

   /* Read MapInfo[block] ONCE. Speed and Type share a single byte, but the
      `CPU.WaitAddress` store below sits between the two uses, and the compiler
      cannot prove it does not alias `Memory.MapInfo` - so it reloaded the byte
      afterwards, costing an extra load and ldrb on every fast-path access.
      Nothing on this path can change the map, so one read is correct. The slow
      path keeps re-reading, because a $420D write there really does rewrite
      the speed table (that is MK3 accuracy fix #3). */
   if (GetAddress >= (uint8_t*) MAP_LAST)
   {
      const SMapInfo mi = Memory.MapInfo[block];

      if (mi.Type == MAP_TYPE_RAM)
         CPU.WaitAddress = CPU.PCAtOpcodeStart;
      byte = GetAddress[Address & 0xffff];

      /* A DMA or HDMA owns the bus and charges its own cycles, so the
         accessors must charge nothing at all while one is running. */
      if (!CPU.InDMA)
      {
         CPU.Cycles += mi.Speed;
         S9xDrainEvents();
      }
      return byte;
   }

   byte = S9xGetByteSlow(Address, GetAddress);
   if (!CPU.InDMA)
   {
      CPU.Cycles += Memory.MapInfo[block].Speed;
      S9xDrainEvents();
   }
   return byte;
}

BUS_HOT uint16_t S9xGetWord(uint32_t Address)
{
   if ((Address & 0x0fff) == 0x0fff)
   {
      OpenBus = S9xGetByte(Address);
      return OpenBus | (S9xGetByte(Address + 1) << 8);
   }

   int32_t block = (Address >> MEMMAP_SHIFT) & MEMMAP_MASK;
   uint8_t* GetAddress = Memory.Map[block];

   if ((intptr_t) GetAddress == MAP_PPU && !CPU.InDMA)
   {
      /* Two separate bus cycles. Reading both halves at one cycle count left
         the high byte of a 16-bit register read stale by one access - for
         $2140-3 that means the SPC700's reply is seen a poll late. */
      uint16_t lo = S9xGetByte(Address);
      return lo | (S9xGetByte(Address + 1) << 8);
   }

   if (!CPU.InDMA)
   {
      CPU.Cycles += (Memory.MapInfo[block].Speed << 1);
      S9xDrainEvents();
   }

   if (GetAddress >= (uint8_t*) MAP_LAST)
   {
      if (Memory.MapInfo[block].Type == MAP_TYPE_RAM)
         CPU.WaitAddress = CPU.PCAtOpcodeStart;
#ifdef FAST_LSB_WORD_ACCESS
      return *(uint16_t*) (GetAddress + (Address & 0xffff));
#else
      return *(GetAddress + (Address & 0xffff)) | (*(GetAddress + (Address & 0xffff) + 1) << 8);
#endif
   }

   switch ((intptr_t) GetAddress)
   {
   case MAP_PPU:
      return S9xGetPPU(Address & 0xffff) | (S9xGetPPU((Address + 1) & 0xffff) << 8);
   case MAP_CPU:
      return S9xGetCPU(Address & 0xffff) | (S9xGetCPU((Address + 1) & 0xffff) << 8);
   case MAP_DSP:
      return S9xGetDSP(Address & 0xffff) | (S9xGetDSP((Address + 1) & 0xffff) << 8);
   case MAP_SA1RAM:
   case MAP_LOROM_SRAM:
      /*Address & 0x7FFF - offset into bank
       *Address & 0xFF0000 - bank
       *bank >> 1 | offset = s-ram address, unbound
       *unbound & SRAMMask = Sram offset */
      /* BJ: no FAST_LSB_WORD_ACCESS here, since if Memory.SRAMMask=0x7ff
       * then the high byte doesn't follow the low byte. */
      return *(Memory.SRAM + ((((Address & 0xFF0000) >> 1) | (Address & 0x7FFF)) & Memory.SRAMMask)) | ((*(Memory.SRAM + (((((Address + 1) & 0xFF0000) >> 1) | ((Address + 1) & 0x7FFF)) & Memory.SRAMMask))) << 8);
   case MAP_RONLY_SRAM:
   case MAP_HIROM_SRAM:
      /* BJ: no FAST_LSB_WORD_ACCESS here, since if Memory.SRAMMask=0x7ff
       * then the high byte doesn't follow the low byte. */
      return *(Memory.SRAM + (((Address & 0x7fff) - 0x6000 + ((Address & 0xf0000) >> 3)) & Memory.SRAMMask)) | (*(Memory.SRAM + ((((Address + 1) & 0x7fff) - 0x6000 + (((Address + 1) & 0xf0000) >> 3)) & Memory.SRAMMask)) << 8);
   case MAP_C4:
      return S9xGetC4(Address & 0xffff) | (S9xGetC4((Address + 1) & 0xffff) << 8);
   case MAP_BWRAM:
   case MAP_SPC7110_ROM:
   case MAP_SPC7110_DRAM:
   case MAP_OBC_RAM:
   case MAP_SETA_DSP:
   case MAP_SETA_RISC:
   default:
      return OpenBus | (OpenBus << 8);
   }
}

/* The write reaches the bus BEFORE its cycles are charged.
 *
 * Charging first timestamped a store to $2140-3 twelve cycles later than the
 * hardware drives it. Both processors ran correct instruction streams, but
 * every handshake byte arrived late and MK3's wait-stateless upload loop
 * drifted a whole iteration out of step. */
/* Same split as S9xGetByteSlow, same reason. */
static NO_INLINE void S9xSetByteSlow(uint8_t Byte, uint32_t Address, uint8_t* SetAddress)
{
   switch ((intptr_t) SetAddress)
   {
   case MAP_PPU:
      S9xSetPPU(Byte, Address & 0xffff);
      return;
   case MAP_CPU:
      S9xSetCPU(Byte, Address & 0xffff);
      return;
   case MAP_DSP:
      S9xSetDSP(Byte, Address & 0xffff);
      return;
   case MAP_LOROM_SRAM:
      if (Memory.SRAMMask)
      {
         *(Memory.SRAM + ((((Address & 0xFF0000) >> 1) | (Address & 0x7FFF)) & Memory.SRAMMask)) = Byte;
         CPU.SRAMModified = true;
      }
      return;
   case MAP_HIROM_SRAM:
      if (Memory.SRAMMask)
      {
         *(Memory.SRAM + (((Address & 0x7fff) - 0x6000 + ((Address & 0xf0000) >> 3)) & Memory.SRAMMask)) = Byte;
         CPU.SRAMModified = true;
      }
      return;
   case MAP_BWRAM:
      return;
   case MAP_SA1RAM:
      *(Memory.SRAM + (Address & 0xffff)) = Byte;
      break;
   case MAP_C4:
      S9xSetC4(Byte, Address & 0xffff);
      return;
   case MAP_OBC_RAM:
      SetOBC1(Byte, Address & 0xFFFF);
      return;
   case MAP_SETA_DSP:
   case MAP_SETA_RISC:
   default:
      return;
   }
}

static INLINE void S9xSetByteBody(uint8_t Byte, uint32_t Address, int32_t block, uint8_t* SetAddress)
{
   if (SetAddress >= (uint8_t*) MAP_LAST)
   {
      SetAddress += Address & 0xffff;
      *SetAddress = Byte;
      return;
   }

   S9xSetByteSlow(Byte, Address, SetAddress);
}

BUS_HOT void S9xSetByte(uint8_t Byte, uint32_t Address)
{
   int32_t  block = (Address >> MEMMAP_SHIFT) & MEMMAP_MASK;
   uint8_t* SetAddress = Memory.Map[block];

   /* One read of MapInfo[block], as in S9xGetByte - the `CPU.WaitAddress`
      store between the Type and Speed uses was forcing a reload. Only the
      fast path may reuse it: on the slow path a $420D write really does
      rewrite the speed table (MK3 accuracy fix #3), so that path re-reads. */
   const SMapInfo mi = Memory.MapInfo[block];

   if (mi.Type == MAP_TYPE_ROM)
      SetAddress = (uint8_t*) MAP_NONE;

   CPU.WaitAddress = NULL;

   if (SetAddress >= (uint8_t*) MAP_LAST)
   {
      SetAddress += Address & 0xffff;
      *SetAddress = Byte;

      if (!CPU.InDMA)
      {
         CPU.Cycles += mi.Speed;
         S9xDrainEvents();
      }
      return;
   }

   S9xSetByteSlow(Byte, Address, SetAddress);

   if (!CPU.InDMA)
   {
      CPU.Cycles += Memory.MapInfo[block].Speed;
      S9xDrainEvents();
   }
}

BUS_HOT void S9xSetWord(uint16_t Word, uint32_t Address)
{
   if ((Address & 0x0FFF) == 0x0FFF)
   {
      S9xSetByte(Word & 0x00FF, Address);
      S9xSetByte(Word >> 8, Address + 1);
      return;
   }

   int32_t block = (Address >> MEMMAP_SHIFT) & MEMMAP_MASK;
   uint8_t* SetAddress = Memory.Map[block];

   CPU.WaitAddress = NULL;

   if (Memory.MapInfo[block].Type == MAP_TYPE_ROM)
      SetAddress = (uint8_t*) MAP_NONE;

   if ((intptr_t) SetAddress == MAP_PPU)
   {
      /* A 16-bit store to a register is two separate bus cycles, and the
         second byte is driven one access after the first. */
      S9xSetByte((uint8_t) Word, Address);
      S9xSetByte(Word >> 8, Address + 1);
      return;
   }

   if (!CPU.InDMA)
   {
      CPU.Cycles += Memory.MapInfo[block].Speed << 1;
      S9xDrainEvents();
   }

   if (SetAddress >= (uint8_t*) MAP_LAST)
   {
      SetAddress += Address & 0xffff;
#ifdef FAST_LSB_WORD_ACCESS
      *(uint16_t*)SetAddress = Word;
#else
      *SetAddress = (uint8_t) Word;
      *(SetAddress + 1) = Word >> 8;
#endif
      return;
   }

   switch ((intptr_t) SetAddress)
   {
   case MAP_PPU:
      S9xSetPPU((uint8_t) Word, Address & 0xffff);
      S9xSetPPU(Word >> 8, (Address & 0xffff) + 1);
      return;
   case MAP_CPU:
      S9xSetCPU((uint8_t) Word, Address & 0xffff);
      S9xSetCPU(Word >> 8, (Address & 0xffff) + 1);
      return;
   case MAP_DSP:
      S9xSetDSP((uint8_t) Word, Address & 0xffff);
      S9xSetDSP(Word >> 8, (Address & 0xffff) + 1);
      return;
   case MAP_LOROM_SRAM:
      if (Memory.SRAMMask)
      {
         /* BJ: no FAST_LSB_WORD_ACCESS here, since if Memory.SRAMMask=0x7ff
          * then the high byte doesn't follow the low byte. */
         *(Memory.SRAM + ((((Address & 0xFF0000) >> 1) | (Address & 0x7FFF)) & Memory.SRAMMask)) = (uint8_t) Word;
         *(Memory.SRAM + (((((Address + 1) & 0xFF0000) >> 1) | ((Address + 1) & 0x7FFF))& Memory.SRAMMask)) = Word >> 8;
         CPU.SRAMModified = true;
      }
      return;
   case MAP_HIROM_SRAM:
      if (Memory.SRAMMask)
      {
         /* BJ: no FAST_LSB_WORD_ACCESS here, since if Memory.SRAMMask=0x7ff
          * then the high byte doesn't follow the low byte. */
         *(Memory.SRAM + (((((Address & 0x7fff) - 0x6000) + ((Address & 0xf0000) >> 3)) & Memory.SRAMMask))) = (uint8_t) Word;
         *(Memory.SRAM + ((((((Address + 1) & 0x7fff) - 0x6000) + (((Address + 1) & 0xf0000) >> 3)) & Memory.SRAMMask))) = (uint8_t)(Word >> 8);
         CPU.SRAMModified = true;
      }
      return;
   case MAP_BWRAM:
      return;
   case MAP_SA1RAM:
      *(Memory.SRAM + (Address & 0xffff)) = (uint8_t) Word;
      *(Memory.SRAM + ((Address + 1) & 0xffff)) = (uint8_t)(Word >> 8);
      break;
   case MAP_C4:
      S9xSetC4(Word & 0xff, Address & 0xffff);
      S9xSetC4((uint8_t)(Word >> 8), (Address + 1) & 0xffff);
      return;
   case MAP_OBC_RAM:
      SetOBC1(Word & 0xff, Address & 0xFFFF);
      SetOBC1((uint8_t)(Word >> 8), (Address + 1) & 0xffff);
      return;
   case MAP_SETA_DSP:
   case MAP_SETA_RISC:
   default:
      return;
   }
}

uint8_t* GetBasePointer(uint32_t Address)
{
   uint8_t* GetAddress = Memory.Map [(Address >> MEMMAP_SHIFT) & MEMMAP_MASK];
   if (GetAddress >= (uint8_t*) MAP_LAST)
      return GetAddress;
   switch ((intptr_t) GetAddress)
   {
   case MAP_PPU: /*just a guess, but it looks like this should match the CPU as a source. */
   case MAP_CPU: /*fixes Ogre Battle's green lines */
   case MAP_OBC_RAM:
      return Memory.FillRAM;
   case MAP_DSP:
      return Memory.FillRAM - 0x6000;
   case MAP_SA1RAM:
   case MAP_LOROM_SRAM:
   case MAP_SETA_DSP:
      return Memory.SRAM;
   case MAP_BWRAM:
      return NULL;
   case MAP_HIROM_SRAM:
      return Memory.SRAM - 0x6000;
   case MAP_C4:
      return Memory.C4RAM - 0x6000;
   default:
      return NULL;
   }
}

uint8_t* S9xGetMemPointer(uint32_t Address)
{
   uint8_t* GetAddress = Memory.Map [(Address >> MEMMAP_SHIFT) & MEMMAP_MASK];
   if (GetAddress >= (uint8_t*) MAP_LAST)
      return GetAddress + (Address & 0xffff);

   switch ((intptr_t) GetAddress)
   {
   case MAP_PPU:
      return Memory.FillRAM + (Address & 0xffff);
   case MAP_CPU:
      return Memory.FillRAM + (Address & 0xffff);
   case MAP_DSP:
      return Memory.FillRAM - 0x6000 + (Address & 0xffff);
   case MAP_SA1RAM:
   case MAP_LOROM_SRAM:
      return Memory.SRAM + (Address & 0xffff);
   case MAP_BWRAM:
      return NULL;
   case MAP_HIROM_SRAM:
      return Memory.SRAM - 0x6000 + (Address & 0xffff);
   case MAP_C4:
      return Memory.C4RAM - 0x6000 + (Address & 0xffff);
   case MAP_OBC_RAM:
      return GetMemPointerOBC1(Address);
   case MAP_SETA_DSP:
      return Memory.SRAM + ((Address & 0xffff) & Memory.SRAMMask);
   default:
      return NULL;
   }
}

void S9xSetPCBase(uint32_t Address)
{
   int32_t block = (Address >> MEMMAP_SHIFT) & MEMMAP_MASK;
   uint8_t* GetAddress = Memory.Map [block];
   CPU.MemSpeed = Memory.MapInfo[block].Speed;
   CPU.MemSpeedx2 = CPU.MemSpeed << 1;

   if (GetAddress >= (uint8_t*) MAP_LAST)
      CPU.PCBase = GetAddress;
   else
   {
      switch ((intptr_t) GetAddress)
      {
      case MAP_PPU:
      case MAP_CPU:
         CPU.PCBase = Memory.FillRAM;
         break;
      case MAP_DSP:
         CPU.PCBase = Memory.FillRAM - 0x6000;
         break;
      // case MAP_BWRAM:
      //    CPU.PCBase = Memory.BWRAM - 0x6000;
      //    break;
      case MAP_HIROM_SRAM:
         CPU.PCBase = Memory.SRAM - 0x6000;
         break;
      case MAP_C4:
         CPU.PCBase = Memory.C4RAM - 0x6000;
         break;
      default:
         CPU.PCBase = Memory.SRAM;
         break;
      }
   }

   CPU.PC = CPU.PCBase + (Address & 0xffff);
}
