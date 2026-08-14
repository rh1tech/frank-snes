/*
 * frank-snes — C2 slave: the CPU-side functions the renderer references
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * The slave links ppu.c for S9xSetPPU, which is the register decode the
 * replayed stream drives. That file's switch also covers the registers the
 * 65816 owns — the APU ports at $2140-3, the CPU registers at $4200-$43ff,
 * DMA, the SuperFX and the S-RTC — so the linker wants those functions even
 * though the slave can never reach them.
 *
 * It cannot reach them because the master's capture filters the stream to
 * $2100..$213f: everything here is outside that window by construction. See
 * ppu_capture.h, where that mask is the reason the filter is 0xffc0 and not
 * 0xff40.
 *
 * So these are unreachable rather than unimplemented, and they are written to
 * make that loud: if the stream ever does carry one, the counter below moves
 * and the slave reports it, instead of silently returning a plausible zero
 * and rendering a subtly wrong frame for the rest of the session.
 */
#include <stdint.h>
#include "snes9x.h"

volatile uint32_t slave_ppu_impossible;   /* read over the link / UART */

static void impossible(void) { slave_ppu_impossible++; }

void    FixROMSpeed(void)                        { impossible(); }
void    fx_dirtySCBR(void)                       { impossible(); }
void    fx_updateRamBank(uint8_t Byte)           { (void)Byte; impossible(); }
int32_t FxEmulate(uint32_t n)                    { (void)n; impossible(); return 0; }
void    FxFlushCache(void)                       { impossible(); }
uint8_t S9xAPUReadPort(int32_t a)                { (void)a; impossible(); return 0; }
void    S9xAPUWritePort(int32_t a, uint8_t b)    { (void)a; (void)b; impossible(); }
void    S9xDoDMA(uint8_t c)                      { (void)c; impossible(); }
uint8_t S9xGetSRTC(uint16_t a)                   { (void)a; impossible(); return 0; }
void    S9xSetIRQ(uint32_t s)                    { (void)s; impossible(); }
void    S9xSetPCBase(uint32_t a)                 { (void)a; impossible(); }
void    S9xSetSRTC(uint8_t d, uint16_t a)        { (void)d; (void)a; impossible(); }

/* Pulled in by S9xResetPPU. There is no input path on the slave at all - the
   master owns input entirely. */
bool S9xReadMousePosition(int32_t which, int32_t *x, int32_t *y, uint32_t *buttons)
{
   (void)which; (void)x; (void)y; (void)buttons;
   impossible();
   return false;
}

/* The master counts APU port reads in ppu.c to tell "the game stopped
   advancing" from "the game is waiting on the sound driver". The slave links
   the same ppu.c for its renderer but not ppu_capture.c, where the master
   defines these - so they need a home here too. Nothing on this chip reads
   them; they exist to satisfy the shared translation unit. */
volatile uint32_t frank_dbg_apu_reads;
volatile uint32_t frank_dbg_apu_last;
