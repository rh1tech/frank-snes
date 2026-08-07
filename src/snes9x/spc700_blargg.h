/*
 * spc700_blargg.h — accurate SPC700, from blargg's SNES_SPC.
 *
 * Copyright (C) 2004-2007 Shay Green. LGPL 2.1 or later; see the notice
 * in spc700_blargg.c.
 *
 * Used only where SOUND_CORE=DSP is selected, which is C2. M1 and M2
 * keep the legacy spc700.c / soundux.c path they have always shipped.
 */
#ifndef SPC700_BLARGG_H
#define SPC700_BLARGG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define REG_COUNT    0x10
#define PORT_COUNT   4
#define TEMPO_UNIT   0x100
#define TIMER_COUNT  3
#define ROM_SIZE     0x40
#define ROM_ADDR     0xFFC0
#define EXTRA_SIZE        16
#define EXTRA_SIZE_DIV_2  8
#define STATE_SIZE   (68 * 1024L)

/* 1024000 SPC clocks per second, one sample pair every 32 clocks. */
#define CLOCKS_PER_SAMPLE 32

typedef struct
{
   int32_t next_time;   /* time of next event */
   int32_t prescaler;
   int32_t period;
   int32_t divider;
   int32_t enabled;
   int32_t counter;
} Timer;

typedef struct
{
   Timer    timers [TIMER_COUNT];
   uint8_t  smp_regs [2] [REG_COUNT];

   struct
   {
      int32_t pc, a, x, y, psw, sp;
   } cpu_regs;

   int32_t  dsp_time;
   int32_t  spc_time;
   int32_t  tempo;
   int32_t  extra_clocks;
   int16_t* buf_begin;
   int16_t* buf_end;
   int16_t* extra_pos;
   int16_t  extra_buf [EXTRA_SIZE];
   int32_t  rom_enabled;
   uint8_t  rom    [ROM_SIZE];
   uint8_t  hi_ram [ROM_SIZE];
   uint8_t  cycle_table [256];

   struct
   {
      /* padding to neutralise address overflow */
      union
      {
         uint8_t  padding1 [0x100];
         uint16_t align;
      } padding1 [1];
      uint8_t ram      [0x10000];
      uint8_t padding2 [0x100];
   } ram;
} spc_state_t;

/* Samples written to the output since it was last set. */
#define SPC_SAMPLE_COUNT() ((m.extra_clocks >> 5) * 2)

/* ---- What the emulator calls ---- */

bool    S9xInitAPU(void);
void    S9xDeinitAPU(void);
void    S9xResetAPU(void);
void    S9xSoftResetAPU(void);

/* ppu.c passes the full address; the port index is masked out here so
 * that file needs no #ifdef. */
uint8_t S9xAPUReadPort(int32_t address);
void    S9xAPUWritePort(int32_t address, uint8_t byte);

/* Run the SPC700 up to the 65816's current cycle, then rebase. Called
 * from cpuexec.c at the end of every scanline. */
void    S9xAPUExecute(void);
void    S9xAPUSetReferenceTime(int32_t cpucycles);

/* The 64 KB the DSP reads BRR data out of. */
uint8_t *spc_apuram(void);

/* Region/tempo, set when a ROM loads. */
void    S9xAPUTimingSetSpeedup(int32_t ticks);
void    S9xAPUAllowTimeOverflow(bool allow);

/* Sample output. The DSP writes stereo pairs straight into this. */
void    S9xAPUSetOutput(int16_t *out, int32_t size);
int32_t S9xAPUSamplesWritten(void);
void    S9xAPUResetOutputCount(void);

/* SPC700 state for savestates: one block, no internal pointers. */
void   *S9xAPUStateBlock(void);
size_t  S9xAPUStateSize(void);

#endif /* SPC700_BLARGG_H */
