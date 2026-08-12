/* This file is part of Snes9x. See LICENSE file. */

#ifndef _SNES9X_H_
#define _SNES9X_H_

#include <stdlib.h>
#include <stdint.h>

#include "port.h"
#include "65c816.h"

#define ROM_NAME_LEN 22

/* SNES screen width and height */
#define SNES_WIDTH            256
#define SNES_HEIGHT           224
#define SNES_HEIGHT_EXTENDED  239

#define SNES_SPRITE_TILE_PER_LINE 34

#define SNES_MAX_NTSC_VCOUNTER  262
#define SNES_MAX_PAL_VCOUNTER   312
#define SNES_HCOUNTER_MAX       341
#define SPC700_TO_65C816_RATIO  2
#define AUTO_FRAMERATE          200

/* NTSC master clock signal 21.47727MHz
 * PPU: master clock / 4
 * 1 / PPU clock * 342 -> 63.695us
 * 63.695us / (1 / 3.579545MHz) -> 228 cycles per scanline
 * From Earth Worm Jim: APU executes an average of 65.14285714 cycles per
 * scanline giving an APU clock speed of 1.022731096MHz                    */

/* PAL master clock signal 21.28137MHz
 * PPU: master clock / 4
 * 1 / PPU clock * 342 -> 64.281us
 * 64.281us / (1 / 3.546895MHz) -> 228 cycles per scanline.  */

#define SNES_SCANLINE_TIME (63.695e-6)
#define SNES_CLOCK_SPEED   (3579545u)

#define SNES_CLOCK_LEN (1.0 / SNES_CLOCK_SPEED)

/* The old expression evaluated to 1368: 63.695us / (1/3579545) is 227.999 CPU
 * cycles, rounded up to 228 and multiplied by six. The hardware scanline is
 * 1364 master cycles (341 dots of four), and every event position below is
 * quoted in those units. */
#define SNES_CYCLES_PER_SCANLINE 1364

#ifdef SNES_OVERCLOCK_CYCLES
#define ONE_CYCLE        (overclock_cycles ? one_c : 6u)
#define SLOW_ONE_CYCLE   (overclock_cycles ? slow_one_c : 8u)
#define TWO_CYCLES       (overclock_cycles ? two_c : 12u)
#else
#define ONE_CYCLE        (6u)
#define SLOW_ONE_CYCLE   (8u)
#define TWO_CYCLES       (12u)
#endif

#define SNES_TR_MASK     (1u << 4)
#define SNES_TL_MASK     (1u << 5)
#define SNES_X_MASK      (1u << 6)
#define SNES_A_MASK      (1u << 7)
#define SNES_RIGHT_MASK  (1u << 8)
#define SNES_LEFT_MASK   (1u << 9)
#define SNES_DOWN_MASK   (1u << 10)
#define SNES_UP_MASK     (1u << 11)
#define SNES_START_MASK  (1u << 12)
#define SNES_SELECT_MASK (1u << 13)
#define SNES_Y_MASK      (1u << 14)
#define SNES_B_MASK      (1u << 15)

extern bool overclock_cycles;
extern int one_c, slow_one_c, two_c;

enum
{
   SNES_MULTIPLAYER5,
   SNES_JOYPAD,
   SNES_MOUSE,
   SNES_SUPERSCOPE,
   SNES_JUSTIFIER,
   SNES_JUSTIFIER_2,
   SNES_MAX_CONTROLLER_OPTIONS
};

#define DEBUG_MODE_FLAG    (1u << 0)
#define TRACE_FLAG         (1u << 1)
#define SINGLE_STEP_FLAG   (1u << 2)
#define BREAK_FLAG         (1u << 3)
#define SCAN_KEYS_FLAG     (1u << 4)
#define SAVE_SNAPSHOT_FLAG (1u << 5)
#define DELAYED_NMI_FLAG   (1u << 6)
#define NMI_FLAG           (1u << 7)
#define PROCESS_SOUND_FLAG (1u << 8)
#define FRAME_ADVANCE_FLAG (1u << 9)
#define DELAYED_NMI_FLAG2  (1u << 10)
#define IRQ_PENDING_FLAG   (1u << 11)

typedef struct
{
   uint32_t Flags;
   bool     BranchSkip;
   bool     NMIActive;
   uint8_t  IRQActive;
   bool     WaitingForInterrupt;
   bool     InDMA;
   uint8_t  WhichEvent;
   uint8_t* PC;
   uint8_t* PCBase;
   uint8_t* PCAtOpcodeStart;
   uint8_t* WaitAddress;
   uint32_t WaitCounter;
   long     Cycles;       /* For savestate compatibility can't change to int32_t */
   long     NextEvent;    /* For savestate compatibility can't change to int32_t */
   long     V_Counter;    /* For savestate compatibility can't change to int32_t */
   long     MemSpeed;     /* For savestate compatibility can't change to int32_t */
   long     MemSpeedx2;   /* For savestate compatibility can't change to int32_t */
   long     FastROMSpeed; /* For savestate compatibility can't change to int32_t */
   uint32_t SaveStateVersion;
   bool     SRAMModified;
   uint32_t NMITriggerPoint;
   bool     UNUSED2;
   bool     TriedInterleavedMode2;
   uint32_t NMICycleCount;
   uint32_t IRQCycleCount;
} SCPUState;

#define ONE_DOT_CYCLE       4
#define ONE_DOT_CYCLE_DIV_2 2

/* Event positions within a scanline, in master cycles. */
#define SNES_HBLANK_END_HC     4     /* H=1   */
#define SNES_HDMA_INIT_HC      20
#define SNES_RENDER_START_HC   192   /* H=48; this core renders a line at once */
#define SNES_WRAM_REFRESH_HC_v2                   538
#define SNES_WRAM_REFRESH_HC_v2_MIN_ONE_DOT_CYCLE 534
#define SNES_HBLANK_START_HC   1096  /* H=274 */
#define SNES_HDMA_START_HC     1106

/* The CPU is held off the bus while WRAM is refreshed, once per scanline.
 * This core never modelled it, handing the 65816 2.9% of every line that the
 * hardware does not - and that the SPC700, clocked separately, never got. */
#define SNES_WRAM_REFRESH_CYCLES 40

/* Six events per scanline, in the order they occur:
 *
 *   HDMA_INIT 20 -> RENDER 192 -> WRAM_REFRESH 534/538 -> HBLANK_START 1096
 *      -> HDMA_START 1106 -> HCOUNTER_MAX 1364 (rebase, next line)
 *
 * There used to be two real positions, HBlankStart and H_Max, with every
 * per-line action crowded onto the latter: the line was rendered, HDMA was
 * started, VBlank began and the APU was resynchronised all at the same
 * instant, at the END of the line.
 *
 * Each base event has an IRQ_x_y twin, scheduled in its place when the
 * programmable H/V timer falls between the two surrounding events. */
#define HC_HBLANK_START_EVENT  1u
#define HC_IRQ_1_3_EVENT       2u
#define HC_HDMA_START_EVENT    3u
#define HC_IRQ_3_5_EVENT       4u
#define HC_HCOUNTER_MAX_EVENT  5u
#define HC_IRQ_5_7_EVENT       6u
#define HC_HDMA_INIT_EVENT     7u
#define HC_IRQ_7_9_EVENT       8u
#define HC_RENDER_EVENT        9u
#define HC_IRQ_9_A_EVENT      10u
#define HC_WRAM_REFRESH_EVENT 11u
#define HC_IRQ_A_1_EVENT      12u

/* Scanline geometry and event positions.
 *
 * Deliberately NOT part of SCPUState: that struct is written to savestates
 * raw and its byte offsets are hardcoded in cpu_asm.S, so it must not grow. */
struct STimings
{
   int32_t H_Max_Master;
   int32_t H_Max;
   int32_t V_Max_Master;
   int32_t V_Max;
   int32_t HBlankStart;
   int32_t HBlankEnd;
   int32_t HDMAInit;
   int32_t HDMAStart;
   int32_t RenderPos;
   int32_t WRAMRefreshPos;
   int32_t DMACPUSync;       /* cycles to resync the CPU to the DMA clock */
   int32_t IRQTriggerCycles; /* comparator -> /IRQ -> taken */
   int32_t NextIRQTimer;     /* absolute cycle deadline of the H/V timer */
   bool    InterlaceField;
};

extern struct STimings Timings;
/* The scanline a timer IRQ actually lands on - a free-standing global rather
   than a PPU field, because PPU is serialised raw into savestates too. */
extern int16_t S9xVTimerPosition;

void S9xInitTimings(void);

typedef struct
{
   /* CPU options */
   bool     APUEnabled;
   bool     Shutdown;
   int32_t  H_Max;
   int32_t  HBlankStart;
   int32_t  CyclesPercentage;
   bool     DisableIRQ;

   /* Joystick options */
   bool     JoystickEnabled;

   /* ROM timing options (see also H_Max above) */
   bool     ForcePAL;
   bool     ForceNTSC;
   bool     PAL;
   uint32_t FrameTimePAL;
   uint32_t FrameTimeNTSC;
   uint32_t FrameTime;

   /* ROM image options */
   bool     ForceLoROM;
   bool     ForceHiROM;
   bool     ForceHeader;
   bool     ForceNoHeader;
   bool     ForceInterleaved;
   bool     ForceInterleaved2;
   bool     ForceNotInterleaved;

   /* Peripheral options */
   bool     ForceSuperFX;
   bool     ForceNoSuperFX;
   bool     ForceDSP1;
   bool     ForceNoDSP1;
   bool     ForceSA1;
   bool     ForceNoSA1;
   bool     ForceC4;
   bool     ForceNoC4;
   bool     ForceSDD1;
   bool     ForceNoSDD1;
   bool     MultiPlayer5;
   bool     Mouse;
   bool     SuperScope;
   bool     SRTC;
   uint32_t ControllerOption;
   bool     MultiPlayer5Master;
   bool     SuperScopeMaster;
   bool     MouseMaster;
   uint8_t  MousePort;            /* 0 = controller port 1, 1 = controller port 2 */

   bool     SuperFX;
   bool     DSP1Master;
   bool     SA1;
   bool     C4;
   bool     SDD1;
   bool     SPC7110;
   bool     SPC7110RTC;
   bool     OBC1;
   uint8_t  DSP;

   /* Sound options */
   uint32_t SoundPlaybackRate;
   uint32_t SoundInputRate;
   bool     TraceSoundDSP;
   bool     EightBitConsoleSound; /* due to caching, this needs S9xSetEightBitConsoleSound() */
   int32_t  SoundBufferSize;
   int32_t  SoundMixInterval;
   bool     SoundEnvelopeHeightReading;
   bool     DisableSoundEcho;
   bool     DisableMasterVolume;
   bool     SoundSync;
   bool     InterpolatedSound;
   bool     ThreadSound;
   bool     Mute;
   bool     NextAPUEnabled;

   /* Others */
   bool     ApplyCheats;

   /* Fixes for individual games */
   bool     StarfoxHack;
   bool     WinterGold;
   bool     BS; /* Japanese Satellite System games. */
   bool     JustifierMaster;
   bool     Justifier;
   bool     SecondJustifier;
   int8_t   SETA;
   bool     HardDisableAudio;
} SSettings;

extern SSettings Settings;
extern SCPUState CPU;
extern char String [513];

void S9xSetPause(uint32_t mask);
void S9xClearPause(uint32_t mask);
#endif
