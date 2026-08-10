/* This file is part of Snes9x. See LICENSE file. */

#ifndef USE_BLARGG_APU

// Use PSRAM allocator on device - MUST be before any stdlib includes
#ifdef PICO_ON_DEVICE
#include "snes_alloc.h"
#define malloc snes_malloc
#define calloc snes_calloc
#define free snes_free
#endif

#include <stdlib.h>
#include <string.h>
#include <errno.h>


#include "snes9x.h"
#include "soundux.h"
#include "apu.h"
#include "memmap.h"
#include "cpuexec.h"

// Step 2: Include audio optimizations
#ifdef PICO_ON_DEVICE
#include "../audio_opt.h"
#endif

#define CLIP16(v) \
(v) = (((v) <= -32768) ? -32768 : (((v) >= 32767) ? 32767 : (v)))

#define CLIP8(v) \
(v) = (((v) <= -128) ? -128 : (((v) >= 127) ? 127 : (v)))

/* Echo[24000] (96KB) lives in PSRAM — only touched when SoundData.echo_enable,
 * and at most once per output sample. Keeps SRAM bss under 512KB budget. */
static int32_t *g_echo_buffer = NULL;

static struct LocalStateStruct {
   int32_t wave[SOUND_BUFFER_SIZE];
   int32_t MixBuffer [SOUND_BUFFER_SIZE];
   int32_t EchoBuffer [SOUND_BUFFER_SIZE];
   int32_t FilterTaps [8];
   uint8_t FilterTapDefinitionBitfield;
   /* In the above, bit I is set if FilterTaps[I] is non-zero. */
   uint32_t Z;
   int32_t Loop [16];

   /* precalculated env rates for S9xSetEnvRate */
   uint32_t AttackERate     [16][10];
   uint32_t DecayERate       [8][10];
   uint32_t SustainERate    [32][10];
   uint32_t IncreaseERate   [32][10];
   uint32_t DecreaseERateExp[32][10];
   uint32_t KeyOffERate         [10];

   /* Used by S9xMixSamplesLowPass() to store the
   * last output samples for the next iteration
   * of the low pass filter. This should go in
   * the SSoundData struct, but doing so would
   * break compatibility with existing save
   * states (with little in the way of tangible
   * benefits) */
   int32_t MixOutputPrev[2];
} LocalStateStorage;
static struct LocalStateStruct *LocalState = &LocalStateStorage;

// Optional kill switch for SNES noise generator (vinyl-like hiss/scratch)
volatile uint32_t kon_total, kon_same_sample;
/*
 * The loop-point directory read is taken at MIX time, i.e. at the end of
 * the frame, long after the voice was keyed. MK3 rewrites one directory
 * entry per voice, so the entry can have moved on to the next sound by
 * then and the voice loops to the wrong sample.
 *
 * loop_dir_checked counts loop points reached; loop_dir_stale counts those
 * where the directory no longer holds what it held at key-on.
 */
volatile uint32_t loop_dir_checked, loop_dir_stale;
static uint16_t kon_loop_addr[8];
static uint8_t  kon_loop_valid;
/* A music driver legitimately re-keys the same instrument on the same voice
 * for every note, so "same sample again" says nothing on its own. What is
 * not legitimate is the same voice re-keying the same sample within a frame
 * or two — far faster than any note. Those are the spurious re-triggers a
 * listener hears as a repeat. */
/* Ring log of every voice start: which sample, on which voice, when.
 * 512 entries at 8 bytes is 4 KB, enough for ~40 s of dense play. */
#ifdef FRANK_SNES_KONLOG
typedef struct { uint32_t frame; uint16_t start; uint8_t ch; uint8_t srcn; } konlog_t;
#define KONLOG_N 512
volatile konlog_t konlog[KONLOG_N];
volatile uint32_t konlog_w;          /* total appends; index = w % KONLOG_N */
#endif
volatile uint32_t kon_retrig_same_frame;
volatile uint32_t kon_retrig_le2_frames;
volatile uint32_t soundux_frame;

static bool g_disable_noise = false;
// Per-channel mute mask for debugging; bit i mutes channel i (0..7)
static uint8_t g_channel_mute_mask = 0x00; // 0 = all channels enabled

#define wave LocalState->wave
#define Echo g_echo_buffer
#define MixBuffer LocalState->MixBuffer
#define EchoBuffer LocalState->EchoBuffer
#define FilterTaps LocalState->FilterTaps
#define FilterTapDefinitionBitfield LocalState->FilterTapDefinitionBitfield
#define Z LocalState->Z
#define Loop LocalState->Loop
#define AttackERate LocalState->AttackERate
#define DecayERate LocalState->DecayERate
#define SustainERate LocalState->SustainERate
#define IncreaseERate LocalState->IncreaseERate
#define DecreaseERateExp LocalState->DecreaseERateExp
#define KeyOffERate LocalState->KeyOffERate
#define MixOutputPrev LocalState->MixOutputPrev

extern int32_t NoiseFreq [32];

/* Cycle-accurate KON/KOFF event queue */
DSPEvent dsp_events[DSP_EVENT_MAX];
uint8_t  dsp_event_count = 0;
int32_t  dsp_frame_start_cycle = 0;

void S9xDSPQueueEvent(uint8_t type, uint8_t data, int32_t cycle)
{
   if (dsp_event_count < DSP_EVENT_MAX)
   {
      dsp_events[dsp_event_count].type  = type;
      dsp_events[dsp_event_count].data  = data;
      dsp_events[dsp_event_count].cycle = cycle;
      dsp_event_count++;
   }
}

void S9xDSPSetFrameStart(int32_t cycle)
{
   dsp_frame_start_cycle = cycle;
   dsp_event_count = 0;
}

/* SFX auto-release state */
static uint8_t  sfx_btn_pending = 0;    /* frames since last button press */
static uint8_t  sfx_channel_mask = 0;   /* channels KON'd during button window */
static uint8_t  sfx_timer[8] = {0};     /* per-channel frames since SFX KON */
static uint8_t  sfx_killed_srcn[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static uint8_t  sfx_killed_ttl[8] = {0}; /* frames to keep remembering killed srcn */
#define SFX_KILLED_MEMORY 60  /* remember killed srcn for 1 second */

void S9xNotifyButtonPress(void)
{
   /* Disabled: SFX auto-release was incorrectly killing music notes
    * in games like Prince of Persia. Real SNES has no such mechanism. */
}

void S9xSFXAutoReleaseTick(void)
{
   /* Disabled: SFX auto-release was killing music notes. */
}

/* Disabled: was killing music loops. */
void S9xSFXLoopRelease(int channel)
{
   (void)channel;
}

/* Called from apu.c when KON happens — disabled (was killing music notes). */
void S9xSFXCheckKON(int channel)
{
   (void)channel;
}

static uint32_t AttackRate [16] =
{
   4100, 2600, 1500, 1000, 640, 380, 260, 160,
   96,   64,   40,   24,   16,  10,  6,   1
};

static uint32_t DecayRate [8] =
{
   1200, 740, 440, 290, 180, 110, 74, 37
};

static uint32_t DecreaseRateExp [32] =
{
   0xFFFFFFFF, 38000, 28000, 24000, 19000, 14000, 12000, 9400,
   7100,       5900,  4700,  3500,  2900,  2400,  1800,  1500,
   1200,       880,   740,   590,   440,   370,   290,   220,
   180,        150,   110,   92,    74,    55,    37,    18
};

static uint32_t IncreaseRate [32] =
{
   0xFFFFFFFF, 4100, 3100, 2600, 2000, 1500, 1300, 1000,
   770,        640,  510,  380,  320,  260,  190,  160,
   130,        96,   80,   64,   48,   40,   32,   24,
   20,         16,   12,   10,   8,    6,    4,    2
};

#define SustainRate DecreaseRateExp

#define FIXED_POINT 0x10000UL
#define FIXED_POINT_REMAINDER 0xffffUL
#define FIXED_POINT_SHIFT 16

#define VOL_DIV16 0x0080
#define ENVX_SHIFT 24

/* Gaussian interpolation table — from real SNES DSP (512 entries, mirrored).
 * NOT const: on RP2350, const goes to flash/XIP which can glitch at high
 * clock speeds. ~1KB in RAM is acceptable. */
static int16_t gauss_table [512] =
{
   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,
   2,   2,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,   5,   5,   5,   5,
   6,   6,   6,   6,   7,   7,   7,   8,   8,   8,   9,   9,   9,  10,  10,  10,
  11,  11,  11,  12,  12,  13,  13,  14,  14,  15,  15,  15,  16,  16,  17,  17,
  18,  19,  19,  20,  20,  21,  21,  22,  23,  23,  24,  24,  25,  26,  27,  27,
  28,  29,  29,  30,  31,  32,  32,  33,  34,  35,  36,  36,  37,  38,  39,  40,
  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,
  58,  59,  60,  61,  62,  64,  65,  66,  67,  69,  70,  71,  73,  74,  76,  77,
  78,  80,  81,  83,  84,  86,  87,  89,  90,  92,  94,  95,  97,  99, 100, 102,
 104, 106, 107, 109, 111, 113, 115, 117, 118, 120, 122, 124, 126, 128, 130, 132,
 134, 137, 139, 141, 143, 145, 147, 150, 152, 154, 156, 159, 161, 163, 166, 168,
 171, 173, 175, 178, 180, 183, 186, 188, 191, 193, 196, 199, 201, 204, 207, 210,
 212, 215, 218, 221, 224, 227, 230, 233, 236, 239, 242, 245, 248, 251, 254, 257,
 260, 263, 267, 270, 273, 276, 280, 283, 286, 290, 293, 297, 300, 304, 307, 311,
 314, 318, 321, 325, 328, 332, 336, 339, 343, 347, 351, 354, 358, 362, 366, 370,
 374, 378, 381, 385, 389, 393, 397, 401, 405, 410, 414, 418, 422, 426, 430, 434,
 439, 443, 447, 451, 456, 460, 464, 469, 473, 477, 482, 486, 491, 495, 499, 504,
 508, 513, 517, 522, 527, 531, 536, 540, 545, 550, 554, 559, 563, 568, 573, 577,
 582, 587, 592, 596, 601, 606, 611, 615, 620, 625, 630, 635, 640, 644, 649, 654,
 659, 664, 669, 674, 678, 683, 688, 693, 698, 703, 708, 713, 718, 723, 728, 732,
 737, 742, 747, 752, 757, 762, 767, 772, 777, 782, 787, 792, 797, 802, 806, 811,
 816, 821, 826, 831, 836, 841, 846, 851, 855, 860, 865, 870, 875, 880, 884, 889,
 894, 899, 904, 908, 913, 918, 923, 927, 932, 937, 941, 946, 951, 955, 960, 965,
 969, 974, 978, 983, 988, 992, 997,1001,1005,1010,1014,1019,1023,1027,1032,1036,
1040,1045,1049,1053,1057,1061,1066,1070,1074,1078,1082,1086,1090,1094,1098,1102,
1106,1109,1113,1117,1121,1125,1128,1132,1136,1139,1143,1146,1150,1153,1157,1160,
1164,1167,1170,1174,1177,1180,1183,1186,1190,1193,1196,1199,1202,1205,1207,1210,
1213,1216,1219,1221,1224,1227,1229,1232,1234,1237,1239,1241,1244,1246,1248,1251,
1253,1255,1257,1259,1261,1263,1265,1267,1269,1270,1272,1274,1275,1277,1279,1280,
1282,1283,1284,1286,1287,1288,1290,1291,1292,1293,1294,1295,1296,1297,1297,1298,
1299,1300,1300,1301,1302,1302,1303,1303,1303,1304,1304,1304,1304,1304,1305,1305,
};

static INLINE int16_t gauss_interpolate(const int16_t *buf, uint32_t frac)
{
   /* Linear interpolation: lerp between buf[2] (current) and buf[3] (next)
    * using the fractional position.  Lighter than 4-point Gaussian but still
    * removes the stair-step artifacts of nearest-sample. */
   int32_t f = (frac >> 1) & 0x7FFF;           /* 0..32767 */
   int32_t out = buf[2] + ((f * (buf[3] - buf[2])) >> 15);
   if (out < -32768) out = -32768;
   else if (out > 32767) out = 32767;
   return (int16_t) out;
}

/* F is channel's current frequency and M is the 16-bit modulation waveform
 * from the previous channel multiplied by the current envelope volume level. */
#define PITCH_MOD(F,M) ((F) * ((((uint32_t) (M)) + 0x800000) >> 16) >> 7)

#define LAST_SAMPLE 0xffffff
#define JUST_PLAYED_LAST_SAMPLE(c) ((c)->sample_pointer >= LAST_SAMPLE)

static INLINE uint8_t* S9xGetSampleAddress(int32_t sample_number)
{
   uint32_t addr = (((APU.DSP[APU_DIR] << 8) + (sample_number << 2)) & 0xffff);
   return (IAPU.RAM + addr);
}

void S9xAPUSetEndOfSample(int32_t i, Channel* ch)
{
   ch->state = SOUND_SILENT;
   ch->mode = MODE_NONE;
   APU.DSP [APU_ENDX] |= 1 << i;
   APU.DSP [APU_KON] &= ~(1 << i);
   APU.DSP [APU_KOFF] &= ~(1 << i);
   APU.KeyedChannels &= ~(1 << i);
}

void S9xAPUSetEndX(int32_t ch)
{
   APU.DSP [APU_ENDX] |= 1 << ch;
}

void S9xSetEnvRate(Channel* ch, uint32_t rate, int32_t direction, int32_t target, uint32_t mode)
{
   ch->envx_target = target;

   if (rate == ~((uint32_t) 0u))
   {
      ch->direction = 0;
      rate = 0;
   }
   else
      ch->direction = direction;

   if (rate == 0 || so.playback_rate == 0)
      ch->erate = 0;
   else
   {
      switch (mode >> 28)
      {
      case 0: /* Attack */
         ch->erate = AttackERate[ch->env_ind_attack][ch->state];
         break;
      case 1: /* Decay */
         ch->erate = DecayERate[ch->env_ind_decay][ch->state];
         break;
      case 2: /* Sustain */
         ch->erate = SustainERate[ch->env_ind_sustain][ch->state];
         break;
      case 3: /* Increase */
         ch->erate = IncreaseERate[mode & 0x1f][ch->state];
         break;
      case 4: /* DecreaseExp */
         ch->erate = DecreaseERateExp[mode & 0x1f][ch->state];
         break;
      case 5: /* KeyOff */
         ch->erate = KeyOffERate[ch->state];
         break;
      }
   }
}

void S9xSetEnvelopeRate(int32_t channel, uint32_t rate, int32_t direction, int32_t target, uint32_t  mode)
{
   S9xSetEnvRate(&SoundData.channels [channel], rate, direction, target, mode);
}

void S9xSetSoundVolume(int32_t channel, int16_t volume_left, int16_t volume_right)
{
   Channel* ch = &SoundData.channels[channel];
   ch->volume_left = volume_left;
   ch->volume_right = volume_right;
   ch-> left_vol_level = (ch->envx * volume_left) / 128;
   ch->right_vol_level = (ch->envx * volume_right) / 128;
}

void S9xSetMasterVolume(int16_t volume_left, int16_t volume_right)
{
   if (Settings.DisableMasterVolume)
      SoundData.master_volume [0] = SoundData.master_volume [1] = 127;
   else
   {
      SoundData.master_volume [0] = volume_left;
      SoundData.master_volume [1] = volume_right;
   }
}

void S9xSetEchoVolume(int16_t volume_left, int16_t volume_right)
{
   SoundData.echo_volume [0] = volume_left;
   SoundData.echo_volume [1] = volume_right;
}

void S9xSetEchoEnable(uint8_t byte)
{
   int32_t i;

   if (!SoundData.echo_write_enabled || Settings.DisableSoundEcho)
      byte = 0;
   if (byte && !SoundData.echo_enable)
   {
      memset(Echo, 0, 24000 * sizeof(int32_t));
      memset(Loop, 0, sizeof(Loop));
   }

   SoundData.echo_enable = byte;
   for (i = 0; i < NUM_CHANNELS; i++)
   {
      if (byte & (1 << i))
         SoundData.channels [i].echo_buf_ptr = EchoBuffer;
      else
         SoundData.channels [i].echo_buf_ptr = NULL;
   }
}

void S9xSetEchoFeedback(int32_t feedback)
{
   CLIP8(feedback);
   SoundData.echo_feedback = feedback;
}

void S9xSetEchoDelay(int32_t delay)
{
   SoundData.echo_buffer_size = (512 * delay * so.playback_rate) / 32040;
   SoundData.echo_buffer_size <<= 1;
   if (SoundData.echo_buffer_size)
      SoundData.echo_ptr %= SoundData.echo_buffer_size;
   else
      SoundData.echo_ptr = 0;
   S9xSetEchoEnable(APU.DSP [APU_EON]);
}

void S9xSetEchoWriteEnable(uint8_t byte)
{
   SoundData.echo_write_enabled = byte;
   S9xSetEchoDelay(APU.DSP [APU_EDL] & 15);
}

void S9xSetFrequencyModulationEnable(uint8_t byte)
{
   SoundData.pitch_mod = byte & 0xFE;
}

void S9xSetSoundKeyOff(int32_t channel)
{
   Channel* ch = &SoundData.channels[channel];

   if (ch->state != SOUND_SILENT)
   {
      ch->state = SOUND_RELEASE;
      ch->mode = MODE_RELEASE;
      S9xSetEnvRate(ch, 8, -1, 0, 5 << 28);
   }
}

void S9xFixSoundAfterSnapshotLoad()
{
   int32_t i;

   SoundData.echo_write_enabled = !(APU.DSP [APU_FLG] & 0x20);
   S9xSetEchoDelay(APU.DSP [APU_EDL] & 0xf);
   S9xSetEchoFeedback((int8_t) APU.DSP [APU_EFB]);

   S9xSetFilterCoefficient(0, (int8_t) APU.DSP [APU_C0]);
   S9xSetFilterCoefficient(1, (int8_t) APU.DSP [APU_C1]);
   S9xSetFilterCoefficient(2, (int8_t) APU.DSP [APU_C2]);
   S9xSetFilterCoefficient(3, (int8_t) APU.DSP [APU_C3]);
   S9xSetFilterCoefficient(4, (int8_t) APU.DSP [APU_C4]);
   S9xSetFilterCoefficient(5, (int8_t) APU.DSP [APU_C5]);
   S9xSetFilterCoefficient(6, (int8_t) APU.DSP [APU_C6]);
   S9xSetFilterCoefficient(7, (int8_t) APU.DSP [APU_C7]);
   for (i = 0; i < 8; i++)
   {
      SoundData.channels[i].needs_decode = true;
      S9xSetSoundFrequency(i, SoundData.channels[i].hertz);
      SoundData.channels [i].envxx = SoundData.channels [i].envx << ENVX_SHIFT;
   }
}

void S9xSetFilterCoefficient(int32_t tap, int32_t value)
{
   FilterTaps [tap & 7] = value;
   if (value == 0 || (tap == 0 && value == 127))
      FilterTapDefinitionBitfield &= ~(1 << (tap & 7));
   else
      FilterTapDefinitionBitfield |= 1 << (tap & 7);
}

void S9xSetSoundADSR(int32_t channel, int32_t attack_ind, int32_t decay_ind, int32_t sustain_ind, int32_t sustain_level, int32_t release_rate)
{
   Channel *ch;
   int32_t attack_rate  = AttackRate [attack_ind];
   int32_t decay_rate   = DecayRate [decay_ind];
   int32_t sustain_rate = SustainRate [sustain_ind];

   /* Hack for ROMs that use a very short attack rate, key on a
      channel, then switch to decay mode. e.g. Final Fantasy II. */
   if(attack_rate == 1)
      attack_rate = 0;


   ch = &SoundData.channels[channel];
   ch->env_ind_attack = attack_ind;
   ch->env_ind_decay = decay_ind;
   ch->env_ind_sustain = sustain_ind;
   ch->attack_rate = attack_rate;
   ch->decay_rate = decay_rate;
   ch->sustain_rate = sustain_rate;
   ch->release_rate = release_rate;
   ch->sustain_level = sustain_level + 1;

   switch (SoundData.channels[channel].state)
   {
   case SOUND_ATTACK:
      S9xSetEnvRate(ch, attack_rate, 1, 127, 0);
      break;
   case SOUND_DECAY:
      S9xSetEnvRate(ch, decay_rate, -1, (MAX_ENVELOPE_HEIGHT * (sustain_level + 1)) >> 3, 1 << 28);
      break;
   case SOUND_SUSTAIN:
      S9xSetEnvRate(ch, sustain_rate, -1, 0, 2 << 28);
      break;
   }
}

void S9xSetEnvelopeHeight(int32_t channel, int32_t level)
{
   Channel* ch = &SoundData.channels[channel];

   ch->envx = level;
   ch->envxx = level << ENVX_SHIFT;

   ch->left_vol_level = (level * ch->volume_left) / 128;
   ch->right_vol_level = (level * ch->volume_right) / 128;

   if (ch->envx == 0 && ch->state != SOUND_SILENT && ch->state != SOUND_GAIN)
      S9xAPUSetEndOfSample(channel, ch);
}

void S9xSetSoundFrequency(int32_t channel, int32_t hertz) /* hertz [0~64K<<1] */
{
   if (SoundData.channels[channel].type == SOUND_NOISE)
      hertz = NoiseFreq [APU.DSP [APU_FLG] & 0x1f];
   SoundData.channels[channel].frequency = (hertz * so.freqbase) >> 11;
}

void S9xSetSoundHertz(int32_t channel, int32_t hertz)
{
   SoundData.channels[channel].hertz = hertz;
   S9xSetSoundFrequency(channel, hertz);
}

void S9xSetSoundType(int32_t channel, int32_t type_of_sound)
{
   SoundData.channels[channel].type = type_of_sound;
}

void DecodeBlock(Channel* ch)
{
   int32_t out;
   uint8_t filter;
   uint8_t shift;
   int8_t sample1, sample2;
   int8_t *compressed;
   int16_t *raw;
   uint32_t i;
   int32_t prev0, prev1;

   if (ch->block_pointer > 0x10000 - 9)
   {
      ch->last_block = true;
      ch->loop = false;
      ch->block = ch->decoded;
      return;
   }

   compressed = (int8_t*) &IAPU.RAM [ch->block_pointer];

   filter = *compressed;
   if ((ch->last_block = (bool) (filter & 1)))
      ch->loop = (bool) (filter & 2);

   raw = ch->block = ch->decoded;

   compressed++;

   prev0 = ch->previous [0];
   prev1 = ch->previous [1];
   shift = filter >> 4;

   switch ((filter >> 2) & 3)
   {
   case 0:
      for (i = 8; i != 0; i--)
      {
         sample1 = *compressed++;
         sample2 = sample1 << 4;
         sample2 >>= 4;
         sample1 >>= 4;
         *raw++ = ((int32_t) sample1 << shift);
         *raw++ = ((int32_t) sample2 << shift);
      }
      prev1 = raw[-2];
      prev0 = raw[-1];
      break;
   case 1:
      for (i = 8; i != 0; i--)
      {
         sample1 = *compressed++;
         sample2 = sample1 << 4;
         sample2 >>= 4;
         sample1 >>= 4;
         prev0 = (int16_t) prev0;
         *raw++ = prev1 = ((int32_t) sample1 << shift) + prev0 - (prev0 >> 4);
         prev1 = (int16_t) prev1;
         *raw++ = prev0 = ((int32_t) sample2 << shift) + prev1 - (prev1 >> 4);
      }
      break;
   case 2:
      for (i = 8; i != 0; i--)
      {
         sample1 = *compressed++;
         sample2 = sample1 << 4;
         sample2 >>= 4;
         sample1 >>= 4;
         out = (sample1 << shift) - prev1 + (prev1 >> 4);
         prev1 = (int16_t) prev0;
         prev0 &= ~3;
         *raw++ = prev0 = out + (prev0 << 1) - (prev0 >> 5) - (prev0 >> 4);
         out = (sample2 << shift) - prev1 + (prev1 >> 4);
         prev1 = (int16_t) prev0;
         prev0 &= ~3;
         *raw++ = prev0 = out + (prev0 << 1) - (prev0 >> 5) - (prev0 >> 4);
      }
      break;
   case 3:
      for (i = 8; i != 0; i--)
      {
         sample1 = *compressed++;
         sample2 = sample1 << 4;
         sample2 >>= 4;
         sample1 >>= 4;
         out = (sample1 << shift);
         out = out - prev1 + (prev1 >> 3) + (prev1 >> 4);
         prev1 = (int16_t) prev0;
         prev0 &= ~3;
         *raw++ = prev0 = out + (prev0 << 1) - (prev0 >> 3) - (prev0 >> 4) - (prev1 >> 6);
         out = (sample2 << shift);
         out = out - prev1 + (prev1 >> 3) + (prev1 >> 4);
         prev1 = (int16_t) prev0;
         prev0 &= ~3;
         *raw++ = prev0 = out + (prev0 << 1) - (prev0 >> 3) - (prev0 >> 4) - (prev1 >> 6);
      }
      break;
   }
   ch->previous [0] = prev0;
   ch->previous [1] = prev1;
   ch->block_pointer += 9;
}

static INLINE void MixStereoSegment(int32_t buf_offset, int32_t sample_count)
{
   int32_t pitch_mod = SoundData.pitch_mod & ~APU.DSP[APU_NON];

   uint32_t J;
   for (J = 0; J < NUM_CHANNELS; J++)
   {
      uint32_t I;
      int32_t VL, VR;
      uint32_t freq0;
      uint8_t mod;
      Channel* ch = &SoundData.channels[J];

      if ((g_channel_mute_mask & (1u << J)) != 0)
         continue;

      if (g_disable_noise && (ch->type == SOUND_NOISE || ch->type == SOUND_EXTRA_NOISE))
         continue;

      if (ch->state == SOUND_SILENT)
         continue;

      freq0 = ch->frequency;
      mod   = pitch_mod & (1 << J);

      if (ch->needs_decode)
      {
         DecodeBlock(ch);
         ch->needs_decode = false;
         ch->sample = ch->block[0];
         ch->sample_pointer = freq0 >> FIXED_POINT_SHIFT;
         if (ch->sample_pointer == 0)
            ch->sample_pointer = 1;
         if (ch->sample_pointer > SOUND_DECODE_LENGTH)
            ch->sample_pointer = SOUND_DECODE_LENGTH - 1;

         ch->next_sample = ch->block[ch->sample_pointer];
         ch->interpolate = 0;

         ch->gauss_buf[0] = ch->gauss_buf[1];
         ch->gauss_buf[1] = ch->gauss_buf[2];
         ch->gauss_buf[2] = ch->gauss_buf[3];
         ch->gauss_buf[3] = ch->next_sample;
      }
      VL = (ch->sample * ch-> left_vol_level) / 128;
      VR = (ch->sample * ch->right_vol_level) / 128;

      for (I = (uint32_t) buf_offset; I < (uint32_t)(buf_offset + sample_count); I += 2)
      {
         uint32_t freq = freq0;

         if (mod)
            freq = PITCH_MOD(freq, wave [I / 2]);

         ch->env_error += ch->erate;
         if (ch->env_error >= FIXED_POINT)
         {
            uint32_t step = ch->env_error >> FIXED_POINT_SHIFT;

            switch (ch->state)
            {
            case SOUND_ATTACK:
               ch->env_error &= FIXED_POINT_REMAINDER;
               ch->envx += step << 1;
               ch->envxx = ch->envx << ENVX_SHIFT;

               if (ch->envx >= 126)
               {
                  ch->envx = 127;
                  ch->envxx = 127 << ENVX_SHIFT;
                  ch->state = SOUND_DECAY;
                  if (ch->sustain_level != 8)
                  {
                     S9xSetEnvRate(ch, ch->decay_rate, -1,
                                   (MAX_ENVELOPE_HEIGHT * ch->sustain_level) >> 3, 1 << 28);
                     break;
                  }
                  ch->state = SOUND_SUSTAIN;
                  S9xSetEnvRate(ch, ch->sustain_rate, -1, 0, 2 << 28);
               }
               break;
            case SOUND_DECAY:
               while (ch->env_error >= FIXED_POINT)
               {
                  ch->envxx = (ch->envxx >> 8) * 255;
                  ch->env_error -= FIXED_POINT;
               }
               ch->envx = ch->envxx >> ENVX_SHIFT;
               if (ch->envx <= ch->envx_target)
               {
                  if (ch->envx <= 0)
                  {
                     S9xAPUSetEndOfSample(J, ch);
                     goto stereo_exit;
                  }
                  ch->state = SOUND_SUSTAIN;
                  S9xSetEnvRate(ch, ch->sustain_rate, -1, 0, 2 << 28);
               }
               break;
            case SOUND_SUSTAIN:
               while (ch->env_error >= FIXED_POINT)
               {
                  ch->envxx = (ch->envxx >> 8) * 255;
                  ch->env_error -= FIXED_POINT;
               }
               ch->envx = ch->envxx >> ENVX_SHIFT;
               if (ch->envx <= 0)
               {
                  S9xAPUSetEndOfSample(J, ch);
                  goto stereo_exit;
               }
               break;
            case SOUND_RELEASE:
               while (ch->env_error >= FIXED_POINT)
               {
                  ch->envxx -= (MAX_ENVELOPE_HEIGHT << ENVX_SHIFT) / 256;
                  ch->env_error -= FIXED_POINT;
               }
               ch->envx = ch->envxx >> ENVX_SHIFT;
               if (ch->envx <= 0)
               {
                  S9xAPUSetEndOfSample(J, ch);
                  goto stereo_exit;
               }
               break;
            case SOUND_INCREASE_LINEAR:
               ch->env_error &= FIXED_POINT_REMAINDER;
               ch->envx += step << 1;
               ch->envxx = ch->envx << ENVX_SHIFT;

               if (ch->envx >= 126)
               {
                  ch->envx = 127;
                  ch->envxx = 127 << ENVX_SHIFT;
                  ch->state = SOUND_GAIN;
                  ch->mode = MODE_GAIN;
                  S9xSetEnvRate(ch, 0, -1, 0, 0);
               }
               break;
            case SOUND_INCREASE_BENT_LINE:
               if (ch->envx >= (MAX_ENVELOPE_HEIGHT * 3) / 4)
               {
                  while (ch->env_error >= FIXED_POINT)
                  {
                     ch->envxx += (MAX_ENVELOPE_HEIGHT << ENVX_SHIFT) / 256;
                     ch->env_error -= FIXED_POINT;
                  }
                  ch->envx = ch->envxx >> ENVX_SHIFT;
               }
               else
               {
                  ch->env_error &= FIXED_POINT_REMAINDER;
                  ch->envx += step << 1;
                  ch->envxx = ch->envx << ENVX_SHIFT;
               }

               if (ch->envx >= 126)
               {
                  ch->envx = 127;
                  ch->envxx = 127 << ENVX_SHIFT;
                  ch->state = SOUND_GAIN;
                  ch->mode = MODE_GAIN;
                  S9xSetEnvRate(ch, 0, -1, 0, 0);
               }
               break;
            case SOUND_DECREASE_LINEAR:
               ch->env_error &= FIXED_POINT_REMAINDER;
               ch->envx -= step << 1;
               ch->envxx = ch->envx << ENVX_SHIFT;
               if (ch->envx <= 0)
               {
                  S9xAPUSetEndOfSample(J, ch);
                  goto stereo_exit;
               }
               break;
            case SOUND_DECREASE_EXPONENTIAL:
               while (ch->env_error >= FIXED_POINT)
               {
                  ch->envxx = (ch->envxx >> 8) * 255;
                  ch->env_error -= FIXED_POINT;
               }
               ch->envx = ch->envxx >> ENVX_SHIFT;
               if (ch->envx <= 0)
               {
                  S9xAPUSetEndOfSample(J, ch);
                  goto stereo_exit;
               }
               break;
            case SOUND_GAIN:
               S9xSetEnvRate(ch, 0, -1, 0, 0);
               break;
            }
            ch-> left_vol_level = (ch->envx * ch->volume_left) / 128;
            ch->right_vol_level = (ch->envx * ch->volume_right) / 128;
            VL = (ch->sample * ch-> left_vol_level) / 128;
            VR = (ch->sample * ch->right_vol_level) / 128;
         }

         ch->count += freq;
         if (ch->count >= FIXED_POINT)
         {
            VL = ch->count >> FIXED_POINT_SHIFT;
            ch->sample_pointer += VL;
            ch->count &= FIXED_POINT_REMAINDER;

            ch->sample = ch->next_sample;
            if (ch->sample_pointer >= SOUND_DECODE_LENGTH)
            {
               if (JUST_PLAYED_LAST_SAMPLE(ch))
               {
                  S9xAPUSetEndOfSample(J, ch);
                  goto stereo_exit;
               }
               do
               {
                  ch->sample_pointer -= SOUND_DECODE_LENGTH;
                  if (ch->last_block)
                  {
                     if (!ch->loop)
                     {
                        ch->sample_pointer = LAST_SAMPLE;
                        ch->next_sample = ch->sample;
                        break;
                     }
                     else
                     {
                        uint8_t *dir;

                        S9xAPUSetEndX(J);
                        ch->last_block = false;
                        dir = S9xGetSampleAddress(ch->sample_number);
                        ch->block_pointer = READ_WORD(dir + 2);
                        if (J < 8 && (kon_loop_valid & (1u << J))) {
                           loop_dir_checked++;
                           if (kon_loop_addr[J] != (uint16_t)ch->block_pointer)
                              loop_dir_stale++;
                        }

                        /* SFX auto-release removed — was killing music loops */
                     }
                  }
                  DecodeBlock(ch);
               }
               while (ch->sample_pointer >= SOUND_DECODE_LENGTH);
               if (!JUST_PLAYED_LAST_SAMPLE(ch))
                  ch->next_sample = ch->block [ch->sample_pointer];
            }
            else
               ch->next_sample = ch->block [ch->sample_pointer];

            if (ch->type == SOUND_SAMPLE)
            {
               /* Shift gaussian ring buffer and insert new sample */
               ch->gauss_buf[0] = ch->gauss_buf[1];
               ch->gauss_buf[1] = ch->gauss_buf[2];
               ch->gauss_buf[2] = ch->gauss_buf[3];
               ch->gauss_buf[3] = ch->next_sample;

               if (Settings.InterpolatedSound && freq < FIXED_POINT && !mod)
                  ch->sample = gauss_interpolate(ch->gauss_buf, ch->count);
               else
                  ch->sample = ch->next_sample;
               ch->interpolate = 0;
            }
            else
            {
               /* Snes9x 1.53's SPC_DSP.cpp, by blargg */
               int32_t feedback = (so.noise_gen << 13) ^ (so.noise_gen << 14);
               so.noise_gen = (feedback & 0x4000) ^ (so.noise_gen >> 1);
               ch->sample = (so.noise_gen << 17) >> 17;
               ch->interpolate = 0;
            }

            VL = (ch->sample * ch-> left_vol_level) / 128;
            VR = (ch->sample * ch->right_vol_level) / 128;
         }
         else
         {
            /* Between sample steps: re-interpolate with advancing fraction */
            if (Settings.InterpolatedSound && ch->type == SOUND_SAMPLE)
            {
               ch->sample = gauss_interpolate(ch->gauss_buf, ch->count);
               VL = (ch->sample * ch-> left_vol_level) / 128;
               VR = (ch->sample * ch->right_vol_level) / 128;
            }
         }

         if (pitch_mod & (1 << (J + 1)))
            wave [I / 2] = ch->sample * ch->envx;

         MixBuffer [I    ] += VL;
         MixBuffer [I + 1] += VR;

         if (!ch->echo_buf_ptr)
            continue;

         ch->echo_buf_ptr [I    ] += VL;
         ch->echo_buf_ptr [I + 1] += VR;
      }
stereo_exit:;
   }
}

static INLINE void MixStereo(int32_t sample_count)
{
   MixStereoSegment(0, sample_count);
}

/* ------------------------------------------------------------------ */
/* Sliced mixing                                                       */
/* ------------------------------------------------------------------ */
/*
 * Why this exists.
 *
 * MK3's sound driver decides a voice has finished by polling ENVX — the
 * live envelope level — about 650 times a second. ENVX (and ENDX) are only
 * written while the mixer runs, and the mixer ran once per emulated frame,
 * *after* S9xMainLoop() had already executed that whole frame's SPC700. So
 * the driver sampled a 20 ms staircase whose every step described the
 * previous frame, and re-keyed voices on the strength of it. That is the
 * repeated sample, and it gets worse with more voices because each one adds
 * envelope transitions that land on stale reads.
 *
 * Mixing the frame in slices as the SPC700 advances keeps the envelopes the
 * driver reads current. The output stage is unchanged: slices accumulate
 * into the same MixBuffer and the frame is converted once at the end.
 */
static int32_t slice_done;        /* stereo samples of this frame mixed  */
static int32_t slice_total;       /* stereo samples this frame will hold */

void S9xSetFrameSampleCount(int32_t stereo_samples)
{
   if (stereo_samples > SOUND_BUFFER_SIZE) stereo_samples = SOUND_BUFFER_SIZE;
   slice_total = stereo_samples & ~1;
}

static void slice_begin(void)
{
   if (SoundData.echo_enable)
      memset(EchoBuffer, 0, slice_total * sizeof(EchoBuffer [0]));
   memset(MixBuffer, 0, slice_total * sizeof(MixBuffer [0]));
   slice_done = 0;
}

/* Mix up to `fraction`/256 of the frame. Called as the SPC700 advances. */
void S9xMixSlice(int32_t fraction)
{
   int32_t upto;

   if (slice_total <= 0) return;
   if (slice_done == 0) slice_begin();

   upto = (int32_t)(((int64_t)slice_total * fraction) >> 8) & ~1;
   if (upto > slice_total) upto = slice_total;
   if (upto <= slice_done)  return;

   MixStereoSegment(slice_done, upto - slice_done);
   slice_done = upto;
}

void S9xMixSamples(int16_t* buffer, int32_t sample_count)
{

   int32_t J;
   int32_t I;

   if (SoundData.echo_enable)
      memset(EchoBuffer, 0, sample_count * sizeof(EchoBuffer [0]));
   memset(MixBuffer, 0, sample_count * sizeof(MixBuffer [0]));
   MixStereo(sample_count);

   /* Mix and convert waveforms */
   if (SoundData.echo_enable && SoundData.echo_buffer_size)
   {
      /* 16-bit stereo sound with echo enabled ... */
      if (FilterTapDefinitionBitfield == 0)
      {
         /* ... but no filter defined. */
         for (J = 0; J < sample_count; J++)
         {
            int32_t E = Echo [SoundData.echo_ptr];
            Echo[SoundData.echo_ptr++] = (E * SoundData.echo_feedback) / 128 + EchoBuffer [J];

            if (SoundData.echo_ptr >= SoundData.echo_buffer_size)
               SoundData.echo_ptr = 0;

            I = (MixBuffer[J] * SoundData.master_volume [J & 1] + E * SoundData.echo_volume [J & 1]) / (VOL_DIV16 * 8);
            CLIP16(I);
            buffer[J] = I;
         }
      }
      else
      {
         /* ... with filter defined. */
         for (J = 0; J < sample_count; J++)
         {
            int32_t E;
            Loop [(Z - 0) & 15] = Echo [SoundData.echo_ptr];
                                             E =  Loop [(Z -  0) & 15] * FilterTaps [0];
            if (FilterTapDefinitionBitfield & 0x02) E += Loop [(Z -  2) & 15] * FilterTaps [1];
            if (FilterTapDefinitionBitfield & 0x04) E += Loop [(Z -  4) & 15] * FilterTaps [2];
            if (FilterTapDefinitionBitfield & 0x08) E += Loop [(Z -  6) & 15] * FilterTaps [3];
            if (FilterTapDefinitionBitfield & 0x10) E += Loop [(Z -  8) & 15] * FilterTaps [4];
            if (FilterTapDefinitionBitfield & 0x20) E += Loop [(Z - 10) & 15] * FilterTaps [5];
            if (FilterTapDefinitionBitfield & 0x40) E += Loop [(Z - 12) & 15] * FilterTaps [6];
            if (FilterTapDefinitionBitfield & 0x80) E += Loop [(Z - 14) & 15] * FilterTaps [7];
            E /= 128;
            Z++;

            Echo[SoundData.echo_ptr++] = (E * SoundData.echo_feedback) / 128 + EchoBuffer[J];

            if (SoundData.echo_ptr >= SoundData.echo_buffer_size)
               SoundData.echo_ptr = 0;

            I = (MixBuffer[J] * SoundData.master_volume [J & 1] + E * SoundData.echo_volume [J & 1]) / (VOL_DIV16 * 8);
            CLIP16(I);
            buffer[J] = I;
         }
      }
   }
   else
   {
      /* 16-bit mono or stereo sound, no echo */
#ifdef PICO_ON_DEVICE
      // Step 2: Use optimized no-echo mixing
      audio_mix_noecho_opt(buffer, sample_count, MixBuffer, SoundData.master_volume);
#else
      for (J = 0; J < sample_count; J++)
      {
         I = (MixBuffer[J] * SoundData.master_volume [J & 1]) / (VOL_DIV16 * 8);
         CLIP16(I);
         buffer[J] = I;
      }
#endif
   }
}

/* Get internal MixBuffer for optimized mixing */
const int32_t* S9xGetMixBuffer(void)
{
   return MixBuffer;
}

/**
 * Mono mixing - mixes to mono output (half the samples of stereo)
 * This is faster because we only process one channel instead of two.
 * The output is single-channel; caller duplicates to stereo for I2S.
 */
#ifdef PICO_ON_DEVICE
__attribute__((hot, section(".time_critical.mix")))
#endif
void S9xMixSamplesMono(int16_t* buffer, int32_t sample_count)
{
   int32_t J;
   int32_t I;

   /* sample_count is the number of mono samples we want */
   int32_t stereo_count = sample_count * 2;
   soundux_frame++;

   /* Finish whatever the slices did not cover. If nothing sliced this
    * frame (slice_total unset, or the caller never advanced us) this is
    * exactly the old behaviour: one memset and one full mix. */
   if (slice_total != stereo_count) {
      S9xSetFrameSampleCount(stereo_count);
      slice_done = 0;
   }
   if (slice_done == 0) slice_begin();
   if (slice_done < stereo_count)
      MixStereoSegment(slice_done, stereo_count - slice_done);
   slice_done = 0;

   /* Mix stereo to mono: average L+R channels */
   /* Use combined master volume (average of L and R) */
   int32_t master_vol = (SoundData.master_volume[0] + SoundData.master_volume[1]) / 2;

   if (SoundData.echo_enable && SoundData.echo_buffer_size)
   {
      /* With echo - simplified (no filter for speed) */
      for (J = 0; J < sample_count; J++)
      {
         /* Get stereo pair and average to mono */
         int32_t left = MixBuffer[J * 2];
         int32_t right = MixBuffer[J * 2 + 1];
         int32_t mono = (left + right) / 2;

         /* Simple echo handling */
         int32_t E = Echo[SoundData.echo_ptr];
         int32_t echo_left = EchoBuffer[J * 2];
         int32_t echo_right = EchoBuffer[J * 2 + 1];
         Echo[SoundData.echo_ptr++] = (E * SoundData.echo_feedback) / 128 + (echo_left + echo_right) / 2;

         if (SoundData.echo_ptr >= SoundData.echo_buffer_size)
            SoundData.echo_ptr = 0;

         int32_t echo_vol = (SoundData.echo_volume[0] + SoundData.echo_volume[1]) / 2;
         I = (mono * master_vol + E * echo_vol) / (VOL_DIV16 * 8);
         CLIP16(I);
         buffer[J] = I;
      }
   }
   else
   {
      /* No echo - fast path */
      for (J = 0; J < sample_count; J++)
      {
         int32_t left = MixBuffer[J * 2];
         int32_t right = MixBuffer[J * 2 + 1];
         int32_t mono = (left + right) / 2;
         I = (mono * master_vol) / (VOL_DIV16 * 8);
         CLIP16(I);
         buffer[J] = I;
      }
   }
}

/* Save/restore minimal channel state for event replay */
typedef struct {
   int32_t  state;
   int32_t  mode;
   int32_t  envx;
   int32_t  envxx;
   uint32_t env_error;
   uint32_t erate;
   int32_t  direction;
   int16_t  envx_target;
   int16_t  left_vol_level;
   int16_t  right_vol_level;
} ChannelSnapshot;

static void SaveChannelState(int c, ChannelSnapshot *snap)
{
   Channel *ch = &SoundData.channels[c];
   snap->state = ch->state;
   snap->mode = ch->mode;
   snap->envx = ch->envx;
   snap->envxx = ch->envxx;
   snap->env_error = ch->env_error;
   snap->erate = ch->erate;
   snap->direction = ch->direction;
   snap->envx_target = ch->envx_target;
   snap->left_vol_level = ch->left_vol_level;
   snap->right_vol_level = ch->right_vol_level;
}

static void RestoreChannelState(int c, const ChannelSnapshot *snap)
{
   Channel *ch = &SoundData.channels[c];
   ch->state = snap->state;
   ch->mode = snap->mode;
   ch->envx = snap->envx;
   ch->envxx = snap->envxx;
   ch->env_error = snap->env_error;
   ch->erate = snap->erate;
   ch->direction = snap->direction;
   ch->envx_target = snap->envx_target;
   ch->left_vol_level = snap->left_vol_level;
   ch->right_vol_level = snap->right_vol_level;
}

static void ApplyDSPEvent(const DSPEvent *ev)
{
   uint8_t mask = 1;
   for (int c = 0; c < 8; c++, mask <<= 1)
   {
      if (ev->data & mask)
      {
         if (ev->type == DSP_EVENT_KON)
            S9xPlaySample(c);
         else
            S9xSetSoundKeyOff(c);
      }
   }
}

void S9xMixSamplesLowPass(int16_t* buffer, int32_t sample_count, int32_t low_pass_range)
{
   int32_t J;
   int32_t I;

   /* Single-pole low-pass filter (6 dB/octave) */
   int32_t low_pass_factor_a = low_pass_range;
   int32_t low_pass_factor_b = 0x10000 - low_pass_factor_a;

   if (SoundData.echo_enable)
      memset(EchoBuffer, 0, sample_count * sizeof(EchoBuffer [0]));
   memset(MixBuffer, 0, sample_count * sizeof(MixBuffer [0]));

   MixStereo(sample_count);

   /* Mix and convert waveforms */
   if (SoundData.echo_enable && SoundData.echo_buffer_size)
   {
      /* 16-bit stereo sound with echo enabled ... */
      if (FilterTapDefinitionBitfield == 0)
      {
         /* ... but no filter defined. */
         for (J = 0; J < sample_count; J++)
         {
            int32_t *low_pass_sample = &MixOutputPrev[J & 0x1];
            int32_t E = Echo [SoundData.echo_ptr];
            Echo[SoundData.echo_ptr++] = (E * SoundData.echo_feedback) / 128 + EchoBuffer [J];

            if (SoundData.echo_ptr >= SoundData.echo_buffer_size)
               SoundData.echo_ptr = 0;

            I = (MixBuffer[J] * SoundData.master_volume [J & 1] + E * SoundData.echo_volume [J & 1]) / (VOL_DIV16 * 8);
            CLIP16(I);

            /* Apply low-pass filter */
            (*low_pass_sample) = ((*low_pass_sample) * low_pass_factor_a) + (I * low_pass_factor_b);
            /* 16.16 fixed point */
            (*low_pass_sample) >>= 16;

            buffer[J] = (int16_t)(*low_pass_sample);
         }
      }
      else
      {
         /* ... with filter defined. */
         for (J = 0; J < sample_count; J++)
         {
            int32_t *low_pass_sample = &MixOutputPrev[J & 0x1];
            int32_t E;
            Loop [(Z - 0) & 15] = Echo [SoundData.echo_ptr];
            E =  Loop [(Z -  0) & 15] * FilterTaps [0];
            if (FilterTapDefinitionBitfield & 0x02) E += Loop [(Z -  2) & 15] * FilterTaps [1];
            if (FilterTapDefinitionBitfield & 0x04) E += Loop [(Z -  4) & 15] * FilterTaps [2];
            if (FilterTapDefinitionBitfield & 0x08) E += Loop [(Z -  6) & 15] * FilterTaps [3];
            if (FilterTapDefinitionBitfield & 0x10) E += Loop [(Z -  8) & 15] * FilterTaps [4];
            if (FilterTapDefinitionBitfield & 0x20) E += Loop [(Z - 10) & 15] * FilterTaps [5];
            if (FilterTapDefinitionBitfield & 0x40) E += Loop [(Z - 12) & 15] * FilterTaps [6];
            if (FilterTapDefinitionBitfield & 0x80) E += Loop [(Z - 14) & 15] * FilterTaps [7];
            E /= 128;
            Z++;

            Echo[SoundData.echo_ptr++] = (E * SoundData.echo_feedback) / 128 + EchoBuffer[J];

            if (SoundData.echo_ptr >= SoundData.echo_buffer_size)
               SoundData.echo_ptr = 0;

            I = (MixBuffer[J] * SoundData.master_volume [J & 1] + E * SoundData.echo_volume [J & 1]) / (VOL_DIV16 * 8);
            CLIP16(I);

            /* Apply low-pass filter */
            (*low_pass_sample) = ((*low_pass_sample) * low_pass_factor_a) + (I * low_pass_factor_b);
            /* 16.16 fixed point */
            (*low_pass_sample) >>= 16;

            buffer[J] = (int16_t)(*low_pass_sample);
         }
      }
   }
   else
   {
      /* 16-bit mono or stereo sound, no echo */
      for (J = 0; J < sample_count; J++)
      {
         int32_t *low_pass_sample = &MixOutputPrev[J & 0x1];
         I = (MixBuffer[J] * SoundData.master_volume [J & 1]) / (VOL_DIV16 * 8);
         CLIP16(I);

         /* Apply low-pass filter */
         (*low_pass_sample) = ((*low_pass_sample) * low_pass_factor_a) + (I * low_pass_factor_b);
         /* 16.16 fixed point */
         (*low_pass_sample) >>= 16;

         buffer[J] = (int16_t)(*low_pass_sample);
      }
   }
}

void S9xResetSound(bool full)
{
   int32_t i;
   for (i = 0; i < 8; i++)
   {
      SoundData.channels[i].state = SOUND_SILENT;
      SoundData.channels[i].mode = MODE_NONE;
      SoundData.channels[i].type = SOUND_SAMPLE;
      SoundData.channels[i].volume_left = 0;
      SoundData.channels[i].volume_right = 0;
      SoundData.channels[i].hertz = 0;
      SoundData.channels[i].count = 0;
      SoundData.channels[i].loop = false;
      SoundData.channels[i].envx_target = 0;
      SoundData.channels[i].env_error = 0;
      SoundData.channels[i].erate = 0;
      SoundData.channels[i].envx = 0;
      SoundData.channels[i].envxx = 0;
      SoundData.channels[i].left_vol_level = 0;
      SoundData.channels[i].right_vol_level = 0;
      SoundData.channels[i].direction = 0;
      SoundData.channels[i].attack_rate = 0;
      SoundData.channels[i].decay_rate = 0;
      SoundData.channels[i].sustain_rate = 0;
      SoundData.channels[i].release_rate = 0;
      SoundData.channels[i].sustain_level = 0;
      /* notaz */
      SoundData.channels[i].env_ind_attack = 0;
      SoundData.channels[i].env_ind_decay = 0;
      SoundData.channels[i].env_ind_sustain = 0;
      SoundData.echo_ptr = 0;
      SoundData.echo_feedback = 0;
      SoundData.echo_buffer_size = 1;
   }
   FilterTaps [0] = 127;
   FilterTaps [1] = 0;
   FilterTaps [2] = 0;
   FilterTaps [3] = 0;
   FilterTaps [4] = 0;
   FilterTaps [5] = 0;
   FilterTaps [6] = 0;
   FilterTaps [7] = 0;
   FilterTapDefinitionBitfield = 0;
   so.noise_gen = 1;

   if (full)
   {
      SoundData.echo_enable = 0;
      SoundData.echo_write_enabled = 0;
      SoundData.pitch_mod = 0;
      SoundData.master_volume[0] = 0;
      SoundData.master_volume[1] = 0;
      SoundData.echo_volume[0] = 0;
      SoundData.echo_volume[1] = 0;
      SoundData.noise_hertz = 0;
   }

   SoundData.master_volume [0] = SoundData.master_volume [1] = 127;
   so.mute_sound = true;

   memset(MixOutputPrev, 0, sizeof(MixOutputPrev));
}

void S9xSetPlaybackRate(uint32_t playback_rate)
{
   int32_t i;

   so.playback_rate = playback_rate;

   if (playback_rate)
   {
      int32_t steps [] = {0, 64, 619, 619, 128, 1, 64, 55, 64, 619};
      int32_t i, u;

      /* notaz: calculate a value (let's call it freqbase) to simplify channel freq calculations later. */
      so.freqbase = (FIXED_POINT << 11) / (playback_rate * 33 / 32);
      /* now precalculate env rates for S9xSetEnvRate */

      for (u = 0 ; u < 10 ; u++)
      {
         int64_t fp1000su = ((int64_t) FIXED_POINT * 1000 * steps[u]);

         for (i = 0 ; i < 16 ; i++)
            AttackERate[i][u] = (uint32_t) (fp1000su / (AttackRate[i] * playback_rate));

         for (i = 0 ; i < 8 ; i++)
            DecayERate[i][u]  = (uint32_t) (fp1000su / (DecayRate[i]  * playback_rate));

         for (i = 0 ; i < 32 ; i++)
         {
            SustainERate[i][u] = (uint32_t) (fp1000su / (SustainRate[i] * playback_rate));
            IncreaseERate[i][u] = (uint32_t) (fp1000su / (IncreaseRate[i] * playback_rate));
            DecreaseERateExp[i][u] = (uint32_t) (fp1000su / (DecreaseRateExp[i] / 2 * playback_rate));
         }

         KeyOffERate[u] = (uint32_t) (fp1000su / (8 * playback_rate));
      }
   }

   S9xSetEchoDelay(APU.DSP [APU_EDL] & 0xf);
   for (i = 0; i < 8; i++)
      S9xSetSoundFrequency(i, SoundData.channels [i].hertz);
}

bool S9xInitSound(int32_t buffer_ms, int32_t lag_ms)
{
   memset(&LocalStateStorage, 0, sizeof(LocalStateStorage));
   if (!g_echo_buffer) {
      g_echo_buffer = (int32_t *)malloc(24000 * sizeof(int32_t));
      if (!g_echo_buffer) return false;
   }
   memset(g_echo_buffer, 0, 24000 * sizeof(int32_t));
   so.playback_rate = 0;
   S9xResetSound(true);
   return true;
}

bool S9xSetSoundMode(int32_t channel, int32_t mode)
{
   Channel* ch = &SoundData.channels[channel];

   switch (mode)
   {
   case MODE_RELEASE:
      if (ch->mode != MODE_NONE)
      {
         ch->mode = MODE_RELEASE;
         return true;
      }
      break;
   case MODE_DECREASE_LINEAR:
   case MODE_DECREASE_EXPONENTIAL:
   case MODE_GAIN:
   case MODE_INCREASE_LINEAR:
   case MODE_INCREASE_BENT_LINE:
      if (ch->mode != MODE_RELEASE)
      {
         ch->mode = mode;
         if (ch->state != SOUND_SILENT)
            ch->state = mode;
         return true;
      }
      break;
   case MODE_ADSR:
      if (ch->mode == MODE_NONE || ch->mode == MODE_ADSR)
      {
         ch->mode = mode;
         return true;
      }
   }

   return false;
}

void S9xPlaySample(int32_t channel)
{
   uint8_t *dir;
   Channel* ch = &SoundData.channels[channel];

   ch->state = SOUND_SILENT;
   ch->mode = MODE_NONE;
   ch->envx = 0;
   ch->envxx = 0;

   S9xFixEnvelope(channel,
                  APU.DSP [APU_GAIN  + (channel << 4)],
                  APU.DSP [APU_ADSR1 + (channel << 4)],
                  APU.DSP [APU_ADSR2 + (channel << 4)]);

   ch->sample_number = APU.DSP [APU_SRCN + channel * 0x10];
   if (APU.DSP [APU_NON] & (1 << channel))
      ch->type = SOUND_NOISE;
   else
      ch->type = SOUND_SAMPLE;


   S9xSetSoundFrequency(channel, ch->hertz);
   ch->loop = false;
   ch->needs_decode = true;
   ch->last_block = false;
   ch->previous [0] = ch->previous[1] = 0;
   ch->gauss_buf[0] = ch->gauss_buf[1] = ch->gauss_buf[2] = ch->gauss_buf[3] = 0;
   dir = S9xGetSampleAddress(ch->sample_number);
   ch->block_pointer = READ_WORD(dir);
   if (channel >= 0 && channel < 8) {
      kon_loop_addr[channel] = (uint16_t)READ_WORD(dir + 2);
      kon_loop_valid |= (uint8_t)(1u << channel);
   }
   /* Re-trigger telemetry. A "repeated sample" that no buffer ever replayed
    * has to be the driver keying the same sound again. Counting a key-on
    * that lands on the same voice with the same start address as that
    * voice's previous key-on separates a genuine re-trigger from a
    * different sound. */
   {
      static uint16_t last_start[8];
      static uint8_t  seen[8];
      kon_total++;
      if (channel >= 0 && channel < 8) {
         static uint32_t last_frame[8];
         if (seen[channel] && last_start[channel] == (uint16_t)ch->block_pointer) {
            uint32_t dt = soundux_frame - last_frame[channel];
            kon_same_sample++;
            if (dt == 0) kon_retrig_same_frame++;
            if (dt <= 2) kon_retrig_le2_frames++;
         }
         last_start[channel] = (uint16_t)ch->block_pointer;
         last_frame[channel] = soundux_frame;
         seen[channel] = 1;
#ifdef FRANK_SNES_KONLOG
         {
            uint32_t w = konlog_w;
            volatile konlog_t *e = &konlog[w % KONLOG_N];
            e->frame = soundux_frame;
            e->start = (uint16_t)ch->block_pointer;
            e->ch    = (uint8_t)channel;
            /* SRCN always equals the voice index in this game, so the byte
             * is better spent on voice pressure: which voices are not
             * SILENT at the instant of this key-on. If the driver is
             * hammering two voices while the rest never free up, it is
             * starved — and a starved driver drops sounds rather than
             * playing them, which is what "too many samples together"
             * looks like from the outside. */
            {
               uint8_t act = 0;
               for (int q = 0; q < 8; q++)
                  if (SoundData.channels[q].state != SOUND_SILENT)
                     act |= (uint8_t)(1u << q);
               e->srcn = act;
            }
            konlog_w = w + 1;
         }
#endif
      }
   }
#ifdef FRANK_SNES_AUDIO_CAPTURE
   /* Diagnosis only: record what this key-on actually resolved to, so a
    * voice that produces no sound can be traced to its sample data. */
   {
      extern void s9x_diag_kon(int, int, unsigned, unsigned, const uint8_t *);
      s9x_diag_kon(channel, ch->sample_number, ch->block_pointer,
                   (unsigned)READ_WORD(dir + 2),
                   &IAPU.RAM[ch->block_pointer & 0xffff]);
   }
#endif
   ch->sample_pointer = 0;
   ch->env_error = 0;
   ch->next_sample = 0;
   ch->interpolate = 0;
   switch (ch->mode)
   {
   case MODE_ADSR:
      if (ch->attack_rate == 0)
      {
         if (ch->decay_rate == 0 || ch->sustain_level == 8)
         {
            ch->state = SOUND_SUSTAIN;
            ch->envx = (MAX_ENVELOPE_HEIGHT * ch->sustain_level) >> 3;
            S9xSetEnvRate(ch, ch->sustain_rate, -1, 0, 2 << 28);
         }
         else
         {
            ch->state = SOUND_DECAY;
            ch->envx = MAX_ENVELOPE_HEIGHT;
            S9xSetEnvRate(ch, ch->decay_rate, -1,
                          (MAX_ENVELOPE_HEIGHT * ch->sustain_level) >> 3, 1 << 28);
         }
         ch-> left_vol_level = (ch->envx * ch->volume_left) / 128;
         ch->right_vol_level = (ch->envx * ch->volume_right) / 128;
      }
      else
      {
         ch->state = SOUND_ATTACK;
         ch->envx = 0;
         ch->left_vol_level = 0;
         ch->right_vol_level = 0;
         S9xSetEnvRate(ch, ch->attack_rate, 1, MAX_ENVELOPE_HEIGHT, 0);
      }
      ch->envxx = ch->envx << ENVX_SHIFT;
      break;
   case MODE_GAIN:
      ch->state = SOUND_GAIN;
      break;
   case MODE_INCREASE_LINEAR:
      ch->state = SOUND_INCREASE_LINEAR;
      break;
   case MODE_INCREASE_BENT_LINE:
      ch->state = SOUND_INCREASE_BENT_LINE;
      break;
   case MODE_DECREASE_LINEAR:
      ch->state = SOUND_DECREASE_LINEAR;
      break;
   case MODE_DECREASE_EXPONENTIAL:
      ch->state = SOUND_DECREASE_EXPONENTIAL;
      break;
   default:
      break;
   }

   S9xFixEnvelope(channel,
                  APU.DSP [APU_GAIN  + (channel << 4)],
                  APU.DSP [APU_ADSR1 + (channel << 4)],
                  APU.DSP [APU_ADSR2 + (channel << 4)]);
}

#endif
