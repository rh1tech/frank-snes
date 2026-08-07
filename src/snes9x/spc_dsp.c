/*
 * spc_dsp.c — accurate S-DSP, extracted from blargg's SPC_DSP.
 *
 * The DSP half of snes9x2005 / snes9x 1.53 apu_blargg.c, lifted verbatim
 * apart from the changes listed below. See spc_dsp.h for why.
 *
 * Copyright (C) 2007 Shay Green. This module is free software; you
 * can redistribute it and/or modify it under the terms of the GNU Lesser
 * General Public License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version. This
 * module is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
 * more details. You should have received a copy of the GNU Lesser General
 * Public License along with this module; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Changes from upstream:
 *   - The SPC700 is not taken; this is the DSP only. src/snes9x/spc700.c
 *     keeps running the CPU and the $2140..$2143 handshake.
 *   - GET_LE16 / SET_LE16 are defined here for little-endian ARM rather
 *     than pulled from blargg_endian.h.
 *   - dsp_run()'s `Settings.HardDisableAudio` early-out is dropped; the
 *     callers here decide whether to run the DSP at all.
 *   - The register-write path is exposed as spc_dsp_write() instead of
 *     being reached through the SPC700's $00F3 store, and the state-copy
 *     (savestate) helpers are left behind — snapshot.c serialises this
 *     module separately.
 */

#include "spc_dsp.h"

#include <string.h>

#ifndef INLINE
#define INLINE inline
#endif

/* Little-endian halfword access. ARM handles unaligned halfwords, and
 * every access here is at least halfword-aligned anyway. */
#define GET_LE16(addr)         (*(uint16_t *)(addr))
#define SET_LE16(addr, data)   (*(uint16_t *)(addr) = (uint16_t)(data))
#define GET_LE16A(addr)        GET_LE16(addr)
#define SET_LE16A(addr, data)  SET_LE16(addr, data)
#define GET_LE16SA(addr)       ((int16_t)GET_LE16(addr))

/* ---- types, from apu_blargg.h ---- */

#define ECHO_HIST_SIZE    8
#define ECHO_HIST_SIZE_X2 16
#define VOICE_COUNT       SPC_DSP_VOICE_COUNT
#define EXTRA_SIZE        16
#define BRR_BUF_SIZE      12
#define BRR_BUF_SIZE_X2   24
#define BRR_BLOCK_SIZE    9
#define REGISTER_COUNT    SPC_DSP_REGISTER_COUNT
#define ROM_WINDOW_BYTES  64

#define R_MVOLL 0x0C
#define R_MVOLR 0x1C
#define R_EVOLL 0x2C
#define R_EVOLR 0x3C
#define R_KON   0x4C
#define R_KOFF  0x5C
#define R_FLG   0x6C
#define R_ENDX  0x7C
#define R_EFB   0x0D
#define R_EON   0x4D
#define R_PMON  0x2D
#define R_NON   0x3D
#define R_DIR   0x5D
#define R_ESA   0x6D
#define R_EDL   0x7D
#define R_FIR   0x0F

#define V_VOLL   0x00
#define V_VOLR   0x01
#define V_PITCHL 0x02
#define V_PITCHH 0x03
#define V_SRCN   0x04
#define V_ADSR0  0x05
#define V_ADSR1  0x06
#define V_GAIN   0x07
#define V_ENVX   0x08
#define V_OUTX   0x09

#define ENV_RELEASE 0
#define ENV_ATTACK  1
#define ENV_DECAY   2
#define ENV_SUSTAIN 3

typedef struct
{
   int32_t  buf [BRR_BUF_SIZE_X2];
   int32_t  buf_pos;
   int32_t  interp_pos;
   int32_t  brr_addr;
   int32_t  brr_offset;
   uint8_t* regs;
   int32_t  vbit;
   int32_t  kon_delay;
   int32_t  env_mode;
   int32_t  env;
   int32_t  hidden_env;
   uint8_t  t_envx_out;
} dsp_voice_t;

typedef struct
{
   uint8_t       regs [REGISTER_COUNT];
   int32_t       echo_hist [ECHO_HIST_SIZE_X2] [2];
   int32_t     (*echo_hist_pos) [2];
   int32_t       every_other_sample;
   int32_t       kon;
   int32_t       noise;
   int32_t       counter;
   int32_t       echo_offset;
   int32_t       echo_length;
   int32_t       phase;

   int32_t       new_kon;
   uint8_t       endx_buf;
   uint8_t       envx_buf;
   uint8_t       outx_buf;

   int32_t       t_pmon;
   int32_t       t_non;
   int32_t       t_eon;
   int32_t       t_dir;
   int32_t       t_koff;

   int32_t       t_brr_next_addr;
   int32_t       t_adsr0;
   int32_t       t_brr_header;
   int32_t       t_brr_byte;
   int32_t       t_srcn;
   int32_t       t_esa;
   int32_t       t_echo_enabled;

   int32_t       t_dir_addr;
   int32_t       t_pitch;
   int32_t       t_output;
   int32_t       t_looped;
   int32_t       t_echo_ptr;

   int32_t       t_main_out [2];
   int32_t       t_echo_out [2];
   int32_t       t_echo_in  [2];

   dsp_voice_t   voices [VOICE_COUNT];

   uint8_t*      ram;
   int16_t*      out;
   int16_t*      out_end;
   int16_t*      out_begin;
   int16_t       extra [EXTRA_SIZE];

   int32_t       rom_enabled;
   uint8_t*      rom;
   uint8_t*      hi_ram;
} dsp_state_t;

static dsp_state_t dsp_m;

/* Copyright (C) 2007 Shay Green. This module is free software; you
can redistribute it and/or modify it under the terms of the GNU Lesser
General Public License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version. This
module is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
details. You should have received a copy of the GNU Lesser General Public
License along with this module; if not, write to the Free Software Foundation,
Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA */

#define CLAMP16(io) \
{ \
   if ((int16_t) io != io) \
      io = (io >> 31) ^ 0x7FFF; \
}

/* Access global DSP register */
#define REG(n) dsp_m.regs [R_##n]

/* Access voice DSP register */
#define VREG(r,n) r [V_##n]

#define WRITE_SAMPLES(l, r, out) \
{\
   out [0] = l; \
   out [1] = r; \
   out += 2; \
   if ( out >= dsp_m.out_end ) \
   { \
      out       = dsp_m.extra; \
      dsp_m.out_end = &dsp_m.extra [EXTRA_SIZE]; \
   } \
}

/* Volume registers and efb are signed! Easy to forget int8_t cast. */
/* Prefixes are to avoid accidental use of locals with same names. */

/* Gaussian interpolation */

static int16_t gauss [512] =
{
0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,
2,   2,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,   5,   5,   5,   5,
6,   6,   6,   6,   7,   7,   7,   8,   8,   8,   9,   9,   9,   10,  10,  10,
11,  11,  11,  12,  12,  13,  13,  14,  14,  15,  15,  15,  16,  16,  17,  17,
18,  19,  19,  20,  20,  21,  21,  22,  23,  23,  24,  24,  25,  26,  27,  27,
28,  29,  29,  30,  31,  32,  32,  33,  34,  35,  36,  36,  37,  38,  39,  40,
41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,
58,  59,  60,  61,  62,  64,  65,  66,  67,  69,  70,  71,  73,  74,  76,  77,
78,  80,  81,  83,  84,  86,  87,  89,  90,  92,  94,  95,  97,  99,  100, 102,
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
969, 974, 978, 983, 988, 992, 997, 1001,1005,1010,1014,1019,1023,1027,1032,1036,
1040,1045,1049,1053,1057,1061,1066,1070,1074,1078,1082,1086,1090,1094,1098,1102,
1106,1109,1113,1117,1121,1125,1128,1132,1136,1139,1143,1146,1150,1153,1157,1160,
1164,1167,1170,1174,1177,1180,1183,1186,1190,1193,1196,1199,1202,1205,1207,1210,
1213,1216,1219,1221,1224,1227,1229,1232,1234,1237,1239,1241,1244,1246,1248,1251,
1253,1255,1257,1259,1261,1263,1265,1267,1269,1270,1272,1274,1275,1277,1279,1280,
1282,1283,1284,1286,1287,1288,1290,1291,1292,1293,1294,1295,1296,1297,1297,1298,
1299,1300,1300,1301,1302,1302,1303,1303,1303,1304,1304,1304,1304,1304,1305,1305,
};

/* Gaussian interpolation */

static INLINE int32_t dsp_interpolate( dsp_voice_t *v )
{
   int32_t offset, out, *in;
   int16_t *fwd, *rev;

   /* Make pointers into gaussian based on fractional position between samples */
   offset = v->interp_pos >> 4 & 0xFF;
   fwd = gauss + 255 - offset;
   rev = gauss       + offset; /* mirror left half of gaussian */

   in = &v->buf [(v->interp_pos >> 12) + v->buf_pos];
   out  = (fwd [  0] * in [0]) >> 11;
   out += (fwd [256] * in [1]) >> 11;
   out += (rev [256] * in [2]) >> 11;
   out = (int16_t) out;
   out += (rev [  0] * in [3]) >> 11;

   CLAMP16( out );
   out &= ~1;
   return out;
}

/* Counters */

/* 30720 =  2048 * 5 * 3 */
#define SIMPLE_COUNTER_RANGE 30720

static uint32_t const counter_rates [32] =
{
   SIMPLE_COUNTER_RANGE + 1, /* never fires */
         2048, 1536,
   1280, 1024,  768,
    640,  512,  384,
    320,  256,  192,
    160,  128,   96,
     80,   64,   48,
     40,   32,   24,
     20,   16,   12,
     10,    8,    6,
      5,    4,    3,
            2,
            1
};

static uint32_t const counter_offsets [32] =
{
     1, 0, 1040,
   536, 0, 1040,
   536, 0, 1040,
   536, 0, 1040,
   536, 0, 1040,
   536, 0, 1040,
   536, 0, 1040,
   536, 0, 1040,
   536, 0, 1040,
   536, 0, 1040,
        0,
        0
};

#define RUN_COUNTERS() \
   if (--dsp_m.counter < 0) \
      dsp_m.counter = SIMPLE_COUNTER_RANGE - 1;

#define READ_COUNTER(rate) (((uint32_t) dsp_m.counter + counter_offsets [rate]) % counter_rates [rate])

/* Envelope */

static INLINE void dsp_run_envelope( dsp_voice_t* const v )
{
   int32_t env, rate, env_data;

   env = v->env;
   env_data = v->regs[V_ADSR1];

   if ( dsp_m.t_adsr0 & 0x80 ) /* 99% ADSR */
   {
      if ( v->env_mode >= ENV_DECAY ) /* 99% */
      {
         env--;
         env -= env >> 8;
         rate = env_data & 0x1F;
         if ( v->env_mode == ENV_DECAY ) /* 1% */
            rate = (dsp_m.t_adsr0 >> 3 & 0x0E) + 0x10;
      }
      else /* ENV_ATTACK */
      {
         rate = (dsp_m.t_adsr0 & 0x0F) * 2 + 1;
         env += rate < 31 ? 0x20 : 0x400;
      }
   }
   else /* GAIN */
   {
      int32_t mode;
      env_data = v->regs[V_GAIN];
      mode = env_data >> 5;
      if ( mode < 4 ) /* direct */
      {
         env = env_data * 0x10;
         rate = 31;
      }
      else
      {
         rate = env_data & 0x1F;
         if ( mode == 4 ) /* 4: linear decrease */
         {
            env -= 0x20;
         }
         else if ( mode < 6 ) /* 5: exponential decrease */
         {
            env--;
            env -= env >> 8;
         }
         else /* 6,7: linear increase */
         {
            env += 0x20;
            if ( mode > 6 && (uint32_t) v->hidden_env >= 0x600 )
               env += 0x8 - 0x20; /* 7: two-slope linear increase */
         }
      }
   }

   /* Sustain level */
   if ( (env >> 8) == (env_data >> 5) && v->env_mode == ENV_DECAY )
      v->env_mode = ENV_SUSTAIN;

   v->hidden_env = env;

   /* unsigned cast because linear decrease going negative also triggers this */
   if ( (uint32_t) env > 0x7FF )
   {
      env = (env < 0 ? 0 : 0x7FF);
      if ( v->env_mode == ENV_ATTACK )
         v->env_mode = ENV_DECAY;
   }

   if (!READ_COUNTER( rate ))
      v->env = env; /* nothing else is controlled by the counter */
}

/* BRR Decoding */

static INLINE void dsp_decode_brr( dsp_voice_t* v )
{
   int32_t nybbles, *pos, *end, header;

   /* Arrange the four input nybbles in 0xABCD order for easy decoding */
   nybbles = dsp_m.t_brr_byte * 0x100 + dsp_m.ram [(v->brr_addr + v->brr_offset + 1) & 0xFFFF];

   header = dsp_m.t_brr_header;

   /* Write to next four samples in circular buffer */
   pos = &v->buf [v->buf_pos];

   if ( (v->buf_pos += 4) >= BRR_BUF_SIZE )
      v->buf_pos = 0;

   /* Decode four samples */
   for ( end = pos + 4; pos < end; pos++, nybbles <<= 4 )
   {
      int32_t filter, p1, p2, s, shift;
      /* Extract nybble and sign-extend */
      s = (int16_t) nybbles >> 12;

      /* Shift sample based on header */
      shift = header >> 4;
      s = (s << shift) >> 1;
      if ( shift >= 0xD ) /* handle invalid range */
         s = (s >> 25) << 11; /* same as: s = (s < 0 ? -0x800 : 0) */

      /* Apply IIR filter (8 is the most commonly used) */
      filter = header & 0x0C;
      p1 = pos [BRR_BUF_SIZE - 1];
      p2 = pos [BRR_BUF_SIZE - 2] >> 1;
      if ( filter >= 8 )
      {
         s += p1;
         s -= p2;
         if ( filter == 8 ) /* s += p1 * 0.953125 - p2 * 0.46875 */
         {
            s += p2 >> 4;
            s += (p1 * -3) >> 6;
         }
         else /* s += p1 * 0.8984375 - p2 * 0.40625 */
         {
            s += (p1 * -13) >> 7;
            s += (p2 * 3) >> 4;
         }
      }
      else if ( filter ) /* s += p1 * 0.46875 */
      {
         s += p1 >> 1;
         s += (-p1) >> 5;
      }

      /* Adjust and write sample */
      CLAMP16( s );
      s = (int16_t) (s * 2);
      pos [BRR_BUF_SIZE] = pos [0] = s; /* second copy simplifies wrap-around */
   }
}

/* Misc */

/* voice 0 doesn't support PMON */

#define MISC_27() dsp_m.t_pmon = dsp_m.regs[R_PMON] & 0xFE;

#define MISC_28() \
   dsp_m.t_non = dsp_m.regs[R_NON]; \
   dsp_m.t_eon = dsp_m.regs[R_EON]; \
   dsp_m.t_dir = dsp_m.regs[R_DIR];

#define MISC_29() \
   if ( (dsp_m.every_other_sample ^= 1) != 0 ) \
      dsp_m.new_kon &= ~dsp_m.kon; /* clears KON 63 clocks after it was last read */

static INLINE void dsp_misc_30()
{
   if ( dsp_m.every_other_sample )
   {
      dsp_m.kon    = dsp_m.new_kon;
      dsp_m.t_koff = dsp_m.regs[R_KOFF];
   }

   RUN_COUNTERS();

   /* Noise */
   if ( !READ_COUNTER( dsp_m.regs[R_FLG] & 0x1F ) )
   {
      int32_t feedback = (dsp_m.noise << 13) ^ (dsp_m.noise << 14);
      dsp_m.noise = (feedback & 0x4000) ^ (dsp_m.noise >> 1);
   }
}

/* Voices */

static INLINE void dsp_voice_V1( dsp_voice_t* const v )
{
   dsp_m.t_dir_addr = dsp_m.t_dir * 0x100 + dsp_m.t_srcn * 4;
   dsp_m.t_srcn = v->regs[V_SRCN];
}

static INLINE void dsp_voice_V2( dsp_voice_t* const v )
{
   uint8_t *entry;

   entry = &dsp_m.ram [dsp_m.t_dir_addr];
   if ( !v->kon_delay )
      entry += 2;

   dsp_m.t_brr_next_addr = GET_LE16( entry );

   dsp_m.t_adsr0 = v->regs [V_ADSR0];


   dsp_m.t_pitch = v->regs [V_PITCHL];
}

static INLINE void dsp_voice_V3a( dsp_voice_t* const v )
{
   dsp_m.t_pitch += (v->regs [V_PITCHH] & 0x3F) << 8;
}

static INLINE void dsp_voice_V3b( dsp_voice_t* const v )
{
   dsp_m.t_brr_byte = dsp_m.ram [(v->brr_addr + v->brr_offset) & 0xffff];
   dsp_m.t_brr_header = dsp_m.ram [v->brr_addr];
}

static void dsp_voice_V3c( dsp_voice_t* const v )
{
   int32_t output;

   /* Pitch modulation using previous voice's output */
   if ( dsp_m.t_pmon & v->vbit )
      dsp_m.t_pitch += ((dsp_m.t_output >> 5) * dsp_m.t_pitch) >> 10;

   if ( v->kon_delay )
   {
      /* Get ready to start BRR decoding on next sample */
      if ( v->kon_delay == 5 )
      {
         v->brr_addr    = dsp_m.t_brr_next_addr;
         v->brr_offset  = 1;
         v->buf_pos     = 0;
         dsp_m.t_brr_header = 0; /* header is ignored on this sample */
      }

      /* Envelope is never run during KON */
      v->env        = 0;
      v->hidden_env = 0;

      /* Disable BRR decoding until last three samples */
      v->interp_pos = 0;
      if ( --v->kon_delay & 3 )
         v->interp_pos = 0x4000;

      /* Pitch is never added during KON */
      dsp_m.t_pitch = 0;
   }

   output = dsp_interpolate( v );

   /* Noise */
   if ( dsp_m.t_non & v->vbit )
      output = (int16_t) (dsp_m.noise * 2);

   /* Apply envelope */
   dsp_m.t_output = (output * v->env) >> 11 & ~1;
   v->t_envx_out = (uint8_t) (v->env >> 4);

   /* Immediate silence due to end of sample or soft reset */
   if ( dsp_m.regs[R_FLG] & 0x80 || (dsp_m.t_brr_header & 3) == 1 )
   {
      v->env_mode = ENV_RELEASE;
      v->env      = 0;
   }

   if ( dsp_m.every_other_sample )
   {
      /* KOFF */
      if ( dsp_m.t_koff & v->vbit )
         v->env_mode = ENV_RELEASE;

      /* KON */
      if ( dsp_m.kon & v->vbit )
      {
         v->kon_delay = 5;
         v->env_mode  = ENV_ATTACK;
      }
   }

   /* Run envelope for next sample */
   if ( !v->kon_delay )
   {
      int32_t env = v->env;
      if ( v->env_mode == ENV_RELEASE ) /* 60% */
      {
         if ( (env -= 0x8) < 0 )
            env = 0;
         v->env = env;
      }
      else
      {
         dsp_run_envelope( v );
      }
   }
}

static INLINE void dsp_voice_output( dsp_voice_t const* v, int32_t ch )
{
   int32_t amp;

   /* Apply left/right volume */
   amp = (dsp_m.t_output * (int8_t) VREG(v->regs,VOLL + ch)) >> 7;

   /* Add to output total */
   dsp_m.t_main_out [ch] += amp;
   CLAMP16( dsp_m.t_main_out [ch] );

   /* Optionally add to echo total */
   if ( dsp_m.t_eon & v->vbit )
   {
      dsp_m.t_echo_out [ch] += amp;
      CLAMP16( dsp_m.t_echo_out [ch] );
   }
}

static INLINE void dsp_voice_V4( dsp_voice_t* const v )
{
   /* Decode BRR */
   dsp_m.t_looped = 0;
   if ( v->interp_pos >= 0x4000 )
   {
      dsp_decode_brr( v );

      if ( (v->brr_offset += 2) >= BRR_BLOCK_SIZE )
      {
         /* Start decoding next BRR block */
         v->brr_addr = (v->brr_addr + BRR_BLOCK_SIZE) & 0xFFFF;
         if ( dsp_m.t_brr_header & 1 )
         {
            v->brr_addr = dsp_m.t_brr_next_addr;
            dsp_m.t_looped = v->vbit;
         }
         v->brr_offset = 1;
      }
   }

   /* Apply pitch */
   v->interp_pos = (v->interp_pos & 0x3FFF) + dsp_m.t_pitch;

   /* Keep from getting too far ahead (when using pitch modulation) */
   if ( v->interp_pos > 0x7FFF )
      v->interp_pos = 0x7FFF;

   /* Output left */
   dsp_voice_output( v, 0 );
}

static INLINE void dsp_voice_V5( dsp_voice_t* const v )
{
   int32_t endx_buf;
   /* Output right */
   dsp_voice_output( v, 1 );

   /* ENDX, OUTX, and ENVX won't update if you wrote to them 1-2 clocks earlier */
   endx_buf = dsp_m.regs[R_ENDX] | dsp_m.t_looped;

   /* Clear bit in ENDX if KON just began */
   if ( v->kon_delay == 5 )
      endx_buf &= ~v->vbit;
   dsp_m.endx_buf = (uint8_t) endx_buf;
}

static INLINE void dsp_voice_V6( dsp_voice_t* const v )
{
   (void) v; /* avoid compiler warning about unused v */
   dsp_m.outx_buf = (uint8_t) (dsp_m.t_output >> 8);
}

static INLINE void dsp_voice_V7( dsp_voice_t* const v )
{
   /* Update ENDX */
   dsp_m.regs[R_ENDX] = dsp_m.endx_buf;

   dsp_m.envx_buf = v->t_envx_out;
}

static INLINE void dsp_voice_V8( dsp_voice_t* const v )
{
   /* Update OUTX */
   v->regs [V_OUTX] = dsp_m.outx_buf;
}

static INLINE void dsp_voice_V9( dsp_voice_t* const v )
{
   v->regs [V_ENVX] = dsp_m.envx_buf;
}

/* Most voices do all these in one clock, so make a handy composite */

static INLINE void dsp_voice_V3( dsp_voice_t* const v )
{
   dsp_voice_V3a( v );
   dsp_voice_V3b( v );
   dsp_voice_V3c( v );
}

/* Common combinations of voice steps on different voices. This greatly reduces
   code size and allows everything to be INLINEd in these functions. */

static void dsp_voice_V7_V4_V1( dsp_voice_t* const v )
{
   dsp_voice_V7(v);
   dsp_voice_V1(v+3);
   dsp_voice_V4(v+1);
}

static void dsp_voice_V8_V5_V2( dsp_voice_t* const v )
{
   dsp_voice_V8(v);
   dsp_voice_V5(v+1);
   dsp_voice_V2(v+2);
}

static void dsp_voice_V9_V6_V3( dsp_voice_t* const v )
{
   dsp_voice_V9(v);
   dsp_voice_V6(v+1);
   dsp_voice_V3(v+2);
}

/* Echo */

/* Current echo buffer pointer for left/right channel */
#define ECHO_PTR( ch )      (&dsp_m.ram [dsp_m.t_echo_ptr + ch * 2])

/* Sample in echo history buffer, where 0 is the oldest */
#define ECHO_FIR( i )       (dsp_m.echo_hist_pos [i])

/* Calculate FIR point for left/right channel */
#define CALC_FIR( i, ch )   ((ECHO_FIR( i + 1 ) [ch] * (int8_t) REG(FIR + i * 0x10)) >> 6)

#define ECHO_READ(ch) \
{ \
   int32_t s; \
   if ( dsp_m.t_echo_ptr >= 0xffc0 && dsp_m.rom_enabled ) \
      s = GET_LE16SA( &dsp_m.hi_ram [dsp_m.t_echo_ptr + ch * 2 - 0xffc0] ); \
   else \
      s = GET_LE16SA( ECHO_PTR( ch ) ); \
   /* second copy simplifies wrap-around handling */ \
   ECHO_FIR( 0 ) [ch] = ECHO_FIR( 8 ) [ch] = s >> 1; \
}

static INLINE void dsp_echo_22()
{
   int32_t l, r;

   if (++dsp_m.echo_hist_pos >= &dsp_m.echo_hist [ECHO_HIST_SIZE])
      dsp_m.echo_hist_pos = dsp_m.echo_hist;

   dsp_m.t_echo_ptr = (dsp_m.t_esa * 0x100 + dsp_m.echo_offset) & 0xFFFF;

   ECHO_READ(0);

   l = (((dsp_m.echo_hist_pos [0 + 1]) [0] * (int8_t) dsp_m.regs [R_FIR + 0 * 0x10]) >> 6);
   r = (((dsp_m.echo_hist_pos [0 + 1]) [1] * (int8_t) dsp_m.regs [R_FIR + 0 * 0x10]) >> 6);

   dsp_m.t_echo_in [0] = l;
   dsp_m.t_echo_in [1] = r;
}

static INLINE void dsp_echo_23()
{
   int32_t l, r;

   l = (((dsp_m.echo_hist_pos [1 + 1]) [0] * (int8_t) dsp_m.regs [R_FIR + 1 * 0x10]) >> 6) + (((dsp_m.echo_hist_pos [2 + 1]) [0] * (int8_t) dsp_m.regs [R_FIR + 2 * 0x10]) >> 6);
   r = (((dsp_m.echo_hist_pos [1 + 1]) [1] * (int8_t) dsp_m.regs [R_FIR + 1 * 0x10]) >> 6) + (((dsp_m.echo_hist_pos [2 + 1]) [1] * (int8_t) dsp_m.regs [R_FIR + 2 * 0x10]) >> 6);

   dsp_m.t_echo_in [0] += l;
   dsp_m.t_echo_in [1] += r;

   ECHO_READ(1);
}

static INLINE void dsp_echo_24()
{
   int32_t l, r;

   l = (((dsp_m.echo_hist_pos [3 + 1]) [0] * (int8_t) dsp_m.regs [R_FIR + 3 * 0x10]) >> 6) + (((dsp_m.echo_hist_pos [4 + 1]) [0] * (int8_t) dsp_m.regs [R_FIR + 4 * 0x10]) >> 6) + (((dsp_m.echo_hist_pos [5 + 1]) [0] * (int8_t) dsp_m.regs [R_FIR + 5 * 0x10]) >> 6);
   r = (((dsp_m.echo_hist_pos [3 + 1]) [1] * (int8_t) dsp_m.regs [R_FIR + 3 * 0x10]) >> 6) + (((dsp_m.echo_hist_pos [4 + 1]) [1] * (int8_t) dsp_m.regs [R_FIR + 4 * 0x10]) >> 6) + (((dsp_m.echo_hist_pos [5 + 1]) [1] * (int8_t) dsp_m.regs [R_FIR + 5 * 0x10]) >> 6);

   dsp_m.t_echo_in [0] += l;
   dsp_m.t_echo_in [1] += r;
}

static INLINE void dsp_echo_25()
{
   int32_t l = dsp_m.t_echo_in [0] + (((dsp_m.echo_hist_pos [6 + 1]) [0] * (int8_t) dsp_m.regs [R_FIR + 6 * 0x10]) >> 6);
   int32_t r = dsp_m.t_echo_in [1] + (((dsp_m.echo_hist_pos [6 + 1]) [1] * (int8_t) dsp_m.regs [R_FIR + 6 * 0x10]) >> 6);

   l = (int16_t) l;
   r = (int16_t) r;

   l += (int16_t) (((dsp_m.echo_hist_pos [7 + 1]) [0] * (int8_t) dsp_m.regs [R_FIR + 7 * 0x10]) >> 6);
   r += (int16_t) (((dsp_m.echo_hist_pos [7 + 1]) [1] * (int8_t) dsp_m.regs [R_FIR + 7 * 0x10]) >> 6);

   if ( (int16_t) l != l )
      l = (l >> 31) ^ 0x7FFF;
   if ( (int16_t) r != r )
      r = (r >> 31) ^ 0x7FFF;

   dsp_m.t_echo_in [0] = l & ~1;
   dsp_m.t_echo_in [1] = r & ~1;
}

#define ECHO_OUTPUT(var, ch) \
{ \
   var = (int16_t) ((dsp_m.t_main_out [ch] * (int8_t) REG(MVOLL + ch * 0x10)) >> 7) + (int16_t) ((dsp_m.t_echo_in [ch] * (int8_t) REG(EVOLL + ch * 0x10)) >> 7); \
   CLAMP16( var ); \
}

static INLINE void dsp_echo_26()
{
   int32_t l, r;

   ECHO_OUTPUT(dsp_m.t_main_out[0], 0 );

   l = dsp_m.t_echo_out [0] + (int16_t) ((dsp_m.t_echo_in [0] * (int8_t) dsp_m.regs [R_EFB]) >> 7);
   r = dsp_m.t_echo_out [1] + (int16_t) ((dsp_m.t_echo_in [1] * (int8_t) dsp_m.regs [R_EFB]) >> 7);

   if ( (int16_t) l != l ) l = (l >> 31) ^ 0x7FFF;
   if ( (int16_t) r != r ) r = (r >> 31) ^ 0x7FFF;

   dsp_m.t_echo_out [0] = l & ~1;
   dsp_m.t_echo_out [1] = r & ~1;
}

static INLINE void dsp_echo_27()
{
   int32_t l, r;
   int16_t *out;

   l = dsp_m.t_main_out [0];
   ECHO_OUTPUT(r, 1);
   dsp_m.t_main_out [0] = 0;
   dsp_m.t_main_out [1] = 0;

   if ( dsp_m.regs [R_FLG] & 0x40 )
   {
      l = 0;
      r = 0;
   }

   out = dsp_m.out;
   out [0] = l;
   out [1] = r;
   out += 2;
   if ( out >= dsp_m.out_end )
   {
      out = dsp_m.extra;
      dsp_m.out_end = &dsp_m.extra [EXTRA_SIZE];
   }
   dsp_m.out = out;
}

#define ECHO_28() dsp_m.t_echo_enabled = dsp_m.regs [R_FLG];

#define ECHO_WRITE(ch) \
   if ( !(dsp_m.t_echo_enabled & 0x20) ) \
   { \
      SET_LE16A( ECHO_PTR( ch ), dsp_m.t_echo_out [ch] ); \
      if ( dsp_m.t_echo_ptr >= 0xffc0 ) \
      { \
         SET_LE16A( &dsp_m.hi_ram [dsp_m.t_echo_ptr + ch * 2 - 0xffc0], dsp_m.t_echo_out [ch] ); \
         if ( dsp_m.rom_enabled ) \
            SET_LE16A( ECHO_PTR( ch ), GET_LE16A( &dsp_m.rom [dsp_m.t_echo_ptr + ch * 2 - 0xffc0] ) ); \
      } \
   } \
   dsp_m.t_echo_out [ch] = 0;

static INLINE void dsp_echo_29()
{
   dsp_m.t_esa = dsp_m.regs [R_ESA];

   if ( !dsp_m.echo_offset )
      dsp_m.echo_length = (dsp_m.regs [R_EDL] & 0x0F) * 0x800;

   dsp_m.echo_offset += 4;
   if ( dsp_m.echo_offset >= dsp_m.echo_length )
      dsp_m.echo_offset = 0;


   ECHO_WRITE(0);

   dsp_m.t_echo_enabled = dsp_m.regs [R_FLG];
}

/* Timing */

/* Execute clock for a particular voice */

/* The most common sequence of clocks uses composite operations
for efficiency. For example, the following are equivalent to the
individual steps on the right:

V(V7_V4_V1,2) -> V(V7,2) V(V4,3) V(V1,5)
V(V8_V5_V2,2) -> V(V8,2) V(V5,3) V(V2,4)
V(V9_V6_V3,2) -> V(V9,2) V(V6,3) V(V3,4) */

/* Voice      0      1      2      3      4      5      6      7 */

/* Runs DSP for specified number of clocks (~1024000 per second). Every 32 clocks
   a pair of samples is be generated. */

static void dsp_run( int32_t clocks_remain )
{
   int32_t phase;
   phase = dsp_m.phase;
   dsp_m.phase = (phase + clocks_remain) & 31;

   switch ( phase )
   {
loop:
      if ( 0 && !--clocks_remain )
         break;
      case 0:
      dsp_voice_V5( &dsp_m.voices [0] );
      dsp_voice_V2( &dsp_m.voices [1] );
      if ( 1 && !--clocks_remain )
         break;
      case 1:
      dsp_voice_V6( &dsp_m.voices [0] );
      dsp_voice_V3( &dsp_m.voices [1] );
      if ( 2 && !--clocks_remain )
         break;
      case 2:
      dsp_voice_V7_V4_V1( &dsp_m.voices [0] );
      if ( 3 && !--clocks_remain )
         break;
      case 3:
      dsp_voice_V8_V5_V2( &dsp_m.voices [0] );
      if ( 4 && !--clocks_remain )
         break;
      case 4:
      dsp_voice_V9_V6_V3( &dsp_m.voices [0] );
      if ( 5 && !--clocks_remain )
         break;
      case 5:
      dsp_voice_V7_V4_V1( &dsp_m.voices [1] );
      if ( 6 && !--clocks_remain )
         break;
      case 6:
      dsp_voice_V8_V5_V2( &dsp_m.voices [1] );
      if ( 7 && !--clocks_remain )
         break;
      case 7:
      dsp_voice_V9_V6_V3( &dsp_m.voices [1] );
      if ( 8 && !--clocks_remain )
         break;
      case 8:
      dsp_voice_V7_V4_V1( &dsp_m.voices [2] );
      if ( 9 && !--clocks_remain )
         break;
      case 9:
      dsp_voice_V8_V5_V2( &dsp_m.voices [2] );
      if ( 10 && !--clocks_remain )
         break;
      case 10:
      dsp_voice_V9_V6_V3( &dsp_m.voices [2] );
      if ( 11 && !--clocks_remain )
         break;
      case 11:
      dsp_voice_V7_V4_V1( &dsp_m.voices [3] );
      if ( 12 && !--clocks_remain )
         break;
      case 12:
      dsp_voice_V8_V5_V2( &dsp_m.voices [3] );
      if ( 13 && !--clocks_remain )
         break;
      case 13:
      dsp_voice_V9_V6_V3( &dsp_m.voices [3] );
      if ( 14 && !--clocks_remain )
         break;
      case 14:
      dsp_voice_V7_V4_V1( &dsp_m.voices [4] );
      if ( 15 && !--clocks_remain )
         break;
      case 15:
      dsp_voice_V8_V5_V2( &dsp_m.voices [4] );
      if ( 16 && !--clocks_remain )
         break;
      case 16:
      dsp_voice_V9_V6_V3( &dsp_m.voices [4] );
      if ( 17 && !--clocks_remain )
         break;
      case 17:
      dsp_voice_V1( &dsp_m.voices [0] );
      dsp_voice_V7( &dsp_m.voices [5] );
      dsp_voice_V4( &dsp_m.voices [6] );
      if ( 18 && !--clocks_remain )
         break;
      case 18:
      dsp_voice_V8_V5_V2( &dsp_m.voices [5] );
      if ( 19 && !--clocks_remain )
         break;
      case 19:
      dsp_voice_V9_V6_V3( &dsp_m.voices [5] );
      if ( 20 && !--clocks_remain )
         break;
      case 20:
      dsp_voice_V1( &dsp_m.voices [1] );
      dsp_voice_V7( &dsp_m.voices [6] );
      dsp_voice_V4( &dsp_m.voices [7] );
      if ( 21 && !--clocks_remain )
         break;
      case 21:
      dsp_voice_V8( &dsp_m.voices [6] );
      dsp_voice_V5( &dsp_m.voices [7] );
      dsp_voice_V2( &dsp_m.voices [0] );
      if ( 22 && !--clocks_remain )
         break;
      case 22:
      dsp_voice_V3a( &dsp_m.voices [0] );
      dsp_voice_V9( &dsp_m.voices [6] );
      dsp_voice_V6( &dsp_m.voices [7] );
      dsp_echo_22();
      if ( 23 && !--clocks_remain )
         break;
      case 23:
      dsp_voice_V7( &dsp_m.voices [7] );
      dsp_echo_23();
      if ( 24 && !--clocks_remain )
         break;
      case 24:
      dsp_voice_V8( &dsp_m.voices [7] );
      dsp_echo_24();
      if ( 25 && !--clocks_remain )
         break;
      case 25:
      dsp_voice_V3b( &dsp_m.voices [0] );
      dsp_voice_V9( &dsp_m.voices [7] );
      dsp_echo_25();
      if ( 26 && !--clocks_remain )
         break;
      case 26:
      dsp_echo_26();
      if ( 27 && !--clocks_remain )
         break;
      case 27:
      MISC_27();
      dsp_echo_27();
      if ( 28 && !--clocks_remain )
         break;
      case 28:
      MISC_28();
      ECHO_28();
      if ( 29 && !--clocks_remain )
         break;
      case 29:
      MISC_29();
      dsp_echo_29();
      if ( 30 && !--clocks_remain )
         break;
      case 30:
      dsp_misc_30();
      dsp_voice_V3c( &dsp_m.voices [0] );
      ECHO_WRITE(1);
      if ( 31 && !--clocks_remain )
         break;
      case 31:
      dsp_voice_V4( &dsp_m.voices [0] );
      dsp_voice_V1( &dsp_m.voices [2] );

      if ( --clocks_remain )
         goto loop;
   }
}

/* Sets destination for output samples. If out is NULL or out_size is 0,
   doesn't generate any. */

static void dsp_set_output( int16_t * out, int32_t size )
{
   if ( !out )
   {
      out  = dsp_m.extra;
      size = EXTRA_SIZE;
   }
   dsp_m.out_begin = out;
   dsp_m.out       = out;
   dsp_m.out_end   = out + size;
}

/* Setup */

static void dsp_soft_reset_common()
{
   dsp_m.noise              = 0x4000;
   dsp_m.echo_hist_pos      = dsp_m.echo_hist;
   dsp_m.every_other_sample = 1;
   dsp_m.echo_offset        = 0;
   dsp_m.phase              = 0;

   dsp_m.counter = 0;
}

/* Resets DSP to power-on state */

static void dsp_reset()
{
   int32_t i;

   uint8_t const initial_regs [REGISTER_COUNT] =
   {
      0x45,0x8B,0x5A,0x9A,0xE4,0x82,0x1B,0x78,0x00,0x00,0xAA,0x96,0x89,0x0E,0xE0,0x80,
      0x2A,0x49,0x3D,0xBA,0x14,0xA0,0xAC,0xC5,0x00,0x00,0x51,0xBB,0x9C,0x4E,0x7B,0xFF,
      0xF4,0xFD,0x57,0x32,0x37,0xD9,0x42,0x22,0x00,0x00,0x5B,0x3C,0x9F,0x1B,0x87,0x9A,
      0x6F,0x27,0xAF,0x7B,0xE5,0x68,0x0A,0xD9,0x00,0x00,0x9A,0xC5,0x9C,0x4E,0x7B,0xFF,
      0xEA,0x21,0x78,0x4F,0xDD,0xED,0x24,0x14,0x00,0x00,0x77,0xB1,0xD1,0x36,0xC1,0x67,
      0x52,0x57,0x46,0x3D,0x59,0xF4,0x87,0xA4,0x00,0x00,0x7E,0x44,0x00,0x4E,0x7B,0xFF,
      0x75,0xF5,0x06,0x97,0x10,0xC3,0x24,0xBB,0x00,0x00,0x7B,0x7A,0xE0,0x60,0x12,0x0F,
      0xF7,0x74,0x1C,0xE5,0x39,0x3D,0x73,0xC1,0x00,0x00,0x7A,0xB3,0xFF,0x4E,0x7B,0xFF
   };

   /* Resets DSP and uses supplied values to initialize registers */

   for (i = 0; i < REGISTER_COUNT; i++)
      dsp_m.regs[i] = initial_regs[i];

   /* Internal state */
   for ( i = VOICE_COUNT; --i >= 0; )
   {
      dsp_voice_t* v = &dsp_m.voices [i];
      v->brr_offset = 1;
      v->vbit       = 1 << i;
      v->regs       = &dsp_m.regs [i * 0x10];
   }
   dsp_m.new_kon = dsp_m.regs[R_KON];
   dsp_m.t_dir   = dsp_m.regs[R_DIR];
   dsp_m.t_esa   = dsp_m.regs[R_ESA];

   dsp_soft_reset_common();
}

/* Initializes DSP and has it use the 64K RAM provided */

static void dsp_init( void* ram_64k )
{
   dsp_m.ram = (uint8_t*) ram_64k;
   dsp_set_output( 0, 0 );
   dsp_reset();
}

/* Emulates pressing reset switch on SNES */

static void dsp_soft_reset(void)
{
   dsp_m.regs[R_FLG] = 0xE0;
   dsp_soft_reset_common();
}


/* ------------------------------------------------------------------ */
/* Public interface                                                    */
/* ------------------------------------------------------------------ */

/* Somewhere to point hi_ram when the caller does not model the IPL ROM
 * window — the echo path writes through it unconditionally. */
static uint8_t spc_dsp_dummy_hi_ram [ROM_WINDOW_BYTES];

void spc_dsp_init(void *ram_64k)
{
   memset(&dsp_m, 0, sizeof(dsp_m));
   dsp_m.rom_enabled = 0;
   dsp_m.hi_ram      = spc_dsp_dummy_hi_ram;
   dsp_m.rom         = spc_dsp_dummy_hi_ram;
   dsp_init(ram_64k);
}

void spc_dsp_init_ram(void *ram_64k)
{
   dsp_m.ram    = (uint8_t *)ram_64k;
   dsp_m.hi_ram = spc_dsp_dummy_hi_ram;
   dsp_m.rom    = spc_dsp_dummy_hi_ram;
}

void spc_dsp_reset(void)      { dsp_reset(); }
void spc_dsp_soft_reset(void) { dsp_soft_reset(); }

void spc_dsp_set_output(int16_t *out, int32_t size)
{
   dsp_set_output(out, size);
}

int32_t spc_dsp_samples_written(void)
{
   return (int32_t)(dsp_m.out - dsp_m.out_begin);
}

void spc_dsp_run(int32_t clocks)
{
   if (clocks > 0)
      dsp_run(clocks);
}

void spc_dsp_write(uint8_t addr, uint8_t data)
{
   /* This is spc_dsp_write() from apu_blargg.c with the register number
    * passed in rather than read out of the SPC700's $00F2 latch. */
   addr &= 0x7F;
   dsp_m.regs [addr] = data;

   switch (addr & 0x0F)
   {
   case V_ENVX:
      dsp_m.envx_buf = data;
      break;

   case V_OUTX:
      dsp_m.outx_buf = data;
      break;

   case 0x0C:
      if (addr == R_KON)
         dsp_m.new_kon = data;

      if (addr == R_ENDX)   /* always cleared, whatever was written */
      {
         dsp_m.endx_buf = 0;
         dsp_m.regs [R_ENDX] = 0;
      }
      break;
   }
}

uint8_t spc_dsp_read(uint8_t addr)
{
   return dsp_m.regs [addr & 0x7F];
}

uint8_t *spc_dsp_regs(void)
{
   return dsp_m.regs;
}

void   *spc_dsp_state(void)       { return &dsp_m; }
size_t  spc_dsp_state_size(void)  { return sizeof(dsp_m); }

int spc_dsp_rom_enabled(void)     { return dsp_m.rom_enabled; }
int16_t *spc_dsp_out_ptr(void)    { return dsp_m.out; }
int16_t *spc_dsp_extra_ptr(void)  { return dsp_m.extra; }
int32_t  spc_dsp_extra_size(void) { return EXTRA_SIZE; }

void spc_dsp_set_rom_window(int enabled, uint8_t *hi_ram, uint8_t *rom)
{
   dsp_m.rom_enabled = enabled;
   dsp_m.hi_ram      = hi_ram ? hi_ram : spc_dsp_dummy_hi_ram;
   dsp_m.rom         = rom    ? rom    : spc_dsp_dummy_hi_ram;
}
