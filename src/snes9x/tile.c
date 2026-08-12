/* This file is part of Snes9x. See LICENSE file. */

#include "snes9x.h"

#include "memmap.h"
#include "ppu.h"
#include "display.h"
#include "gfx.h"
#include "tile.h"
#include "colormath.h"

#if PICO_ON_DEVICE
/* Assembly-optimized tile pixel functions */
extern void WRITE_4PIXELS16_asm(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors);
extern void WRITE_4PIXELS16_FLIPPED_asm(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors);
extern void WRITE_4PIXELS16_OPAQUE_asm(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors);
extern void WRITE_4PIXELS16_FLIPPED_OPAQUE_asm(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors);
extern void WRITE_4PIXELS16x2_asm(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors);
extern void WRITE_4PIXELS16_FLIPPEDx2_asm(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors);
extern void WRITE_4PIXELS16x2_OPAQUE_asm(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors);
extern void WRITE_4PIXELS16_FLIPPEDx2_OPAQUE_asm(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors);
/* New 8-pixel row functions - reduce call overhead by handling full row at once */
extern void WRITE_8PIXELS16_OPAQUE_ROW_asm(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors);
extern void WRITE_8PIXELS16_FLIPPED_OPAQUE_ROW_asm(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors);
/* 4bpp tile conversion in assembly - handles most common tile format */
extern uint8_t ConvertTile4bpp_asm(uint8_t* pCache, uint32_t TileAddr);
#endif

/* Include optimized tile conversion header */
#if PICO_ON_DEVICE
#include "../tile_convert_opt.h"
#endif

static const uint32_t HeadMask[4] =
{
#ifdef MSB_FIRST
   0xffffffff, 0x00ffffff, 0x0000ffff, 0x000000ff
#else
   0xffffffff, 0xffffff00, 0xffff0000, 0xff000000
#endif
};

static const uint32_t TailMask[5] =
{
#ifdef MSB_FIRST
   0x00000000, 0xff000000, 0xffff0000, 0xffffff00, 0xffffffff
#else
   0x00000000, 0x000000ff, 0x0000ffff, 0x00ffffff, 0xffffffff
#endif
};

/* Lookup tables - exported for assembly optimizations */
const uint32_t odd[4][16] =
{
#ifdef MSB_FIRST
   {0x00000000, 0x00000001, 0x00000100, 0x00000101, 0x00010000, 0x00010001, 0x00010100, 0x00010101, 0x01000000, 0x01000001, 0x01000100, 0x01000101, 0x01010000, 0x01010001, 0x01010100, 0x01010101},
   {0x00000000, 0x00000004, 0x00000400, 0x00000404, 0x00040000, 0x00040004, 0x00040400, 0x00040404, 0x04000000, 0x04000004, 0x04000400, 0x04000404, 0x04040000, 0x04040004, 0x04040400, 0x04040404},
   {0x00000000, 0x00000010, 0x00001000, 0x00001010, 0x00100000, 0x00100010, 0x00101000, 0x00101010, 0x10000000, 0x10000010, 0x10001000, 0x10001010, 0x10100000, 0x10100010, 0x10101000, 0x10101010},
   {0x00000000, 0x00000040, 0x00004000, 0x00004040, 0x00400000, 0x00400040, 0x00404000, 0x00404040, 0x40000000, 0x40000040, 0x40004000, 0x40004040, 0x40400000, 0x40400040, 0x40404000, 0x40404040}
#else
   {0x00000000, 0x01000000, 0x00010000, 0x01010000, 0x00000100, 0x01000100, 0x00010100, 0x01010100, 0x00000001, 0x01000001, 0x00010001, 0x01010001, 0x00000101, 0x01000101, 0x00010101, 0x01010101},
   {0x00000000, 0x04000000, 0x00040000, 0x04040000, 0x00000400, 0x04000400, 0x00040400, 0x04040400, 0x00000004, 0x04000004, 0x00040004, 0x04040004, 0x00000404, 0x04000404, 0x00040404, 0x04040404},
   {0x00000000, 0x10000000, 0x00100000, 0x10100000, 0x00001000, 0x10001000, 0x00101000, 0x10101000, 0x00000010, 0x10000010, 0x00100010, 0x10100010, 0x00001010, 0x10001010, 0x00101010, 0x10101010},
   {0x00000000, 0x40000000, 0x00400000, 0x40400000, 0x00004000, 0x40004000, 0x00404000, 0x40404000, 0x00000040, 0x40000040, 0x00400040, 0x40400040, 0x00004040, 0x40004040, 0x00404040, 0x40404040}
#endif
};

const uint32_t even[4][16] =
{
#ifdef MSB_FIRST
   {0x00000000, 0x00000002, 0x00000200, 0x00000202, 0x00020000, 0x00020002, 0x00020200, 0x00020202, 0x02000000, 0x02000002, 0x02000200, 0x02000202, 0x02020000, 0x02020002, 0x02020200, 0x02020202},
   {0x00000000, 0x00000008, 0x00000800, 0x00000808, 0x00080000, 0x00080008, 0x00080800, 0x00080808, 0x08000000, 0x08000008, 0x08000800, 0x08000808, 0x08080000, 0x08080008, 0x08080800, 0x08080808},
   {0x00000000, 0x00000020, 0x00002000, 0x00002020, 0x00200000, 0x00200020, 0x00202000, 0x00202020, 0x20000000, 0x20000020, 0x20002000, 0x20002020, 0x20200000, 0x20200020, 0x20202000, 0x20202020},
   {0x00000000, 0x00000080, 0x00008000, 0x00008080, 0x00800000, 0x00800080, 0x00808000, 0x00808080, 0x80000000, 0x80000080, 0x80008000, 0x80008080, 0x80800000, 0x80800080, 0x80808000, 0x80808080}
#else
   {0x00000000, 0x02000000, 0x00020000, 0x02020000, 0x00000200, 0x02000200, 0x00020200, 0x02020200, 0x00000002, 0x02000002, 0x00020002, 0x02020002, 0x00000202, 0x02000202, 0x00020202, 0x02020202},
   {0x00000000, 0x08000000, 0x00080000, 0x08080000, 0x00000800, 0x08000800, 0x00080800, 0x08080800, 0x00000008, 0x08000008, 0x00080008, 0x08080008, 0x00000808, 0x08000808, 0x00080808, 0x08080808},
   {0x00000000, 0x20000000, 0x00200000, 0x20200000, 0x00002000, 0x20002000, 0x00202000, 0x20202000, 0x00000020, 0x20000020, 0x00200020, 0x20200020, 0x00002020, 0x20002020, 0x00202020, 0x20202020},
   {0x00000000, 0x80000000, 0x00800000, 0x80800000, 0x00008000, 0x80008000, 0x00808000, 0x80808000, 0x00000080, 0x80000080, 0x00800080, 0x80800080, 0x00008080, 0x80008080, 0x00808080, 0x80808080}
#endif
};

/*
 * Optimized tile conversion - branch-free version for better pipeline efficiency
 * 
 * Key optimizations:
 * 1. Remove conditional branches per pixel - always load and OR
 * 2. Compute opaque flag using bit manipulation instead of per-pixel boolean
 * 3. Use local variables to help register allocation
 * 4. On PICO, use ASM for 4bpp tiles (most common case)
 */
static uint8_t ConvertTile(uint8_t* pCache, uint32_t TileAddr)
{
   uint8_t* tp = &Memory.VRAM[TileAddr];
   uint32_t* p = (uint32_t*) pCache;
   uint32_t non_zero = 0;
   uint32_t has_transparent = 0;
   uint8_t line;
   uint32_t p1, p2;
   uint8_t pix;

   // Detect zero bytes in a 32-bit word (classic bit trick)
   #define HAS_ZERO_BYTE(x) (((x) - 0x01010101u) & ~(x) & 0x80808080u)

   switch (BG.BitShift)
   {
   case 8:
      for (line = 8; line != 0; line--, tp += 2)
      {
         // Branch-free: always load and OR, zero entries in tables produce 0
         pix = tp[0];
         p1 = odd[0][pix >> 4];
         p2 = odd[0][pix & 0xf];
         
         pix = tp[1];
         p1 |= even[0][pix >> 4];
         p2 |= even[0][pix & 0xf];
         
         pix = tp[16];
         p1 |= odd[1][pix >> 4];
         p2 |= odd[1][pix & 0xf];
         
         pix = tp[17];
         p1 |= even[1][pix >> 4];
         p2 |= even[1][pix & 0xf];
         
         pix = tp[32];
         p1 |= odd[2][pix >> 4];
         p2 |= odd[2][pix & 0xf];
         
         pix = tp[33];
         p1 |= even[2][pix >> 4];
         p2 |= even[2][pix & 0xf];
         
         pix = tp[48];
         p1 |= odd[3][pix >> 4];
         p2 |= odd[3][pix & 0xf];
         
         pix = tp[49];
         p1 |= even[3][pix >> 4];
         p2 |= even[3][pix & 0xf];
         
         *p++ = p1;
         *p++ = p2;
         non_zero |= p1 | p2;
         has_transparent |= HAS_ZERO_BYTE(p1) | HAS_ZERO_BYTE(p2);
      }
      break;

   case 4:
      // 4bpp tiles (Mode 1 backgrounds) - C version (ASM has bugs)
      for (line = 8; line != 0; line--, tp += 2)
      {
         pix = tp[0];
         p1 = odd[0][pix >> 4];
         p2 = odd[0][pix & 0xf];
         
         pix = tp[1];
         p1 |= even[0][pix >> 4];
         p2 |= even[0][pix & 0xf];
         
         pix = tp[16];
         p1 |= odd[1][pix >> 4];
         p2 |= odd[1][pix & 0xf];
         
         pix = tp[17];
         p1 |= even[1][pix >> 4];
         p2 |= even[1][pix & 0xf];
         
         *p++ = p1;
         *p++ = p2;
         non_zero |= p1 | p2;
         has_transparent |= HAS_ZERO_BYTE(p1) | HAS_ZERO_BYTE(p2);
      }
      break;

   case 2:
      for (line = 8; line != 0; line--, tp += 2)
      {
         pix = tp[0];
         p1 = odd[0][pix >> 4];
         p2 = odd[0][pix & 0xf];
         
         pix = tp[1];
         p1 |= even[0][pix >> 4];
         p2 |= even[0][pix & 0xf];
         
         *p++ = p1;
         *p++ = p2;
         non_zero |= p1 | p2;
         has_transparent |= HAS_ZERO_BYTE(p1) | HAS_ZERO_BYTE(p2);
      }
      break;
   }
   #undef HAS_ZERO_BYTE

   // Low 5 bits are the legacy validity/depth encoding; bit 0x20 marks opaque tiles
   if (!non_zero)
      return BLANK_TILE;
   return (0x10 | BG.Depth) | (has_transparent ? 0 : 0x20);
}

#define PLOT_PIXEL(screen, pixel) (pixel)

/*
 * Tile pixel-writing functions organized by platform:
 * - PICO_ON_DEVICE: Uses hand-optimized assembly
 * - ARM_ARCH: Uses C with manual unrolling
 * - Other: Uses simple loop-based C
 */

#if PICO_ON_DEVICE
/* ============ PICO: Assembly-optimized versions ============ */

static INLINE void WRITE_4PIXELS16(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   WRITE_4PIXELS16_asm(Offset, Pixels, ScreenColors);
}

static INLINE void WRITE_4PIXELS16_FLIPPED(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   WRITE_4PIXELS16_FLIPPED_asm(Offset, Pixels, ScreenColors);
}

static INLINE void WRITE_4PIXELS16_OPAQUE(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   WRITE_4PIXELS16_OPAQUE_asm(Offset, Pixels, ScreenColors);
}

static INLINE void WRITE_4PIXELS16_FLIPPED_OPAQUE(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   WRITE_4PIXELS16_FLIPPED_OPAQUE_asm(Offset, Pixels, ScreenColors);
}

static INLINE void WRITE_4PIXELS16x2_OPAQUE(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   WRITE_4PIXELS16x2_OPAQUE_asm(Offset, Pixels, ScreenColors);
}

static INLINE void WRITE_4PIXELS16_FLIPPEDx2_OPAQUE(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   WRITE_4PIXELS16_FLIPPEDx2_OPAQUE_asm(Offset, Pixels, ScreenColors);
}

static INLINE void WRITE_4PIXELS16x2(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   WRITE_4PIXELS16x2_asm(Offset, Pixels, ScreenColors);
}

static INLINE void WRITE_4PIXELS16_FLIPPEDx2(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   WRITE_4PIXELS16_FLIPPEDx2_asm(Offset, Pixels, ScreenColors);
}

#elif defined(__ARM_ARCH_8M_MAIN__) || defined(__ARM_ARCH_7M__)
/* ============ ARM: Unrolled C versions ============ */

static INLINE void WRITE_4PIXELS16(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.DB + Offset;
   uint8_t Z1 = GFX.Z1;
   uint8_t Z2 = GFX.Z2;
   uint8_t Pixel;
   
   if (Z1 > Depth[0] && (Pixel = Pixels[0])) { Screen[0] = ScreenColors[Pixel]; Depth[0] = Z2; }
   if (Z1 > Depth[1] && (Pixel = Pixels[1])) { Screen[1] = ScreenColors[Pixel]; Depth[1] = Z2; }
   if (Z1 > Depth[2] && (Pixel = Pixels[2])) { Screen[2] = ScreenColors[Pixel]; Depth[2] = Z2; }
   if (Z1 > Depth[3] && (Pixel = Pixels[3])) { Screen[3] = ScreenColors[Pixel]; Depth[3] = Z2; }
}

static INLINE void WRITE_4PIXELS16_FLIPPED(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.DB + Offset;
   uint8_t Z1 = GFX.Z1;
   uint8_t Z2 = GFX.Z2;
   uint8_t Pixel;
   
   if (Z1 > Depth[0] && (Pixel = Pixels[3])) { Screen[0] = ScreenColors[Pixel]; Depth[0] = Z2; }
   if (Z1 > Depth[1] && (Pixel = Pixels[2])) { Screen[1] = ScreenColors[Pixel]; Depth[1] = Z2; }
   if (Z1 > Depth[2] && (Pixel = Pixels[1])) { Screen[2] = ScreenColors[Pixel]; Depth[2] = Z2; }
   if (Z1 > Depth[3] && (Pixel = Pixels[0])) { Screen[3] = ScreenColors[Pixel]; Depth[3] = Z2; }
}

static INLINE void WRITE_4PIXELS16_OPAQUE(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.DB + Offset;
   uint8_t Z1 = GFX.Z1;
   uint8_t Z2 = GFX.Z2;

   if (Z1 > Depth[0]) { Screen[0] = ScreenColors[Pixels[0]]; Depth[0] = Z2; }
   if (Z1 > Depth[1]) { Screen[1] = ScreenColors[Pixels[1]]; Depth[1] = Z2; }
   if (Z1 > Depth[2]) { Screen[2] = ScreenColors[Pixels[2]]; Depth[2] = Z2; }
   if (Z1 > Depth[3]) { Screen[3] = ScreenColors[Pixels[3]]; Depth[3] = Z2; }
}

static INLINE void WRITE_4PIXELS16_FLIPPED_OPAQUE(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.DB + Offset;
   uint8_t Z1 = GFX.Z1;
   uint8_t Z2 = GFX.Z2;

   if (Z1 > Depth[0]) { Screen[0] = ScreenColors[Pixels[3]]; Depth[0] = Z2; }
   if (Z1 > Depth[1]) { Screen[1] = ScreenColors[Pixels[2]]; Depth[1] = Z2; }
   if (Z1 > Depth[2]) { Screen[2] = ScreenColors[Pixels[1]]; Depth[2] = Z2; }
   if (Z1 > Depth[3]) { Screen[3] = ScreenColors[Pixels[0]]; Depth[3] = Z2; }
}

static INLINE void WRITE_4PIXELS16x2_OPAQUE(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.DB + Offset;
   uint8_t Z1 = GFX.Z1;
   uint8_t Z2 = GFX.Z2;

   if (Z1 > Depth[0]) { Screen[0] = Screen[1] = ScreenColors[Pixels[0]]; Depth[0] = Depth[1] = Z2; }
   if (Z1 > Depth[2]) { Screen[2] = Screen[3] = ScreenColors[Pixels[1]]; Depth[2] = Depth[3] = Z2; }
   if (Z1 > Depth[4]) { Screen[4] = Screen[5] = ScreenColors[Pixels[2]]; Depth[4] = Depth[5] = Z2; }
   if (Z1 > Depth[6]) { Screen[6] = Screen[7] = ScreenColors[Pixels[3]]; Depth[6] = Depth[7] = Z2; }
}

static INLINE void WRITE_4PIXELS16_FLIPPEDx2_OPAQUE(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.DB + Offset;
   uint8_t Z1 = GFX.Z1;
   uint8_t Z2 = GFX.Z2;

   if (Z1 > Depth[0]) { Screen[0] = Screen[1] = ScreenColors[Pixels[3]]; Depth[0] = Depth[1] = Z2; }
   if (Z1 > Depth[2]) { Screen[2] = Screen[3] = ScreenColors[Pixels[2]]; Depth[2] = Depth[3] = Z2; }
   if (Z1 > Depth[4]) { Screen[4] = Screen[5] = ScreenColors[Pixels[1]]; Depth[4] = Depth[5] = Z2; }
   if (Z1 > Depth[6]) { Screen[6] = Screen[7] = ScreenColors[Pixels[0]]; Depth[6] = Depth[7] = Z2; }
}

#else
/* ============ Fallback: Simple loop-based C ============ */

static INLINE void WRITE_4PIXELS16(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t  Pixel, N;
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.DB + Offset;

   for (N = 0; N < 4; N++)
   {
      if (GFX.Z1 > Depth [N] && (Pixel = Pixels[N]))
      {
         Screen [N] = ScreenColors [Pixel];
         Depth [N] = GFX.Z2;
      }
   }
}

static INLINE void WRITE_4PIXELS16_FLIPPED(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t  Pixel, N;
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.DB + Offset;

   for (N = 0; N < 4; N++)
   {
      if (GFX.Z1 > Depth [N] && (Pixel = Pixels[3 - N]))
      {
         Screen [N] = ScreenColors [Pixel];
         Depth [N] = GFX.Z2;
      }
   }
}

/* Opaque variants not implemented for non-ARM - use regular versions */
#define WRITE_4PIXELS16_OPAQUE WRITE_4PIXELS16
#define WRITE_4PIXELS16_FLIPPED_OPAQUE WRITE_4PIXELS16_FLIPPED
#define WRITE_4PIXELS16x2_OPAQUE WRITE_4PIXELS16x2
#define WRITE_4PIXELS16_FLIPPEDx2_OPAQUE WRITE_4PIXELS16_FLIPPEDx2

#endif /* platform selection */

static void WRITE_4PIXELS16_HALFWIDTH(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t  Pixel, N;
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.DB + Offset;

   for (N = 0; N < 4; N += 2)
   {
      if (GFX.Z1 > Depth [N] && (Pixel = Pixels[N]))
      {
         Screen [N >> 1] = ScreenColors [Pixel];
         Depth [N >> 1] = GFX.Z2;
      }
   }
}

static void WRITE_4PIXELS16_FLIPPED_HALFWIDTH(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t  Pixel, N;
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.DB + Offset;

   for (N = 0; N < 4; N += 2)
   {
      if (GFX.Z1 > Depth [N] && (Pixel = Pixels[2 - N]))
      {
         Screen [N >> 1] = ScreenColors [Pixel];
         Depth [N >> 1] = GFX.Z2;
      }
   }
}

#if !PICO_ON_DEVICE
/* Non-PICO platforms use loop-based x2 functions */
static void WRITE_4PIXELS16x2(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t  Pixel, N;
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.DB + Offset;

   for (N = 0; N < 4; N++)
   {
      if (GFX.Z1 > Depth [N * 2] && (Pixel = Pixels[N]))
      {
         Screen [N * 2] = Screen [N * 2 + 1] = ScreenColors [Pixel];
         Depth [N * 2] = Depth [N * 2 + 1] = GFX.Z2;
      }
   }
}

static void WRITE_4PIXELS16_FLIPPEDx2(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t  Pixel, N;
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.DB + Offset;

   for (N = 0; N < 4; N++)
   {
      if (GFX.Z1 > Depth [N * 2] && (Pixel = Pixels[3 - N]))
      {
         Screen [N * 2] = Screen [N * 2 + 1] = ScreenColors [Pixel];
         Depth [N * 2] = Depth [N * 2 + 1] = GFX.Z2;
      }
   }
}
#endif /* !PICO_ON_DEVICE */

static void WRITE_4PIXELS16x2x2(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t  Pixel, N;
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.DB + Offset;

   for (N = 0; N < 4; N++)
   {
      if (GFX.Z1 > Depth [N * 2] && (Pixel = Pixels[N]))
      {
         Screen [N * 2] = Screen [N * 2 + 1] = Screen [GFX.RealPitch + N * 2] = Screen [GFX.RealPitch + N * 2 + 1] = ScreenColors [Pixel];
         Depth [N * 2] = Depth [N * 2 + 1] = Depth [GFX.RealPitch + N * 2] = Depth [GFX.RealPitch + N * 2 + 1] = GFX.Z2;
      }
   }
}

static void WRITE_4PIXELS16_FLIPPEDx2x2(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t  Pixel, N;
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.DB + Offset;

   for (N = 0; N < 4; N++)
   {
      if (GFX.Z1 > Depth [N * 2] && (Pixel = Pixels[3 - N]))
      {
         Screen [N * 2] = Screen [N * 2 + 1] = Screen [GFX.RealPitch + N * 2] = Screen [GFX.RealPitch + N * 2 + 1] = ScreenColors [Pixel];
         Depth [N * 2] = Depth [N * 2 + 1] = Depth [GFX.RealPitch + N * 2] = Depth [GFX.RealPitch + N * 2 + 1] = GFX.Z2;
      }
   }
}

/*
 * Direct-decode tile rendering for 4bpp tiles
 * Decodes planar VRAM data inline without using the tile cache.
 * Saves ConvertTile overhead when VRAM is frequently written (animation-heavy games).
 *
 * The planar layout for 4bpp:
 *   Row R: bytes at offset R*2+0, R*2+1, R*2+16, R*2+17
 *   Each byte contains one bit of each of 8 pixels
 *
 * Uses the same odd/even lookup tables as ConvertTile to decode 4 pixels at a time.
 */
#if PICO_ON_DEVICE && defined(FRANK_SNES_FAST_MODE)
#define DIRECT_DECODE_TILES 1

static void DrawTile16_Direct4bpp(uint32_t Tile, int32_t Offset, uint32_t StartLine, uint32_t LineCount)
{
   uint32_t TileAddr = BG.TileAddress + ((Tile & 0x3ff) << BG.TileShift);
   if ((Tile & 0x1ff) >= 256)
      TileAddr += BG.NameSelect;
   TileAddr &= 0xffff;
   
   uint8_t* tp = &Memory.VRAM[TileAddr + (StartLine >> 3) * 2];  /* Starting row in tile */
   uint16_t* ScreenColors;
   
   if (BG.DirectColourMode)
      ScreenColors = &IPPU.DirectColors[((Tile >> 10) & BG.PaletteMask) << 8];
   else
      ScreenColors = &IPPU.ScreenColors[(((Tile >> 10) & BG.PaletteMask) << BG.PaletteShift) + BG.StartPalette];
   
   uint8_t* Screen = GFX.S + Offset;
   uint8_t* Depth = GFX.DB + Offset;
   uint8_t Z1 = GFX.Z1;
   uint8_t Z2 = GFX.Z2;
   
   uint32_t hflip = Tile & H_FLIP;
   uint32_t vflip = Tile & V_FLIP;
   
   if (vflip) {
      tp = &Memory.VRAM[TileAddr + (7 - (StartLine >> 3)) * 2];
   }
   
   for (uint32_t l = LineCount; l != 0; l--)
   {
      /* Decode one row: 8 pixels from 4 bitplanes */
      uint8_t b0 = tp[0];
      uint8_t b1 = tp[1];
      uint8_t b2 = tp[16];
      uint8_t b3 = tp[17];
      
      /* Decode high 4 pixels using lookup tables */
      uint32_t p1 = odd[0][b0 >> 4] | even[0][b1 >> 4] | odd[1][b2 >> 4] | even[1][b3 >> 4];
      /* Decode low 4 pixels */
      uint32_t p2 = odd[0][b0 & 0xf] | even[0][b1 & 0xf] | odd[1][b2 & 0xf] | even[1][b3 & 0xf];
      
      uint8_t* pixels = (uint8_t*)&p1;  /* First 4 pixels */
      uint8_t* pixels2 = (uint8_t*)&p2; /* Next 4 pixels */
      
      if (hflip) {
         /* Write pixels in reverse order */
         for (int i = 0; i < 4; i++) {
            uint8_t pix = pixels2[3-i];
            if (Z1 > Depth[i] && pix) { Screen[i] = ScreenColors[pix]; Depth[i] = Z2; }
         }
         for (int i = 0; i < 4; i++) {
            uint8_t pix = pixels[3-i];
            if (Z1 > Depth[4+i] && pix) { Screen[4+i] = ScreenColors[pix]; Depth[4+i] = Z2; }
         }
      } else {
         /* Normal order */
         for (int i = 0; i < 4; i++) {
            uint8_t pix = pixels[i];
            if (Z1 > Depth[i] && pix) { Screen[i] = ScreenColors[pix]; Depth[i] = Z2; }
         }
         for (int i = 0; i < 4; i++) {
            uint8_t pix = pixels2[i];
            if (Z1 > Depth[4+i] && pix) { Screen[4+i] = ScreenColors[pix]; Depth[4+i] = Z2; }
         }
      }
      
      Screen += GFX.PPL;
      Depth += GFX.PPL;
      if (vflip) tp -= 2; else tp += 2;
   }
}

/* Direct decode version for clipped tiles - handles StartPixel and Width for edge tiles */
static void DrawClippedTile16_Direct4bpp(uint32_t Tile, int32_t Offset, uint32_t StartPixel, uint32_t Width, uint32_t StartLine, uint32_t LineCount)
{
   uint32_t TileAddr = BG.TileAddress + ((Tile & 0x3ff) << BG.TileShift);
   if ((Tile & 0x1ff) >= 256)
      TileAddr += BG.NameSelect;
   TileAddr &= 0xffff;
   
   uint8_t* tp = &Memory.VRAM[TileAddr + (StartLine >> 3) * 2];
   uint16_t* ScreenColors;
   
   if (BG.DirectColourMode)
      ScreenColors = &IPPU.DirectColors[((Tile >> 10) & BG.PaletteMask) << 8];
   else
      ScreenColors = &IPPU.ScreenColors[(((Tile >> 10) & BG.PaletteMask) << BG.PaletteShift) + BG.StartPalette];
   
   uint8_t* Screen = GFX.S + Offset;
   uint8_t* Depth = GFX.DB + Offset;
   uint8_t Z1 = GFX.Z1;
   uint8_t Z2 = GFX.Z2;
   
   uint32_t hflip = Tile & H_FLIP;
   uint32_t vflip = Tile & V_FLIP;
   
   if (vflip) {
      tp = &Memory.VRAM[TileAddr + (7 - (StartLine >> 3)) * 2];
   }
   
   /* Precompute which pixels to actually render */
   uint32_t endPixel = StartPixel + Width;
   
   for (uint32_t l = LineCount; l != 0; l--)
   {
      /* Decode one row: 8 pixels from 4 bitplanes */
      uint8_t b0 = tp[0];
      uint8_t b1 = tp[1];
      uint8_t b2 = tp[16];
      uint8_t b3 = tp[17];
      
      /* Decode all 8 pixel indices */
      uint32_t p1 = odd[0][b0 >> 4] | even[0][b1 >> 4] | odd[1][b2 >> 4] | even[1][b3 >> 4];
      uint32_t p2 = odd[0][b0 & 0xf] | even[0][b1 & 0xf] | odd[1][b2 & 0xf] | even[1][b3 & 0xf];
      
      uint8_t pixels[8];
      *(uint32_t*)&pixels[0] = p1;
      *(uint32_t*)&pixels[4] = p2;
      
      /* Only render pixels within [StartPixel, StartPixel+Width) */
      for (uint32_t i = StartPixel; i < endPixel; i++) {
         uint32_t srcIdx = hflip ? (7 - i) : i;
         uint8_t pix = pixels[srcIdx];
         if (Z1 > Depth[i] && pix) {
            Screen[i] = ScreenColors[pix];
            Depth[i] = Z2;
         }
      }
      
      Screen += GFX.PPL;
      Depth += GFX.PPL;
      if (vflip) tp -= 2; else tp += 2;
   }
}
#endif

/* Counted, not timed: 6,000+ calls/frame means a time_us_32() pair per call
   would cost far more than the thing being measured. Dividing the main-screen
   timer by this gives the true per-tile cost. */
volatile uint32_t frank_tile_calls;
volatile uint32_t frank_tile_lines;

void DrawTile16(uint32_t Tile, int32_t Offset, uint32_t StartLine, uint32_t LineCount)
{
   frank_tile_calls++;
   frank_tile_lines += LineCount;
#if defined(DIRECT_DECODE_TILES)
   /* For 4bpp tiles, use direct decode to skip tile cache entirely */
   if (BG.BitShift == 4) {
      DrawTile16_Direct4bpp(Tile, Offset, StartLine, LineCount);
      return;
   }
#endif

   uint8_t* bp;
   TILE_PREAMBLE_VARS();
   TILE_PREAMBLE_CODE();
#if PICO_ON_DEVICE
   /* Use full ASM tile renderer - processes all rows in one call */
   RENDER_TILE_FULL_ASM();
#else
   RENDER_TILE(WRITE_4PIXELS16, WRITE_4PIXELS16_FLIPPED, 4);
#endif
}

void DrawClippedTile16(uint32_t Tile, int32_t Offset, uint32_t StartPixel, uint32_t Width, uint32_t StartLine, uint32_t LineCount)
{
#if defined(DIRECT_DECODE_TILES)
   /* For 4bpp tiles, use direct decode to skip tile cache entirely */
   if (BG.BitShift == 4) {
      DrawClippedTile16_Direct4bpp(Tile, Offset, StartPixel, Width, StartLine, LineCount);
      return;
   }
#endif

   uint8_t* bp;
   TILE_PREAMBLE_VARS();
   TILE_CLIP_PREAMBLE_VARS();
   RENDER_CLIPPED_TILE_VARS();
   TILE_PREAMBLE_CODE();
   TILE_CLIP_PREAMBLE_CODE();
   RENDER_CLIPPED_TILE_CODE(WRITE_4PIXELS16, WRITE_4PIXELS16_FLIPPED, 4);
}

void DrawTile16HalfWidth(uint32_t Tile, int32_t Offset, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* bp;
   TILE_PREAMBLE_VARS();
   TILE_PREAMBLE_CODE();
   RENDER_TILE(WRITE_4PIXELS16_HALFWIDTH, WRITE_4PIXELS16_FLIPPED_HALFWIDTH, 2);
}

void DrawClippedTile16HalfWidth(uint32_t Tile, int32_t Offset, uint32_t StartPixel, uint32_t Width, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* bp;
   TILE_PREAMBLE_VARS();
   TILE_CLIP_PREAMBLE_VARS();
   RENDER_CLIPPED_TILE_VARS();
   TILE_PREAMBLE_CODE();
   TILE_CLIP_PREAMBLE_CODE();
   RENDER_CLIPPED_TILE_CODE(WRITE_4PIXELS16_HALFWIDTH, WRITE_4PIXELS16_FLIPPED_HALFWIDTH, 2);
}

void DrawTile16x2(uint32_t Tile, int32_t Offset, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* bp;
   TILE_PREAMBLE_VARS();
   TILE_PREAMBLE_CODE();
#if PICO_ON_DEVICE
   RENDER_TILE_OPAQUE(WRITE_4PIXELS16x2, WRITE_4PIXELS16_FLIPPEDx2, WRITE_4PIXELS16x2_OPAQUE, WRITE_4PIXELS16_FLIPPEDx2_OPAQUE, 8);
#else
   RENDER_TILE(WRITE_4PIXELS16x2, WRITE_4PIXELS16_FLIPPEDx2, 8);
#endif
}

void DrawClippedTile16x2(uint32_t Tile, int32_t Offset, uint32_t StartPixel, uint32_t Width, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* bp;
   TILE_PREAMBLE_VARS();
   TILE_CLIP_PREAMBLE_VARS();
   RENDER_CLIPPED_TILE_VARS();
   TILE_PREAMBLE_CODE();
   TILE_CLIP_PREAMBLE_CODE();
   RENDER_CLIPPED_TILE_CODE(WRITE_4PIXELS16x2, WRITE_4PIXELS16_FLIPPEDx2, 8);
}

void DrawTile16x2x2(uint32_t Tile, int32_t Offset, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* bp;
   TILE_PREAMBLE_VARS();
   TILE_PREAMBLE_CODE();
   RENDER_TILE(WRITE_4PIXELS16x2x2, WRITE_4PIXELS16_FLIPPEDx2x2, 8);
}

void DrawClippedTile16x2x2(uint32_t Tile, int32_t Offset, uint32_t StartPixel, uint32_t Width, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* bp;
   TILE_PREAMBLE_VARS();
   TILE_CLIP_PREAMBLE_VARS();
   RENDER_CLIPPED_TILE_VARS();
   TILE_PREAMBLE_CODE();
   TILE_CLIP_PREAMBLE_CODE();
   RENDER_CLIPPED_TILE_CODE(WRITE_4PIXELS16x2x2, WRITE_4PIXELS16_FLIPPEDx2x2, 8);
}

void DrawLargePixel16(uint32_t Tile, int32_t Offset, uint32_t StartPixel, uint32_t Pixels, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t pixel;
   uint8_t *sp;
   uint8_t *Depth;
   TILE_PREAMBLE_VARS();
   TILE_PREAMBLE_CODE();
   sp = GFX.S + Offset;
   Depth = GFX.DB + Offset;
   RENDER_TILE_LARGE(ScreenColors [pixel], PLOT_PIXEL);
}

void DrawLargePixel16HalfWidth(uint32_t Tile, int32_t Offset, uint32_t StartPixel, uint32_t Pixels, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t pixel;
   uint8_t *sp;
   uint8_t *Depth;
   TILE_PREAMBLE_VARS();
   TILE_PREAMBLE_CODE();
   sp = GFX.S + Offset;
   Depth = GFX.DB + Offset;
   RENDER_TILE_LARGE_HALFWIDTH(ScreenColors [pixel], PLOT_PIXEL);
}

static void WRITE_4PIXELS16_ADD(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t  Pixel, N;
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.ZBuffer + Offset;
   uint8_t*  SubDepth = GFX.SubZBuffer + Offset;

   for (N = 0; N < 4; N++)
   {
      if (GFX.Z1 > Depth [N] && (Pixel = Pixels[N]))
      {
         uint8_t fg = ScreenColors[Pixel];
         switch (SubDepth[N])
         {
         case 0:
            Screen[N] = fg;
            break;
         case 1:
            Screen[N] = colormath_fixed_add(fg, GFX.FixedColour15);
            break;
         default:
            Screen[N] = colormath_add(fg, *(Screen + GFX.Delta + N));
            break;
         }
         Depth [N] = GFX.Z2;
      }
   }
}

static void WRITE_4PIXELS16_FLIPPED_ADD(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t  Pixel, N;
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.ZBuffer + Offset;
   uint8_t*  SubDepth = GFX.SubZBuffer + Offset;

   for (N = 0; N < 4; N++)
   {
      if (GFX.Z1 > Depth [N] && (Pixel = Pixels[3 - N]))
      {
         uint8_t fg = ScreenColors[Pixel];
         switch (SubDepth[N])
         {
         case 0:
            Screen[N] = fg;
            break;
         case 1:
            Screen[N] = colormath_fixed_add(fg, GFX.FixedColour15);
            break;
         default:
            Screen[N] = colormath_add(fg, *(Screen + GFX.Delta + N));
            break;
         }
         Depth [N] = GFX.Z2;
      }
   }
}

static void WRITE_4PIXELS16_ADD1_2(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t  Pixel, N;
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.ZBuffer + Offset;
   uint8_t*  SubDepth = GFX.SubZBuffer + Offset;

   for (N = 0; N < 4; N++)
   {
      if (GFX.Z1 > Depth [N] && (Pixel = Pixels[N]))
      {
         uint8_t fg = ScreenColors[Pixel];
         switch (SubDepth[N])
         {
         case 0:
            Screen[N] = fg;
            break;
         case 1:
            Screen[N] = colormath_fixed_add(fg, GFX.FixedColour15);
            break;
         default:
            Screen[N] = colormath_add_half(fg, *(Screen + GFX.Delta + N));
            break;
         }
         Depth [N] = GFX.Z2;
      }
   }
}

static void WRITE_4PIXELS16_FLIPPED_ADD1_2(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t  Pixel, N;
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.ZBuffer + Offset;
   uint8_t*  SubDepth = GFX.SubZBuffer + Offset;

   for (N = 0; N < 4; N++)
   {
      if (GFX.Z1 > Depth [N] && (Pixel = Pixels[3 - N]))
      {
         uint8_t fg = ScreenColors[Pixel];
         switch (SubDepth[N])
         {
         case 0:
            Screen[N] = fg;
            break;
         case 1:
            Screen[N] = colormath_fixed_add(fg, GFX.FixedColour15);
            break;
         default:
            Screen[N] = colormath_add_half(fg, *(Screen + GFX.Delta + N));
            break;
         }
         Depth [N] = GFX.Z2;
      }
   }
}

static void WRITE_4PIXELS16_SUB(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t  Pixel, N;
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.ZBuffer + Offset;
   uint8_t*  SubDepth = GFX.SubZBuffer + Offset;

   for (N = 0; N < 4; N++)
   {
      if (GFX.Z1 > Depth [N] && (Pixel = Pixels[N]))
      {
         uint8_t fg = ScreenColors[Pixel];
         switch (SubDepth[N])
         {
         case 0:
            Screen[N] = fg;
            break;
         case 1:
            Screen[N] = colormath_fixed_sub(fg, GFX.FixedColour15);
            break;
         default:
            Screen[N] = colormath_sub(fg, *(Screen + GFX.Delta + N));
            break;
         }
         Depth [N] = GFX.Z2;
      }
   }
}

static void WRITE_4PIXELS16_FLIPPED_SUB(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t  Pixel, N;
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.ZBuffer + Offset;
   uint8_t*  SubDepth = GFX.SubZBuffer + Offset;

   for (N = 0; N < 4; N++)
   {
      if (GFX.Z1 > Depth [N] && (Pixel = Pixels[3 - N]))
      {
         uint8_t fg = ScreenColors[Pixel];
         switch (SubDepth[N])
         {
         case 0:
            Screen[N] = fg;
            break;
         case 1:
            Screen[N] = colormath_fixed_sub(fg, GFX.FixedColour15);
            break;
         default:
            Screen[N] = colormath_sub(fg, *(Screen + GFX.Delta + N));
            break;
         }
         Depth [N] = GFX.Z2;
      }
   }
}

static void WRITE_4PIXELS16_SUB1_2(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t  Pixel, N;
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.ZBuffer + Offset;
   uint8_t*  SubDepth = GFX.SubZBuffer + Offset;

   for (N = 0; N < 4; N++)
   {
      if (GFX.Z1 > Depth [N] && (Pixel = Pixels[N]))
      {
         uint8_t fg = ScreenColors[Pixel];
         switch (SubDepth[N])
         {
         case 0:
            Screen[N] = fg;
            break;
         case 1:
            Screen[N] = colormath_fixed_sub(fg, GFX.FixedColour15);
            break;
         default:
            Screen[N] = colormath_sub_half(fg, *(Screen + GFX.Delta + N));
            break;
         }
         Depth [N] = GFX.Z2;
      }
   }
}

static void WRITE_4PIXELS16_FLIPPED_SUB1_2(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t  Pixel, N;
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.ZBuffer + Offset;
   uint8_t*  SubDepth = GFX.SubZBuffer + Offset;

   for (N = 0; N < 4; N++)
   {
      if (GFX.Z1 > Depth [N] && (Pixel = Pixels[3 - N]))
      {
         uint8_t fg = ScreenColors[Pixel];
         switch (SubDepth[N])
         {
         case 0:
            Screen[N] = fg;
            break;
         case 1:
            Screen[N] = colormath_fixed_sub(fg, GFX.FixedColour15);
            break;
         default:
            Screen[N] = colormath_sub_half(fg, *(Screen + GFX.Delta + N));
            break;
         }
         Depth [N] = GFX.Z2;
      }
   }
}


void DrawTile16Add(uint32_t Tile, int32_t Offset, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* bp;
   TILE_PREAMBLE_VARS();
   TILE_PREAMBLE_CODE();
   RENDER_TILE(WRITE_4PIXELS16_ADD, WRITE_4PIXELS16_FLIPPED_ADD, 4);
}

void DrawClippedTile16Add(uint32_t Tile, int32_t Offset, uint32_t StartPixel, uint32_t Width, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* bp;
   TILE_PREAMBLE_VARS();
   TILE_CLIP_PREAMBLE_VARS();
   RENDER_CLIPPED_TILE_VARS();
   TILE_PREAMBLE_CODE();
   TILE_CLIP_PREAMBLE_CODE();
   RENDER_CLIPPED_TILE_CODE(WRITE_4PIXELS16_ADD, WRITE_4PIXELS16_FLIPPED_ADD, 4);
}

void DrawTile16Add1_2(uint32_t Tile, int32_t Offset, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* bp;
   TILE_PREAMBLE_VARS();
   TILE_PREAMBLE_CODE();
   RENDER_TILE(WRITE_4PIXELS16_ADD1_2, WRITE_4PIXELS16_FLIPPED_ADD1_2, 4);
}

void DrawClippedTile16Add1_2(uint32_t Tile, int32_t Offset, uint32_t StartPixel, uint32_t Width, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* bp;
   TILE_PREAMBLE_VARS();
   TILE_CLIP_PREAMBLE_VARS();
   RENDER_CLIPPED_TILE_VARS();
   TILE_PREAMBLE_CODE();
   TILE_CLIP_PREAMBLE_CODE();
   RENDER_CLIPPED_TILE_CODE(WRITE_4PIXELS16_ADD1_2, WRITE_4PIXELS16_FLIPPED_ADD1_2, 4);
}

void DrawTile16Sub(uint32_t Tile, int32_t Offset, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* bp;
   TILE_PREAMBLE_VARS();
   TILE_PREAMBLE_CODE();
   RENDER_TILE(WRITE_4PIXELS16_SUB, WRITE_4PIXELS16_FLIPPED_SUB, 4);
}

void DrawClippedTile16Sub(uint32_t Tile, int32_t Offset, uint32_t StartPixel, uint32_t Width,  uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* bp;
   TILE_PREAMBLE_VARS();
   TILE_CLIP_PREAMBLE_VARS();
   RENDER_CLIPPED_TILE_VARS();
   TILE_PREAMBLE_CODE();
   TILE_CLIP_PREAMBLE_CODE();
   RENDER_CLIPPED_TILE_CODE(WRITE_4PIXELS16_SUB, WRITE_4PIXELS16_FLIPPED_SUB, 4);
}

void DrawTile16Sub1_2(uint32_t Tile, int32_t Offset, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* bp;
   TILE_PREAMBLE_VARS();
   TILE_PREAMBLE_CODE();
   RENDER_TILE(WRITE_4PIXELS16_SUB1_2, WRITE_4PIXELS16_FLIPPED_SUB1_2, 4);
}

void DrawClippedTile16Sub1_2(uint32_t Tile, int32_t Offset, uint32_t StartPixel, uint32_t Width, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* bp;
   TILE_PREAMBLE_VARS();
   TILE_CLIP_PREAMBLE_VARS();
   RENDER_CLIPPED_TILE_VARS();
   TILE_PREAMBLE_CODE();
   TILE_CLIP_PREAMBLE_CODE();
   RENDER_CLIPPED_TILE_CODE(WRITE_4PIXELS16_SUB1_2, WRITE_4PIXELS16_FLIPPED_SUB1_2, 4);
}

static void WRITE_4PIXELS16_ADDF1_2(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t  Pixel, N;
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.ZBuffer + Offset;
   uint8_t*  SubDepth = GFX.SubZBuffer + Offset;

   for (N = 0; N < 4; N++)
   {
      if (GFX.Z1 > Depth [N] && (Pixel = Pixels[N]))
      {
         if (SubDepth[N] == 1)
            Screen[N] = colormath_fixed_add_half(ScreenColors[Pixel], GFX.FixedColour15);
         else
            Screen[N] = ScreenColors[Pixel];
         Depth [N] = GFX.Z2;
      }
   }
}

static void WRITE_4PIXELS16_FLIPPED_ADDF1_2(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t  Pixel, N;
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.ZBuffer + Offset;
   uint8_t*  SubDepth = GFX.SubZBuffer + Offset;

   for (N = 0; N < 4; N++)
   {
      if (GFX.Z1 > Depth [N] && (Pixel = Pixels[3 - N]))
      {
         if (SubDepth[N] == 1)
            Screen[N] = colormath_fixed_add_half(ScreenColors[Pixel], GFX.FixedColour15);
         else
            Screen[N] = ScreenColors[Pixel];
         Depth [N] = GFX.Z2;
      }
   }
}

static void WRITE_4PIXELS16_SUBF1_2(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t  Pixel, N;
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.ZBuffer + Offset;
   uint8_t*  SubDepth = GFX.SubZBuffer + Offset;

   for (N = 0; N < 4; N++)
   {
      if (GFX.Z1 > Depth [N] && (Pixel = Pixels[N]))
      {
         if (SubDepth[N] == 1)
            Screen[N] = colormath_fixed_sub_half(ScreenColors[Pixel], GFX.FixedColour15);
         else
            Screen[N] = ScreenColors[Pixel];
         Depth [N] = GFX.Z2;
      }
   }
}

static void WRITE_4PIXELS16_FLIPPED_SUBF1_2(int32_t Offset, uint8_t* Pixels, uint16_t* ScreenColors)
{
   uint8_t  Pixel, N;
   uint8_t* Screen = GFX.S + Offset;
   uint8_t*  Depth = GFX.ZBuffer + Offset;
   uint8_t*  SubDepth = GFX.SubZBuffer + Offset;

   for (N = 0; N < 4; N++)
   {
      if (GFX.Z1 > Depth [N] && (Pixel = Pixels[3 - N]))
      {
         if (SubDepth[N] == 1)
            Screen[N] = colormath_fixed_sub_half(ScreenColors[Pixel], GFX.FixedColour15);
         else
            Screen[N] = ScreenColors[Pixel];
         Depth [N] = GFX.Z2;
      }
   }
}

void DrawTile16FixedAdd1_2(uint32_t Tile, int32_t Offset, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* bp;
   TILE_PREAMBLE_VARS();
   TILE_PREAMBLE_CODE();
   RENDER_TILE(WRITE_4PIXELS16_ADDF1_2, WRITE_4PIXELS16_FLIPPED_ADDF1_2, 4);
}

void DrawClippedTile16FixedAdd1_2(uint32_t Tile, int32_t Offset, uint32_t StartPixel, uint32_t Width,  uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* bp;
   TILE_PREAMBLE_VARS();
   TILE_CLIP_PREAMBLE_VARS();
   RENDER_CLIPPED_TILE_VARS();
   TILE_PREAMBLE_CODE();
   TILE_CLIP_PREAMBLE_CODE();
   RENDER_CLIPPED_TILE_CODE(WRITE_4PIXELS16_ADDF1_2, WRITE_4PIXELS16_FLIPPED_ADDF1_2, 4);
}

void DrawTile16FixedSub1_2(uint32_t Tile, int32_t Offset, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* bp;
   TILE_PREAMBLE_VARS();
   TILE_PREAMBLE_CODE();
   RENDER_TILE(WRITE_4PIXELS16_SUBF1_2, WRITE_4PIXELS16_FLIPPED_SUBF1_2, 4);
}

void DrawClippedTile16FixedSub1_2(uint32_t Tile, int32_t Offset, uint32_t StartPixel, uint32_t Width, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* bp;
   TILE_PREAMBLE_VARS();
   TILE_CLIP_PREAMBLE_VARS();
   RENDER_CLIPPED_TILE_VARS();
   TILE_PREAMBLE_CODE();
   TILE_CLIP_PREAMBLE_CODE();
   RENDER_CLIPPED_TILE_CODE(WRITE_4PIXELS16_SUBF1_2, WRITE_4PIXELS16_FLIPPED_SUBF1_2, 4);
}

void DrawLargePixel16Add(uint32_t Tile, int32_t Offset, uint32_t StartPixel, uint32_t Pixels, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* sp = GFX.S + Offset;
   uint8_t*  Depth = GFX.ZBuffer + Offset;
   uint8_t pixel;
   TILE_PREAMBLE_VARS();
   TILE_PREAMBLE_CODE();

#define LARGE_ADD_PIXEL(s, p) \
(Depth [z + GFX.DepthDelta] ? (Depth [z + GFX.DepthDelta] != 1 ? \
                colormath_add(p, *(s + GFX.Delta)) : \
                colormath_fixed_add(p, GFX.FixedColour15)) : p)

   RENDER_TILE_LARGE(ScreenColors [pixel], LARGE_ADD_PIXEL);
}

void DrawLargePixel16Add1_2(uint32_t Tile, int32_t Offset, uint32_t StartPixel, uint32_t Pixels, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* sp = GFX.S + Offset;
   uint8_t*  Depth = GFX.ZBuffer + Offset;
   uint8_t pixel;
   TILE_PREAMBLE_VARS();
   TILE_PREAMBLE_CODE();

#define LARGE_ADD_PIXEL1_2(s, p) \
(Depth [z + GFX.DepthDelta] ? (Depth [z + GFX.DepthDelta] != 1 ? \
                colormath_add_half(p, *(s + GFX.Delta)) : \
                colormath_fixed_add(p, GFX.FixedColour15)) : p)

   RENDER_TILE_LARGE(ScreenColors [pixel], LARGE_ADD_PIXEL1_2);
}

void DrawLargePixel16Sub(uint32_t Tile, int32_t Offset, uint32_t StartPixel, uint32_t Pixels, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* sp = GFX.S + Offset;
   uint8_t*  Depth = GFX.ZBuffer + Offset;
   uint8_t pixel;
   TILE_PREAMBLE_VARS();
   TILE_PREAMBLE_CODE();

#define LARGE_SUB_PIXEL(s, p) \
(Depth [z + GFX.DepthDelta] ? (Depth [z + GFX.DepthDelta] != 1 ? \
                colormath_sub(p, *(s + GFX.Delta)) : \
                colormath_fixed_sub(p, GFX.FixedColour15)) : p)

   RENDER_TILE_LARGE(ScreenColors [pixel], LARGE_SUB_PIXEL);
}

void DrawLargePixel16Sub1_2(uint32_t Tile, int32_t Offset, uint32_t StartPixel, uint32_t Pixels, uint32_t StartLine, uint32_t LineCount)
{
   uint8_t* sp = GFX.S + Offset;
   uint8_t*  Depth = GFX.ZBuffer + Offset;
   uint8_t pixel;
   TILE_PREAMBLE_VARS();
   TILE_PREAMBLE_CODE();

#define LARGE_SUB_PIXEL1_2(s, p) \
(Depth [z + GFX.DepthDelta] ? (Depth [z + GFX.DepthDelta] != 1 ? \
                colormath_sub_half(p, *(s + GFX.Delta)) : \
                colormath_fixed_sub(p, GFX.FixedColour15)) : p)

   RENDER_TILE_LARGE(ScreenColors [pixel], LARGE_SUB_PIXEL1_2);
}
