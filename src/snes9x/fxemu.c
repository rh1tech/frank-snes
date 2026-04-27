/* SuperFX (GSU) emulator for frank-snes
 * Based on snes9x2005 reference, optimized for RP2350
 * Uses compact dispatch: vCurrentOp stores opcode, register index = op & 0xF */

#include "fxemu.h"
#include "memmap.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* Place every opcode handler in SRAM so the hot dispatch loop never pays
 * XIP cache misses fetching instruction code from flash.  Each handler is
 * tiny (30-120 bytes) and the total fits easily into RP2350 SRAM. */
#ifdef PICO_ON_DEVICE
#define FX_OP __attribute__((hot, section(".time_critical.fx_op")))
#else
#define FX_OP
#endif

FxRegs_s GSU;

/* --- Diagnostics --- */
#ifdef FRANK_SNES_SUPERFX_DIAG
volatile uint32_t g_gsu_call_count = 0;
volatile uint32_t g_gsu_inst_count = 0;
volatile uint32_t g_gsu_plot_count = 0;
volatile uint32_t g_gsu_start_fail = 0;
volatile uint32_t g_gsu_stop_count = 0;
volatile uint32_t g_gsu_last_scmr = 0;
volatile uint32_t g_gsu_last_scbr = 0;
volatile uint32_t g_gsu_last_mode = 0;
volatile uint32_t g_gsu_plot_xmin = 255;
volatile uint32_t g_gsu_plot_xmax = 0;
volatile uint32_t g_gsu_plot_ymin = 255;
volatile uint32_t g_gsu_plot_ymax = 0;
volatile uint32_t g_gsu_plot_colors = 0;  /* bitmask of seen color bytes lo nibble */
#endif

/* --- Internal macros --- */

#define TF(a) (GSU.vStatusReg &  FLG_##a)
#define CF(a) (GSU.vStatusReg &= ~FLG_##a)
#define SF(a) (GSU.vStatusReg |=  FLG_##a)

#define SEX16(a)  ((int32_t)((int16_t)(a)))
#define SEX8(a)   ((int32_t)((int8_t)(a)))
#define USEX16(a) ((uint32_t)((uint16_t)(a)))
#define USEX8(a)  ((uint32_t)((uint8_t)(a)))
#define SUSEX16(a) ((int32_t)((uint16_t)(a)))

#define CLRFLAGS \
   GSU.vStatusReg &= ~(FLG_ALT1|FLG_ALT2|FLG_B); \
   GSU.pvDreg = GSU.pvSreg = &GSU.avReg[0]

#define RAM(adr)      (GSU.pvRamBank[USEX16(adr)])
#define ROM(idx)      (GSU.pvRomBank[USEX16(idx)])
#define PIPE          GSU.vPipe
#define PRGBANK(idx)  GSU.pvPrgBank[USEX16(idx)]
#define FETCHPIPE     { PIPE = PRGBANK(GSU.avReg[15]); }
#define SREG          (*GSU.pvSreg)
#define DREG          (*GSU.pvDreg)
#define READR14       GSU.vRomBuffer = ROM(GSU.avReg[14])
#define TESTR14       if (GSU.pvDreg == &GSU.avReg[14]) READR14

#define R0  GSU.avReg[0]
#define R1  GSU.avReg[1]
#define R2  GSU.avReg[2]
#define R3  GSU.avReg[3]
#define R4  GSU.avReg[4]
#define R5  GSU.avReg[5]
#define R6  GSU.avReg[6]
#define R7  GSU.avReg[7]
#define R8  GSU.avReg[8]
#define R9  GSU.avReg[9]
#define R10 GSU.avReg[10]
#define R11 GSU.avReg[11]
#define R12 GSU.avReg[12]
#define R13 GSU.avReg[13]
#define R14 GSU.avReg[14]
#define R15 GSU.avReg[15]

#define COLR GSU.vColorReg
#define POR  GSU.vPlotOptionReg
#define SCBR USEX8(GSU.pvRegisters[GSU_SCBR])
#define SCMR USEX8(GSU.pvRegisters[GSU_SCMR])

#define CUR_REG (GSU.vCurrentOp & 0xF)

/* --- Plot/Rpix functions --- */

FX_OP static void fx_plot_2bit(void)
{
   uint32_t x = USEX8(R1);
   uint32_t y = USEX8(R2);
   uint8_t *a, v, c;
   R15++; CLRFLAGS; R1++;
   c = (POR & 0x02) ? ((x ^ y) & 1 ? (uint8_t)(COLR >> 4) : (uint8_t)COLR) : (uint8_t)COLR;
   if (!(POR & 0x01) && !(c & 0xf)) return;
#ifdef FRANK_SNES_SUPERFX_DIAG
   g_gsu_plot_count++;
#endif
   a = GSU.apvScreen[y >> 3] + GSU.x[x >> 3] + ((y & 7) << 1);
   v = 128 >> (x & 7);
   if (c & 0x01) a[0] |= v; else a[0] &= ~v;
   if (c & 0x02) a[1] |= v; else a[1] &= ~v;
}

FX_OP static void fx_rpix_2bit(void)
{
   uint32_t x = USEX8(R1);
   uint32_t y = USEX8(R2);
   uint8_t *a, v;
   R15++; CLRFLAGS;
   a = GSU.apvScreen[y >> 3] + GSU.x[x >> 3] + ((y & 7) << 1);
   v = 128 >> (x & 7);
   DREG = ((uint32_t)((a[0] & v) != 0)) | (((uint32_t)((a[1] & v) != 0)) << 1);
   TESTR14;
}

FX_OP static void fx_plot_4bit(void)
{
   uint32_t x = USEX8(R1);
   uint32_t y = USEX8(R2);
   uint8_t *a, v, c;
   R15++; CLRFLAGS; R1++;
   c = (POR & 0x02) ? ((x ^ y) & 1 ? (uint8_t)(COLR >> 4) : (uint8_t)COLR) : (uint8_t)COLR;
   if (!(POR & 0x01) && !(c & 0xf)) return;
#ifdef FRANK_SNES_SUPERFX_DIAG
   g_gsu_plot_count++;
#endif
   a = GSU.apvScreen[y >> 3] + GSU.x[x >> 3] + ((y & 7) << 1);
   v = 128 >> (x & 7);
   if (c & 0x01) a[0x00] |= v; else a[0x00] &= ~v;
   if (c & 0x02) a[0x01] |= v; else a[0x01] &= ~v;
   if (c & 0x04) a[0x10] |= v; else a[0x10] &= ~v;
   if (c & 0x08) a[0x11] |= v; else a[0x11] &= ~v;
}

FX_OP static void fx_rpix_4bit(void)
{
   uint32_t x = USEX8(R1);
   uint32_t y = USEX8(R2);
   uint8_t *a, v;
   R15++; CLRFLAGS;
   a = GSU.apvScreen[y >> 3] + GSU.x[x >> 3] + ((y & 7) << 1);
   v = 128 >> (x & 7);
   DREG = ((uint32_t)((a[0x00] & v) != 0))
        | (((uint32_t)((a[0x01] & v) != 0)) << 1)
        | (((uint32_t)((a[0x10] & v) != 0)) << 2)
        | (((uint32_t)((a[0x11] & v) != 0)) << 3);
   TESTR14;
}

FX_OP static void fx_plot_8bit(void)
{
   uint32_t x = USEX8(R1);
   uint32_t y = USEX8(R2);
   uint8_t *a, v, c;
   R15++; CLRFLAGS; R1++;
   c = (uint8_t)COLR;
   if (!(POR & 0x10)) { if (!(POR & 0x01) && !(c & 0xf)) return; }
   else if (!(POR & 0x01) && !c) return;
#ifdef FRANK_SNES_SUPERFX_DIAG
   g_gsu_plot_count++;
   if (x < g_gsu_plot_xmin) g_gsu_plot_xmin = x;
   if (x > g_gsu_plot_xmax) g_gsu_plot_xmax = x;
   if (y < g_gsu_plot_ymin) g_gsu_plot_ymin = y;
   if (y > g_gsu_plot_ymax) g_gsu_plot_ymax = y;
   g_gsu_plot_colors |= (1u << (c & 0x1F));
#endif
   a = GSU.apvScreen[y >> 3] + GSU.x[x >> 3] + ((y & 7) << 1);
   v = 128 >> (x & 7);
   if (c & 0x01) a[0x00] |= v; else a[0x00] &= ~v;
   if (c & 0x02) a[0x01] |= v; else a[0x01] &= ~v;
   if (c & 0x04) a[0x10] |= v; else a[0x10] &= ~v;
   if (c & 0x08) a[0x11] |= v; else a[0x11] &= ~v;
   if (c & 0x10) a[0x20] |= v; else a[0x20] &= ~v;
   if (c & 0x20) a[0x21] |= v; else a[0x21] &= ~v;
   if (c & 0x40) a[0x30] |= v; else a[0x30] &= ~v;
   if (c & 0x80) a[0x31] |= v; else a[0x31] &= ~v;
}

FX_OP static void fx_rpix_8bit(void)
{
   uint32_t x = USEX8(R1);
   uint32_t y = USEX8(R2);
   uint8_t *a, v;
   R15++; CLRFLAGS;
   a = GSU.apvScreen[y >> 3] + GSU.x[x >> 3] + ((y & 7) << 1);
   v = 128 >> (x & 7);
   DREG = ((uint32_t)((a[0x00] & v) != 0))
        | (((uint32_t)((a[0x01] & v) != 0)) << 1)
        | (((uint32_t)((a[0x10] & v) != 0)) << 2)
        | (((uint32_t)((a[0x11] & v) != 0)) << 3)
        | (((uint32_t)((a[0x20] & v) != 0)) << 4)
        | (((uint32_t)((a[0x21] & v) != 0)) << 5)
        | (((uint32_t)((a[0x30] & v) != 0)) << 6)
        | (((uint32_t)((a[0x31] & v) != 0)) << 7);
   GSU.vZero = DREG;
   TESTR14;
}

FX_OP static void fx_obj_func(void) { }

static void (*fx_plot_table[])(void) = {
   &fx_plot_2bit, &fx_plot_4bit, &fx_plot_4bit, &fx_plot_8bit, &fx_obj_func,
   &fx_rpix_2bit, &fx_rpix_4bit, &fx_rpix_4bit, &fx_rpix_8bit, &fx_obj_func,
};

/* --- Opcode implementations --- */

/* 00 - stop */
FX_OP static void fx_stop(void)
{
#ifdef FRANK_SNES_SUPERFX_DIAG
   g_gsu_stop_count++;
#endif
   CF(G);
   GSU.vCounter = 0;
   GSU.vInstCount = GSU.vCounter;
   if (!(GSU.pvRegisters[GSU_CFGR] & 0x80))
      SF(IRQ);
   POR = 0;
   PIPE = 1;
   CLRFLAGS;
   R15++;
}

/* 01 - nop */
FX_OP static void fx_nop(void) { CLRFLAGS; R15++; }

/* 02 - cache */
FX_OP static void fx_cache(void)
{
   uint32_t c = R15 & 0xfff0;
   if (GSU.vCacheBaseReg != c || !GSU.bCacheActive) {
      GSU.bCacheActive = false;
      GSU.vCacheBaseReg = c;
      GSU.bCacheActive = true;
   }
   CLRFLAGS; R15++;
}

/* 03 - lsr */
FX_OP static void fx_lsr(void)
{
   GSU.vCarry = SREG & 1;
   uint32_t v = USEX16(SREG) >> 1;
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v; TESTR14; CLRFLAGS;
}

/* 04 - rol */
FX_OP static void fx_rol(void)
{
   uint32_t v = USEX16((SREG << 1) + GSU.vCarry);
   GSU.vCarry = (SREG >> 15) & 1;
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v; TESTR14; CLRFLAGS;
}

/* 05 - bra */
FX_OP static void fx_bra(void)
{
   uint8_t v = PIPE; R15++; FETCHPIPE; R15 += SEX8(v);
}

/* 06-0F - conditional branches */
#define TEST_S  (GSU.vSign & 0x8000)
#define TEST_Z  (USEX16(GSU.vZero) == 0)
#define TEST_OV (GSU.vOverflow >= 0x8000 || GSU.vOverflow < -0x8000)
#define TEST_CY (GSU.vCarry & 1)

#define BRA_COND(cond) { uint8_t v = PIPE; R15++; FETCHPIPE; if (cond) R15 += SEX8(v); else R15++; }

FX_OP static void fx_blt(void) { BRA_COND((TEST_S != 0) != (TEST_OV != 0)); }
FX_OP static void fx_bge(void) { BRA_COND((TEST_S != 0) == (TEST_OV != 0)); }
FX_OP static void fx_bne(void) { BRA_COND(!TEST_Z); }
FX_OP static void fx_beq(void) { BRA_COND(TEST_Z); }
FX_OP static void fx_bpl(void) { BRA_COND(!TEST_S); }
FX_OP static void fx_bmi(void) { BRA_COND(TEST_S); }
FX_OP static void fx_bcc(void) { BRA_COND(!TEST_CY); }
FX_OP static void fx_bcs(void) { BRA_COND(TEST_CY); }
FX_OP static void fx_bvc(void) { BRA_COND(!TEST_OV); }
FX_OP static void fx_bvs(void) { BRA_COND(TEST_OV); }

/* 10-1F - to rn / move rn */
FX_OP static void fx_to(void)
{
   uint32_t reg = CUR_REG;
   if (TF(B)) {
      GSU.avReg[reg] = SREG;
      CLRFLAGS;
      if (reg == 14) READR14;
      if (reg == 15) return; /* Don't increment PC when destination is PC */
   } else {
      GSU.pvDreg = &GSU.avReg[reg];
   }
   R15++;
}

/* 20-2F - with rn */
FX_OP static void fx_with(void)
{
   SF(B);
   GSU.pvSreg = GSU.pvDreg = &GSU.avReg[CUR_REG];
   R15++;
}

/* 30-3B - stw (rn) */
FX_OP static void fx_stw(void)
{
   uint32_t reg = CUR_REG;
   GSU.vLastRamAdr = GSU.avReg[reg];
   RAM(GSU.avReg[reg]) = (uint8_t)SREG;
   RAM(GSU.avReg[reg] ^ 1) = (uint8_t)(SREG >> 8);
   CLRFLAGS; R15++;
}

/* 30-3B(ALT1) - stb (rn) */
FX_OP static void fx_stb(void)
{
   uint32_t reg = CUR_REG;
   GSU.vLastRamAdr = GSU.avReg[reg];
   RAM(GSU.avReg[reg]) = (uint8_t)SREG;
   CLRFLAGS; R15++;
}

/* 3C - loop */
FX_OP static void fx_loop(void)
{
   GSU.vSign = GSU.vZero = --R12;
   if ((uint16_t)R12 != 0) R15 = R13; else R15++;
   CLRFLAGS;
}

/* 3D-3F - alt mode set */
FX_OP static void fx_alt1(void) { SF(ALT1); CF(B); R15++; }
FX_OP static void fx_alt2(void) { SF(ALT2); CF(B); R15++; }
FX_OP static void fx_alt3(void) { SF(ALT1); SF(ALT2); CF(B); R15++; }

/* 40-4B - ldw (rn) */
FX_OP static void fx_ldw(void)
{
   uint32_t reg = CUR_REG;
   GSU.vLastRamAdr = GSU.avReg[reg];
   uint32_t v = (uint32_t)RAM(GSU.avReg[reg]);
   v |= ((uint32_t)RAM(GSU.avReg[reg] ^ 1)) << 8;
   R15++; DREG = v; TESTR14; CLRFLAGS;
}

/* 40-4B(ALT1) - ldb (rn) */
FX_OP static void fx_ldb(void)
{
   uint32_t reg = CUR_REG;
   GSU.vLastRamAdr = GSU.avReg[reg];
   uint32_t v = (uint32_t)RAM(GSU.avReg[reg]);
   R15++; DREG = v; TESTR14; CLRFLAGS;
}

/* 4D - swap */
FX_OP static void fx_swap(void)
{
   uint8_t c = (uint8_t)SREG, d = (uint8_t)(SREG >> 8);
   uint32_t v = (((uint32_t)c) << 8) | ((uint32_t)d);
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v; TESTR14; CLRFLAGS;
}

/* 4E - color */
FX_OP static void fx_color(void)
{
   uint8_t c = (uint8_t)SREG;
   if (POR & 0x04) c = (c & 0xf0) | (c >> 4);
   if (POR & 0x08) { COLR &= 0xf0; COLR |= c & 0x0f; }
   else COLR = USEX8(c);
   CLRFLAGS; R15++;
}

/* 4E(ALT1) - cmode */
FX_OP static void fx_cmode(void)
{
   POR = SREG;
   if (POR & 0x10) GSU.vScreenHeight = 256;
   else GSU.vScreenHeight = GSU.vScreenRealHeight;
   fx_computeScreenPointers();
   CLRFLAGS; R15++;
}

/* 4F - not */
FX_OP static void fx_not(void)
{
   uint32_t v = ~SREG;
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v; TESTR14; CLRFLAGS;
}

/* 50-5F - add rn */
FX_OP static void fx_add(void)
{
   uint32_t reg = CUR_REG;
   int32_t s = SUSEX16(SREG) + SUSEX16(GSU.avReg[reg]);
   GSU.vCarry = s >= 0x10000;
   GSU.vOverflow = ~(SREG ^ GSU.avReg[reg]) & (GSU.avReg[reg] ^ s) & 0x8000;
   GSU.vSign = s; GSU.vZero = s;
   R15++; DREG = s; TESTR14; CLRFLAGS;
}

/* 50-5F(ALT1) - adc rn */
FX_OP static void fx_adc(void)
{
   uint32_t reg = CUR_REG;
   int32_t s = SUSEX16(SREG) + SUSEX16(GSU.avReg[reg]) + SEX16(GSU.vCarry);
   GSU.vCarry = s >= 0x10000;
   GSU.vOverflow = ~(SREG ^ GSU.avReg[reg]) & (GSU.avReg[reg] ^ s) & 0x8000;
   GSU.vSign = s; GSU.vZero = s;
   R15++; DREG = s; TESTR14; CLRFLAGS;
}

/* 50-5F(ALT2) - add #n */
FX_OP static void fx_add_i(void)
{
   uint32_t imm = CUR_REG;
   int32_t s = SUSEX16(SREG) + imm;
   GSU.vCarry = s >= 0x10000;
   GSU.vOverflow = ~(SREG ^ imm) & (imm ^ s) & 0x8000;
   GSU.vSign = s; GSU.vZero = s;
   R15++; DREG = s; TESTR14; CLRFLAGS;
}

/* 50-5F(ALT3) - adc #n */
FX_OP static void fx_adc_i(void)
{
   uint32_t imm = CUR_REG;
   int32_t s = SUSEX16(SREG) + imm + SUSEX16(GSU.vCarry);
   GSU.vCarry = s >= 0x10000;
   GSU.vOverflow = ~(SREG ^ imm) & (imm ^ s) & 0x8000;
   GSU.vSign = s; GSU.vZero = s;
   R15++; DREG = s; TESTR14; CLRFLAGS;
}

/* 60-6F - sub rn */
FX_OP static void fx_sub(void)
{
   uint32_t reg = CUR_REG;
   int32_t s = SUSEX16(SREG) - SUSEX16(GSU.avReg[reg]);
   GSU.vCarry = s >= 0;
   GSU.vOverflow = (SREG ^ GSU.avReg[reg]) & (SREG ^ s) & 0x8000;
   GSU.vSign = s; GSU.vZero = s;
   R15++; DREG = s; TESTR14; CLRFLAGS;
}

/* 60-6F(ALT1) - sbc rn */
FX_OP static void fx_sbc(void)
{
   uint32_t reg = CUR_REG;
   int32_t s = SUSEX16(SREG) - SUSEX16(GSU.avReg[reg]) - (SUSEX16(GSU.vCarry ^ 1));
   GSU.vCarry = s >= 0;
   GSU.vOverflow = (SREG ^ GSU.avReg[reg]) & (SREG ^ s) & 0x8000;
   GSU.vSign = s; GSU.vZero = s;
   R15++; DREG = s; TESTR14; CLRFLAGS;
}

/* 60-6F(ALT2) - sub #n */
FX_OP static void fx_sub_i(void)
{
   uint32_t imm = CUR_REG;
   int32_t s = SUSEX16(SREG) - imm;
   GSU.vCarry = s >= 0;
   GSU.vOverflow = (SREG ^ imm) & (SREG ^ s) & 0x8000;
   GSU.vSign = s; GSU.vZero = s;
   R15++; DREG = s; TESTR14; CLRFLAGS;
}

/* 60-6F(ALT3) - cmp rn */
FX_OP static void fx_cmp(void)
{
   uint32_t reg = CUR_REG;
   int32_t s = SUSEX16(SREG) - SUSEX16(GSU.avReg[reg]);
   GSU.vCarry = s >= 0;
   GSU.vOverflow = (SREG ^ GSU.avReg[reg]) & (SREG ^ s) & 0x8000;
   GSU.vSign = s; GSU.vZero = s;
   R15++; CLRFLAGS;
}

/* 70 - merge */
FX_OP static void fx_merge(void)
{
   uint32_t v = (R7 & 0xff00) | ((R8 & 0xff00) >> 8);
   R15++; DREG = v;
   GSU.vOverflow = (v & 0xc0c0) << 16;
   GSU.vZero = !(v & 0xf0f0);
   GSU.vSign = ((v | (v << 8)) & 0x8000);
   GSU.vCarry = (v & 0xe0e0) != 0;
   TESTR14; CLRFLAGS;
}

/* 71-7F - and rn */
FX_OP static void fx_and(void)
{
   uint32_t v = SREG & GSU.avReg[CUR_REG];
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v; TESTR14; CLRFLAGS;
}

/* 71-7F(ALT1) - bic rn */
FX_OP static void fx_bic(void)
{
   uint32_t v = SREG & ~GSU.avReg[CUR_REG];
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v; TESTR14; CLRFLAGS;
}

/* 71-7F(ALT2) - and #n */
FX_OP static void fx_and_i(void)
{
   uint32_t v = SREG & CUR_REG;
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v; TESTR14; CLRFLAGS;
}

/* 71-7F(ALT3) - bic #n */
FX_OP static void fx_bic_i(void)
{
   uint32_t v = SREG & ~((uint32_t)CUR_REG);
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v; TESTR14; CLRFLAGS;
}

/* 80-8F - mult rn (signed 8x8→16) */
FX_OP static void fx_mult(void)
{
   uint32_t v = (uint32_t)(SEX8(SREG) * SEX8(GSU.avReg[CUR_REG]));
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v; TESTR14; CLRFLAGS;
}

/* 80-8F(ALT1) - umult rn (unsigned 8x8→16) */
FX_OP static void fx_umult(void)
{
   uint32_t v = USEX8(SREG) * USEX8(GSU.avReg[CUR_REG]);
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v; TESTR14; CLRFLAGS;
}

/* 80-8F(ALT2) - mult #n */
FX_OP static void fx_mult_i(void)
{
   uint32_t v = (uint32_t)(SEX8(SREG) * ((int32_t)CUR_REG));
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v; TESTR14; CLRFLAGS;
}

/* 80-8F(ALT3) - umult #n */
FX_OP static void fx_umult_i(void)
{
   uint32_t v = USEX8(SREG) * ((uint32_t)CUR_REG);
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v; TESTR14; CLRFLAGS;
}

/* 90 - sbk */
FX_OP static void fx_sbk(void)
{
   RAM(GSU.vLastRamAdr) = (uint8_t)SREG;
   RAM(GSU.vLastRamAdr ^ 1) = (uint8_t)(SREG >> 8);
   CLRFLAGS; R15++;
}

/* 91-94 - link #n (opcode 0x91=link 1, 0x92=link 2, etc.) */
FX_OP static void fx_link(void)
{
   R11 = R15 + CUR_REG;
   CLRFLAGS; R15++;
}

/* 95 - sex */
FX_OP static void fx_sex(void)
{
   uint32_t v = (uint32_t)SEX8(SREG);
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v; TESTR14; CLRFLAGS;
}

/* 96 - asr */
FX_OP static void fx_asr(void)
{
   GSU.vCarry = SREG & 1;
   uint32_t v = (uint32_t)(SEX16(SREG) >> 1);
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v; TESTR14; CLRFLAGS;
}

/* 96(ALT1) - div2 */
FX_OP static void fx_div2(void)
{
   int32_t s = SEX16(SREG);
   GSU.vCarry = s & 1;
   uint32_t v = (s == -1) ? 0 : (uint32_t)(s >> 1);
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v; TESTR14; CLRFLAGS;
}

/* 97 - ror */
FX_OP static void fx_ror(void)
{
   uint32_t v = (USEX16(SREG) >> 1) | (GSU.vCarry << 15);
   GSU.vCarry = SREG & 1;
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v; TESTR14; CLRFLAGS;
}

/* 98-9D - jmp rn */
FX_OP static void fx_jmp(void)
{
   R15 = GSU.avReg[CUR_REG];
   CLRFLAGS;
}

/* 98-9D(ALT1) - ljmp rn */
FX_OP static void fx_ljmp(void)
{
   uint32_t reg = CUR_REG;
   GSU.vPrgBankReg = GSU.avReg[reg] & 0x7f;
   GSU.pvPrgBank = GSU.apvRomBank[GSU.vPrgBankReg];
   R15 = SREG;
   GSU.bCacheActive = false;
   fx_cache();
   R15--;
}

/* 9E - lob */
FX_OP static void fx_lob(void)
{
   uint32_t v = USEX8(SREG);
   R15++; DREG = v; GSU.vSign = v << 8; GSU.vZero = v << 8; TESTR14; CLRFLAGS;
}

/* 9F - fmult (16x16→32, upper 16 bits) */
FX_OP static void fx_fmult(void)
{
   uint32_t c = (uint32_t)(SEX16(SREG) * SEX16(R6));
   uint32_t v = c >> 16;
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v;
   GSU.vCarry = (c >> 15) & 1;
   TESTR14; CLRFLAGS;
}

/* 9F(ALT1) - lmult (16x16→32, full result) */
FX_OP static void fx_lmult(void)
{
   uint32_t c = (uint32_t)(SEX16(SREG) * SEX16(R6));
   R4 = c;
   uint32_t v = c >> 16;
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v;
   GSU.vCarry = (R4 >> 15) & 1;
   TESTR14; CLRFLAGS;
}

/* A0-AF - ibt rn,#pp */
FX_OP static void fx_ibt(void)
{
   uint32_t reg = CUR_REG;
   uint8_t v = PIPE; R15++; FETCHPIPE; R15++;
   GSU.avReg[reg] = SEX8(v);
   CLRFLAGS;
   if (reg == 14) READR14;
}

/* A0-AF(ALT1) - lms rn,(yy) */
FX_OP static void fx_lms(void)
{
   uint32_t reg = CUR_REG;
   GSU.vLastRamAdr = ((uint32_t)PIPE) << 1;
   R15++; FETCHPIPE; R15++;
   GSU.avReg[reg] = (uint32_t)RAM(GSU.vLastRamAdr);
   GSU.avReg[reg] |= ((uint32_t)RAM(GSU.vLastRamAdr + 1)) << 8;
   CLRFLAGS;
   if (reg == 14) READR14;
}

/* A0-AF(ALT2) - sms (yy),rn */
FX_OP static void fx_sms(void)
{
   uint32_t reg = CUR_REG;
   uint32_t v = GSU.avReg[reg];
   GSU.vLastRamAdr = ((uint32_t)PIPE) << 1;
   R15++; FETCHPIPE;
   RAM(GSU.vLastRamAdr) = (uint8_t)v;
   RAM(GSU.vLastRamAdr + 1) = (uint8_t)(v >> 8);
   CLRFLAGS; R15++;
}

/* B0-BF - from rn / moves rn */
FX_OP static void fx_from(void)
{
   uint32_t reg = CUR_REG;
   if (TF(B)) {
      uint32_t v = GSU.avReg[reg];
      R15++; DREG = v;
      GSU.vOverflow = (v & 0x80) << 16;
      GSU.vSign = v; GSU.vZero = v;
      TESTR14; CLRFLAGS;
   } else {
      GSU.pvSreg = &GSU.avReg[reg];
      R15++;
   }
}

/* C0 - hib */
FX_OP static void fx_hib(void)
{
   uint32_t v = USEX8(SREG >> 8);
   R15++; DREG = v; GSU.vSign = v << 8; GSU.vZero = v << 8; TESTR14; CLRFLAGS;
}

/* C1-CF - or rn */
FX_OP static void fx_or(void)
{
   uint32_t v = SREG | GSU.avReg[CUR_REG];
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v; TESTR14; CLRFLAGS;
}

/* C1-CF(ALT1) - xor rn */
FX_OP static void fx_xor(void)
{
   uint32_t v = SREG ^ GSU.avReg[CUR_REG];
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v; TESTR14; CLRFLAGS;
}

/* C1-CF(ALT2) - or #n */
FX_OP static void fx_or_i(void)
{
   uint32_t v = SREG | CUR_REG;
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v; TESTR14; CLRFLAGS;
}

/* C1-CF(ALT3) - xor #n */
FX_OP static void fx_xor_i(void)
{
   uint32_t v = SREG ^ CUR_REG;
   R15++; DREG = v; GSU.vSign = v; GSU.vZero = v; TESTR14; CLRFLAGS;
}

/* D0-DE - inc rn */
FX_OP static void fx_inc(void)
{
   uint32_t reg = CUR_REG;
   GSU.avReg[reg]++;
   GSU.vSign = GSU.avReg[reg]; GSU.vZero = GSU.avReg[reg];
   CLRFLAGS; R15++;
   if (reg == 14) READR14;
}

/* DF - getc */
FX_OP static void fx_getc(void)
{
   uint8_t c = GSU.vRomBuffer;
   if (POR & 0x04) c = (c & 0xf0) | (c >> 4);
   if (POR & 0x08) { COLR &= 0xf0; COLR |= c & 0x0f; }
   else COLR = USEX8(c);
   CLRFLAGS; R15++;
}

/* DF(ALT2) - ramb */
FX_OP static void fx_ramb(void)
{
   GSU.vRamBankReg = SREG & (FX_RAM_BANKS - 1);
   GSU.pvRamBank = GSU.apvRamBank[GSU.vRamBankReg & 0x3];
   CLRFLAGS; R15++;
}

/* DF(ALT3) - romb */
FX_OP static void fx_romb(void)
{
   GSU.vRomBankReg = USEX8(SREG) & 0x7f;
   GSU.pvRomBank = GSU.apvRomBank[GSU.vRomBankReg];
   CLRFLAGS; R15++;
}

/* E0-EE - dec rn */
FX_OP static void fx_dec(void)
{
   uint32_t reg = CUR_REG;
   GSU.avReg[reg]--;
   GSU.vSign = GSU.avReg[reg]; GSU.vZero = GSU.avReg[reg];
   CLRFLAGS; R15++;
   if (reg == 14) READR14;
}

/* EF - getb */
FX_OP static void fx_getb(void)
{
   uint32_t v = (uint32_t)GSU.vRomBuffer;
   R15++; DREG = v; TESTR14; CLRFLAGS;
}

/* EF(ALT1) - getbh */
FX_OP static void fx_getbh(void)
{
   uint32_t v = USEX8(SREG) | (USEX8(GSU.vRomBuffer) << 8);
   R15++; DREG = v; TESTR14; CLRFLAGS;
}

/* EF(ALT2) - getbl */
FX_OP static void fx_getbl(void)
{
   uint32_t v = (SREG & 0xff00) | USEX8(GSU.vRomBuffer);
   R15++; DREG = v; TESTR14; CLRFLAGS;
}

/* EF(ALT3) - getbs */
FX_OP static void fx_getbs(void)
{
   uint32_t v = SEX8(GSU.vRomBuffer);
   R15++; DREG = v; TESTR14; CLRFLAGS;
}

/* F0-FF - iwt rn,#xx */
FX_OP static void fx_iwt(void)
{
   uint32_t reg = CUR_REG;
   uint32_t v = PIPE; R15++; FETCHPIPE; R15++;
   v |= USEX8(PIPE) << 8; FETCHPIPE; R15++;
   GSU.avReg[reg] = v;
   CLRFLAGS;
   if (reg == 14) READR14;
}

/* F0-FF(ALT1) - lm rn,(xx) */
FX_OP static void fx_lm(void)
{
   uint32_t reg = CUR_REG;
   GSU.vLastRamAdr = PIPE; R15++; FETCHPIPE; R15++;
   GSU.vLastRamAdr |= USEX8(PIPE) << 8; FETCHPIPE; R15++;
   GSU.avReg[reg] = RAM(GSU.vLastRamAdr);
   GSU.avReg[reg] |= USEX8(RAM(GSU.vLastRamAdr ^ 1)) << 8;
   CLRFLAGS;
   if (reg == 14) READR14;
}

/* F0-FF(ALT2) - sm (xx),rn */
FX_OP static void fx_sm(void)
{
   uint32_t reg = CUR_REG;
   uint32_t v = GSU.avReg[reg];
   GSU.vLastRamAdr = PIPE; R15++; FETCHPIPE; R15++;
   GSU.vLastRamAdr |= USEX8(PIPE) << 8; FETCHPIPE;
   RAM(GSU.vLastRamAdr) = (uint8_t)v;
   RAM(GSU.vLastRamAdr ^ 1) = (uint8_t)(v >> 8);
   CLRFLAGS; R15++;
}

/* --- Opcode table (4 x 256 entries) --- */

/* Helper: fill 16 entries with same function pointer */
#define P16(f) f,f,f,f,f,f,f,f,f,f,f,f,f,f,f,f
/* Helper: fill 15 entries (C1-CF, 71-7F style: skip entry 0 = separate opcode) */
#define P15(f) f,f,f,f,f,f,f,f,f,f,f,f,f,f,f
/* Helper: fill 12 entries (30-3B, 40-4B) */
#define P12(f) f,f,f,f,f,f,f,f,f,f,f,f
/* Helper: fill 6 entries (98-9D) */
#define P6(f) f,f,f,f,f,f

static void (*fx_opcode_table[1024])(void) =
{
   /* ALT0 Table (0x000-0x0FF) */
   /* 00-0F */ &fx_stop, &fx_nop, &fx_cache, &fx_lsr, &fx_rol, &fx_bra, &fx_bge, &fx_blt,
               &fx_bne, &fx_beq, &fx_bpl, &fx_bmi, &fx_bcc, &fx_bcs, &fx_bvc, &fx_bvs,
   /* 10-1F */ P16(&fx_to),
   /* 20-2F */ P16(&fx_with),
   /* 30-3F */ P12(&fx_stw), &fx_loop, &fx_alt1, &fx_alt2, &fx_alt3,
   /* 40-4F */ P12(&fx_ldw), &fx_plot_2bit, &fx_swap, &fx_color, &fx_not,
   /* 50-5F */ P16(&fx_add),
   /* 60-6F */ P16(&fx_sub),
   /* 70-7F */ &fx_merge, P15(&fx_and),
   /* 80-8F */ P16(&fx_mult),
   /* 90-9F */ &fx_sbk, &fx_link, &fx_link, &fx_link, &fx_link, &fx_sex, &fx_asr, &fx_ror,
               P6(&fx_jmp), &fx_lob, &fx_fmult,
   /* A0-AF */ P16(&fx_ibt),
   /* B0-BF */ P16(&fx_from),
   /* C0-CF */ &fx_hib, P15(&fx_or),
   /* D0-DF */ P15(&fx_inc), &fx_getc,
   /* E0-EF */ P15(&fx_dec), &fx_getb,
   /* F0-FF */ P16(&fx_iwt),

   /* ALT1 Table (0x100-0x1FF) */
   /* 00-0F */ &fx_stop, &fx_nop, &fx_cache, &fx_lsr, &fx_rol, &fx_bra, &fx_bge, &fx_blt,
               &fx_bne, &fx_beq, &fx_bpl, &fx_bmi, &fx_bcc, &fx_bcs, &fx_bvc, &fx_bvs,
   /* 10-1F */ P16(&fx_to),
   /* 20-2F */ P16(&fx_with),
   /* 30-3F */ P12(&fx_stb), &fx_loop, &fx_alt1, &fx_alt2, &fx_alt3,
   /* 40-4F */ P12(&fx_ldb), &fx_rpix_2bit, &fx_swap, &fx_cmode, &fx_not,
   /* 50-5F */ P16(&fx_adc),
   /* 60-6F */ P16(&fx_sbc),
   /* 70-7F */ &fx_merge, P15(&fx_bic),
   /* 80-8F */ P16(&fx_umult),
   /* 90-9F */ &fx_sbk, &fx_link, &fx_link, &fx_link, &fx_link, &fx_sex, &fx_div2, &fx_ror,
               P6(&fx_ljmp), &fx_lob, &fx_lmult,
   /* A0-AF */ P16(&fx_lms),
   /* B0-BF */ P16(&fx_from),
   /* C0-CF */ &fx_hib, P15(&fx_xor),
   /* D0-DF */ P15(&fx_inc), &fx_getc,
   /* E0-EF */ P15(&fx_dec), &fx_getbh,
   /* F0-FF */ P16(&fx_lm),

   /* ALT2 Table (0x200-0x2FF) */
   /* 00-0F */ &fx_stop, &fx_nop, &fx_cache, &fx_lsr, &fx_rol, &fx_bra, &fx_bge, &fx_blt,
               &fx_bne, &fx_beq, &fx_bpl, &fx_bmi, &fx_bcc, &fx_bcs, &fx_bvc, &fx_bvs,
   /* 10-1F */ P16(&fx_to),
   /* 20-2F */ P16(&fx_with),
   /* 30-3F */ P12(&fx_stw), &fx_loop, &fx_alt1, &fx_alt2, &fx_alt3,
   /* 40-4F */ P12(&fx_ldw), &fx_plot_2bit, &fx_swap, &fx_color, &fx_not,
   /* 50-5F */ P16(&fx_add_i),
   /* 60-6F */ P16(&fx_sub_i),
   /* 70-7F */ &fx_merge, P15(&fx_and_i),
   /* 80-8F */ P16(&fx_mult_i),
   /* 90-9F */ &fx_sbk, &fx_link, &fx_link, &fx_link, &fx_link, &fx_sex, &fx_asr, &fx_ror,
               P6(&fx_jmp), &fx_lob, &fx_fmult,
   /* A0-AF */ P16(&fx_sms),
   /* B0-BF */ P16(&fx_from),
   /* C0-CF */ &fx_hib, P15(&fx_or_i),
   /* D0-DF */ P15(&fx_inc), &fx_ramb,
   /* E0-EF */ P15(&fx_dec), &fx_getbl,
   /* F0-FF */ P16(&fx_sm),

   /* ALT3 Table (0x300-0x3FF) */
   /* 00-0F */ &fx_stop, &fx_nop, &fx_cache, &fx_lsr, &fx_rol, &fx_bra, &fx_bge, &fx_blt,
               &fx_bne, &fx_beq, &fx_bpl, &fx_bmi, &fx_bcc, &fx_bcs, &fx_bvc, &fx_bvs,
   /* 10-1F */ P16(&fx_to),
   /* 20-2F */ P16(&fx_with),
   /* 30-3F */ P12(&fx_stb), &fx_loop, &fx_alt1, &fx_alt2, &fx_alt3,
   /* 40-4F */ P12(&fx_ldb), &fx_rpix_2bit, &fx_swap, &fx_cmode, &fx_not,
   /* 50-5F */ P16(&fx_adc_i),
   /* 60-6F */ P16(&fx_cmp),
   /* 70-7F */ &fx_merge, P15(&fx_bic_i),
   /* 80-8F */ P16(&fx_umult_i),
   /* 90-9F */ &fx_sbk, &fx_link, &fx_link, &fx_link, &fx_link, &fx_sex, &fx_div2, &fx_ror,
               P6(&fx_ljmp), &fx_lob, &fx_lmult,
   /* A0-AF */ P16(&fx_lms),
   /* B0-BF */ P16(&fx_from),
   /* C0-CF */ &fx_hib, P15(&fx_xor_i),
   /* D0-DF */ P15(&fx_inc), &fx_romb,
   /* E0-EF */ P15(&fx_dec), &fx_getbs,
   /* F0-FF */ P16(&fx_lm),
};

/* --- Execution loop --- */

#ifdef PICO_ON_DEVICE
#define FX_HOT __attribute__((hot, section(".time_critical.fx_run")))
#else
#define FX_HOT
#endif

/* All opcode handlers are now in .time_critical to avoid XIP cache misses
 * on every dispatch.  They fit comfortably in SRAM.
 * Applied retroactively to all `static void fx_...` above via objcopy — see
 * CMake `set_source_files_properties` in the build rules. */

FX_HOT static uint32_t fx_run(uint32_t nInstructions)
{
   GSU.vCounter = nInstructions;
   READR14;
   while (TF(G) && (GSU.vCounter-- > 0)) {
      uint32_t vOpcode = (uint32_t)PIPE;
      FETCHPIPE;
      GSU.vCurrentOp = (uint8_t)vOpcode;
      (*fx_opcode_table[(GSU.vStatusReg & 0x300) | vOpcode])();
   }
   return nInstructions - GSU.vInstCount;
}

/* --- Register space read/write --- */

FX_OP static void fx_readRegisterSpaceForCheck(void)
{
   R15 = (uint32_t)READ_WORD(&GSU.pvRegisters[30]);
}

FX_OP static void fx_readRegisterSpaceForUse(void)
{
   static const uint32_t avHeight[] = { 128, 160, 192, 256 };
   static const uint32_t avMult[] = { 16, 32, 32, 64 };
   int32_t i;
   uint8_t *p = GSU.pvRegisters;

   for (i = 0; i < 15; i++, p += 2)
      GSU.avReg[i] = (uint32_t)READ_WORD(p);

   p = GSU.pvRegisters;
   GSU.vStatusReg = (uint32_t)READ_WORD(&p[GSU_SFR]);
   GSU.vPrgBankReg = (uint32_t)p[GSU_PBR];
   GSU.vRomBankReg = (uint32_t)p[GSU_ROMBR];
   GSU.vRamBankReg = ((uint32_t)p[GSU_RAMBR]) & (FX_RAM_BANKS - 1);
   GSU.vCacheBaseReg = (uint32_t)READ_WORD(&p[GSU_CBR]);

   GSU.vZero = !(GSU.vStatusReg & FLG_Z);
   GSU.vSign = (GSU.vStatusReg & FLG_S) << 12;
   GSU.vOverflow = (GSU.vStatusReg & FLG_OV) << 16;
   GSU.vCarry = (GSU.vStatusReg & FLG_CY) >> 2;

   GSU.pvRamBank = GSU.apvRamBank[GSU.vRamBankReg & 0x3];
   GSU.pvRomBank = GSU.apvRomBank[GSU.vRomBankReg];
   GSU.pvPrgBank = GSU.apvRomBank[GSU.vPrgBankReg];

   GSU.pvScreenBase = &GSU.pvRam[USEX8(p[GSU_SCBR]) << 10];
   i  =  (int32_t)(!!(p[GSU_SCMR] & 0x04));
   i |= ((int32_t)(!!(p[GSU_SCMR] & 0x20))) << 1;
   GSU.vScreenHeight = GSU.vScreenRealHeight = avHeight[i];
   GSU.vMode = p[GSU_SCMR] & 0x03;

   if (i == 3)
      GSU.vScreenSize = 32768;
   else
      GSU.vScreenSize = GSU.vScreenHeight * 4 * avMult[GSU.vMode];

   if (POR & 0x10)
      GSU.vScreenHeight = 256;

   if (GSU.pvScreenBase + GSU.vScreenSize > GSU.pvRam + (GSU.nRamBanks * 65536))
      GSU.pvScreenBase = GSU.pvRam + (GSU.nRamBanks * 65536) - GSU.vScreenSize;

   GSU.pfPlot = fx_plot_table[GSU.vMode];
   GSU.pfRpix = fx_plot_table[GSU.vMode + 5];

   /* Patch opcode table for current plot/rpix mode */
   fx_opcode_table[0x04c] = GSU.pfPlot;
   fx_opcode_table[0x14c] = GSU.pfRpix;
   fx_opcode_table[0x24c] = GSU.pfPlot;
   fx_opcode_table[0x34c] = GSU.pfRpix;

   if (GSU.vMode != GSU.vPrevMode || GSU.vPrevScreenHeight != GSU.vScreenHeight || GSU.vSCBRDirty)
      fx_computeScreenPointers();
}

FX_OP static void fx_writeRegisterSpaceAfterCheck(void)
{
   WRITE_WORD(&GSU.pvRegisters[30], R15);
}

FX_OP static void fx_writeRegisterSpaceAfterUse(void)
{
   int32_t i;
   uint8_t *p = GSU.pvRegisters;

   for (i = 0; i < 15; i++, p += 2)
      WRITE_WORD(p, GSU.avReg[i]);

   if (USEX16(GSU.vZero) == 0) SF(Z); else CF(Z);
   if (GSU.vSign & 0x8000) SF(S); else CF(S);
   if (GSU.vOverflow >= 0x8000 || GSU.vOverflow < -0x8000) SF(OV); else CF(OV);
   if (GSU.vCarry) SF(CY); else CF(CY);

   p = GSU.pvRegisters;
   WRITE_WORD(&p[GSU_SFR], GSU.vStatusReg);
   p[GSU_PBR] = (uint8_t)GSU.vPrgBankReg;
   p[GSU_ROMBR] = (uint8_t)GSU.vRomBankReg;
   p[GSU_RAMBR] = (uint8_t)GSU.vRamBankReg;
   WRITE_WORD(&p[GSU_CBR], GSU.vCacheBaseReg);
}

/* --- Public API --- */

void FxFlushCache(void)
{
   GSU.vCacheBaseReg = 0;
   GSU.bCacheActive = false;
}

void fx_dirtySCBR(void)
{
   GSU.vSCBRDirty = true;
}

void fx_updateRamBank(uint8_t Byte)
{
   GSU.vRamBankReg = (uint32_t)Byte & (FX_RAM_BANKS - 1);
   GSU.pvRamBank = GSU.apvRamBank[Byte & 0x3];
}

void fx_computeScreenPointers(void)
{
   int32_t i, j;
   uint32_t apvIncrement, vMode, xIncrement;
   GSU.vSCBRDirty = false;

   vMode = GSU.vMode;
   int32_t condition = vMode - 2;
   int32_t mask = (condition | -condition) >> 31;
   vMode = (vMode & mask) | (3 & ~mask);
   vMode++;

   GSU.x[0] = 0;
   GSU.apvScreen[0] = GSU.pvScreenBase;
   apvIncrement = vMode << 4;

   if (GSU.vScreenHeight == 256) {
      GSU.x[16] = vMode << 12;
      GSU.apvScreen[16] = GSU.pvScreenBase + (vMode << 13);
      apvIncrement <<= 4;
      xIncrement = vMode << 4;
      for (i = 1, j = 17; i < 16; i++, j++) {
         GSU.x[i] = GSU.x[i-1] + xIncrement;
         GSU.apvScreen[i] = GSU.apvScreen[i-1] + apvIncrement;
         GSU.x[j] = GSU.x[j-1] + xIncrement;
         GSU.apvScreen[j] = GSU.apvScreen[j-1] + apvIncrement;
      }
   } else {
      xIncrement = (vMode * GSU.vScreenHeight) << 1;
      for (i = 1; i < 32; i++) {
         GSU.x[i] = GSU.x[i-1] + xIncrement;
         GSU.apvScreen[i] = GSU.apvScreen[i-1] + apvIncrement;
      }
   }
   GSU.vPrevMode = GSU.vMode;
   GSU.vPrevScreenHeight = GSU.vScreenHeight;
}

void FxReset(FxInit_s *psFxInfo)
{
   int32_t i;
   memset(&GSU, 0, sizeof(FxRegs_s));
   GSU.pvSreg = GSU.pvDreg = &R0;

   GSU.pvRegisters = psFxInfo->pvRegisters;
   GSU.nRamBanks = psFxInfo->nRamBanks;
   GSU.pvRam = psFxInfo->pvRam;
   GSU.nRomBanks = psFxInfo->nRomBanks;
   GSU.pvRom = psFxInfo->pvRom;
   GSU.vPrevScreenHeight = ~0;
   GSU.vPrevMode = ~0;

   if (GSU.nRomBanks > 0x20)
      GSU.nRomBanks = 0x20;

   memset(GSU.pvRegisters, 0, 0x300);
   GSU.pvRegisters[0x3b] = 0;

   for (i = 0; i < 256; i++) {
      uint32_t b = i & 0x7f;
      if (b >= 0x40) {
         if (GSU.nRomBanks > 2) b %= GSU.nRomBanks;
         else b &= 1;
         GSU.apvRomBank[i] = &GSU.pvRom[b << 16];
      } else {
         b %= GSU.nRomBanks * 2;
         GSU.apvRomBank[i] = &GSU.pvRom[(b << 16) + 0x200000];
      }
   }

   for (i = 0; i < 4; i++) {
      GSU.apvRamBank[i] = &GSU.pvRam[(i % GSU.nRamBanks) << 16];
      GSU.apvRomBank[0x70 + i] = GSU.apvRamBank[i];
   }

   GSU.vPipe = 0x01;
   fx_readRegisterSpaceForCheck();
   fx_readRegisterSpaceForUse();
}

static bool fx_checkStartAddress(void)
{
   if (GSU.bCacheActive && R15 >= GSU.vCacheBaseReg && R15 < (GSU.vCacheBaseReg + 512))
      return true;
   if (GSU.vPrgBankReg >= 0x60 && GSU.vPrgBankReg <= 0x6f)
      return false;
   if (GSU.vPrgBankReg >= 0x74)
      return false;
   if (GSU.vPrgBankReg >= 0x70 && !(SCMR & (1 << 3)))
      return false;
   if (!(SCMR & (1 << 4)))
      return false;
   return true;
}

int32_t FxEmulate(uint32_t nInstructions)
{
   uint32_t vCount;
#ifdef FRANK_SNES_SUPERFX_DIAG
   g_gsu_call_count++;
#endif
   fx_readRegisterSpaceForCheck();

   if (!fx_checkStartAddress()) {
#ifdef FRANK_SNES_SUPERFX_DIAG
      g_gsu_start_fail++;
#endif
      CF(G);
      fx_writeRegisterSpaceAfterCheck();
      return 0;
   }

   fx_readRegisterSpaceForUse();
#ifdef FRANK_SNES_SUPERFX_DIAG
   g_gsu_last_scmr = GSU.pvRegisters[GSU_SCMR];
   g_gsu_last_scbr = GSU.pvRegisters[GSU_SCBR];
   g_gsu_last_mode = GSU.vMode;
#endif
   CF(IRQ);
   vCount = fx_run(nInstructions);
#ifdef FRANK_SNES_SUPERFX_DIAG
   g_gsu_inst_count += vCount;
#endif

   fx_writeRegisterSpaceAfterCheck();
   fx_writeRegisterSpaceAfterUse();
   return vCount;
}
