/*
 * spc_dsp.h — accurate S-DSP, extracted from blargg's SPC_DSP.
 *
 * Source: snes9x2005 / snes9x 1.53 apu_blargg.c, the DSP half only.
 * Copyright (C) 2007 Shay Green. LGPL 2.1 or later; see the notice in
 * spc_dsp.c.
 *
 * Why this exists
 * ---------------
 * The sound core this port shipped with is snes9x 1.43-era soundux.c: a
 * 16-sample-block BRR decoder with rate-table envelopes, driven once per
 * emulated frame. It cannot represent what the real DSP does within a
 * frame, and on games that stream digitised speech with GAIN envelopes —
 * Mortal Kombat 3 is the one that exposed it here — the result is cut
 * samples, clicks and bursts of noise.
 *
 * This module is the real thing: a cycle-driven DSP that runs the same
 * 32-clock sample pipeline the hardware does, decodes BRR the way the
 * hardware does, and applies the true 4-point Gaussian.
 *
 * Only the DSP is taken. The SPC700 in src/snes9x/spc700.c stays exactly
 * as it is, and so does the $2140..$2143 port handshake with the 65816 —
 * that code works and is what makes the C2 split possible at all.
 *
 * Interface
 * ---------
 * The DSP owns no memory of its own: spc_dsp_init() is handed the 64 KB
 * APU RAM and reads BRR data straight out of it, exactly as the hardware
 * reads the same bus. That is what lets the C2 slave run this against
 * its mirror of APU RAM with no further plumbing.
 */
#ifndef SPC_DSP_H
#define SPC_DSP_H

#include <stdint.h>
#include <stddef.h>

#define SPC_DSP_VOICE_COUNT      8
#define SPC_DSP_REGISTER_COUNT   128
#define SPC_DSP_CLOCKS_PER_SAMPLE 32

/* Bring up the DSP against `ram_64k` (the emulator's APU RAM) and reset
 * it to power-on state. */
void spc_dsp_init(void *ram_64k);

/* Re-point the DSP at APU RAM without resetting it. Used after a
 * savestate load, where the restored state carries the saved build's
 * pointer. */
void spc_dsp_init_ram(void *ram_64k);

/* Power-on reset (register file to hardware defaults). */
void spc_dsp_reset(void);

/* The SNES reset switch: FLG = 0xE0, internal state cleared, registers
 * left alone. */
void spc_dsp_soft_reset(void);

/* Where stereo sample pairs are written. Interleaved L,R. Passing NULL
 * makes the DSP run without producing output, which is what the master
 * does when the C2 slave owns sample generation. */
void spc_dsp_set_output(int16_t *out, int32_t size);

/* Sample pairs written since the last spc_dsp_set_output(). */
int32_t spc_dsp_samples_written(void);

/* Run the DSP for `clocks` cycles. 32 clocks is one sample pair at
 * 32 kHz, and one clock is one SPC700 cycle. */
void spc_dsp_run(int32_t clocks);

/* A write to DSP register `addr` (0..127). Equivalent to the SPC700
 * storing to $00F3 with $00F2 holding `addr`. */
void spc_dsp_write(uint8_t addr, uint8_t data);

/* Read back a DSP register. ENVX, OUTX and ENDX are maintained in the
 * register file by the running DSP, so this is a plain fetch. */
uint8_t spc_dsp_read(uint8_t addr);

/* The whole 128-byte register file, for bulk transfer. On C2 this is
 * what the slave sends back so the master can answer the SPC700's reads
 * without a round trip. */
uint8_t *spc_dsp_regs(void);

/* The IPL ROM window at $FFC0..$FFFF. The DSP has to know about it
 * because an echo buffer placed there writes to the RAM *under* the ROM
 * rather than to the visible bytes. `hi_ram` is the 64 bytes underneath
 * (the emulator's APU.ExtraRAM) and `rom` the ROM image itself; pass
 * enabled = 0 to ignore the overlay entirely, which is what the C2 slave
 * does since the master owns that state. */
void spc_dsp_set_rom_window(int enabled, uint8_t *hi_ram, uint8_t *rom);
int  spc_dsp_rom_enabled(void);

/* The DSP's current output pointer and its internal overflow buffer.
 * blargg's SPC700 needs both to work out how many samples ran past the
 * end of the frame it asked for; nothing else should touch them. */
int16_t *spc_dsp_out_ptr(void);
int16_t *spc_dsp_extra_ptr(void);
int32_t  spc_dsp_extra_size(void);

/* The DSP's whole state, for savestates. It contains no pointers into
 * itself, so it serialises as one opaque block — except `ram`, which is
 * re-pointed by spc_dsp_init() on load. */
void   *spc_dsp_state(void);
size_t  spc_dsp_state_size(void);

#endif /* SPC_DSP_H */
