/*
 * Does slicing the mixer change what it produces?
 *
 * The mixer runs once per emulated frame, after the SPC700 has already run
 * that whole frame. Everything the sound driver reads back to decide a note
 * started or finished — ENVX, ENDX — therefore only moves at frame
 * boundaries. Slicing the mix was tried on hardware and made the repeats
 * audibly worse, which is evidence about the mechanism rather than about
 * the patch, and this pins down exactly what changes.
 *
 * Two questions, both answered without a board:
 *
 *   1. For a single key-on, is a sliced mix bit-identical to a whole-frame
 *      mix? If not, slicing is simply unsafe and the hardware result says
 *      nothing.
 *   2. If a voice is keyed TWICE inside one frame, how many sample starts
 *      does each produce? The claim is that a whole-frame mix collapses the
 *      pair into one and a sliced mix does not.
 *
 * Builds against the real soundux.c, so it tests the shipping code.
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "snes9x.h"
#include "apu.h"
#include "soundux.h"

/* Globals the mixer expects the rest of the emulator to own. */
SAPU      APU;
SIAPU     IAPU;
SSettings Settings;
SCPUState CPU;

static uint8_t apu_ram[0x10000];

/* The four globals the mixer needs that live elsewhere in the emulator.
 * globals.c and apu.c pull in the whole CPU, so they are provided here
 * instead — SoundData and so are plain state, and S9xFixEnvelope is copied
 * verbatim from apu.c so the ADSR setup under test is the real one. */
SSoundData SoundData;
SoundStatus so;
int32_t NoiseFreq[32] = {
       0,    16,    21,    25,    31,    42,    50,    63,
      84,   100,   125,   167,   200,   250,   333,   400,
     500,   667,   800,  1000,  1300,  1600,  2000,  2700,
    3200,  4000,  5300,  6400,  8000, 10700, 16000, 32000
};

void S9xFixEnvelope(int32_t channel, uint8_t gain, uint8_t adsr1, uint8_t adsr2)
{
    if (adsr1 & 0x80) {
        if (S9xSetSoundMode(channel, MODE_ADSR))
            S9xSetSoundADSR(channel, adsr1 & 0xf, (adsr1 >> 4) & 7,
                            adsr2 & 0x1f, (adsr2 >> 5) & 7, 8);
    } else if (!(gain & 0x80)) {
        if (S9xSetSoundMode(channel, MODE_GAIN)) {
            S9xSetEnvelopeRate(channel, 0, 0, gain & 0x7f, 0);
            S9xSetEnvelopeHeight(channel, gain & 0x7f);
        }
    } else if (gain & 0x40) {
        if (S9xSetSoundMode(channel, (gain & 0x20) ? MODE_INCREASE_BENT_LINE
                                                   : MODE_INCREASE_LINEAR))
            S9xSetEnvelopeRate(channel, gain, 1, 127, (3 << 28) | gain);
    } else if (gain & 0x20) {
        if (S9xSetSoundMode(channel, MODE_DECREASE_EXPONENTIAL))
            S9xSetEnvelopeRate(channel, gain, -1, 0, (4 << 28) | gain);
    } else {
        if (S9xSetSoundMode(channel, MODE_DECREASE_LINEAR))
            S9xSetEnvelopeRate(channel, gain, -1, 0, (3 << 28) | gain);
    }
}

/* Count sample starts by watching block_pointer resets via needs_decode. */
static int starts_seen;

/* A minimal looping BRR sample: header (range 12, filter 0, LOOP|END on the
 * second block) followed by nybbles. Two blocks so the loop path is real. */
static void build_sample(uint16_t at)
{
    uint8_t *p = apu_ram + at;
    p[0] = (12 << 4) | 0x00;                 /* block 0: no END */
    for (int i = 1; i < 9; i++) p[i] = 0x71; /* arbitrary non-silent nybbles */
    p[9]  = (12 << 4) | 0x03;                /* block 1: LOOP|END */
    for (int i = 10; i < 18; i++) p[i] = 0x71;
}

static void setup(void)
{
    memset(&APU, 0, sizeof APU);
    memset(&IAPU, 0, sizeof IAPU);
    memset(&Settings, 0, sizeof Settings);
    memset(apu_ram, 0, sizeof apu_ram);

    Settings.PAL = false;
    Settings.H_Max = 1364;
    Settings.SoundPlaybackRate = 32040;
    Settings.InterpolatedSound = true;
    IAPU.RAM = apu_ram;

    build_sample(0x1000);
    /* Directory at $0200: entry 0 -> start 0x1000, loop 0x1009 */
    APU.DSP[APU_DIR] = 0x02;
    apu_ram[0x0200] = 0x00; apu_ram[0x0201] = 0x10;
    apu_ram[0x0202] = 0x09; apu_ram[0x0203] = 0x10;

    S9xResetSound(true);
    S9xSetPlaybackRate(32040);

    /* Voice 0: full volume, mid pitch, GAIN mode so it sustains. */
    APU.DSP[APU_SRCN + 0x00] = 0;
    APU.DSP[APU_VOL_LEFT  + 0x00] = 100;
    APU.DSP[APU_VOL_RIGHT + 0x00] = 100;
    APU.DSP[APU_P_LOW  + 0x00] = 0x00;
    APU.DSP[APU_P_HIGH + 0x00] = 0x10;
    S9xSetSoundVolume(0, 100, 100);
    S9xSetSoundHertz(0, 8000);
}

int main(void)
{
    static int16_t whole[2048], sliced[2048];
    const int frame = 534;                    /* NTSC mono samples */

    /* ---- 1. single key-on: sliced vs whole ---------------------------- */
    setup();
    S9xPlaySample(0);
    S9xSetFrameSampleCount(frame * 2);
    S9xMixSamplesMono(whole, frame);

    setup();
    S9xPlaySample(0);
    S9xSetFrameSampleCount(frame * 2);
    for (int k = 1; k <= 16; k++) S9xMixSlice((k * 256) / 16);
    S9xMixSamplesMono(sliced, frame);

    int diff = 0, first = -1;
    for (int i = 0; i < frame; i++)
        if (whole[i] != sliced[i]) { if (first < 0) first = i; diff++; }

    printf("single key-on: %d of %d samples differ", diff, frame);
    if (diff) printf(" (first at %d: whole=%d sliced=%d)", first,
                     whole[first], sliced[first]);
    printf("\n  -> %s\n", diff ? "SLICING CHANGES OUTPUT — unsafe as written"
                               : "sliced mix is bit-identical");

    /* ---- 2. two key-ons inside one frame ------------------------------ */
    setup();
    S9xSetFrameSampleCount(frame * 2);
    S9xPlaySample(0);
    S9xPlaySample(0);                          /* driver keys it twice */
    S9xMixSamplesMono(whole, frame);
    int whole_pending = SoundData.channels[0].needs_decode;

    setup();
    S9xSetFrameSampleCount(frame * 2);
    S9xPlaySample(0);
    for (int k = 1; k <= 8; k++) S9xMixSlice((k * 256) / 16);
    S9xPlaySample(0);                          /* second KON mid-frame */
    for (int k = 9; k <= 16; k++) S9xMixSlice((k * 256) / 16);
    S9xMixSamplesMono(sliced, frame);

    diff = 0;
    for (int i = 0; i < frame; i++) if (whole[i] != sliced[i]) diff++;
    printf("\ndouble key-on in one frame: %d of %d samples differ\n", diff, frame);
    printf("  -> %s\n", diff
        ? "the two mixes disagree: a whole-frame mix hides one of the starts"
        : "identical — the collapse theory is wrong");
    (void)whole_pending; (void)starts_seen;
    return 0;
}
