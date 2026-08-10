/*
 * frank-snes — the rate-matching FIFO between the mixer and the DAC.
 *
 * Lifted out of main.c unchanged so it can be driven on a host. It sits
 * on the one seam where audio can actually be lost: the emulator
 * produces a frame's worth whenever a frame finishes, and the DAC takes
 * a chunk every 16.7 ms whatever happens. Everything here exists to
 * reconcile those two rates, so a fault here is heard as sound cut short
 * — and until tests/audio_path_test.c there was no way to observe it
 * except by ear on hardware.
 *
 * Header-only and static: main.c includes it exactly where the code used
 * to be, so there is no linkage or build change, and the test includes
 * the same text the firmware compiles.
 */
#ifndef AUDIO_RATE_H
#define AUDIO_RATE_H

static int16_t  sfifo[SFIFO_FRAMES * AUDIO_CH];
/* Both cursors stay wrapped inside the ring. A free-running q16 read cursor
 * only spans 65535 frames — two seconds at 32 kHz — before it wraps and
 * starts reading stale samples. */
#define SFIFO_SPAN_Q16 ((uint32_t)SFIFO_FRAMES << 16)
static uint32_t sfifo_rd;        /* read cursor, frames, q16 fixed point  */
static uint32_t sfifo_wr;        /* write cursor, whole frames            */
static uint32_t sfifo_fill;      /* frames available                      */

static void sfifo_reset(void)
{
    sfifo_rd = 0;
    sfifo_wr = 0;
    sfifo_fill = 0;
}

static void sfifo_push(const int16_t *src, uint32_t frames)
{
    for (uint32_t i = 0; i < frames; i++) {
        int16_t *d = &sfifo[sfifo_wr * AUDIO_CH];
        for (uint32_t c = 0; c < AUDIO_CH; c++) d[c] = src[i * AUDIO_CH + c];
        if (++sfifo_wr >= SFIFO_FRAMES) sfifo_wr = 0;
        if (sfifo_fill < SFIFO_FRAMES) {
            sfifo_fill++;
        } else {                            /* overwrote the oldest frame */
            sfifo_rd += 1u << 16;
            if (sfifo_rd >= SFIFO_SPAN_Q16) sfifo_rd -= SFIFO_SPAN_Q16;
            audio_discards++;               /* the only place audio is lost */
        }
    }
}

/* Pull one chunk, resampling at `ratio` (q16 input frames per output frame).
 * The caller derives the ratio from ring depth, which is the only signal
 * that reflects the DAC's true rate. */
/* How often the ring could not supply real audio. A slow producer should
 * come out as *slow* sound, never as holes: every one of these is a
 * fraction of a millisecond of held sample where emulated audio should
 * have been. Non-zero means the resampler could not stretch far enough. */
uint32_t sfifo_dry;

static void sfifo_pull(int16_t *dst, uint32_t frames, int32_t ratio)
{

    for (uint32_t i = 0; i < frames; i++) {
        uint32_t whole = sfifo_rd >> 16;
        if (sfifo_fill < 2) {
            sfifo_dry++;
            /*
             * Dry mid-chunk. Emitting zeros here punches a hole into
             * whatever was sounding — measured at 13 holes even with the
             * producer running at a perfect 60 fps, because the caller's
             * `need` estimate rounds down and the last frame or two of a
             * chunk can fall off the end. Repeating the last frame
             * instead costs a fraction of a millisecond of held sample,
             * which is inaudible, where a zero is a click.
             */
            for (uint32_t c = 0; c < AUDIO_CH; c++)
                dst[i * AUDIO_CH + c] = i ? dst[(i - 1) * AUDIO_CH + c] : 0;
            continue;
        }
        uint32_t frac = sfifo_rd & 0xffff;
        uint32_t nxt  = (whole + 1 >= SFIFO_FRAMES) ? 0 : whole + 1;
        const int16_t *a = &sfifo[whole * AUDIO_CH];
        const int16_t *b = &sfifo[nxt   * AUDIO_CH];
        for (uint32_t c = 0; c < AUDIO_CH; c++)
            dst[i * AUDIO_CH + c] =
                (int16_t)(a[c] + (((int32_t)(b[c] - a[c]) * (int32_t)frac) >> 16));
        sfifo_rd += ratio;
        uint32_t consumed = (sfifo_rd >> 16) - whole;
        if (sfifo_rd >= SFIFO_SPAN_Q16) {
            sfifo_rd -= SFIFO_SPAN_Q16;
            consumed = (sfifo_rd >> 16) + SFIFO_FRAMES - whole;
        }
        if (consumed)
            sfifo_fill = (sfifo_fill > consumed) ? sfifo_fill - consumed : 0;
    }
}


#endif /* AUDIO_RATE_H */
