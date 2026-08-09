/*
 * Plays a known-good render through the real delivery path and records
 * what the DAC would actually have emitted.
 *
 * The host render is the reference: the user reports it as perfect, and
 * it is produced by writing S9xMixSamples output straight to a file,
 * touching none of the device's ring, DMA or chain. This harness takes
 * that same audio, pushes it through the real drivers/audio.c at a
 * chosen producer rate, and writes out whatever the chain played. Diff
 * the two and every fault in the last stage becomes a number instead of
 * something to argue about by ear.
 *
 * usage: audio_path_test <reference.wav> <producer fps> [stall chunks]
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <hardware/dma.h>
#include <hardware/pio.h>

int g_irq_disabled;
const void *dma_read_addr[16];
uint32_t dma_count[16];
bool dma_read_increment[16];
static dma_hw_t dma_hw_inst;
dma_hw_t *dma_hw = &dma_hw_inst;
pio_hw_t pio0_hw_inst, pio1_hw_inst;

#include "../drivers/audio.c"

/* The real FIFO the firmware uses, same text. */
#define AUDIO_BUFFER_LENGTH 534
#define AUDIO_CH 2
#define AUDIO_QUEUE_DEPTH 8
#define SFIFO_CHUNKS 4
#define SFIFO_FRAMES (AUDIO_BUFFER_LENGTH * SFIFO_CHUNKS)
static uint32_t audio_discards;
#include "../src/audio_rate.h"

/* Mirrors the servo in main.c's rate-matching loop. Kept here rather
 * than shared because that loop is still inline in the emulation loop;
 * extracting it is the next step, and until then this must be kept in
 * step by hand. */
static int32_t servo_ratio(uint32_t ring_depth, uint32_t frame_samples)
{
    static int32_t base_i;
    uint32_t buffered = ring_depth * AUDIO_BUFFER_LENGTH + sfifo_fill;
    const uint32_t target =
        (AUDIO_QUEUE_DEPTH * AUDIO_BUFFER_LENGTH + SFIFO_FRAMES) / 2u;
    int32_t err = (int32_t)buffered - (int32_t)target;
    int32_t mag = err < 0 ? -err : err;
    int32_t adj = (int32_t)(((int64_t)err * mag * 7864) /
                            ((int64_t)target * target));
    const int32_t nom = (int32_t)(((uint32_t)frame_samples << 16) / AUDIO_BUFFER_LENGTH);
    const int32_t lo = (int32_t)(((int64_t)nom * 32768) >> 16);
    const int32_t hi = (int32_t)(((int64_t)nom * 85197) >> 16);
    if (base_i == 0) base_i = nom;
    base_i += err * 32;
    if (base_i < lo) base_i = lo;
    if (base_i > hi) base_i = hi;
    int32_t r = base_i + adj;
    if (r < lo) r = lo;
    if (r > hi) r = hi;
    return r;
}

#define FRAMES 534            /* one chunk, and one DAC period at 32040 */
#define WARMUP_STEPS 60       /* startup silence before the servo is primed */

static uint32_t *ref;         /* reference, stereo pairs packed as u32 */
static uint32_t ref_frames;

static uint32_t *out;         /* what the chain actually played */
static uint32_t out_frames;

static uint32_t load_wav(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); exit(2); }
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 44, SEEK_SET);
    ref_frames = (uint32_t)((n - 44) / 4);
    ref = malloc(ref_frames * 4);
    if (fread(ref, 4, ref_frames, f) != ref_frames) { }
    fclose(f);
    return ref_frames;
}

static void write_wav(const char *path, const uint32_t *d, uint32_t frames)
{
    FILE *f = fopen(path, "wb");
    uint32_t bytes = frames * 4, riff = 36 + bytes, sz = 16, rate = 32040,
             brate = rate * 4;
    uint16_t fmt = 1, ch = 2, bp = 4, bits = 16;
    fwrite("RIFF", 1, 4, f); fwrite(&riff, 4, 1, f);
    fwrite("WAVEfmt ", 1, 8, f); fwrite(&sz, 4, 1, f);
    fwrite(&fmt, 2, 1, f); fwrite(&ch, 2, 1, f); fwrite(&rate, 4, 1, f);
    fwrite(&brate, 4, 1, f); fwrite(&bp, 2, 1, f); fwrite(&bits, 2, 1, f);
    fwrite("data", 1, 4, f); fwrite(&bytes, 4, 1, f);
    fwrite(d, 4, frames, f); fclose(f);
}

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: %s ref.wav fps [stall]\n", argv[0]); return 2; }
    double fps = atof(argv[2]);
    int stall_at = argc > 3 ? atoi(argv[3]) : -1;

    load_wav(argv[1]);
    uint32_t chunks = ref_frames / FRAMES;
    out = calloc(chunks + 64, FRAMES * 4);

    i2s_config_t cfg = i2s_get_default_config();
    cfg.dma_trans_count = FRAMES;
    i2s_init(&cfg);
    sfifo_reset();

    /*
     * The DAC is the fixed clock: one chunk every 1/60 s of real time,
     * always, whatever the producer manages. The producer runs at `fps`,
     * which is what an emulator below full speed looks like from here.
     */
    const double dac_period = 1.0 / 60.0;
    const double prod_period = 1.0 / fps;
    double t_dac = 0, t_prod = 0;
    /* main.c holds AUDIO_QUEUE_DEPTH packed chunks between the FIFO and
     * the DMA; without it the FIFO overflows and discards on every burst. */
    static int16_t queue[AUDIO_QUEUE_DEPTH][AUDIO_BUFFER_LENGTH * AUDIO_CH];
    uint32_t qw = 0, qr = 0, qn = 0;
    uint32_t next_in = 0, step = 0;
    uint32_t queue_underruns = 0;
    bool pipeline_started = false;
    static int16_t silence[AUDIO_BUFFER_LENGTH * AUDIO_CH];
    int ch = -1;

    for (uint32_t i = 0; i < PREROLL_BUFFERS && next_in < chunks; i++)
        i2s_dma_write(&cfg, (const int16_t *)&ref[next_in++ * FRAMES]);
    ch = dma_channel_a;

    while (step < chunks) {
        if (t_prod <= t_dac && next_in < chunks) {
            bool stalled = (stall_at >= 0 && (int)next_in >= stall_at &&
                            (int)next_in < stall_at + 10);
            if (!stalled)
                sfifo_push((const int16_t *)&ref[next_in * FRAMES], FRAMES);
            next_in++;
            /* drain the FIFO into the ring exactly as main.c does */
            for (;;) {
                uint32_t depth = 0;
                for (uint32_t b = 0; b < DMA_BUFFER_COUNT; b++)
                    if (!(dma_buffers_free_mask & (1u << b))) depth++;
                if (qn >= AUDIO_QUEUE_DEPTH) break;   /* main.c's packed ring */
                (void)depth;
                int32_t ratio = servo_ratio(qn, FRAMES);
                uint32_t need = ((AUDIO_BUFFER_LENGTH * (uint32_t)ratio) >> 16) + 4;
                if (sfifo_fill < need) break;
                sfifo_pull(queue[qw], AUDIO_BUFFER_LENGTH, ratio);
                qw = (qw + 1) % AUDIO_QUEUE_DEPTH; qn++;
            }
            t_prod += prod_period;
            continue;
        }
        /* One DAC period elapsed: record the channel that actually played,
         * then complete it so the IRQ starts the opposite channel. */
        const uint32_t *p = (const uint32_t *)dma_read_addr[ch];
        if (dma_read_increment[ch]) {
            memcpy(&out[out_frames], p, FRAMES * 4);
        } else {
            for (uint32_t i = 0; i < FRAMES; i++)
                out[out_frames + i] = *p;
        }
        out_frames += FRAMES;
        dma_hw->ints0 = (1u << ch);
        audio_dma_irq_handler();

        /* Core 1 always feeds the newly released DMA slot. An empty packed
         * queue becomes a fade-to-silence chunk in firmware; count that as
         * a path underrun once the pipeline has started. */
        if (dma_buffers_free_mask & (1u << fill_idx)) {
            if (qn) {
                i2s_dma_write(&cfg, queue[qr]);
                qr = (qr + 1) % AUDIO_QUEUE_DEPTH;
                qn--;
                pipeline_started = true;
            } else {
                if (pipeline_started && step >= WARMUP_STEPS)
                    queue_underruns++;
                i2s_dma_write(&cfg, silence);
            }
        }

        ch = (ch == dma_channel_a) ? dma_channel_b : dma_channel_a;
        t_dac += dac_period;
        step++;
    }

    write_wav("/tmp/host/played.wav", out, out_frames);

    /*
     * Exact accounting, not content matching.
     *
     * Comparing output chunk c against reference chunk c measures the
     * pipeline's delay, not its losses — the path delays by pre-roll
     * plus FIFO depth and the resampler shifts it further, so any
     * index-based comparison reports phantom holes. The invariants
     * below need no alignment and are exact:
     *
     *   every chain step played either a chunk the producer filled, or
     *   a counted silence; no chunk was played twice; and nothing was
     *   dropped on the way in.
     *
     * If those hold, the path is lossless by construction.
     */
    uint32_t replay = 0;
    for (uint32_t c = 1; c + 1 <= out_frames / FRAMES; c++)
    {   bool sil = true;
        for (uint32_t i = 0; i < FRAMES; i++) if (out[c*FRAMES+i]) { sil=false; break; }
        if (!sil && !memcmp(&out[c * FRAMES], &out[(c - 1) * FRAMES], FRAMES * 4))
            replay++; }
    uint32_t steps = out_frames / FRAMES;
    printf("  fps %-5.1f  chain steps %u   replays %u   dropped-in %u"
           "   queue-underruns %u\n",
           fps, steps, replay, audio_discards, queue_underruns);
    bool lossless =
        i2s_buffers_consumed + i2s_starved == steps &&
        replay == 0 && audio_discards == 0 && i2s_starved == 0 &&
        queue_underruns == 0;
    printf("  lossless: %s   (played %u = fed %u + starved %u)\n",
           lossless ? "YES" : "NO",
           steps, i2s_buffers_consumed, i2s_starved);
    printf("  driver counters: fed %u consumed %u starved %u\n",
           i2s_buffers_fed, i2s_buffers_consumed, i2s_starved);
    return lossless ? 0 : 1;
}
