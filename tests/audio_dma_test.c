/*
 * Host test for the I2S DMA buffer handoff in drivers/audio.c.
 *
 * The fault this guards against is not a crash, it is a lie: the chain
 * playing a buffer the CPU has not refilled, so the DAC emits audio the
 * emulator never produced a second time. On speech that is a word said
 * twice, and it is invisible to every producer-side diagnostic in this
 * tree because the producer did nothing wrong.
 *
 * drivers/audio.c is included rather than linked so the statics that
 * hold the ownership state are visible, and the SDK is stubbed under
 * tests/shim so the real code runs unmodified.
 */
#include <stdio.h>
#include <string.h>
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

#define FRAMES 534

static int failures;
static void check(int cond, const char *what)
{
    if (!cond) { printf("  FAIL: %s\n", what); failures++; }
}

/* Every chunk written carries a unique tag in its first word, so a
 * replay is detectable by seeing the same tag played twice. */
static uint32_t next_tag = 1;
static void produce(i2s_config_t *cfg)
{
    static uint32_t buf[FRAMES];
    uint32_t tag = next_tag++;
    for (int i = 0; i < FRAMES; i++) buf[i] = tag;

    /* The slot about to be handed over must not be one the DMA is armed
     * on, or the memcpy lands in a buffer that is mid-play. */
    i2s_dma_write(cfg, (const int16_t *)buf);
}

/* One playback step: record what the named channel emitted, then complete
 * it so the IRQ can select and start the opposite channel. */
static uint32_t played[4096];
static uint32_t n_played;
static void complete(int ch)
{
    const uint32_t *p = (const uint32_t *)dma_read_addr[ch];
    played[n_played++] = p ? p[0] : 0;
    dma_hw->ints0 = (1u << ch);
    audio_dma_irq_handler();
}

int main(void)
{
    i2s_config_t cfg = i2s_get_default_config();
    cfg.dma_trans_count = FRAMES;
    i2s_init(&cfg);

    /* Pre-roll, then alternate: producer keeps up. */
    for (int i = 0; i < PREROLL_BUFFERS; i++) produce(&cfg);
    check(audio_running, "playback starts after pre-roll");

    int ch = dma_channel_a;
    for (int step = 0; step < 200; step++) {
        complete(ch);
        produce(&cfg);
        ch = (ch == dma_channel_a) ? dma_channel_b : dma_channel_a;
    }

    /* Producer stalls for ten chain steps, then resumes. This is the
     * case that used to replay. */
    for (int step = 0; step < 10; step++) {
        complete(ch);
        ch = (ch == dma_channel_a) ? dma_channel_b : dma_channel_a;
    }
    for (int step = 0; step < 100; step++) {
        complete(ch);
        produce(&cfg);
        ch = (ch == dma_channel_a) ? dma_channel_b : dma_channel_a;
    }

    /* Invariant 1: no chunk is ever played twice. Tag 0 is silence and
     * may repeat as often as it likes. */
    static uint8_t seen[8192];
    int replays = 0;
    int reordered = 0;
    uint32_t last_tag = 0;
    for (uint32_t i = 0; i < n_played; i++) {
        uint32_t t = played[i];
        if (!t) continue;
        if (t < sizeof seen && seen[t]) replays++;
        if (t < sizeof seen) seen[t] = 1;
        if (t <= last_tag) reordered++;
        last_tag = t;
    }
    printf("  %u chain steps, %d replays, %d reordered\n",
           n_played, replays, reordered);
    check(replays == 0, "no buffer is ever played twice");
    check(reordered == 0, "fresh buffers retain producer order");

    /* Invariant 2: the starvation is counted, not hidden. */
    printf("  fed %u, consumed %u, starved %u\n",
           i2s_buffers_fed, i2s_buffers_consumed, i2s_starved);
    check(i2s_starved > 0, "the ten-step stall is reported as starvation");
    check(i2s_buffers_consumed + i2s_starved == n_played,
          "every chain step is either a real buffer or a counted gap");
    check(i2s_buffers_consumed <= i2s_buffers_fed,
          "nothing is played that was never filled");

    printf(failures ? "FAILED (%d)\n" : "ok\n", failures);
    return failures != 0;
}
