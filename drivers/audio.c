/*
 * frank-snes - I2S Audio Driver with Alternating Double Buffer DMA
 * Based on murmgenesis audio driver by Mikhail Matveev
 *
 * Uses two DMA channels in ping-pong configuration. The completion IRQ
 * starts the opposite channel only after selecting a freshly filled
 * buffer or silence, so a missed producer deadline cannot replay audio.
 *
 * Each channel completion raises DMA_IRQ_0; the IRQ handler releases the
 * completed buffer and starts the opposite channel.
 *
 * No pico-extras dependency - direct PIO + DMA.
 */

#include "audio.h"
#include "board_config.h"
#include "audio_i2s.pio.h"

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "hardware/irq.h"

//=============================================================================
// State - alternating double-buffer (ping-pong) DMA
//=============================================================================

// NOTE: HDMI uses DMA_IRQ_1 with an exclusive handler.
// Audio uses DMA_IRQ_0 to avoid conflicts.
#define AUDIO_DMA_IRQ DMA_IRQ_0

// Fixed DMA channels for audio (keep away from dynamically-claimed HDMI channels)
#define AUDIO_DMA_CH_A 10
#define AUDIO_DMA_CH_B 11

/* M1/M2 do not have room for four static DMA buffers. Two provide one
 * active slot and one refill window; missed windows play silence rather
 * than stale data. */
#define DMA_BUFFER_COUNT 2
// Max buffer size in stereo frames (32-bit words).
// SNES at 32kHz/60fps = 533 frames; generous headroom.
#define DMA_BUFFER_MAX_SAMPLES 600

static uint32_t __attribute__((aligned(4))) dma_buffers[DMA_BUFFER_COUNT][DMA_BUFFER_MAX_SAMPLES];

// Bitmask of buffers the CPU is allowed to write (1 = free)
static volatile uint32_t dma_buffers_free_mask = 0;

/*
 * Which buffers hold audio the DMA has not played yet.
 *
 * Without this the scheduler simply re-armed channel A on buffer 0 and B on
 * buffer 1 every time they finished, whether or not the CPU had refilled
 * them. Miss one deadline and the DMA plays the same buffer a second
 * time — and because the DAC's rate is fixed, that is not heard as a
 * click but as the last 16.7 ms of sound said twice. On speech it is a
 * word repeating, which is exactly the fault reported.
 *
 * A buffer is marked ready when the CPU finishes filling it and cleared
 * when the DMA takes it. If it is not ready when its channel comes round,
 * the channel is pointed at silence instead. A gap is honest; a replay
 * invents audio the emulator never produced.
 */
static volatile uint32_t dma_buffers_ready_mask = 0;
static uint32_t __attribute__((aligned(4))) dma_silence;

/* Health, and the reason this was invisible for so long: every counter
 * in this codebase watched the producer, and the loss was here. */
volatile uint32_t i2s_buffers_fed = 0;
volatile uint32_t i2s_buffers_consumed = 0;
volatile uint32_t i2s_starved = 0;

// Pre-roll: fill both buffers before starting playback
#define PREROLL_BUFFERS 2
static volatile int preroll_count = 0;

/* Playback order is ring order: the producer fills strictly in sequence,
 * with channel A owning slot 0 and channel B slot 1. */
static volatile uint8_t fill_idx = 0;
static bool dma_a_has_data;
static bool dma_b_has_data;

static int dma_channel_a = -1;
static int dma_channel_b = -1;
static dma_channel_config dma_cfg_a_data;
static dma_channel_config dma_cfg_a_silence;
static dma_channel_config dma_cfg_b_data;
static dma_channel_config dma_cfg_b_silence;
static PIO audio_pio;
static uint audio_sm;
static uint32_t dma_transfer_count;

static volatile bool audio_running = false;

static void audio_dma_irq_handler(void);
static inline void start_channel(int ch, uint8_t slot, bool *has_data);

//=============================================================================
// I2S Implementation
//=============================================================================

i2s_config_t i2s_get_default_config(void) {
    i2s_config_t config = {
        .sample_freq = 18000,
        .channel_count = 2,
        .data_pin = I2S_DATA_PIN,
        .clock_pin_base = I2S_CLOCK_PIN_BASE,
        .pio = pio1,   // PIO1 - HDMI uses PIO0
        .sm = 0,
        .dma_channel = 0,
        .dma_trans_count = 300,
        .dma_buf = NULL,
        .volume = 0,
    };
    return config;
}

void i2s_init(i2s_config_t *config) {
    audio_pio = config->pio;
    dma_transfer_count = config->dma_trans_count;

    // Determine GPIO function based on which PIO we're using
    uint8_t func = (config->pio == pio0) ? GPIO_FUNC_PIO0 : GPIO_FUNC_PIO1;
    gpio_set_function(config->data_pin, func);
    gpio_set_function(config->clock_pin_base, func);
    gpio_set_function(config->clock_pin_base + 1, func);

    gpio_set_drive_strength(config->data_pin, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(config->clock_pin_base, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(config->clock_pin_base + 1, GPIO_DRIVE_STRENGTH_12MA);

    // Claim state machine
    audio_sm = pio_claim_unused_sm(audio_pio, true);
    config->sm = audio_sm;

    // Add PIO program
    uint offset = pio_add_program(audio_pio, &audio_i2s_program);
    audio_i2s_program_init(audio_pio, audio_sm, offset,
                           config->data_pin, config->clock_pin_base);

    // Drain the TX FIFO
    pio_sm_clear_fifos(audio_pio, audio_sm);

    // Set clock divider for sample rate
    uint32_t sys_clk = clock_get_hz(clk_sys);
    uint32_t divider = sys_clk * 4 / config->sample_freq;
    pio_sm_set_clkdiv_int_frac(audio_pio, audio_sm, divider >> 8u, divider & 0xffu);

    // Validate transfer count fits our static buffers
    dma_transfer_count = config->dma_trans_count;
    if (dma_transfer_count == 0) dma_transfer_count = 1;
    if (dma_transfer_count > DMA_BUFFER_MAX_SAMPLES) dma_transfer_count = DMA_BUFFER_MAX_SAMPLES;
    config->dma_trans_count = (uint16_t)dma_transfer_count;

    // Initialize DMA buffers with silence
    memset(dma_buffers, 0, sizeof(dma_buffers));
    config->dma_buf = (uint16_t *)(void *)dma_buffers[0];

    // Clear audio DMA IRQ flags (IRQ1)
    dma_hw->ints0 = (1u << AUDIO_DMA_CH_A) | (1u << AUDIO_DMA_CH_B);

    // Use fixed DMA channels for audio
    dma_channel_abort(AUDIO_DMA_CH_A);
    dma_channel_abort(AUDIO_DMA_CH_B);
    while (dma_channel_is_busy(AUDIO_DMA_CH_A) || dma_channel_is_busy(AUDIO_DMA_CH_B)) {
        tight_loop_contents();
    }

    dma_channel_unclaim(AUDIO_DMA_CH_A);
    dma_channel_unclaim(AUDIO_DMA_CH_B);
    dma_channel_claim(AUDIO_DMA_CH_A);
    dma_channel_claim(AUDIO_DMA_CH_B);
    dma_channel_a = AUDIO_DMA_CH_A;
    dma_channel_b = AUDIO_DMA_CH_B;
    config->dma_channel = (uint8_t)dma_channel_a;

    // Configure two channels; the completion IRQ alternates them.
    dma_cfg_a_data = dma_channel_get_default_config(dma_channel_a);
    channel_config_set_read_increment(&dma_cfg_a_data, true);
    channel_config_set_write_increment(&dma_cfg_a_data, false);
    channel_config_set_transfer_data_size(&dma_cfg_a_data, DMA_SIZE_32);
    channel_config_set_dreq(&dma_cfg_a_data,
                            pio_get_dreq(audio_pio, audio_sm, true));
    dma_cfg_a_silence = dma_cfg_a_data;
    channel_config_set_read_increment(&dma_cfg_a_silence, false);

    dma_cfg_b_data = dma_channel_get_default_config(dma_channel_b);
    channel_config_set_read_increment(&dma_cfg_b_data, true);
    channel_config_set_write_increment(&dma_cfg_b_data, false);
    channel_config_set_transfer_data_size(&dma_cfg_b_data, DMA_SIZE_32);
    channel_config_set_dreq(&dma_cfg_b_data,
                            pio_get_dreq(audio_pio, audio_sm, true));
    dma_cfg_b_silence = dma_cfg_b_data;
    channel_config_set_read_increment(&dma_cfg_b_silence, false);

    dma_channel_configure(
        dma_channel_a, &dma_cfg_a_silence,
        &audio_pio->txf[audio_sm], &dma_silence, dma_transfer_count, false);

    dma_channel_configure(
        dma_channel_b, &dma_cfg_b_silence,
        &audio_pio->txf[audio_sm], &dma_silence, dma_transfer_count, false);

    // Set up DMA IRQ1 handler (avoid HDMI's DMA_IRQ_0 exclusive handler)
    irq_set_exclusive_handler(AUDIO_DMA_IRQ, audio_dma_irq_handler);
    irq_set_priority(AUDIO_DMA_IRQ, 0x80);
    irq_set_enabled(AUDIO_DMA_IRQ, true);

    // Enable IRQ1 for both channels
    dma_hw->ints0 = (1u << dma_channel_a) | (1u << dma_channel_b);
    dma_channel_set_irq0_enabled(dma_channel_a, true);
    dma_channel_set_irq0_enabled(dma_channel_b, true);

    // Enable PIO state machine
    pio_sm_set_enabled(audio_pio, audio_sm, true);

    // Initialize state
    preroll_count = 0;
    dma_buffers_free_mask = (1u << DMA_BUFFER_COUNT) - 1u; // both free
    dma_buffers_ready_mask = 0;
    fill_idx = 0;
    dma_a_has_data = false;
    dma_b_has_data = false;
    audio_running = false;
}

void i2s_dma_reset(i2s_config_t *config) {
    uint32_t irq_state = save_and_disable_interrupts();

    dma_channel_abort(dma_channel_a);
    dma_channel_abort(dma_channel_b);
    while (dma_channel_is_busy(dma_channel_a) ||
           dma_channel_is_busy(dma_channel_b)) {
        tight_loop_contents();
    }

    dma_hw->ints0 = (1u << dma_channel_a) | (1u << dma_channel_b);
    dma_channel_set_config(dma_channel_a, &dma_cfg_a_silence, false);
    dma_channel_set_config(dma_channel_b, &dma_cfg_b_silence, false);
    dma_channel_set_read_addr(dma_channel_a, &dma_silence, false);
    dma_channel_set_read_addr(dma_channel_b, &dma_silence, false);
    dma_channel_set_trans_count(dma_channel_a, dma_transfer_count, false);
    dma_channel_set_trans_count(dma_channel_b, dma_transfer_count, false);

    dma_buffers_free_mask = (1u << DMA_BUFFER_COUNT) - 1u;
    dma_buffers_ready_mask = 0;
    preroll_count = 0;
    fill_idx = 0;
    dma_a_has_data = false;
    dma_b_has_data = false;
    audio_running = false;
    config->dma_buf = (uint16_t *)(void *)dma_buffers[0];

    restore_interrupts(irq_state);
}

void i2s_write(const i2s_config_t *config, const int16_t *samples, const size_t len) {
    for (size_t i = 0; i < len; i++) {
        pio_sm_put_blocking(config->pio, config->sm, (uint32_t)samples[i]);
    }
}

void i2s_dma_write(i2s_config_t *config, const int16_t *samples) {
    // Wait for a free buffer, then claim it (atomically vs DMA IRQ)
    uint8_t buf_index = 0;
    while (true) {
        uint32_t irq_state = save_and_disable_interrupts();
        uint32_t free_mask = dma_buffers_free_mask;

        if (!audio_running) {
            // Pre-roll fills buffer 0 then buffer 1 to preserve ordering
            buf_index = (uint8_t)preroll_count;
            if (buf_index < DMA_BUFFER_COUNT && (free_mask & (1u << buf_index))) {
                dma_buffers_free_mask &= ~(1u << buf_index);
                restore_interrupts(irq_state);
                break;
            }
        } else {
            if (free_mask & (1u << fill_idx)) {
                buf_index = fill_idx;
                fill_idx = (uint8_t)((fill_idx + 1u) & (DMA_BUFFER_COUNT - 1u));
                dma_buffers_free_mask &= ~(1u << buf_index);
                restore_interrupts(irq_state);
                break;
            }
        }

        restore_interrupts(irq_state);
        tight_loop_contents();
    }

    uint32_t *dst = dma_buffers[buf_index];
    const uint32_t *src = (const uint32_t *)samples;
    const uint shift = config->volume;

    if (shift == 0) {
        memcpy(dst, src, dma_transfer_count * sizeof(uint32_t));
    } else {
#ifdef PICO_ON_DEVICE
        audio_volume_shift(dst, src, dma_transfer_count, shift);
#else
        for (uint32_t i = 0; i < dma_transfer_count; i++) {
            uint32_t v = src[i];
            int16_t l = (int16_t)(v >> 16);
            int16_t r = (int16_t)(v & 0xFFFF);
            l >>= shift;
            r >>= shift;
            dst[i] = ((uint32_t)(uint16_t)l << 16) | (uint16_t)r;
        }
#endif
    }

    // Memory barrier to ensure writes are visible before DMA reads
    __dmb();

    /* Now, and only now, is this slot worth playing. The DMA takes it on
     * the next playback turn and clears the bit; if the CPU is late the bit
     * is still clear and the scheduler plays silence rather than replaying
     * what is already in the buffer. */
    {
        /* The DMA IRQ runs on this same core and clears bits in this mask
         * (see arm_channel), so a plain read-modify-write here can lose
         * its clear: the buffer would be marked ready again after the DMA
         * had already taken it, and the scheduler would play it a second
         * time. That is a hard 16.7 ms replay, and while the IRQ phase
         * holds it recurs every round — audible as a stutter on anything
         * sustained, which is what appeared on the boot jingle. */
        uint32_t irq = save_and_disable_interrupts();
        dma_buffers_free_mask &= ~(1u << buf_index);
        dma_buffers_ready_mask |= (1u << buf_index);
        restore_interrupts(irq);
    }
    i2s_buffers_fed++;

    if (!audio_running) {
        preroll_count++;
        if (preroll_count >= PREROLL_BUFFERS) {
            // Both buffers are filled and queued; start playback on channel A.
            start_channel(dma_channel_a, 0, &dma_a_has_data);
            audio_running = true;
        }
    }
}

void i2s_volume(i2s_config_t *config, uint8_t volume) {
    if (volume > 16) volume = 16;
    config->volume = volume;
}

void i2s_increase_volume(i2s_config_t *config) {
    if (config->volume > 0) config->volume--;
}

void i2s_decrease_volume(i2s_config_t *config) {
    if (config->volume < 16) config->volume++;
}

//=============================================================================
// DMA IRQ Handler
//=============================================================================

/* Start `ch` from its freshly filled slot, or from silence if the producer
 * missed this boundary. A slot that arrives while its channel is playing
 * silence remains queued for the following turn. */
static inline void start_channel(int ch, uint8_t slot, bool *has_data)
{
    uint32_t bit = 1u << slot;
    if (dma_buffers_ready_mask & bit) {
        dma_buffers_ready_mask &= ~bit;
        dma_channel_set_config(
            ch, ch == dma_channel_a ? &dma_cfg_a_data : &dma_cfg_b_data,
            false);
        dma_channel_set_read_addr(ch, dma_buffers[slot], false);
        *has_data = true;
    } else {
        /* Reserve this channel's slot while it emits silence. Otherwise
         * the producer could fill this older slot and the opposite newer
         * slot before the next IRQ, after which fixed A/B alternation
         * would play the newer buffer first. */
        dma_buffers_free_mask &= ~bit;
        dma_channel_set_config(
            ch, ch == dma_channel_a ? &dma_cfg_a_silence
                                    : &dma_cfg_b_silence,
            false);
        dma_channel_set_read_addr(ch, &dma_silence, false);
        *has_data = false;
    }
    dma_channel_set_trans_count(ch, dma_transfer_count, false);
    dma_channel_start(ch);
}

static inline void finish_channel(uint8_t slot, bool had_data)
{
    if (had_data) {
        dma_buffers_free_mask |= 1u << slot;
        i2s_buffers_consumed++;
    } else {
        uint32_t bit = 1u << slot;
        if (!(dma_buffers_ready_mask & bit))
            dma_buffers_free_mask |= bit;
        i2s_starved++;
    }
}

static void audio_dma_irq_handler(void) {
    uint32_t ints = dma_hw->ints0;
    uint32_t mask = 0;
    if (dma_channel_a >= 0) mask |= (1u << dma_channel_a);
    if (dma_channel_b >= 0) mask |= (1u << dma_channel_b);
    ints &= mask;
    if (!ints) return;

    if ((dma_channel_a >= 0) && (ints & (1u << dma_channel_a))) {
        dma_hw->ints0 = (1u << dma_channel_a);
        finish_channel(0, dma_a_has_data);
        start_channel(dma_channel_b, 1, &dma_b_has_data);
    }

    if ((dma_channel_b >= 0) && (ints & (1u << dma_channel_b))) {
        dma_hw->ints0 = (1u << dma_channel_b);
        finish_channel(1, dma_b_has_data);
        start_channel(dma_channel_a, 0, &dma_a_has_data);
    }
}
