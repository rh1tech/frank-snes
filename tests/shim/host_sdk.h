#ifndef TESTS_HOST_SDK_H
#define TESTS_HOST_SDK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef unsigned int uint;

typedef struct {
    volatile uint32_t txf[4];
} pio_hw_t;

typedef pio_hw_t *PIO;

extern pio_hw_t pio0_hw_inst;
extern pio_hw_t pio1_hw_inst;

#define pio0 (&pio0_hw_inst)
#define pio1 (&pio1_hw_inst)

struct pio_program {
    uint8_t unused;
};

static inline uint pio_claim_unused_sm(PIO pio, bool required)
{
    (void)pio;
    (void)required;
    return 0;
}

static inline uint pio_add_program(PIO pio, const struct pio_program *program)
{
    (void)pio;
    (void)program;
    return 0;
}

static inline void pio_sm_clear_fifos(PIO pio, uint sm)
{
    (void)pio;
    (void)sm;
}

static inline void pio_sm_set_clkdiv_int_frac(PIO pio, uint sm,
                                               uint16_t div_int,
                                               uint8_t div_frac)
{
    (void)pio;
    (void)sm;
    (void)div_int;
    (void)div_frac;
}

static inline uint pio_get_dreq(PIO pio, uint sm, bool is_tx)
{
    (void)pio;
    (void)sm;
    (void)is_tx;
    return 0;
}

static inline void pio_sm_set_enabled(PIO pio, uint sm, bool enabled)
{
    (void)pio;
    (void)sm;
    (void)enabled;
}

static inline void pio_sm_put_blocking(PIO pio, uint sm, uint32_t value)
{
    (void)pio;
    (void)sm;
    (void)value;
}

typedef struct {
    volatile uint32_t ints0;
} dma_hw_t;

typedef struct {
    bool read_increment;
} dma_channel_config;

extern dma_hw_t *dma_hw;
extern const void *dma_read_addr[16];
extern uint32_t dma_count[16];
extern bool dma_read_increment[16];

#define DMA_SIZE_32 2

static inline void dma_channel_abort(uint channel)
{
    (void)channel;
}

static inline bool dma_channel_is_busy(uint channel)
{
    (void)channel;
    return false;
}

static inline void dma_channel_unclaim(uint channel)
{
    (void)channel;
}

static inline void dma_channel_claim(uint channel)
{
    (void)channel;
}

static inline dma_channel_config dma_channel_get_default_config(uint channel)
{
    (void)channel;
    return (dma_channel_config){0};
}

static inline void channel_config_set_read_increment(dma_channel_config *cfg,
                                                      bool increment)
{
    cfg->read_increment = increment;
}

static inline void channel_config_set_write_increment(dma_channel_config *cfg,
                                                       bool increment)
{
    (void)cfg;
    (void)increment;
}

static inline void channel_config_set_transfer_data_size(
    dma_channel_config *cfg, uint size)
{
    (void)cfg;
    (void)size;
}

static inline void channel_config_set_dreq(dma_channel_config *cfg, uint dreq)
{
    (void)cfg;
    (void)dreq;
}

static inline void channel_config_set_chain_to(dma_channel_config *cfg,
                                                uint channel)
{
    (void)cfg;
    (void)channel;
}

static inline void dma_channel_configure(uint channel,
                                         const dma_channel_config *cfg,
                                         volatile void *write_addr,
                                         const void *read_addr,
                                         uint32_t transfer_count,
                                         bool trigger)
{
    (void)cfg;
    (void)write_addr;
    (void)trigger;
    dma_read_addr[channel] = read_addr;
    dma_count[channel] = transfer_count;
    dma_read_increment[channel] = cfg->read_increment;
}

static inline void dma_channel_set_config(uint channel,
                                          const dma_channel_config *cfg,
                                          bool trigger)
{
    (void)channel;
    (void)trigger;
    dma_read_increment[channel] = cfg->read_increment;
}

static inline void dma_channel_start(uint channel)
{
    (void)channel;
}

static inline void dma_channel_set_irq0_enabled(uint channel, bool enabled)
{
    (void)channel;
    (void)enabled;
}

static inline void dma_channel_set_read_addr(uint channel,
                                             const void *read_addr,
                                             bool trigger)
{
    (void)trigger;
    dma_read_addr[channel] = read_addr;
}

static inline void dma_channel_set_trans_count(uint channel,
                                               uint32_t transfer_count,
                                               bool trigger)
{
    (void)trigger;
    dma_count[channel] = transfer_count;
}

enum {
    clk_sys,
};

static inline uint32_t clock_get_hz(uint clock)
{
    (void)clock;
    return 252000000u;
}

#define GPIO_FUNC_PIO0 0
#define GPIO_FUNC_PIO1 1
#define GPIO_DRIVE_STRENGTH_12MA 3

static inline void gpio_set_function(uint pin, uint function)
{
    (void)pin;
    (void)function;
}

static inline void gpio_set_drive_strength(uint pin, uint strength)
{
    (void)pin;
    (void)strength;
}

extern int g_irq_disabled;

static inline uint32_t save_and_disable_interrupts(void)
{
    uint32_t previous = (uint32_t)g_irq_disabled;
    g_irq_disabled = 1;
    return previous;
}

static inline void restore_interrupts(uint32_t state)
{
    g_irq_disabled = (int)state;
}

#define __dmb() __asm__ volatile("" ::: "memory")

#define DMA_IRQ_0 0

static inline void irq_set_exclusive_handler(uint irq, void (*handler)(void))
{
    (void)irq;
    (void)handler;
}

static inline void irq_set_priority(uint irq, uint8_t priority)
{
    (void)irq;
    (void)priority;
}

static inline void irq_set_enabled(uint irq, bool enabled)
{
    (void)irq;
    (void)enabled;
}

static inline void tight_loop_contents(void)
{
}

#endif
