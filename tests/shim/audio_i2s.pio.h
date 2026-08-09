#ifndef TESTS_AUDIO_I2S_PIO_H
#define TESTS_AUDIO_I2S_PIO_H

#include "host_sdk.h"

static const struct pio_program audio_i2s_program = {0};

static inline void audio_i2s_program_init(PIO pio, uint sm, uint offset,
                                          uint data_pin,
                                          uint clock_pin_base)
{
    (void)pio;
    (void)sm;
    (void)offset;
    (void)data_pin;
    (void)clock_pin_base;
}

#endif
