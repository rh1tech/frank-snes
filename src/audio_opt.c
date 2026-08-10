/*
 * Audio optimization functions - Step 1: C implementation
 * This matches the original logic exactly for validation
 */

#include "audio_opt.h"

// From main.c
static inline int16_t clamp16(int32_t v) {
    if (v < -32768) return -32768;
    if (v > 32767) return 32767;
    return v;
}

static inline int16_t soft_limit16(int32_t v) {
    // Two-stage soft limiter for smooth compression at high gain
    // Stage 1: gentle compression above ±24000
    if (v > 24000) {
        v = 24000 + ((v - 24000) >> 2);
    } else if (v < -24000) {
        v = -24000 + ((v + 24000) >> 2);
    }
    // Stage 2: harder compression above ±30000
    if (v > 30000) {
        v = 30000 + ((v - 30000) >> 3);
    } else if (v < -30000) {
        v = -30000 + ((v + 30000) >> 3);
    }
    return clamp16(v);
}

#ifndef PICO_ON_DEVICE
// C implementation for non-device builds
void audio_pack_opt(uint32_t* dst, const int16_t* src, uint32_t count,
                     int gain_num, int gain_den, bool use_soft_limit)
{
    for (uint32_t i = 0; i < count; i++) {
        int32_t left = (src[i * 2] * gain_num) / gain_den;
        int32_t right = (src[i * 2 + 1] * gain_num) / gain_den;
        
        if (use_soft_limit) {
            left = soft_limit16(left);
            right = soft_limit16(right);
        } else {
            left = clamp16(left);
            right = clamp16(right);
        }
        
        dst[i] = ((uint32_t)(uint16_t)left << 16) | (uint16_t)right;
    }
}

void audio_pack_mono_to_stereo(uint32_t* dst, const int16_t* src, uint32_t count,
                                int gain_num, int gain_den, bool use_soft_limit)
{
    for (uint32_t i = 0; i < count; i++) {
        int32_t mono = (src[i] * gain_num) / gain_den;
        
        if (use_soft_limit) {
            mono = soft_limit16(mono);
        } else {
            mono = clamp16(mono);
        }
        
        // Duplicate mono to both L and R channels
        dst[i] = ((uint32_t)(uint16_t)mono << 16) | (uint16_t)mono;
    }
}
#else
// Device implementation - use the assembly version or this C fallback
void audio_pack_mono_to_stereo(uint32_t* dst, const int16_t* src, uint32_t count,
                                int gain_num, int gain_den, bool use_soft_limit)
{
    for (uint32_t i = 0; i < count; i++) {
        int32_t mono = (src[i] * gain_num) / gain_den;
        
        if (use_soft_limit) {
            mono = soft_limit16(mono);
        } else {
            mono = clamp16(mono);
        }
        
        // Duplicate mono to both L and R channels
        dst[i] = ((uint32_t)(uint16_t)mono << 16) | (uint16_t)mono;
    }
}
#endif

/**
 * Optimized no-echo mixing
 * Matches the logic from soundux.c lines 838-843:
 *   for (J = 0; J < sample_count; J++) {
 *     I = (MixBuffer[J] * SoundData.master_volume[J & 1]) / VOL_DIV16;
 *     CLIP16(I);
 *     buffer[J] = I;
 *   }
 * VOL_DIV16 = 128 (0x80)
 */
void audio_mix_noecho_opt(int16_t* buffer, int32_t sample_count,
                           const int32_t* MixBuffer, const int16_t* master_volume)
{
    const int32_t left_vol = master_volume[0];
    const int32_t right_vol = master_volume[1];

    int32_t i = 0;
    for (; i < sample_count - 1; i += 2) {
        int32_t l = (MixBuffer[i]     * left_vol)  >> 10;
        int32_t r = (MixBuffer[i + 1] * right_vol) >> 10;
        if (l < -32768) l = -32768; else if (l > 32767) l = 32767;
        if (r < -32768) r = -32768; else if (r > 32767) r = 32767;
        buffer[i]     = l;
        buffer[i + 1] = r;
    }
    if (i < sample_count) {
        int32_t l = (MixBuffer[i] * left_vol) >> 10;
        if (l < -32768) l = -32768; else if (l > 32767) l = 32767;
        buffer[i] = l;
    }
}
