/*
 * frank-snes - HDMI_ALT driver header
 *
 * Alternative HDMI path backed by libdvi (PicoDVI-audio fork).  Compiled
 * when -DHDMI_DRIVER=ALT is passed to CMake.  Exposes the same
 * graphics_* / startVIDEO API as drivers/HDMI.h so src/main.c does not
 * branch.
 *
 * The original drivers/HDMI.h is included for type definitions
 * (g_out, video_mode_t, graphics_mode_t, tab_color[]).  This header
 * intentionally re-uses that file so palette tables and enums match.
 */
#ifndef HDMI_ALT_H_
#define HDMI_ALT_H_

#include "HDMI.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Push a single 32-bit packed stereo frame (L<<16 | R) into the HDMI
 * audio data-island ring.  Called by Core 1's audio loop in main.c when
 * FRANK_SNES_HDMI_ALT is defined, replacing i2s_dma_write().
 *
 * Returns the number of frames actually written.  Drops samples
 * silently when the ring is full so we never block emulation. */
uint32_t hdmi_alt_audio_write(const int16_t *frames_lr, uint32_t num_frames);

/* Number of free audio frames in the ring; useful for pacing. */
uint32_t hdmi_alt_audio_free(void);

#ifdef __cplusplus
}
#endif

#endif /* HDMI_ALT_H_ */
