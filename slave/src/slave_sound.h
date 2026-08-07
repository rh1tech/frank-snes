/*
 * frank-snes — C2 slave firmware
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * slave_sound.h — the sound half of the emulator, driven by the
 * master's event stream instead of by an SPC700.
 */
#ifndef SLAVE_SOUND_H
#define SLAVE_SOUND_H

#include <stdbool.h>
#include <stdint.h>

#include "link_proto.h"

/* Bring up soundux and the DSP register file. Called once at boot. */
void slave_sound_init(void);

/* LINK_OP_RESET: back to the state S9xResetAPU() leaves behind. */
void slave_sound_reset(void);

/* LINK_OP_CONFIG: mirror the master's Settings. */
void slave_sound_config(const link_sound_config_t *cfg);

/* Where the master's APU RAM lands. The mixer reads BRR sample data
 * straight out of this, so it is the same array snes9x calls IAPU.RAM. */
uint8_t *slave_sound_aram(void);

/* How many int16 samples `chunks` frames' worth comes to, derived from
 * the last LINK_OP_CONFIG. The master derives the same number from the
 * same config and the same chunk count, which is what lets the sample
 * bulk carry no length of its own.
 *
 * chunks is normally 1. It rises when the emulator is running below
 * 60 fps and main.c's wall-clock catch-up is asking for extra audio —
 * see LINK_SAMPLES_PER_CHUNK in link_proto.h. */
uint32_t slave_sound_want(uint32_t chunks);

/* LINK_OP_FRAME: replay `n_events` DSP writes, then mix `want` int16
 * samples into `out`. Fills in the parts of `reply` the mixer owns and
 * returns how many samples were produced.
 *
 * Replaying every write before mixing is not an approximation — it is
 * exactly what the single-chip build does. S9xMixSamples*() is called
 * once per frame from the emulation loop, after that frame's DSP writes
 * have all landed, and the sub-frame KON/KOFF queue in soundux.h
 * (S9xDSPQueueEvent) is dead code that nothing calls. So the event
 * timestamps are carried for diagnostics and for any future sub-frame
 * work, not because ordering within the frame currently matters. */
uint32_t slave_sound_frame(const link_event_t *events, uint32_t n_events,
                           int16_t *out, uint32_t want,
                           link_frame_reply_t *reply);

#endif /* SLAVE_SOUND_H */
