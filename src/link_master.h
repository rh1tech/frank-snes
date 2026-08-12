/*
 * frank-snes — C2 inter-processor sound link, master half
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * link_master.h — bring-up and the per-frame exchange with the sound
 * slave. Only compiled for BOARD_VARIANT=C2 without C2_LOCAL_SOUND.
 *
 * The emulator never talks to link_bus/link_session directly; it calls
 * the four entry points below and sound_backend_link.c fills in what
 * travels. Keeping the wire in one file is what makes the "slave is
 * absent" path a single early return rather than a condition threaded
 * through the audio code.
 */
#ifndef LINK_MASTER_H
#define LINK_MASTER_H

#ifdef C2_SOUND_LINK

#include <stdbool.h>
#include <stdint.h>

#include "link_proto.h"

/* Bring up the PIO link and probe for the slave. Returns false if the
 * slave did not answer LINK_OP_HELLO — the caller carries on regardless
 * and the machine simply runs silent, which is a far better failure than
 * refusing to boot because one of two chips is unprogrammed. */
bool link_master_init(void);

/* True once a HELLO has been answered and no exchange has failed since.
 * Everything in sound_backend_link.c short-circuits on this. */
bool link_master_online(void);

/* Try to bring a dropped link back: quiesce the doorbells, wait for the
 * slave to do the same, then re-HELLO. Cheap to call and safe to call
 * repeatedly — it does not touch the PIO or the DMA channels. On success
 * the caller still owes the slave a full state push, because everything
 * sent while the link was down is gone. */
bool link_master_reprobe(void);

/* Push the emulator's sound settings. Called after a ROM loads and
 * whenever the settings menu changes something the slave mirrors. */
bool link_master_send_config(const link_sound_config_t *cfg);

/* Reset the slave's DSP and mixer, then push the whole 64 KB of APU RAM.
 * Used after S9xResetAPU() and after a savestate load, where a
 * dirty-page diff would be the entire image anyway. */
bool link_master_reset_and_sync_aram(const uint8_t *aram);

/* The steady-state exchange.
 *
 *   events / n_events   this frame's DSP writes, in timestamp order
 *   runs / n_runs       which stretches of APU RAM changed
 *   chunks              how many frames' worth of audio to mix back
 *   aram                the live 64 KB APU RAM; runs index into it and
 *                       the DMA reads it in place, so nothing is staged
 *   samples / n_samples where the slave's mixer output lands
 *   reply               the mixer state the SPC700 can read back
 *
 * Returns false and leaves `samples` untouched if the slave did not
 * answer; the caller substitutes silence. A single failure takes the
 * link offline until the next reset, because a half-completed exchange
 * has left the doorbells in an unknown phase. */
bool link_master_frame_exchange(const link_event_t *events, uint32_t n_events,
                                const link_aram_run_t *runs, uint32_t n_runs,
                                uint32_t chunks, const uint8_t *aram,
                                int16_t *samples, uint32_t n_samples,
                                link_frame_reply_t *reply);

#ifdef FRANK_SNES_PPU_CAPTURE
/* PPU offload. The stream goes out inside the SAME exchange as the sound
 * frame and the rendered picture comes back in the same reply, because the
 * first assembled board measured that the doorbell PHASES, not the bytes,
 * are what an exchange costs - see the note on LINK_OP_FRAME. A separate
 * PPU exchange would double the per-frame round trips to move 4 KB across a
 * link that is 99.94% idle.
 *
 *   ppu_stream / ppu_len   the captured PPU command stream for this frame
 *   fb / fb_max            where the slave's finished 8bpp picture lands
 *
 * Staged, not passed: link_master_frame_exchange() picks these up and sends
 * the stream between the sound payloads and the reply, so it lands in the
 * same phase group. Handing them to a separate call would put them in an
 * exchange of their own, which is exactly the cost being avoided.
 *
 *   ppu_stream / ppu_len   the captured PPU command stream for this frame
 *   fb / fb_max            where the slave's finished 8bpp picture lands
 *
 * Staging len == 0 leaves the exchange exactly as it was, so the sound path
 * is unaffected when the offload is off. */
void link_master_ppu_stage(const uint8_t *ppu_stream, uint32_t ppu_len,
                           uint8_t *fb, uint32_t fb_max);

extern uint32_t g_ppu_palette[256];
extern volatile bool g_ppu_pal_valid;

/* Bytes of framebuffer the last exchange returned, 0 if none. */
uint32_t link_master_ppu_got(void);
#endif

/* Per-second counters for the profile build. */
void link_master_get_stats(uint32_t *exchanges, uint32_t *failures,
                           uint32_t *last_us);

#endif /* C2_SOUND_LINK */

#endif /* LINK_MASTER_H */
