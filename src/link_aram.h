/*
 * frank-snes — C2 inter-processor sound link
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * link_aram.h — which pages of APU RAM the slave still needs.
 *
 * Split out of sound_backend_link.c so it can be compiled and tested on
 * a host: it depends on nothing but link_proto.h, and getting it wrong
 * is silent — the slave mixes from stale sample data and the only
 * symptom is that a sound is subtly not the one the game asked for.
 */
#ifndef LINK_ARAM_H
#define LINK_ARAM_H

#include <stdint.h>

#include "link_proto.h"

/* The SPC700 wrote this address. Only the page number is kept. */
void link_aram_mark(uint32_t address);

/* Send the whole 64 KB on the next collect. Used after a reset, a
 * savestate load, or a link failure, where a dirty-page diff would be
 * the entire image anyway. */
void link_aram_force_resync(void);

/*
 * Fill `out` (LINK_ARAM_MAX_RUNS entries) with the runs to send this
 * frame and clear the dirty set. Returns the run count.
 *
 * `esa` and `edl` are the DSP's echo registers, which say where the
 * slave's own DSP is writing its delay line — see the comment on the
 * implementation for why the collector has to know.
 */
uint32_t link_aram_collect(link_aram_run_t *out, uint8_t esa, uint8_t edl);

/* True if `page` is inside the echo delay line described by esa/edl.
 * Exposed for the host test. */
int link_aram_page_is_echo(uint32_t page, uint8_t esa, uint8_t edl);

#endif /* LINK_ARAM_H */
