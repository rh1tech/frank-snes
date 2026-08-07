/*
 * frank-snes — C2 inter-processor sound link
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * link_proto.h — wire protocol between the C2 master (65816, PPU, video,
 * SPC700, I2S) and the C2 slave (S-DSP: BRR decode, 8-voice mixer, echo).
 *
 * Control frames use the doorbell handshake from link_bus.h:
 *
 *   master                              slave
 *   ------                              -----
 *   DB_MS = 1  ------------------------>  sees DB_MS, arms RX
 *   waits for DB_SM  <-----------------   DB_SM = 1  ("armed")
 *   VALID_A=1, stream, VALID_A=0
 *   DB_MS = 0  ------------------------>  RX DMA completes
 *                    <-----------------   DB_SM = 0
 *
 * Nothing depends on the two chips agreeing about absolute time, so the
 * slave can boot seconds after the master and still join cleanly.
 *
 * =====================================================================
 * Why the split is here and not at the SPC700
 * =====================================================================
 *
 * frank-genesis moves its whole sound CPU (Z80) to the slave. The
 * obvious analogue would be the SPC700, and it does not work: the 65816
 * talks to the SPC700 through the four ports at $2140..$2143, and games
 * spin on those in tight loops — during the entire boot-time upload and
 * on every music command. The Genesis got away with batching a frame at
 * a time because the 68K reads Z80 RAM about 0.3 times per frame; the
 * SNES would need a doorbell round trip inside a spin loop, thousands of
 * times per frame.
 *
 * One layer down the coupling collapses. Snes9x funnels every DSP
 * register write through S9xSetAPUDSP() and every read back through
 * S9xGetAPUDSP(), and soundux.c — BRR decode, the 8 voices, ADSR, echo,
 * the FIR — touches exactly four things outside itself:
 *
 *   IAPU.RAM[]          64 KB APU RAM, read-only (BRR sample data)
 *   APU.DSP[]           the 128 DSP registers
 *   APU.KeyedChannels   which voices are keyed on
 *   Settings/so         a handful of static flags
 *
 * So the master keeps the SPC700 and the $2140..$2143 handshake exactly
 * as it is today — bit-identical to M1/M2 — and the slave takes the
 * sample-generating half. See docs/C2_SOUND_SPLIT.md.
 */
#ifndef LINK_PROTO_H
#define LINK_PROTO_H

#include <stdint.h>

#define LINK_MAGIC        0x53534E53u   /* "SNSS" — snes sound */
#define LINK_PROTO_VER    1u

/* Fixed control-frame size. Must be a multiple of 4 (PIO autopush is
 * 32-bit) and large enough for the biggest payload struct below.
 *
 * 256, not 128: the frame reply carries the DSP's whole 128-byte
 * register file so the master can answer the SPC700's reads without a
 * round trip. The extra 128 bytes on the wire is about 5 us a frame. */
#define LINK_CTRL_BYTES   256u

/* ---- Opcodes ---- */
enum {
    LINK_OP_HELLO        = 0x0001,  /* M->S: are you there?               */
    LINK_OP_HELLO_ACK    = 0x0002,  /* S->M: payload link_node_info_t     */

    LINK_OP_RESET        = 0x0010,  /* M->S: reset the DSP and mixer      */
    LINK_OP_RESET_ACK    = 0x0011,

    LINK_OP_CONFIG       = 0x0012,  /* M->S: payload link_sound_config_t  */
    LINK_OP_CONFIG_ACK   = 0x0013,

    /* Full 64 KB APU RAM push. Used once after a reset or a savestate
     * load, where a dirty-page diff would be the whole image anyway.
     * arg0 = byte offset, arg1 = chunk length; the chunk follows as a
     * bulk transfer. */
    LINK_OP_ARAM_BEGIN     = 0x0020,
    LINK_OP_ARAM_BEGIN_ACK = 0x0021,
    LINK_OP_ARAM_CHUNK     = 0x0022,
    LINK_OP_ARAM_CHUNK_ACK = 0x0023,
    LINK_OP_ARAM_END       = 0x0024,  /* arg0 = CRC-32 of the whole image */
    LINK_OP_ARAM_END_ACK   = 0x0025,  /* arg0 = the CRC the slave saw     */

    /* Steady state, one per emulated frame.
     * arg0 = DSP event count.
     * arg1 = dirty APU RAM run count, plus the number of audio chunks
     *        the master wants back in the high half — see
     *        LINK_FRAME_RUNS() / LINK_FRAME_CHUNKS() below.
     *
     * The run table rides in this frame's payload rather than in a bulk
     * of its own: it is at most LINK_ARAM_MAX_RUNS * 4 = 32 bytes
     * against 104 bytes of payload, and a bulk phase costs a doorbell
     * round trip whatever its length. Measured on the first assembled
     * board, the phases — not the bytes — are what the exchange costs. */
    LINK_OP_FRAME        = 0x0030,  /* M->S: header + run table, then the
                                     *       event bulk, then one bulk
                                     *       per run                      */
    LINK_OP_FRAME_ACK    = 0x0031,  /* S->M: payload link_frame_reply_t,
                                     *       then the sample bulk         */

    LINK_OP_PING         = 0x0040,  /* M->S: liveness / latency probe     */
    LINK_OP_PONG         = 0x0041,
};

/* ---- Frame header (24 bytes), followed by payload, zero-padded to
 *      LINK_CTRL_BYTES. ---- */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t op;
    uint16_t proto_ver;
    uint32_t seq;
    uint32_t arg0;
    uint32_t arg1;
    uint32_t crc;      /* CRC-32 of the whole frame with crc treated as 0 */
} link_hdr_t;

#define LINK_PAYLOAD_BYTES (LINK_CTRL_BYTES - sizeof(link_hdr_t))

/* =====================================================================
 * The event stream
 *
 * Every DSP register write the SPC700 performs becomes one of these,
 * tagged with the master's CPU.Cycles at which it happened. The slave
 * replays them in order and then mixes the frame.
 *
 * That is not an approximation of what the single-chip build does — it
 * is exactly what it does. S9xMixSamples*() is called once per frame
 * from the emulation loop, after all of that frame's DSP writes have
 * landed, and soundux.c's sub-frame KON/KOFF queue (S9xDSPQueueEvent)
 * is dead code that nothing calls. So a frame's audio generated on the
 * slave is bit-identical to the same frame generated on the master.
 *
 * The timestamps are therefore carried for diagnostics, and so that
 * sub-frame replay remains possible later without a protocol change —
 * not because ordering within the frame currently matters.
 *
 * 8 bytes each, same shape as the Genesis event so the two firmwares
 * stay recognisably the same code.
 * ===================================================================== */
enum {
    LINK_EV_DSP_WRITE    = 0x01,  /* addr = DSP register, val = data      */
    LINK_EV_RUN_UNTIL    = 0x02,  /* time marker: end of the frame        */
};

/* One frame in the master's event timebase is V_Counter * H_Max +
 * CPU.Cycles, and both halves must agree on how long that is.
 *
 * There is deliberately no macro for it here. There used to be, fixed at
 * 262 scanlines, and on a PAL ROM — 312 lines — that clamped every write
 * from scanline 262 to 311 onto one timestamp at the end of the frame.
 * That band is most of PAL V-blank, which is where a sound driver does
 * its work, so key-ons arrived bunched at the frame edge instead of
 * where they belonged.
 *
 * The span depends on the ROM's region and can only be known at runtime,
 * so the master computes it in link_frame_span() and sends it in
 * LINK_OP_CONFIG. The slave uses what it is told and assumes nothing. */

typedef struct __attribute__((packed)) {
    uint32_t cycles;   /* position in the frame; span comes from CONFIG */
    uint8_t  type;     /* LINK_EV_*                            */
    uint8_t  val;
    uint16_t addr;
} link_event_t;

/* ---- Payload: who am I ---- */
typedef struct __attribute__((packed)) {
    uint8_t  chip_id[8];
    uint8_t  package_is_a;     /* 1 = QFN-60 (RP2350A) — the slave  */
    uint8_t  rp2350_rev;
    uint16_t fw_version;       /* (major << 8) | minor              */
    uint32_t sys_clk_hz;
    uint32_t psram_bytes;      /* 0 if the probe failed             */
    uint32_t proto_ver;
} link_node_info_t;

/* ---- Payload: emulation settings the slave needs ----
 *
 * Mirrors the master's settings so muting and the interpolation and echo
 * switches behave identically on both halves. Sent on ROM load and
 * whenever settings change. */
typedef struct __attribute__((packed)) {
    uint8_t  sound_enabled;
    uint8_t  interpolated;       /* Settings.InterpolatedSound          */
    uint8_t  echo_disabled;      /* Settings.DisableSoundEcho           */
    uint8_t  master_volume_off;  /* Settings.DisableMasterVolume        */
    uint8_t  mono;               /* 1 = master wants S9xMixSamplesMono  */
    uint8_t  reserved[3];
    uint32_t playback_rate;      /* 32040                               */
    uint32_t samples_per_frame;  /* 534 mono for 32040 Hz at 60 Hz      */
    /* The master's frame timebase: V_Counter * H_Max + CPU.Cycles at
     * end of frame. Event timestamps are in these units, and the slave
     * needs the span to map them onto DSP clocks. It is sent rather than
     * assumed because H_Max depends on the ROM's region and timing. */
    uint32_t frame_span;
} link_sound_config_t;

/* ---- Payload: what came back with LINK_OP_FRAME_ACK ----
 *
 * The slave returns the mixer's output exactly as soundux.c produced it
 * — int16, and mono or stereo according to the config — and nothing
 * else. It does not apply gain, the soft limiter or the volume setting,
 * and it does not pack to the I2S frame format.
 *
 * That is deliberate, and for the same reason frank-genesis returns raw
 * chip buffers: all of that is master-side state. audio_pack_opt()
 * scales by g_settings.volume, and main.c's wall-clock catch-up decides
 * how many chunks a frame actually owes based on how long the master's
 * last frame took. The slave has no idea. Returning the mixer's raw
 * output leaves src/main.c's audio path and drivers/audio.c untouched
 * and costs the same bytes on the wire.
 *
 * The rest of the reply is the small amount of state the mixer owns that
 * the SPC700 can observe through S9xGetAPUDSP(): ENVX and OUTX per
 * voice, and the effect of a sample running out.
 *
 * Those last bits travel as *deltas*, not as the raw register bytes, and
 * that distinction matters. The slave computed them from frame N while
 * the master has since been running frame N+1 and writing KON, KOFF and
 * ENDX of its own; shipping the slave's bytes wholesale would stamp on
 * the master's newer writes once per frame. What the mixer actually did
 * is monotonic and tiny — "these voices reached the end of their sample"
 * and "these reached a loop point" — so the master replays exactly the
 * S9xAPUSetEndOfSample() / S9xAPUSetEndX() bookkeeping against whatever
 * its registers hold now. Order stops mattering and the frame of lag
 * costs nothing. It is the same problem frank-genesis solves with a
 * dirty bitmap over Z80 RAM, made trivial by being four bits wide. */
typedef struct __attribute__((packed)) {
    uint32_t seq;              /* which frame these samples belong to   */
    uint32_t samples;          /* int16 samples that follow in the bulk */

    /* The DSP's entire register file as of the end of the frame the
     * slave just rendered.
     *
     * This replaces the delta scheme the legacy mixer needed. The
     * accurate DSP maintains ENVX, OUTX and ENDX inside the register
     * file itself — exactly as the hardware does — so there is nothing
     * to reconstruct: the master copies these 128 bytes into its shadow
     * and answers every S9xGetAPUDSP() from it. Registers the master
     * wrote during the frame it is *currently* running are re-applied on
     * top by the event list, so a stale byte cannot outlive one
     * exchange. */
    uint8_t  dsp_regs[128];

    uint32_t events_replayed;  /* how many of the frame's events landed */
    uint32_t overflows;        /* frames whose event list was truncated */
    uint32_t mix_us;           /* slave-side cost of the mix, for perf  */
} link_frame_reply_t;

/* APU RAM is 64 KB. It travels as 256-byte pages so the steady state is
 * nearly free: a game uploads its sample bank once and then writes
 * almost nothing, whereas a full image every frame would be 64 KB
 * against a budget of 16 ms.
 *
 * The dirty set is described as *runs* of consecutive pages rather than
 * as a bitmap, and each run is then sent straight out of IAPU.RAM. That
 * choice is about master SRAM, not about the wire: staging the marked
 * pages into one contiguous buffer would need a second 64 KB array, and
 * the master has nothing like that spare once the emulator, APU RAM and
 * the frame buffers are placed. Runs let the DMA read the live array.
 *
 * The cost of a run is one doorbell handshake, so the master coalesces
 * across small gaps until the count fits LINK_ARAM_MAX_RUNS. In the
 * steady state there is exactly one run — pages 0 and 1, the zero page
 * and the stack, which are re-sent unconditionally. */
#define LINK_ARAM_BYTES      65536u
#define LINK_ARAM_PAGE_BITS  8u
#define LINK_ARAM_PAGE_BYTES (1u << LINK_ARAM_PAGE_BITS)      /* 256 */
#define LINK_ARAM_PAGES      (LINK_ARAM_BYTES / LINK_ARAM_PAGE_BYTES) /* 256 */
#define LINK_ARAM_BITMAP_BYTES (LINK_ARAM_PAGES / 8u)         /* 32  */

/* Eight runs is 8 handshakes, about 300 us at the measured control-frame
 * cost. Beyond that coalescing wins even when it re-sends clean pages. */
#define LINK_ARAM_MAX_RUNS   8u

/* One run of consecutive dirty pages. Sent as an array of arg1 entries
 * in a single small bulk, ahead of the run payloads themselves. */
typedef struct __attribute__((packed)) {
    uint16_t first_page;
    uint16_t page_count;
} link_aram_run_t;

/* 32040 Hz at 60 Hz is 534 mono samples per frame; stereo doubles it.
 *
 * A frame does not owe exactly one chunk. main.c paces audio against the
 * wall clock, and whenever the emulator runs below 60 fps it calls
 * S9xMixSamples*() again — up to AUDIO_CATCHUP_MAX + 1 = 7 times — to
 * keep the I2S ring from starving. On a single-chip build those extra
 * calls run the mixer again and produce real audio.
 *
 * So the master has to be able to ask for more than one chunk, and the
 * slave has to mix that much, or every catch-up chunk is silence. On
 * Doom at 26.7 fps that is most of the audio: it sounds gappy and lagged
 * because the I2S ring fills with zeros while the real samples queue up
 * behind them. The master sizes each request from what main.c actually
 * consumed on the previous frame.
 *
 * The ceiling is deliberately well below main.c's 7. Each chunk is
 * ~400 us of mixing on the slave, and the slave cannot answer the next
 * frame's doorbell until it has finished; at 8 chunks that is over 3 ms
 * of latency on top of everything else, and the link drops on the
 * handshake timeout. Three bounds it to ~1.2 ms, and the FIFO in
 * sound_backend_link.c spreads a bigger backlog over several frames
 * instead of demanding it all at once. */
#define LINK_SAMPLES_PER_CHUNK  534u
#define LINK_MAX_CHUNKS         3u

/* The master's ring has to hold its queued backlog *and* the reply that
 * is about to land on top of it, or the receive has nowhere to go and
 * the link drops. Sizing it for twice LINK_MAX_CHUNKS gives room for a
 * full backlog plus a full request; sound_backend_link.c trims the
 * backlog to LINK_MAX_CHUNKS to keep the other half free. The 2u is the
 * stereo case, where a chunk is 1068 samples rather than 534. */
#define LINK_MAX_SAMPLES        (LINK_SAMPLES_PER_CHUNK * 2u * \
                                 (LINK_MAX_CHUNKS * 2u))
#define LINK_AUDIO_BYTES        (LINK_MAX_SAMPLES * sizeof(int16_t))

/* LINK_OP_FRAME arg1 packing. */
#define LINK_FRAME_ARG1(runs, chunks) (((uint32_t)(runs) & 0xFFFFu) | \
                                       ((uint32_t)(chunks) << 16))
#define LINK_FRAME_RUNS(a1)    ((a1) & 0xFFFFu)
#define LINK_FRAME_CHUNKS(a1)  ((a1) >> 16)

/* Upper bound on DSP writes in one frame. A frame that would exceed this
 * is truncated rather than dropped, and the overflow is counted — losing
 * the tail of a frame's writes degrades sound, whereas losing frame
 * alignment desynchronises the whole stream.
 *
 * 2048 is generous: a busy driver writes on the order of 200 DSP
 * registers per frame, and the ceiling only matters for the pathological
 * case of a game clearing all 128 registers in a loop. */
#define LINK_MAX_EVENTS     2048u
#define LINK_EVENT_BYTES    (LINK_MAX_EVENTS * sizeof(link_event_t))

/* APU RAM upload chunk, for the one-shot full push. 16 KiB, matching
 * frank-genesis: both halves stage chunks through SRAM rather than
 * letting DMA touch the PSRAM XIP window. Only affects how many
 * handshakes a rare full sync costs. */
#define LINK_ARAM_CHUNK_BYTES (16u * 1024u)

/* The PIO FIFOs are 32 bits wide and autopull/autopush are word-sized,
 * so every bulk length must be a whole number of words. Sample counts
 * are not naturally aligned, so both sides round the same way from the
 * same count in the reply and stay in step. Buffers therefore need two
 * bytes of slack past the largest count they will ever carry. */
#define LINK_ALIGN4(n) (((n) + 3u) & ~3u)

/* ---- Helpers shared by both firmwares ---- */
uint32_t link_crc32(const void *data, uint32_t len);

/* Build a control frame into `frame` (must be LINK_CTRL_BYTES, 4-byte
 * aligned). Copies `payload_len` bytes of payload and fills in the CRC. */
void link_frame_build(void *frame, uint16_t op, uint32_t seq,
                      uint32_t arg0, uint32_t arg1,
                      const void *payload, uint32_t payload_len);

/* Validate magic, version and CRC. Returns 1 on success. */
int link_frame_check(const void *frame);

#endif /* LINK_PROTO_H */
