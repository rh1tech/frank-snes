# C2 sound split — moving the S-DSP and the mixer to the slave

Design reference for the dual-RP2350 FRANK Core 2 (`BOARD_VARIANT=C2`)
build. The master keeps the 65816, the PPU, video, SD, input, the SPC700
and the I2S DAC; the slave takes sample generation.

**M1 and M2 must be unaffected.** Everything here is behind a
compile-time seam; on those boards the sound path compiles to exactly
what it does today. Verified by build: M1's `.bss` is byte-for-byte
unchanged against the pre-C2 tree.

## Why the split is not where frank-genesis put it

frank-genesis moves its whole sound CPU — the Z80 — to the slave. The
obvious analogue here is the SPC700, and it does not work.

The 68K reaches the Genesis sound subsystem through ten call sites, and
only three of them need data back. One is free (it is pure master-side
state), one is answered by a master-side timer shadow, and the last —
68K reads of Z80 RAM — happens about **0.3 times per frame**, rare enough
to serve with an occasional mid-frame `LINK_OP_SYNC` round trip.

The SNES has nothing like that ratio. The 65816 talks to the SPC700
through four ports at `$2140..$2143` (`src/snes9x/ppu.c:627` and `:872`,
into `S9xAPUReadPort`/`S9xAPUWritePort`), and games **spin** on them: the
entire boot-time driver upload is a handshake loop, and so is every music
command. A doorbell round trip is tens of microseconds. Putting one
inside those spin loops is not a tuning problem, it is a different
machine.

## Where it is instead

One layer down, the coupling collapses. Snes9x funnels every DSP register
write through `S9xSetAPUDSP()` and every read back through
`S9xGetAPUDSP()`, and `soundux.c` — BRR decode, eight voices, ADSR, echo,
the FIR, the mixer — reaches outside itself for exactly four things:

| What | Direction |
|---|---|
| `IAPU.RAM[]` — 64 KB APU RAM | read only (BRR sample data) |
| `APU.DSP[]` — the 128 DSP registers | read and write |
| `APU.KeyedChannels` | write |
| `Settings.*` / `so` | read (a handful of flags) |

So the seam is `S9xSetAPUDSP` / `S9xGetAPUDSP`, and the `$2140..$2143`
handshake stays on the master, bit-identical to M1/M2.

## Ownership

**Master:** 65816, PPU, video, SD, USB HID input, **the SPC700 and all
64 KB of APU RAM**, the DSP register shadow, and the I2S DAC — it is
wired to master GPIO 9/10/11, so the samples come back over the link.

**Slave:** `soundux.c` and the DSP register decode half of `apu.c`,
compiled from the same shared tree the master builds from. Its APU RAM is
a mirror the master keeps up to date.

The slave's own PSRAM (U5) goes unused. Its only allocation is soundux's
96 KB echo buffer, which is touched once per output sample; PSRAM read
latency in that inner loop would be the wrong trade against 520 KB of
SRAM and 160 KB of demand. `slave/src/snes_alloc.h` shadows the master's
copy on the include path to redirect it.

## The frame is already the right unit

This is the part that makes the split cheap, and it is worth being
precise about.

`S9xMixSamples*()` is called **once per emulated frame**, from the
emulation loop in `src/main.c`, after all of that frame's DSP writes have
landed. soundux's sub-frame KON/KOFF queue (`S9xDSPQueueEvent`,
`dsp_events`) is dead code — nothing calls it, and `MixStereo()` mixes
the whole frame in a single `MixStereoSegment(0, count)`.

So "ship the frame's DSP writes, replay them in order, then mix" is not
an approximation of the single-chip build. It **is** the single-chip
build. A frame's audio generated on the slave is bit-identical to the
same frame generated on the master.

The event timestamps (`CPU.Cycles`) are carried anyway, for diagnostics
and so that sub-frame replay stays possible later without a protocol
change.

## Getting APU RAM across

The mixer reads BRR sample data straight out of APU RAM, so the slave
needs a live mirror of all 64 KB. Sending it every frame would be 64 KB
against a 16 ms budget, so it travels as 256-byte dirty pages.

**Only one write path can move sample data.** Every SPC700 store to APU
RAM outside the zero page, the stack and the register file goes through
the `else` branch of `S9xAPUSetByte()` in `spc700.c` — that is the single
`s9x_apu_ram_dirty()` call site. Everything else in the emulator that
writes `IAPU.RAM` (apu.c's timers and ports, cpuexec.c's timer ticks, the
`Push`/`PushW` macros, the `DirectPage` store) is confined to pages
`0x00` and `0x01`, and those two are **re-sent unconditionally every
frame**. 512 bytes a frame — 31 KB/s against 48 MB/s — buys immunity from
ever having missed a write site, which is the kind of bug that shows up
as one game with quietly wrong samples. Page `0xFF` is the IPL ROM window
and is marked by hand in `S9xSetAPUControl()`.

Dirty pages travel as **runs of consecutive pages**, not as a bitmap.
That is a master-SRAM decision, not a wire decision: staging the marked
pages into one contiguous buffer would need a second 64 KB array, and the
master has nothing like that spare. Runs let the DMA read the live array
in place. A run costs one doorbell handshake, so the master coalesces
across gaps until the count fits `LINK_ARAM_MAX_RUNS`. In the steady
state there is exactly **one** run: pages 0 and 1.

A full 64 KB push is used after a reset, after a savestate load, and
after the link recovers from a failure — cases where a diff would be the
whole image anyway.

### The echo buffer must not be coalesced over

Coalescing re-sends *clean* pages, and there is one region where a clean
page is not free: the echo delay line. It lives inside APU RAM, and on
C2 it is the **slave** that writes it — its DSP stores each sample at
`ESA` and reads it back tens of milliseconds later for feedback. The
master's mirror never sees those stores, so over the echo region the two
copies are legitimately different and the master's is the stale one.

A run that swallows a gap crossing the echo buffer therefore drops stale
bytes on top of a live delay line, and a delay line replaying old content
is heard as **a sample repeating** — intermittently, only while echo is
enabled. That is the symptom that outlived every fix aimed at the mixer
itself, and it is not a mixer bug at all.

So the merge picks the *cheapest* gap rather than the smallest: a clean
page inside the echo region is priced at 64 ordinary pages, which means
every other gap is merged first and the delay line is crossed only when
nothing else still fits the run cap. Pages the SPC700 genuinely wrote are
always sent, echo region or not — a game may place its buffer over memory
it also uses, and its write is newer than anything the DSP put there.

The tracker is `src/link_aram.c`, split out of `sound_backend_link.c`
precisely because this failure is silent: no counter moves and no link
error is raised, the slave just mixes from bytes that are not what the
SPC700 wrote. `tests/link_aram_test.c` compiles against it on a host and
covers the discriminating case — the echo buffer sitting in the smallest
gap, which the old rule merged straight through, sending 24 pages of
stale bytes over the live buffer.

```sh
cc -O1 -Wall -Isrc -Ilink -o /tmp/link_aram_test \
    tests/link_aram_test.c src/link_aram.c && /tmp/link_aram_test
```

## The two reads

`S9xGetAPUDSP()` answers everything out of the master's own 128-byte
register shadow, and the reply refreshes that shadow wholesale from the
slave's register file once a frame.

That is simpler than it was under the legacy mixer, and the accurate DSP
is why. **ENVX, OUTX and ENDX live inside the register file itself**, as
they do on hardware, so there is nothing to reconstruct — the master
copies 128 bytes and answers every read from them. Registers the master
writes during the frame it is *currently* running are re-applied on top
by the event list, so a stale byte cannot outlive one exchange.

Bytes the DSP authored are therefore one frame stale, the same bargain
frank-genesis makes with the YM2612 status byte. Bytes the master
authored never are.

(An earlier revision shipped ENDX and the key-on bookkeeping as
*deltas* — two bitmasks replayed against whatever the master's registers
held — because the legacy mixer kept that state outside the register
file and a wholesale copy would have stamped on the master's newer
writes. The accurate DSP removed the need.)

## Sync model

One exchange per emulated frame, on core 0, immediately before the audio
is consumed — but **pipelined**, so the master never waits for the mixer:

```
frame N   core0: 65816 + PPU + SPC700 run
                 DSP writes    -> event list
                 APU RAM writes-> dirty pages
          core0: ship frame N's events + dirty pages
                 collect frame N-1's samples        <- already mixed
          core1: HDMI scanline pump (untouched)

slave     reply immediately with frame N-1
          then mix frame N while the master runs frame N+1
```

frank-genesis puts its exchange on core 1 and runs a frame behind. Core 1
here is the HDMI scanline pump and cannot be borrowed, so the frame of
lag lives on the slave instead — same pipeline, other end of the wire.
The cost is one extra frame of audio latency, 16.7 ms.

This is the difference between the split being worth having and not.
Mixing before replying makes the master block for the mix as well as the
wire: 683 µs against 219 µs, which is more than the mixing it saves.

The dirty-page scheme matters more here than the ROM upload did in
frank-genesis: in the steady state a frame is ~100–200 DSP events plus
one 512-byte run, and the doorbell handshakes dominate, not the bytes.
That is also why the run table rides in the `LINK_OP_FRAME` payload
rather than in a bulk of its own.

A consequence worth stating plainly: **the master blocks on the link once
per frame.** If the slave stops answering, `link_master_online()` goes
false, every path short-circuits, and the machine runs silent rather than
stalling — until recovery brings it back.

### The frame is as long as the region says it is

Event timestamps are `V_Counter * H_Max + CPU.Cycles`, and the slave maps
them onto DSP clocks by dividing through the frame span the master sent
at `LINK_OP_CONFIG`. The span was hardcoded at 262 scanlines. That is
NTSC.

On a PAL ROM the frame is 312 lines (`cpuexec.c` wraps `V_Counter` at
`SNES_MAX_PAL_VCOUNTER`), so `link_dsp_now()`'s clamp collapsed every
write from scanline 262 to 311 onto a single timestamp at the end of the
frame. That band is most of PAL V-blank — exactly where a sound driver
does its work — so key-ons arrived bunched at the frame edge instead of
where they belonged, and 16% of every frame's timeline was flattened.

The span is now derived from `Settings.PAL` at the point it is sent, in
`link_frame_span()`. Read off a running board with Mortal Kombat 3
(Europe) loaded: `Settings.PAL = 1`, `H_Max = 1504`, `FrameTime = 20000`
— a PAL ROM, so this was live, not theoretical.

## Failure is silent, not fatal

A slave that never answers `LINK_OP_HELLO` is not an error. The master
logs it and runs with no sound. That is deliberate: refusing to boot
because one of two chips is unprogrammed would be a far worse outcome
than a quiet SNES, and it is exactly the state a board is in the first
time it is flashed.

Any mid-exchange failure takes the link offline immediately, because a
half-completed exchange leaves the doorbells in an unknown phase and
guessing where the peer got to is worse than stopping. Two things are
treated as fatal rather than clamped, for the same reason: a protocol
version mismatch, and a reply claiming more samples than were requested
(the slave is about to put those bytes on the wire whatever the master
does, and receiving fewer would desynchronise every exchange after it).

Offline is **not permanent**, and an earlier revision of this document
was wrong to say it was. The slave's loop times out after a second and
goes back to waiting for a control frame, so the wire is usable again
almost immediately — and yet the machine used to play silent until the
next ROM load. One glitched handshake cost the rest of the session's
sound. `link_master_reprobe()` now retries once a second: drop the
doorbell, abort any armed receive, wait for the slave to go idle, then
re-HELLO. It deliberately does **not** repeat `link_init()`, which claims
PIO state machines and DMA channels and panics if claimed twice.

Re-establishing the wire is only half of recovery. Every register write
and every dirty page sent while the link was down is gone, so replaying
from there would leave the DSP holding whichever patch it had when the
link dropped — voices keyed on samples that have since been replaced,
volumes from another scene. So recovery pushes the whole of the state the
mixer reads: 64 KB of APU RAM, then all 128 DSP registers out of the
shadow, stamped at cycle 0 so they land at the head of the next frame
rather than wherever the 65816 happens to be.

**KON is the one register that is not replayed.** It is a latch the DSP
consumes, not a level the game holds, so re-issuing the last value keys
those voices a second time — a duplicated sample, the exact fault the
whole path exists to avoid. Which voices were sounding is not recoverable
anyway; zero is the honest answer, and the driver keys its next note
normally. `link_recoveries` counts successful recoveries and is readable
over SWD.

The two halves must be built at the same `CPU_SPEED` — the receiving PIO
program has to finish its loop inside the transmitter's byte period, and
each side derives that from its own system clock. The master checks this
over the wire at HELLO and refuses to proceed on a mismatch, rather than
running a link that works in one direction.

## What is not on this board

GPIO20..43 are the link, so C2 has **no NES pad header and no PS/2
header**. USB HID is the only input path, and `CMakeLists.txt` forces
`USB_HID_ENABLED` on. The PS/2 keyboard driver would also want PIO2,
which the link needs — bus B reaches GPIO39, and only a PIO instance
given the upper GPIO window (`pio_set_gpio_base(16)`) can see it.

PIO allocation on the C2 master: **PIO0** HDMI, **PIO1** I2S audio,
**PIO2** the link.

Rather than thread `#ifdef BOARD_C2` through two dozen input call sites
in `main.c` and `rom_selector.c`, the drivers are left out of the build
and `src/c2_input_stubs.c` answers their API with "nothing connected" —
which is a state the existing code already handles.

LD1 is a WS2812B rather than a plain LED, so the boot indicator is
skipped on C2 rather than spending a fourth PIO state machine on it.

## Files

| Path | Role |
|---|---|
| `boards/frank_core2_master.h` | RP2350B, 48 GPIOs, PIO GPIO base |
| `boards/frank_core2_slave.h` | RP2350A, UART1 on J4, LED on GPIO26 |
| `link/` | transport + wire protocol, built into both halves |
| `src/link_master.c` | bring-up, the per-frame exchange, recovery |
| `src/link_aram.c` | which APU RAM pages the slave still needs |
| `src/sound_backend_link.c` | the master's half of the seam |
| `tests/link_aram_test.c` | host test for the page tracker |
| `src/c2_input_stubs.c` | no NES pad, no PS/2 |
| `slave/src/main.c` | reactive opcode loop |
| `slave/src/slave_sound.c` | replay events, drive soundux |
| `slave/src/slave_glue.c` | the snes9x globals the sound code needs |
| `slave/src/snes_alloc.h` | shadows the master's, keeps echo in SRAM |

`link/link_bus.{c,h,pio}`, `link/link_session.{c,h}` and
`link/link_proto.c` are frank-genesis's, unchanged — the board is the
same silicon. Only `link/link_proto.h` is SNES-specific.

## Building and flashing

```sh
./build.sh C2            # builds both halves
./flash.sh --both        # master, then slave (prompts for BOOTSEL)
```

`C2_LOCAL_SOUND=1 ./build.sh C2` keeps sound on the master, for A/B
comparison against the split on identical hardware.

The slave has no screen; it reports once a second over UART1 on the J4
header at 115200.

## Measured on hardware

First assembled C2, both halves at 504 MHz, flashed over SWD with two
CMSIS-DAP probes (master = the RP2350B, `PACKAGE_SEL` 0; slave = the
RP2350A, `PACKAGE_SEL` 1). Video captured off HDMI at 640x480.

| | Super Mario World | Doom |
|---|---|---|
| Emulated frame rate | 60.0 fps | 27.3 fps |
| Master stalled in the exchange | 147 µs | 236 µs |
| Slave mixing, overlapped | 319 µs | 861 µs |
| Audio underruns | 0 /s | 5.2 /s |
| Link failures | 0 | 0 |

Doom with `C2_LOCAL_SOUND=1` on the same board and ROM: **26.9 fps**
against **27.3** offloaded. The mixing the master no longer does is worth
more than the wire costs.

### What it took to get there

Three things had to be right, and only the first was in the original
design.

**The exchange has to be pipelined.** The slave replies with the audio it
mixed during the *previous* exchange and mixes the current frame
afterwards. Mixing first and then replying makes the master sit in
`link_m_recv_ctrl()` for the whole mix — 683 µs of master stall against
219 µs — so the two processors take turns instead of working at once,
and the offload is strictly worse than mixing on the master. It also
breaks outright under load: a multi-chunk mix outlasts the doorbell
timeout and the link drops.

**A frame does not owe exactly one chunk.** `main.c` paces audio against
the wall clock and calls `S9xMixSamples*()` again — up to seven times —
whenever the emulator runs below 60 fps. On a single-chip build those
extra calls produce real audio. Serving them with silence, which is what
the first version did, means most of the audio is silence on anything
slower than 60 fps: Doom was 60 underruns a second. The master now sizes
each request from what was actually consumed last frame and the slave
mixes that much.

**Leftovers have to queue, not be dropped.** The slave advances its mixer
by exactly what it was asked for, so every sample it returns is a slice
of the song that has already happened. Discarding a leftover is not
discarding spare audio, it is skipping forward in the music. Samples now
sit in a FIFO in `sound_backend_link.c` and are consumed in order, with
about a chunk of backlog to absorb estimate error.

**A multi-chunk request must not become a multi-chunk mixer call.**
`soundux.c`'s `MixBuffer` and `EchoBuffer` are fixed `int32_t[2133]`, and
`S9xMixSamplesMono()` doubles its count internally to index them as
stereo pairs — so a mono call is bounded at 1066 samples, and the
534-sample chunk only just fits. Passing two chunks in one call overruns
both arrays by 4 KB; three overruns by more. It sounds like hard
clipping, and it is what Mortal Kombat 3 exposed. The slave now loops,
mixing one chunk per call, which is also exactly what `main.c` does on a
single-chip build. Verified on hardware at a forced three chunks: no
clipped samples, and the waveform runs continuously across both 534-sample
seams.

`LINK_MAX_CHUNKS` is capped at 3 for a related reason: the slave cannot
answer the next frame's doorbell until it has finished mixing, so an
8-chunk request is over 3 ms of added latency and the link drops on the
handshake timeout. The FIFO spreads a larger backlog over several frames
instead.

The backlog cap and the chunk ceiling are coupled, and getting it wrong
drops the link rather than degrading gracefully: the master's ring has to
hold its queued backlog *and* the reply landing on top of it, so
`LINK_MAX_SAMPLES` is sized for twice `LINK_MAX_CHUNKS` and
`FIFO_MAX_CHUNKS` is trimmed to keep the other half free. An earlier
revision allowed a 4-chunk backlog against a 3-chunk ring and went
offline whenever both were near maximum.

### Corrections to earlier revisions of this document

Two claims recorded here after the first bring-up session were wrong, and
are worth stating plainly because both cost time.

**There is no head-of-buffer corruption.** Earlier notes described the
first ~24 samples of every frame as garbage — a full-scale spike among
samples that otherwise peak around 300 — and blamed first the pipelining
and then the receive framing. It was an artifact of reading `sample_ring`
after halting the core mid-DMA. Read live, with the core running, the
buffer is clean end to end: peak 453, rms 238, mean |Δ|/rms 0.12, DC −2,
and no sample above 8000 anywhere. Poking a marker across the head and
letting the board run 60 frames overwrites all of it.

**The pipelining was never the problem.** It was reverted on the strength
of that phantom defect. It is now enabled and is what makes the split
worth having.

The general lesson for anyone debugging this: **read the master's memory
with the core running.** `halt` on this target is unreliable — `resume`
frequently fails with "context restore failed" — and anything read out of
a DMA landing area after a halt may be torn. Deriving symbol addresses
freshly from the current ELF matters too; several confusing measurements
here came from reusing addresses across a rebuild.

## Status

Both halves build clean and run on hardware. The split is functionally
correct and now a net win: roughly 170 µs of master time per frame
recovered at 60 fps and 625 µs at Doom's frame rate, with no audio
underruns at 60 fps and no link failures observed over minutes of play.

Not done: the analog I2S output has never been captured, so the audio has
been verified by inspecting the sample stream rather than by listening
through an instrument. There is no comparison against an M1 or M2 board,
only against `C2_LOCAL_SOUND=1` on the same hardware.
