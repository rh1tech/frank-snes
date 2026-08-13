#
# frank-snes — C2 slave firmware (RP2350A, U6)
#
# Runs the SNES sound DSP and mixer, driven by the master's event stream.
# Shares src/snes9x/soundux.c and src/snes9x/apu.c with the master build
# so the two halves can never generate sound differently.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
cmake_minimum_required(VERSION 3.13)

set(REPO_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)

# The slave is the RP2350A (QFN-60): 30 GPIOs, PSRAM CS on GPIO0, blue
# LED on GPIO26, UART1 on the J4 header.
set(PICO_BOARD_HEADER_DIRS ${REPO_ROOT}/boards CACHE PATH "" FORCE)
set(PICO_BOARD "frank_core2_slave" CACHE STRING "Pico board type" FORCE)

# Both halves must be built at the same CPU_SPEED: the receiving PIO
# program has to finish its loop inside the transmitter's byte period,
# and each side derives that from its own system clock. Mismatched
# clocks give a link that works one way and drops bytes the other. The
# master checks this over the wire at HELLO and refuses to run on a
# mismatch, so a wrong value here fails loudly rather than subtly.
set(CPU_SPEED "252" CACHE STRING "CPU clock in MHz")

if(CPU_SPEED GREATER_EQUAL 504)
    set(CPU_VOLTAGE "VREG_VOLTAGE_1_65")
elseif(CPU_SPEED GREATER_EQUAL 378)
    set(CPU_VOLTAGE "VREG_VOLTAGE_1_60")
else()
    set(CPU_VOLTAGE "VREG_VOLTAGE_1_50")
endif()

message(STATUS "C2 slave: CPU ${CPU_SPEED} MHz, vreg ${CPU_VOLTAGE}")

include(${REPO_ROOT}/pico_sdk_import.cmake)
project(frank-snes-slave C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_subdirectory(${REPO_ROOT}/link link)

# The accurate S-DSP, compiled from the shared tree exactly as the
# master compiles it — so the two halves can never render sound
# differently.
#
# Nothing else from snes9x comes with it. spc_dsp.c depends on no
# emulator globals: it owns its state and is handed a pointer to APU
# RAM. That is what shrank the slave from a dozen stubbed globals and a
# 96 KB echo buffer down to this.
set(SLAVE_SOUND_SOURCES
    ${REPO_ROOT}/src/snes9x/spc_dsp.c
)

# The renderer, for the PPU offload. Scoping build: these are the files the
# slave needs to replay a PPU command stream and draw the frame.
set(SLAVE_PPU_SOURCES
    ${REPO_ROOT}/src/snes9x/gfx.c
    ${REPO_ROOT}/src/snes9x/tile.c
    ${REPO_ROOT}/src/snes9x/clip.c
    ${REPO_ROOT}/src/snes9x/colormath.c
    ${REPO_ROOT}/src/snes9x/ppu.c
    ${REPO_ROOT}/src/snes9x/globals.c
    ${REPO_ROOT}/src/snes9x/tile_asm.S

    # The 448 KB of tile caches cannot fit in the slave's SRAM (356 KB free
    # with the renderer linked), so the slave's PSRAM on U5 - unused until
    # now - is brought up for them. Everything the renderer touches per pixel
    # stays in SRAM.
    ${REPO_ROOT}/drivers/psram_init.c
    ${REPO_ROOT}/drivers/psram_allocator.c
)

option(FRANK_SNES_PPU_SLAVE "PPU offload: render on the slave" OFF)

add_executable(frank-snes-slave
    src/main.c
    src/slave_glue.c
    src/slave_sound.c
    src/slave_ppu.c
    src/slave_ppu_stubs.c
    ${SLAVE_SOUND_SOURCES}
    ${SLAVE_PPU_SOURCES}
)

# slave/src comes first so its snes_alloc.h shadows the master's, which
# would otherwise route soundux.c's echo buffer into PSRAM. See the
# comment at the top of slave/src/snes_alloc.h.
target_include_directories(frank-snes-slave PRIVATE
    src
    ${REPO_ROOT}/src
    ${REPO_ROOT}/src/snes9x
    ${REPO_ROOT}/link
    ${REPO_ROOT}/drivers
    ${REPO_ROOT}/drivers/libdvi
)

# Diagnostic: force the DSP's echo unit off. The echo delay line lives
# in APU RAM, which is mirrored rather than shared, so it is the one
# piece of DSP state the two halves cannot keep identical.
option(C2_DISABLE_ECHO "Force the S-DSP echo unit off on the slave" OFF)

# Diagnostic: reply with the frame just mixed instead of the previous
# one, so the DSP register file (ENDX in particular) is not a frame
# stale. Costs the master ~3 ms of stall per frame.
option(C2_NO_PIPELINE "Mix before replying (fresh DSP registers)" OFF)

target_compile_definitions(frank-snes-slave PRIVATE
    BOARD_C2
    $<$<BOOL:${C2_DISABLE_ECHO}>:C2_DISABLE_ECHO=1>
    $<$<BOOL:${C2_NO_PIPELINE}>:C2_NO_PIPELINE=1>
    CPU_CLOCK_MHZ=${CPU_SPEED}
    CPU_VOLTAGE=${CPU_VOLTAGE}
    PICO_ON_DEVICE=1
    C2_SOUND_SLAVE=1
)

# The DSP is the whole reason this chip exists; give it the same
# optimisation the master applies to the snes9x sources.
set_source_files_properties(${SLAVE_SOUND_SOURCES} PROPERTIES COMPILE_FLAGS
    "-O3 -fno-strict-aliasing -funroll-loops -fomit-frame-pointer")

target_compile_options(frank-snes-slave PRIVATE
    -Wall
    -Wno-unused-parameter
    -Wno-unused-function
)

target_link_libraries(frank-snes-slave
    pico_stdlib
    # The renderer runs on core 1 so the link loop never blocks on it.
    pico_multicore
    hardware_clocks
    hardware_dma
    hardware_gpio
    hardware_pio
    hardware_sync
    hardware_vreg
    link

    hardware_xip_cache)

# Console on BOTH UART1 (J4 header) and USB-CDC.
#
# The USB half is not a convenience. Measured on this board: SWD on J3 cannot
# read IDR even with the target sitting in BOOTSEL running the bootrom, and a
# beacon firmware printing unconditionally to UART1 delivered 0 bytes to the
# host — so both of the "primary" debug routes are dead at the wire, and the
# slave has no screen. USB-CDC is the one channel proven to work: the same
# beacon came through on it immediately.
#
# It also buys remote recovery. `picotool reboot -u -f` over this interface
# puts the slave back in BOOTSEL without anyone touching the board, which is
# what makes flashing it unattended possible at all.
pico_enable_stdio_uart(frank-snes-slave 1)
pico_enable_stdio_usb(frank-snes-slave 1)

pico_add_extra_outputs(frank-snes-slave)

if(FRANK_SNES_PPU_SLAVE)
    # NOT FRANK_SNES_PPU_CAPTURE: that switches on the capture hooks inside the
    # shared gfx.c/ppu.c/dma.c, which is the MASTER's role. The slave links the
    # same files to REPLAY, and must not capture what it renders.
    target_compile_definitions(frank-snes-slave PRIVATE FRANK_SNES_PPU_SLAVE=1)

    # The three flags that change what the RENDERER does. They are set on the
    # master (see the root CMakeLists) and were not set here, so the two halves
    # were compiling the same gfx.c/tile.c/clip.c into different renderers -
    # exactly what slave_ppu.c's note about g_settings warns against, but in
    # the build system where no comment was watching.
    #
    # NO_WINDOW_CLIPPING is the one that blanks the picture. Without it the
    # slave calls ComputeClipWindows() every frame; with it, the master never
    # calls it at all for a non-SuperFX game. IPPU.Clip drives the draw loops,
    # so the two builds disagree about which pixels exist.
    #
    # NO_ZERO_LUT also frees the slave's 128 KB GFX.ZERO allocation, and
    # SIMPLE_COLOR_MATH takes the same colour-math path the master takes.
    #
    # Found by diffing the two builds' flags.make rather than by reading the
    # renderer, which is the only way this class of mismatch shows up:
    # FRANK_SNES_FAST_MODE alone changes the BG cache, the tile-dirty
    # tracking, the colour-math path, the backdrop fill and whether
    # FLUSH_REDRAW_EFFECT does anything at all.
    target_compile_definitions(frank-snes-slave PRIVATE
        NO_WINDOW_CLIPPING=1
        SIMPLE_COLOR_MATH=1
        NO_ZERO_LUT=1
        FRANK_SNES_FAST_MODE=1)
endif()
