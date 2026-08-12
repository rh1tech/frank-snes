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

add_executable(frank-snes-slave
    src/main.c
    src/slave_glue.c
    src/slave_sound.c
    src/slave_ppu.c
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
    hardware_clocks
    hardware_dma
    hardware_gpio
    hardware_pio
    hardware_sync
    hardware_vreg
    link
)

# Console on UART1 via the J4 header — the slave's USB-C port stays free
# for BOOTSEL, and SWD on J3 is the primary debug route anyway.
pico_enable_stdio_uart(frank-snes-slave 1)
pico_enable_stdio_usb(frank-snes-slave 0)

pico_add_extra_outputs(frank-snes-slave)
