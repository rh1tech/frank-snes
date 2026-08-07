cmake_minimum_required(VERSION 3.13)

# Board configuration variant (M1, M2, C2)
set(BOARD_VARIANT "M1" CACHE STRING "Board variant: M1, M2 or C2")

if(BOARD_VARIANT STREQUAL "C2")
    # FRANK Core 2 master is an RP2350B. The stock pico2 definition
    # declares PICO_RP2350A 1, capping NUM_BANK0_GPIOS at 30 and
    # switching off PIO's movable GPIO window — the link bus (GPIO20..43),
    # the WS2812 (GPIO46) and PSRAM CS (GPIO47) would alias down into
    # 0..29. Our own header sets PICO_RP2350A 0 / PICO_PIO_USE_GPIO_BASE 1.
    set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards CACHE PATH "" FORCE)
    set(PICO_BOARD "frank_core2_master" CACHE STRING "Pico board type" FORCE)
    # GPIO20..43 are the inter-processor link, so there is no NES pad
    # header and no PS/2 header — USB HID is the only input path.
    set(USB_HID_ENABLED ON CACHE BOOL "USB HID host" FORCE)
else()
    # Set board to Pico 2 (RP2350)
    set(PICO_BOARD pico2 CACHE STRING "Pico board type")
endif()

# C2 offloads the DSP and the mixer to the slave, so the master also
# carries its half of the link.
# C2_LOCAL_SOUND builds the C2 master with sound still generated locally,
# for A/B comparison against the offloaded path on identical hardware.
set(C2_LOCAL_SOUND "0" CACHE STRING "C2: generate sound locally instead of on the slave")

if(BOARD_VARIANT STREQUAL "C2" AND NOT C2_LOCAL_SOUND)
    set(C2_SOUND_LINK ON)
endif()

# CPU overclock configuration (252, 378, 504 MHz)
set(CPU_SPEED "504" CACHE STRING "CPU clock in MHz: 252, 378, 504")

# PSRAM speed configuration (84, 100, 133 MHz target)
set(PSRAM_SPEED "133" CACHE STRING "PSRAM max frequency in MHz: 84, 100, 133")

# Lightweight on-device profiling (prints once per second over UART)
option(FRANK_SNES_PROFILE "Enable periodic performance profiling output" OFF)

# DSP command logging (KON/KOFF/FLG/GAIN to serial)
option(FRANK_SNES_DSP_LOG "Enable DSP command logging output" OFF)

# Re-hash the ROM in PSRAM once a second and count mismatches. Proves
# whether the PSRAM overclock is stable at the configured speed.
option(FRANK_SNES_PSRAM_CHECK "Verify the ROM image in PSRAM at runtime" OFF)

# Record the stream going to the DAC into PSRAM, for offline analysis.
option(FRANK_SNES_AUDIO_CAPTURE "Capture output audio to PSRAM" OFF)

# Test override for the runtime frameskip setting (0-4).
set(FRANK_SNES_FORCE_FRAMESKIP "" CACHE STRING "Force frameskip level 0-4")

# SuperFX diagnostics: per-second counters printed to serial
option(FRANK_SNES_SUPERFX_DIAG "Enable SuperFX GSU execution diagnostics" OFF)

# Autoboot: skip welcome + selector, hardcode Doom path (debugging)
option(FRANK_SNES_AUTOBOOT "Auto-load Doom on boot (skip UI)" OFF)

# Autopad: scripted gamepad inputs to drive title/menus (debugging)
option(FRANK_SNES_AUTOPAD "Inject scripted button presses" OFF)

# Fast mode: aggressively reduce quality for higher framerate
option(FRANK_SNES_FAST_MODE "Enable fast mode (reduced quality for higher FPS)" ON)

# USB HID gamepad/keyboard support (enabled by default)
option(USB_HID_ENABLED "Enable USB HID host for gamepads and keyboards" ON)

# Sound core selection.
#   LEGACY — the original snes9x 1.43-era path: soundux.c's block mixer
#            driven by spc700.c's scaled-cycle SPC700. This is what M1
#            and M2 have always shipped and what they keep: those boards
#            are single-chip and cannot afford the accurate core, and
#            changing a sound path that works on released hardware is
#            not something to do as a side effect.
#   DSP    — accurate S-DSP (spc_dsp.c) on the slave, driven by the
#            original SPC700. The DSP is the part that renders sound, so
#            this gets the accuracy where it matters while leaving the
#            master's CPU cost unchanged. C2 default.
#   ACCURATE — as DSP, but also replaces the SPC700 with blargg's
#            cycle-driven core (spc700_blargg.c). Strictly more correct
#            — the $2140..$2143 handshake is emulated rather than
#            approximated — but it is markedly more expensive on the
#            master: Mortal Kombat 3 drops from 60 fps to about 42 in a
#            fight, and everything then runs in slow motion. Kept for
#            when accuracy matters more than speed.
#
# Defaults per board; override explicitly to compare.
if(BOARD_VARIANT STREQUAL "C2")
    set(SOUND_CORE_DEFAULT "DSP")
else()
    set(SOUND_CORE_DEFAULT "LEGACY")
endif()
set(SOUND_CORE "${SOUND_CORE_DEFAULT}" CACHE STRING "Sound core: DSP or LEGACY")
set_property(CACHE SOUND_CORE PROPERTY STRINGS DSP ACCURATE LEGACY)
message(STATUS "Sound core: ${SOUND_CORE}")

# HDMI driver selection.  PIO (default) is the bespoke palette-indexed
# scanline driver in drivers/HDMI.c; ALT switches to the libdvi-backed
# path with HDMI audio data-island support, vendored under
# drivers/libdvi/.
set(HDMI_DRIVER "PIO" CACHE STRING "HDMI driver: PIO or ALT")
set_property(CACHE HDMI_DRIVER PROPERTY STRINGS PIO ALT)
message(STATUS "HDMI driver: ${HDMI_DRIVER}")

# CPU voltage selection based on speed
if(CPU_SPEED GREATER_EQUAL 504)
    set(CPU_VOLTAGE "VREG_VOLTAGE_1_65")
elseif(CPU_SPEED GREATER_EQUAL 378)
    set(CPU_VOLTAGE "VREG_VOLTAGE_1_60")
else()
    set(CPU_VOLTAGE "VREG_VOLTAGE_1_50")
endif()

message(STATUS "Board: ${BOARD_VARIANT}, CPU: ${CPU_SPEED} MHz, PSRAM: ${PSRAM_SPEED} MHz, Voltage: ${CPU_VOLTAGE}")

include(pico_sdk_import.cmake)

project(frank-snes C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Read version from version.txt (format: "MAJOR MINOR", e.g. "1 00" -> "1.00")
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" FRANK_SNES_VERSION_RAW)
string(STRIP "${FRANK_SNES_VERSION_RAW}" FRANK_SNES_VERSION_RAW)
string(REPLACE " " "." FRANK_SNES_VERSION_STR "${FRANK_SNES_VERSION_RAW}")

add_subdirectory(src/fatfs)
add_subdirectory(drivers/sdcard)
add_subdirectory(drivers/usbhid)

# The NES pad and the PS/2 headers do not exist on C2 — their GPIOs are
# the inter-processor link — and the PS/2 keyboard would want PIO2, which
# the link needs for its upper-window bus. Both drivers are left out
# rather than built and never called.
if(NOT BOARD_VARIANT STREQUAL "C2")
    add_subdirectory(drivers/nespad)
    add_subdirectory(drivers/ps2)
    add_subdirectory(src/ps2kbd)
endif()

# Inter-processor sound link — C2 only. The slave half is built by
# slave/CMakeLists.txt, which links the same library.
if(C2_SOUND_LINK)
    add_subdirectory(link)
endif()

if(HDMI_DRIVER STREQUAL "ALT")
    add_subdirectory(drivers/libdvi)
endif()

# Per-driver source set
if(HDMI_DRIVER STREQUAL "ALT")
    set(HDMI_DRIVER_SOURCES drivers/hdmi_alt.c)
else()
    set(HDMI_DRIVER_SOURCES drivers/HDMI.c drivers/hdmi_scanline.S)
endif()

# Driver library
add_library(drivers
    ${HDMI_DRIVER_SOURCES}
    drivers/psram_init.c
    drivers/psram_allocator.c
    drivers/audio.c
    drivers/audio_i2s_opt.S
)
target_include_directories(drivers PUBLIC drivers src)
target_compile_definitions(drivers PRIVATE
    BOARD_${BOARD_VARIANT}
    PSRAM_MAX_FREQ_MHZ=${PSRAM_SPEED}
)

if(HDMI_DRIVER STREQUAL "ALT")
    target_compile_definitions(drivers PUBLIC FRANK_SNES_HDMI_ALT=1)
    target_link_libraries(drivers libdvi)
endif()

pico_generate_pio_header(drivers ${CMAKE_CURRENT_LIST_DIR}/drivers/audio_i2s.pio)

target_link_libraries(drivers pico_stdlib hardware_dma hardware_pio hardware_spi hardware_gpio)

# Snes9x emulator sources
set(SNES9X_SOURCES
    src/snes9x/c4.c
    src/snes9x/c4emu.c
    src/snes9x/clip.c
    src/snes9x/cpu.c
    src/snes9x/cpuexec.c
    src/snes9x/cpuops.c
    src/snes9x/dma.c
    src/snes9x/dsp.c
    src/snes9x/fxemu.c
    src/snes9x/getset.c
    src/snes9x/colormath.c
    src/snes9x/gfx.c
    src/snes9x/globals.c
    src/snes9x/memmap.c
    src/snes9x/obc1.c
    src/snes9x/ppu.c
    src/snes9x/snapshot.c
    src/snes9x/srtc.c
    src/snes9x/tile.c
    src/snes9x/cpu_asm.S
    src/snes9x/tile_asm.S
)

# The sample-generating half of the APU. On C2 it runs on the slave
# instead, and src/sound_backend_link.c stands in its place — see
# docs/C2_SOUND_SPLIT.md. The slave build compiles the same sources, so
# the two halves can never generate sound differently.
#
# These stay out of SNES9X_SOURCES: that list carries the upstream
# Snes9x -O3 flags, and the link code is ours.
# The APU. Two whole cores, selected together — the SPC700 and the DSP
# are a matched pair and cannot be mixed.
set(C2_SOURCES "")
if(SOUND_CORE STREQUAL "ACCURATE")
    # blargg's SPC700 clocking blargg's S-DSP.
    list(APPEND SNES9X_SOURCES
        src/snes9x/spc700_blargg.c
        src/snes9x/spc_dsp.c
        src/snes9x/apu_dsp.c)
else()
    # The original SPC700 and its register decode. LEGACY adds the block
    # mixer behind it; DSP leaves sample generation to spc_dsp.c, which
    # on C2 lives on the slave.
    list(APPEND SNES9X_SOURCES
        src/snes9x/apu.c
        src/snes9x/spc700.c
        src/snes9x/apu_core1.c)
    if(SOUND_CORE STREQUAL "LEGACY")
        list(APPEND SNES9X_SOURCES src/snes9x/soundux.c)
    else()
        list(APPEND SNES9X_SOURCES src/snes9x/spc_dsp.c src/snes9x/apu_dsp.c)
    endif()
endif()

# On C2 with the offload, sound_backend_link.c replaces apu_dsp.c: the
# DSP runs on the slave, so the master needs the seam and the audio
# plumbing but not spc_dsp.c's renderer.
if(C2_SOUND_LINK)
    list(APPEND C2_SOURCES src/sound_backend_link.c src/link_master.c
                           src/link_aram.c)
    list(REMOVE_ITEM SNES9X_SOURCES src/snes9x/apu_dsp.c)
endif()

# C2 has no NES pad and no PS/2 header; the stubs answer their API with
# "nothing connected" so main.c and rom_selector.c compile unchanged.
if(BOARD_VARIANT STREQUAL "C2")
    list(APPEND C2_SOURCES src/c2_input_stubs.c)
endif()

# Assembly optimizations
set(ASM_OPT_SOURCES
    src/asm/tile_convert_optimized.S
    src/tile_convert_opt.c
    src/audio_opt.c
    src/asm/audio_pack_asm.S
)

# UI sources
set(UI_SOURCES
    src/menu_ui.c
    src/rom_selector.c
    src/settings.c
)

add_executable(frank-snes
    src/main.c
    src/frank_snes_profile.c
    ${SNES9X_SOURCES}
    ${ASM_OPT_SOURCES}
    ${UI_SOURCES}
    ${C2_SOURCES}
)

set_source_files_properties(${SNES9X_SOURCES} PROPERTIES COMPILE_FLAGS
    "-O3 -fno-strict-aliasing -funroll-loops -fomit-frame-pointer")

target_include_directories(frank-snes PRIVATE
    src
    src/snes9x
    src/fatfs
    drivers
    drivers/sdcard
    drivers/nespad
    drivers/ps2
    src/ps2kbd
    link
)

target_compile_definitions(frank-snes PRIVATE
    BOARD_${BOARD_VARIANT}
    CPU_CLOCK_MHZ=${CPU_SPEED}
    CPU_VOLTAGE=${CPU_VOLTAGE}
    PSRAM_MAX_FREQ_MHZ=${PSRAM_SPEED}
    PICO_ON_DEVICE=1
    FRANK_SNES_VERSION="${FRANK_SNES_VERSION_STR}"
    NO_WINDOW_CLIPPING=1
    SIMPLE_COLOR_MATH=1
    NO_ZERO_LUT=1
)

if(SOUND_CORE STREQUAL "ACCURATE")
    # Switches every shared snes9x file onto its blargg branch: cpuexec.c,
    # cpuops.c, dma.c, memmap.c, globals.c, apu.h, soundux.h, spc700.h.
    # That scaffolding was already in the tree from upstream.
    target_compile_definitions(frank-snes PRIVATE
        USE_BLARGG_APU=1
        SOUND_CORE_DSP=1
        SPC700_ACCURATE=1)
elseif(SOUND_CORE STREQUAL "DSP")
    # Original SPC700, accurate DSP. apu.c's S9xSetAPUDSP/S9xGetAPUDSP
    # are replaced by the DSP driver.
    target_compile_definitions(frank-snes PRIVATE SOUND_CORE_DSP=1)
endif()

if(FRANK_SNES_PROFILE)
    target_compile_definitions(frank-snes PRIVATE FRANK_SNES_PROFILE=1)
endif()

if(FRANK_SNES_DSP_LOG)
    target_compile_definitions(frank-snes PRIVATE FRANK_SNES_DSP_LOG=1)
endif()

if(FRANK_SNES_PSRAM_CHECK)
    target_compile_definitions(frank-snes PRIVATE FRANK_SNES_PSRAM_CHECK=1)
endif()

if(FRANK_SNES_AUDIO_CAPTURE)
    target_compile_definitions(frank-snes PRIVATE FRANK_SNES_AUDIO_CAPTURE=1)
endif()

if(NOT FRANK_SNES_FORCE_FRAMESKIP STREQUAL "")
    target_compile_definitions(frank-snes PRIVATE
        FRANK_SNES_FORCE_FRAMESKIP=${FRANK_SNES_FORCE_FRAMESKIP})
endif()

if(FRANK_SNES_SUPERFX_DIAG)
    target_compile_definitions(frank-snes PRIVATE FRANK_SNES_SUPERFX_DIAG=1)
endif()

# Development aid: boot this exact ROM path instead of the built-in
# default. Plain if(), not a generator expression: ROM paths contain
# parentheses ("Super Mario World (USA).sfc") and those are syntax
# inside $<>, so the define would silently never appear.
set(AUTOBOOT_PATH "" CACHE STRING "FRANK_SNES_AUTOBOOT: boot this exact ROM path")

if(FRANK_SNES_AUTOBOOT)
    target_compile_definitions(frank-snes PRIVATE FRANK_SNES_AUTOBOOT=1)
    if(AUTOBOOT_PATH)
        target_compile_definitions(frank-snes PRIVATE AUTOBOOT_PATH="${AUTOBOOT_PATH}")
        message(STATUS "AUTOBOOT: ${AUTOBOOT_PATH}")
    endif()
endif()

if(FRANK_SNES_AUTOPAD)
    target_compile_definitions(frank-snes PRIVATE FRANK_SNES_AUTOPAD=1)
endif()

if(FRANK_SNES_FAST_MODE)
    target_compile_definitions(frank-snes PRIVATE FRANK_SNES_FAST_MODE=1)
    message(STATUS "FAST MODE enabled")
endif()

if(HDMI_DRIVER STREQUAL "ALT")
    target_compile_definitions(frank-snes PRIVATE FRANK_SNES_HDMI_ALT=1)
endif()


# Peripheral pins per board variant
if(BOARD_VARIANT STREQUAL "M1")
    target_compile_definitions(frank-snes PRIVATE
        SDCARD_PIN_SPI0_CS=5
        SDCARD_PIN_SPI0_SCK=2
        SDCARD_PIN_SPI0_MOSI=3
        SDCARD_PIN_SPI0_MISO=4
        NESPAD_GPIO_CLK=14
        NESPAD_GPIO_LATCH=15
        NESPAD_GPIO_DATA=16
    )
elseif(BOARD_VARIANT STREQUAL "M2")
    target_compile_definitions(frank-snes PRIVATE
        SDCARD_PIN_SPI0_CS=5
        SDCARD_PIN_SPI0_SCK=6
        SDCARD_PIN_SPI0_MOSI=7
        SDCARD_PIN_SPI0_MISO=4
        NESPAD_GPIO_CLK=20
        NESPAD_GPIO_LATCH=21
        NESPAD_GPIO_DATA=26
    )
elseif(BOARD_VARIANT STREQUAL "C2")
    # FRANK Core 2 master. HDMI/SD/I2S/PSRAM match M2 exactly; GPIO20..43
    # belong to the inter-processor link, so there are no gamepad or PS/2
    # pins — NESPAD_GPIO_* and PS2_PIN_* stay undefined and main.c skips
    # those drivers.
    target_compile_definitions(frank-snes PRIVATE
        SDCARD_PIN_SPI0_CS=5
        SDCARD_PIN_SPI0_SCK=6
        SDCARD_PIN_SPI0_MOSI=7
        SDCARD_PIN_SPI0_MISO=4
    )
endif()

if(C2_SOUND_LINK)
    target_compile_definitions(frank-snes PRIVATE C2_SOUND_LINK=1)
    target_link_libraries(frank-snes link)
    message(STATUS "C2: sound offloaded to the slave (build slave/ too)")
elseif(BOARD_VARIANT STREQUAL "C2")
    message(STATUS "C2: C2_LOCAL_SOUND — sound stays on the master")
endif()

target_link_libraries(frank-snes
    pico_stdlib
    pico_multicore
    hardware_spi
    hardware_dma
    hardware_pio
    hardware_pwm
    hardware_irq
    sdcard
    fatfs
    drivers
    usbhid
)

if(NOT BOARD_VARIANT STREQUAL "C2")
    target_link_libraries(frank-snes nespad ps2kbd)
endif()

# Dev builds: USB CDC serial for printf. Release builds (USB_HID): no serial.
#
# C2 is the exception: USB is the HID host and cannot also be a CDC
# console, but the board has a UART header (J2, GPIO0/1) that the debug
# probe sits on. Without this the C2 master would have no console at all
# — no way to see whether the sound slave answered, which is the one
# thing worth watching on this board.
if(BOARD_VARIANT STREQUAL "C2")
    pico_enable_stdio_uart(frank-snes 1)
    pico_enable_stdio_usb(frank-snes 0)
else()
    pico_enable_stdio_uart(frank-snes 0)
    if(USB_HID_ENABLED)
        pico_enable_stdio_usb(frank-snes 0)
    else()
        pico_enable_stdio_usb(frank-snes 1)
    endif()
endif()

pico_add_extra_outputs(frank-snes)
