cmake_minimum_required(VERSION 3.13)

# Set board to Pico 2 (RP2350)
set(PICO_BOARD pico2 CACHE STRING "Pico board type")

# Board configuration variant (M1, M2)
set(BOARD_VARIANT "M1" CACHE STRING "Board variant: M1 or M2")

# CPU overclock configuration (252, 378, 504 MHz)
set(CPU_SPEED "504" CACHE STRING "CPU clock in MHz: 252, 378, 504")

# PSRAM speed configuration (84, 100, 133 MHz target)
set(PSRAM_SPEED "133" CACHE STRING "PSRAM max frequency in MHz: 84, 100, 133")

# Lightweight on-device profiling (prints once per second over UART)
option(FRANK_SNES_PROFILE "Enable periodic performance profiling output" OFF)

# DSP command logging (KON/KOFF/FLG/GAIN to serial)
option(FRANK_SNES_DSP_LOG "Enable DSP command logging output" OFF)

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
add_subdirectory(drivers/nespad)
add_subdirectory(drivers/ps2kbd)
add_subdirectory(drivers/usbhid)

# Driver library
add_library(drivers
    drivers/HDMI.c
    drivers/hdmi_scanline.S
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

pico_generate_pio_header(drivers ${CMAKE_CURRENT_LIST_DIR}/drivers/audio_i2s.pio)

target_link_libraries(drivers pico_stdlib hardware_dma hardware_pio hardware_spi hardware_gpio)

# Snes9x emulator sources
set(SNES9X_SOURCES
    src/snes9x/apu.c
    src/snes9x/apu_core1.c
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
    src/snes9x/soundux.c
    src/snes9x/spc700.c
    src/snes9x/srtc.c
    src/snes9x/tile.c
    src/snes9x/cpu_asm.S
    src/snes9x/tile_asm.S
)

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
    drivers/ps2kbd
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

if(FRANK_SNES_PROFILE)
    target_compile_definitions(frank-snes PRIVATE FRANK_SNES_PROFILE=1)
endif()

if(FRANK_SNES_DSP_LOG)
    target_compile_definitions(frank-snes PRIVATE FRANK_SNES_DSP_LOG=1)
endif()

if(FRANK_SNES_SUPERFX_DIAG)
    target_compile_definitions(frank-snes PRIVATE FRANK_SNES_SUPERFX_DIAG=1)
endif()

if(FRANK_SNES_AUTOBOOT)
    target_compile_definitions(frank-snes PRIVATE FRANK_SNES_AUTOBOOT=1)
endif()

if(FRANK_SNES_AUTOPAD)
    target_compile_definitions(frank-snes PRIVATE FRANK_SNES_AUTOPAD=1)
endif()

if(FRANK_SNES_FAST_MODE)
    target_compile_definitions(frank-snes PRIVATE FRANK_SNES_FAST_MODE=1)
    message(STATUS "FAST MODE enabled")
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
    nespad
    ps2kbd
    usbhid
)

# Dev builds: USB CDC serial for printf. Release builds (USB_HID): no serial.
pico_enable_stdio_uart(frank-snes 0)
if(USB_HID_ENABLED)
    pico_enable_stdio_usb(frank-snes 0)
else()
    pico_enable_stdio_usb(frank-snes 1)
endif()

pico_add_extra_outputs(frank-snes)
