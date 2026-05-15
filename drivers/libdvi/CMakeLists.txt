########################################################################
# Vendored libdvi
#
# Source: https://github.com/Wren6991/PicoDVI (with audio additions from
# https://github.com/shuichitakano/PicoDVI-audio).  Copied verbatim from
# ~/Documents/GitHub/PicoDVI-audio/software/libdvi.
#
# Used by the HDMI_ALT path (FRANK_SNES_HDMI_ALT).  Not built when the
# default HDMI_PIO driver is selected.
########################################################################

add_library(libdvi INTERFACE)

target_sources(libdvi INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/dvi.c
    ${CMAKE_CURRENT_LIST_DIR}/dvi_serialiser.c
    ${CMAKE_CURRENT_LIST_DIR}/dvi_timing.c
    ${CMAKE_CURRENT_LIST_DIR}/tmds_encode.c
    ${CMAKE_CURRENT_LIST_DIR}/tmds_encode.S
    ${CMAKE_CURRENT_LIST_DIR}/data_packet.c
    ${CMAKE_CURRENT_LIST_DIR}/audio_ring.c
)

target_include_directories(libdvi INTERFACE ${CMAKE_CURRENT_LIST_DIR})

# Match pico-zxspectrum's MURMULATOR2 target: 3 TMDS buffers.  This
# gives the IRQ handler enough slack that occasional Core 0 SRAM
# bursts (rom-selector carousel sprite scaling, etc.) don't underrun
# the FIFOs and flash a red "late scanline" indicator.
target_compile_definitions(libdvi INTERFACE
    DVI_N_TMDS_BUFFERS=3
)

target_link_libraries(libdvi INTERFACE
    pico_base_headers
    pico_util
    hardware_dma
    hardware_interp
    hardware_pio
    hardware_pwm
)

pico_generate_pio_header(libdvi ${CMAKE_CURRENT_LIST_DIR}/dvi_serialiser.pio)
pico_generate_pio_header(libdvi ${CMAKE_CURRENT_LIST_DIR}/tmds_encode_1bpp.pio)
pico_generate_pio_header(libdvi ${CMAKE_CURRENT_LIST_DIR}/dvi_clock.pio)
