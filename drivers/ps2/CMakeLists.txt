add_library(ps2_driver STATIC
    ${CMAKE_CURRENT_LIST_DIR}/ps2.c
)

pico_generate_pio_header(ps2_driver ${CMAKE_CURRENT_LIST_DIR}/ps2.pio)

target_include_directories(ps2_driver PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(ps2_driver PUBLIC
    pico_stdlib
    hardware_pio
    hardware_gpio
    hardware_clocks
    hardware_irq
    hardware_sync
)

if(USB_HID_ENABLED)
    target_compile_definitions(ps2_driver PRIVATE USB_HID_ENABLED=1)
endif()
