add_library(ps2kbd STATIC
    ${CMAKE_CURRENT_LIST_DIR}/ps2kbd_mrmltr.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ps2kbd_wrapper.cpp
)

target_include_directories(ps2kbd PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_include_directories(ps2kbd PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/drivers
)

target_link_libraries(ps2kbd PUBLIC
    pico_stdlib
    ps2_driver
)

# Board variant define (for pin selection in board_config.h)
if(BOARD_VARIANT STREQUAL "M2")
    target_compile_definitions(ps2kbd PRIVATE BOARD_M2)
else()
    target_compile_definitions(ps2kbd PRIVATE BOARD_M1)
endif()
