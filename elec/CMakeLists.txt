# the rp2040 firmware, added by the root CMakeLists.txt in its firmware mode

# a missing tinyusb submodule only warns in pico-sdk, and the boards would build without usb
if(NOT TARGET pico_stdio_usb)
    message(FATAL_ERROR "pico-sdk has no USB support because its tinyusb submodule is missing. Run "
        "`git -C vendor/pico-sdk submodule update --init lib/tinyusb` from the repo root, then build again")
endif()

string(REPLACE "." "\\." arm_re "${arm_version}")
if(NOT CMAKE_C_COMPILER_VERSION MATCHES "^${arm_re}(\\.|$)")
    message(FATAL_ERROR "arm-gcc is ${CMAKE_C_COMPILER_VERSION}, but the build pins ${arm_release} "
        "(elec/cmake/pinned_tools.cmake). If the pin just changed, run `just clean` from elec/ and build again")
endif()

add_subdirectory(src/common)

# a board built from its folder's main.cpp, named after the folder so `just flash <folder>` finds it
# arguments are extra libraries on top of board_common
function(add_board)
    get_filename_component(board ${CMAKE_CURRENT_SOURCE_DIR} NAME)
    add_executable(${board} main.cpp)
    target_compile_options(${board} PRIVATE -Wall -Wextra)
    target_link_libraries(${board} PRIVATE board_common pico_bootsel_via_double_reset ${ARGN})
    pico_enable_stdio_uart(${board} 0)
    pico_enable_stdio_usb(${board} 1)
    # fail the build if the board links the heap allocator, before the .uf2 is written
    add_custom_command(TARGET ${board} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -DNM=${CMAKE_NM} -DELF=$<TARGET_FILE:${board}>
                -P ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/cmake/forbid_heap.cmake)
    pico_add_extra_outputs(${board})
    # pico_add_extra_outputs defines the picotool target the build found, in this folder
    add_custom_target(flash_${board}
        COMMAND picotool load -f -x $<TARGET_FILE_DIR:${board}>/${board}.uf2
        DEPENDS ${board}
        USES_TERMINAL)
endfunction()

# firmware inventory; add a board with an explicit line
add_subdirectory(src/front_aux)
add_subdirectory(src/back_aux)
add_subdirectory(src/joulemeter)
add_subdirectory(src/devboard)
add_subdirectory(src/template-project)
add_subdirectory(src/fbaux_test)
