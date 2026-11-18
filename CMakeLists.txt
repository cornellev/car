cmake_minimum_required(VERSION 3.21)

# firmware and host code need different compilers, so each preset in CMakePresets.json
# builds one of them in its own folder
set(CEV_BUILD "" CACHE STRING "what this build folder builds: firmware or host")
set_property(CACHE CEV_BUILD PROPERTY STRINGS firmware host)
if(NOT CEV_BUILD MATCHES "^(firmware|host)$")
    message(FATAL_ERROR "CEV_BUILD is \"${CEV_BUILD}\". Configure with `cmake --preset firmware` or "
        "`cmake --preset host` from the repo root")
endif()
# a build folder keeps the compiler it was first configured with
if(DEFINED CACHE{CEV_BUILD_FIRST} AND NOT CEV_BUILD STREQUAL CEV_BUILD_FIRST)
    message(FATAL_ERROR "${CMAKE_BINARY_DIR} was configured with CEV_BUILD=${CEV_BUILD_FIRST}, "
        "so it can't switch to ${CEV_BUILD}. Use the other preset, which has its own folder")
endif()
set(CEV_BUILD_FIRST ${CEV_BUILD} CACHE INTERNAL "")

if(CEV_BUILD STREQUAL "firmware")
    # the toolchain must be chosen before project()
    set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/vendor/pico-sdk)
    if(NOT EXISTS ${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
        message(FATAL_ERROR "vendor/pico-sdk is empty. Run `git submodule update --init` and "
            "`git -C vendor/pico-sdk submodule update --init lib/tinyusb` from the repo root, then build again")
    endif()
    include(elec/cmake/pinned_tools.cmake)
    include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
    project(urban_concept C CXX ASM)
else()
    project(urban_concept CXX)
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CEV_BUILD STREQUAL "firmware")
    pico_sdk_init()
    add_subdirectory(cev-lib/RP2040_MCP251863_Driver)
    add_subdirectory(cev-lib/chuds)
    add_subdirectory(elec)
else()
    # ctest reads the tests from the top build folder
    enable_testing()
    set(CHUDS_BUILD_HOST_TARGETS ON)
    add_subdirectory(cev-lib/chuds)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        add_subdirectory(telem/shm)
    endif()
endif()
