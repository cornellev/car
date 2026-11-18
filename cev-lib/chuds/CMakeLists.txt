# core library
add_library(chuds INTERFACE)
target_include_directories(chuds INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
target_compile_features(chuds INTERFACE cxx_std_23)

# each adapter exists only where it works, so linking one elsewhere fails with library not found
# opt-in Linux SocketCAN adapter
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_library(chuds_socketcan INTERFACE)
    target_link_libraries(chuds_socketcan INTERFACE chuds)
endif()

# opt-in MCP251863 adapter for RP2040, needs the chip driver target added first
if(TARGET rp2040_mcp251863)
    add_library(chuds_mcp INTERFACE)
    target_link_libraries(chuds_mcp INTERFACE chuds rp2040_mcp251863)
endif()

# warnings for every host target below
add_library(chuds_warnings INTERFACE)
target_compile_options(chuds_warnings INTERFACE -Wall -Wextra -Wconversion)

# the root's host build sets this to build the tests, and on Linux the examples and tools
if(CHUDS_BUILD_HOST_TARGETS)
    add_subdirectory(tests)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        add_subdirectory(examples)
        add_subdirectory(tools)
    endif()
endif()
