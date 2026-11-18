# library target
add_library(rp2040_mcp251863 STATIC
    src/mcp251863.cpp
)

target_include_directories(rp2040_mcp251863 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(
        rp2040_mcp251863 PUBLIC
        hardware_spi
        hardware_dma
        pico_stdlib
)

target_compile_features(rp2040_mcp251863 PUBLIC cxx_std_23)

target_compile_options(rp2040_mcp251863 PRIVATE -Wall -Wextra)

# build the examples so the build and lint check them
foreach(example interrupt_test_rx interrupt_test_tx multiple_fifos_test_rx multiple_fifos_test_tx)
    add_executable(mcp251863_${example} examples/${example}.cpp)
    target_link_libraries(mcp251863_${example} PRIVATE rp2040_mcp251863)
    target_compile_options(mcp251863_${example} PRIVATE -Wall -Wextra)
endforeach()
