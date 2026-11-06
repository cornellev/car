# board_common (src/common) already brings pico_stdlib, spi, gpio, and the chuds mcp transport
add_board(hardware_adc hardware_pwm hardware_watchdog hardware_irq hardware_sync)
target_sources(precharge PRIVATE state_machine.cpp)

# printf over USB waits up to 500 ms by default when a terminal is open but not reading,
# which would trip the 100 ms watchdog. Drop the output after 1 ms instead.
target_compile_definitions(precharge PRIVATE PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000)
