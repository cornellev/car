add_board(hardware_pwm hardware_adc hardware_i2c)

# drop telemetry after 1 ms of a full usb buffer instead of the default 500 ms, which would
# stall the main loop (and trip the command timeout) whenever the host isn't reading
target_compile_definitions(merceves PRIVATE PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000)
