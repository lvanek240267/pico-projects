include(../pico_sdk_import.cmake)

add_executable(my_blink
        my_blink.c
        )

# pull in common dependencies
target_link_libraries(my_blink pico_stdlib hardware_i2c)

# create map/bin/hex file etc.
pico_add_extra_outputs(my_blink)

# add url via pico_set_program_url
example_auto_set_url(my_blink)

# Enable usb output, disable uart output
pico_enable_stdio_usb(my_blink 1)
pico_enable_stdio_uart(my_blink 1)
