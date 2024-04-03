include_directories(include)

add_executable(spd_main src/main.c src/bno055.c)

# pull in common dependencies
target_link_libraries(spd_main 
                pico_stdio
                pico_stdlib
                hardware_gpio
                hardware_spi
                hardware_irq
                hardware_i2c
                hardware_adc)

pico_enable_stdio_usb(spd_main 1)
pico_enable_stdio_uart(spd_main 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(spd_main)