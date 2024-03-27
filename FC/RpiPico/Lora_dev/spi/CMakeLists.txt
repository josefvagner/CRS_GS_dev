include_directories(include)

add_executable(main_tx
        src/main_tx.c src/sx1280_spi.c
        )

add_executable(main_rx
        src/main_rx.c src/sx1280_spi.c
        )

# pull in common dependencies
target_link_libraries(main_tx 
                pico_stdio
                pico_stdlib
                hardware_gpio
                hardware_spi
                hardware_irq
                hardware_adc)
target_link_libraries(main_rx 
                pico_stdio
                pico_stdlib
                hardware_gpio
                hardware_spi
                hardware_irq
                hardware_adc)

pico_enable_stdio_usb(main_tx 1)
pico_enable_stdio_uart(main_tx 0)
                
pico_enable_stdio_usb(main_rx 1)
pico_enable_stdio_uart(main_rx 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(main_tx)
pico_add_extra_outputs(main_rx)