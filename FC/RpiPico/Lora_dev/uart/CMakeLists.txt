include_directories(include)

add_executable(main_tx
        src/main_tx.c src/sx1280_uart.c
        )

add_executable(main_rx
        src/main_rx.c src/sx1280_uart.c
        )

add_executable(testing_tx
        src/testing_tx.c src/sx1280_uart.c
)

add_executable(testing_rx
        src/testing_rx.c src/sx1280_uart.c src/cJSON.c
)

# pull in common dependencies
target_link_libraries(main_tx pico_stdlib)
target_link_libraries(main_rx pico_stdlib)
target_link_libraries(testing_tx pico_stdlib)
target_link_libraries(testing_rx pico_stdlib)

pico_enable_stdio_usb(main_tx 1)
pico_enable_stdio_uart(main_tx 0)

pico_enable_stdio_usb(main_rx 1)
pico_enable_stdio_uart(main_rx 0)

pico_enable_stdio_usb(testing_tx 1)
pico_enable_stdio_uart(testing_tx 0)

pico_enable_stdio_usb(testing_rx 1)
pico_enable_stdio_uart(testing_rx 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(main_tx)
pico_add_extra_outputs(main_rx)
pico_add_extra_outputs(testing_tx)
pico_add_extra_outputs(testing_rx)