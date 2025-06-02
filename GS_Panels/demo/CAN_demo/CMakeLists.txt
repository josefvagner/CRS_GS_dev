include_directories(include)

add_executable(CAN_demo_tx
        src/CAN_demo_tx.c
        )

add_executable(CAN_demo_rx
        src/CAN_demo_rx.c
        )

# Add the standard library to the build
target_link_libraries(CAN_demo_rx
        pico_stdlib
        can
        ws2812)
target_link_libraries(CAN_demo_tx
        pico_stdlib
        can
        ws2812)



pico_enable_stdio_usb(CAN_demo_tx 1)

pico_enable_stdio_usb(CAN_demo_rx 1)


# create map/bin/hex file etc.
pico_add_extra_outputs(CAN_demo_tx)
pico_add_extra_outputs(CAN_demo_rx)