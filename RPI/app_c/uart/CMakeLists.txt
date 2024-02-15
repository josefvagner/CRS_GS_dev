include_directories(include)

add_executable(main_rx
        src/main_rx.c src/sx1280_uart.c src/cJSON.c
        )

# pull in common dependencies
target_link_libraries(main_rx pigpiod_if2)
