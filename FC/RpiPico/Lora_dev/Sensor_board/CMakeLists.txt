include_directories(include)

# Set the name
set(NAME Sensor_board)

add_executable(${NAME}
        src/main.c
        src/bmp3.c
        src/bno055.c
        src/common.c
        src/GFX.c
        src/SSD1309.c
        )

# Add the standard library to the build
target_link_libraries(${NAME}
        pico_stdlib
        pico_stdio
        hardware_i2c
        pico_malloc
        pico_mem_ops)

pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)


# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})