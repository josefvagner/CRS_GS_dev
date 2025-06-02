include_directories(include)

# Set the name
set(NAME cansat_panel)


add_executable(${NAME}
        src/main.c
)

# Add the standard library to the build
target_link_libraries(${NAME}
        pico_stdlib
        can
        pico_mem_ops
        ws2812
)

pico_enable_stdio_usb(${NAME} 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})
