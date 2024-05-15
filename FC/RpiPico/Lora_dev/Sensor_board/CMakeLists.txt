# Set the name
set(NAME_TX TX)
set(NAME_RX RX)

# Include directories
include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

# Add the main executable
add_executable(${NAME_TX}
    src/transciever.c
    src/bmp3.c
    src/bno055.c
    src/common.c
    src/sx1280_spi.c
    src/GFX_wrapper.cpp
)

# Add the main executable
add_executable(${NAME_RX}
    src/reciever.c
    src/bmp3.c
    src/bno055.c
    src/common.c
    src/sx1280_spi.c
    src/GFX_wrapper.cpp
)

# Add the SSD1309 library as a static library
add_library(ssd1309 STATIC
    src/SSD1309.cpp
    src/GFX.cpp
)

# Include directories for the SSD1309 library
target_include_directories(ssd1309 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(ssd1309
    pico_stdlib
    hardware_i2c
    pico_malloc
    pico_mem_ops
    hardware_spi
    hardware_irq
    hardware_pio
)

# Add the standard library and hardware libraries to the build
target_link_libraries(${NAME_TX}
    pico_stdlib
    pico_stdio
    hardware_i2c
    pico_malloc
    pico_mem_ops
    hardware_spi
    hardware_irq
    hardware_pio
    ssd1309
)

target_link_libraries(${NAME_RX}
    pico_stdlib
    pico_stdio
    hardware_i2c
    pico_malloc
    pico_mem_ops
    hardware_spi
    hardware_irq
    hardware_pio
    ssd1309
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(${NAME_TX} 1)
pico_enable_stdio_uart(${NAME_TX} 0)

pico_enable_stdio_usb(${NAME_RX} 1)
pico_enable_stdio_uart(${NAME_RX} 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${NAME_TX})
pico_add_extra_outputs(${NAME_RX})
