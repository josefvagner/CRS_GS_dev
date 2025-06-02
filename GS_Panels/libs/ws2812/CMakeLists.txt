include_directories(include)

# Set the name
set(LIB_NAME ws2812)

# Add your source files
set(LIB_SOURCES
    src/ws2812.c
)

# Create the library
add_library(${LIB_NAME} ${LIB_SOURCES})


# Include directories for the library
target_include_directories(${LIB_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    # Add other include directories if necessary
)

# Link other libraries
target_link_libraries(${LIB_NAME}
        pico_stdlib
        pico_stdlib
        pico_malloc
        pico_mem_ops
        hardware_pio
        hardware_dma
        hardware_pwm
)