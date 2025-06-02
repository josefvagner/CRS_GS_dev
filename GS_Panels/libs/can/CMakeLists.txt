include_directories(include)

# Set the name
set(LIB_NAME can)

# Add your source files
set(LIB_SOURCES
    src/can.c
)

# Create the library
add_library(${LIB_NAME} ${LIB_SOURCES})

target_sources(${LIB_NAME} PRIVATE
    src/can2040.c
)

# Include directories for the library
target_include_directories(${LIB_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link other libraries
target_link_libraries(${LIB_NAME}
        pico_stdlib
        hardware_irq
        hardware_pio
        pico_base_headers
        hardware_dma
        cmsis_core
)
