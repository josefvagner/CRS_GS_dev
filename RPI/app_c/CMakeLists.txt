cmake_minimum_required(VERSION 3.12)

project(rpi_sx1280 C)
set(CMAKE_C_STANDARD 11)


#add_subdirectory(spi)
add_subdirectory(uart)

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )
