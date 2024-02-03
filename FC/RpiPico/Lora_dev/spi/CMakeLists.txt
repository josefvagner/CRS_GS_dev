add_executable(spi_dma
        spi_dma.c
        )

target_link_libraries(spi_dma pico_stdlib hardware_spi hardware_dma)

# create map/bin/hex file etc.
pico_add_extra_outputs(spi_dma)
