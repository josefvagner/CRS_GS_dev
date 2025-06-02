project(main C)

include_directories(include)

add_executable(main
        src/main.c src/sx1280_spi.c src/cJSON.c
        )

target_link_libraries(main lgpio)
