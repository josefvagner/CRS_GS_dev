project(bootup C)

add_executable(bootup
        main.c
        )

target_link_libraries(bootup lgpio)
