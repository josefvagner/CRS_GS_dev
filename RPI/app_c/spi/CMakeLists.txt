include_directories(include)

#add_executable(main_rx
#        src/main_rx.c src/sx1280_spi.c src/cJSON.c
#        )

add_executable(main
        src/main.c src/sx1280_spi.c src/cJSON.c
        )

#add_executable(testing_rx
#        src/testing_rx.c src/sx1280_spi.c src/cJSON.c
#        )

# pull in common dependencies
#target_link_libraries(main_rx pigpiod_if2)
#target_link_libraries(testing_rx pigpiod_if2)
target_link_libraries(main pigpiod_if2)
