# Set the name
file(GLOB SRCS_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/*")
foreach(src_dir ${SRCS_DIRS})
    # TODO: Enable once kalman is fixed
    if(IS_DIRECTORY ${src_dir} AND NOT src_dir MATCHES "kalman")
        add_subdirectory(${src_dir})
    endif()
endforeach()
