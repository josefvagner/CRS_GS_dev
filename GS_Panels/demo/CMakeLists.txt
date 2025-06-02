# Set the name
file(GLOB SRCS_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/*")
foreach(src_dir ${SRCS_DIRS})
    if(IS_DIRECTORY ${src_dir})
        add_subdirectory(${src_dir})
    endif()
endforeach()
