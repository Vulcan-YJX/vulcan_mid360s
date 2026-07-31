file(GLOB_RECURSE files "*.c")

set_source_files_properties(${files} PROPERTIES COMPILE_FLAGS -Wno-error)

add_library(Fusion ${files})

if(UNIX AND NOT APPLE)
    target_link_libraries(Fusion m) # link math library for Linux
endif()
