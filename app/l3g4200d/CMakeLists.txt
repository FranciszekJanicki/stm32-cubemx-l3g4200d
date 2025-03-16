add_library(l3g4200d STATIC)

target_sources(l3g4200d PRIVATE
    "l3g4200d.cpp"
)

target_include_directories(l3g4200d PUBLIC 
    "."
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(l3g4200d PRIVATE
    utility
    stm32cubemx
)

target_compile_options(l3g4200d PUBLIC
    -std=c++23
    -Wall
    -Wextra
    -Wconversion
    -Wshadow
    -Wpedantic
    -Wnarrowing
    -Waddress
    -pedantic
    -Wdeprecated
    -Wsign-conversion
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wnull-dereference
    -Wdouble-promotion
    -Wimplicit-fallthrough
    -Wcast-align
    -fconcepts
)
