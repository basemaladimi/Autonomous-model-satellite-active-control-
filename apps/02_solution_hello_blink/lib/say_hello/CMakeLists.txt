# Create the library target
add_library(hello_lib STATIC src/say_hello.c)

# Add the library's header file(s)
target_include_directories(
    hello_lib 
    PUBLIC   
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Required to set Zephyr compile options (e.g. for ARM)
target_link_libraries(hello_lib PRIVATE zephyr_interface)
