cmake_minimum_required(VERSION 3.22.0)

set(ZEPHYR_EXTRA_MODULES "${CMAKE_SOURCE_DIR}/../../modules/mcp9808")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(thread_demo)

target_sources(app PRIVATE src/main.c)