cmake_minimum_required(VERSION 3.20.0)

# Add additional modules (before find_package)
set(ZEPHYR_EXTRA_MODULES "${CMAKE_SOURCE_DIR}/../../modules/mcp9808")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(read_temp)

target_sources(app PRIVATE src/main.c)